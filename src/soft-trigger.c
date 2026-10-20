/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2014 Bert Vermeulen <bert@biot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <string.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

/** @cond PRIVATE */
#define LOG_PREFIX "soft-trigger"
/** @endcond */
#include <stdint.h>
#include <limits.h>
/*
 * AVX2/AVX-512 scanning is not used by default in 64-bit Windows GCC builds
 * (MinGW/MXE): GCC assumes 32-byte stack alignment for spilled 256-bit
 * values there, which Win64 does not provide (GCC bug 54412), and the
 * resulting crash is not worth it -- the plain 64-bit path below still
 * scans ~4 GB/s. Define SOFT_TRIG_WIN64_SIMD to use AVX there anyway.
 */
#if (defined(__x86_64__) || defined(__i386__)) && (defined(__GNUC__) || defined(__clang__)) && \
	!(defined(_WIN64) && !defined(__clang__) && !defined(SOFT_TRIG_WIN64_SIMD))
#include <immintrin.h>
#define STRIG_X86_SIMD 1
#else
#define STRIG_X86_SIMD 0
#endif

/*
 * 1 = an edge at the first sample of a buffer is checked against the last
 *     sample of the previous buffer (stl->prev_sample), so edges that fall
 *     exactly on a buffer boundary are not missed. Only the very first sample
 *     of an acquisition (stl->count == 0) cannot fire an edge trigger.
 * 0 = old behaviour of the 8bit/sipeed variants: never fire an edge trigger
 *     on sample 0 of any buffer.
 *
 * This only selects the behaviour of soft_trigger_logic_check8bit() and
 * soft_trigger_logic_check_sipeed(). soft_trigger_logic_check() always
 * checks edges across buffers, as upstream libsigrok does: with 0 there, an
 * edge exactly on a USB transfer boundary was never seen.
 */
#ifndef SOFT_TRIG_EDGE_ACROSS_BUFFERS
#define SOFT_TRIG_EDGE_ACROSS_BUFFERS 0
#endif

#define STRIG_ALWAYS_INLINE static inline __attribute__((always_inline))

typedef struct {
	uint64_t ms, vs;   /* current-sample mask / required value  */
	uint64_t mp, vp;   /* previous-sample mask / required value */
	uint64_t me;       /* channels that must toggle (EDGE)      */
	uint64_t lsb;      /* lowest bit of every W-bit field       */
	unsigned W;        /* sample width in bits: 1,2,4,8,16,32,64 */
	int has_edge;
	int impossible;    /* stage can never match                 */
} trig_prog_t;
SR_PRIV int logic_channel_unitsize(GSList *channels)
{
	int number = 0;
	struct sr_channel *channel;
	GSList *l;

	for (l = channels; l; l = l->next) {
		channel = l->data;
		if (channel->type == SR_CHANNEL_LOGIC)
			number++;
	}

	return (number + 7) / 8;
}

SR_PRIV struct soft_trigger_logic *soft_trigger_logic_new(
		const struct sr_dev_inst *sdi, struct sr_trigger *trigger,
		int pre_trigger_samples)
{
	struct soft_trigger_logic *stl;

	stl = g_malloc0(sizeof(struct soft_trigger_logic));
	stl->sdi = sdi;
	stl->trigger = trigger;
	stl->unitsize = logic_channel_unitsize(sdi->channels);
	stl->prev_sample = g_malloc0(stl->unitsize);
	stl->pre_trigger_size = stl->unitsize * pre_trigger_samples;
	stl->pre_trigger_buffer = g_try_malloc(stl->pre_trigger_size);
	if (pre_trigger_samples > 0 && !stl->pre_trigger_buffer) {
		/*
		 * Error out if g_try_malloc() failed (or was invoked as
		 * g_try_malloc(0)) *and* more than 0 pretrigger samples
		 * were requested.
		 */
		soft_trigger_logic_free(stl);
		return NULL;
	}
	stl->pre_trigger_head = stl->pre_trigger_buffer;

	if (stl->pre_trigger_size > 0 && !stl->pre_trigger_buffer) {
		soft_trigger_logic_free(stl);
		return NULL;
	}

	return stl;
}

SR_PRIV void soft_trigger_logic_free(struct soft_trigger_logic *stl)
{
	g_free(stl->pre_trigger_buffer);
	g_free(stl->prev_sample);
	g_free(stl);
}

static void pre_trigger_append(struct soft_trigger_logic *stl,
		uint8_t *buf, int len)
{
	/* Avoid uselessly copying more than the pre-trigger size. */
	if (len > stl->pre_trigger_size) {
		buf += len - stl->pre_trigger_size;
		len = stl->pre_trigger_size;
	}

	/* Update the filling level of the pre-trigger circular buffer. */
	stl->pre_trigger_fill = MIN(stl->pre_trigger_fill + len,
	                            stl->pre_trigger_size);

	/* Actually copy data to the pre-trigger circular buffer. */
	while (len > 0) {
		size_t size = MIN(stl->pre_trigger_buffer + stl->pre_trigger_size
		                  - stl->pre_trigger_head, len);
		memcpy(stl->pre_trigger_head, buf, size);
		stl->pre_trigger_head += size;
		if (stl->pre_trigger_head >= stl->pre_trigger_buffer
		                             + stl->pre_trigger_size)
			stl->pre_trigger_head = stl->pre_trigger_buffer;
		buf += size;
		len -= size;
	}
}

static void pre_trigger_send(struct soft_trigger_logic *stl,
		int *pre_trigger_samples)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;
	logic.unitsize = stl->unitsize;

	if (pre_trigger_samples)
		*pre_trigger_samples = 0;

	/* If pre-trigger buffer not full, rewind head to the first valid sample. */
	if (stl->pre_trigger_fill < stl->pre_trigger_size)
		stl->pre_trigger_head = stl->pre_trigger_buffer;

	/* Send logic packets for the pre-trigger circular buffer content. */
	while (stl->pre_trigger_fill > 0) {
		size_t size = MIN(stl->pre_trigger_buffer + stl->pre_trigger_size
		                  - stl->pre_trigger_head, stl->pre_trigger_fill);
		logic.length = size;
		logic.data = stl->pre_trigger_head;
		sr_session_send(stl->sdi, &packet);
		stl->pre_trigger_head = stl->pre_trigger_buffer;
		stl->pre_trigger_fill -= size;
		if (pre_trigger_samples)
			*pre_trigger_samples += size / stl->unitsize;
	}
}

static gboolean logic_check_match(struct soft_trigger_logic *stl,
		uint8_t *sample, struct sr_trigger_match *match)
{
	int bit, prev_bit;
	gboolean result;

	stl->count++;
	result = FALSE;
	bit = *(sample + match->channel->index / 8 )& (1 << (match->channel->index % 8));
	if (match->match == SR_TRIGGER_ZERO)
		result = bit == 0;
	else if (match->match == SR_TRIGGER_ONE)
		result = bit != 0;
	else {
		/* Edge matches. */
		if (stl->count == 1)
			/* First sample, don't have enough for an edge match yet. */
			return FALSE;
		prev_bit = *(stl->prev_sample+ match->channel->index / 8 )	& (1 << (match->channel->index % 8));
		if (match->match == SR_TRIGGER_RISING)
			result = prev_bit == 0 && bit != 0;
		else if (match->match == SR_TRIGGER_FALLING)
			result = prev_bit != 0 && bit == 0;
		else if (match->match == SR_TRIGGER_EDGE)
			result = prev_bit != bit;
	}

	return result;
}
G_GNUC_UNUSED static gboolean logic_check_match8(struct soft_trigger_logic *stl,
		uint8_t *sample, struct sr_trigger_match *match)
{
	int bit, prev_bit;
	gboolean result;

	stl->count++;
	result = FALSE;
	bit = (*sample )& (1 << (match->channel->index));
	if (match->match == SR_TRIGGER_ZERO)
		result = bit == 0;
	else if (match->match == SR_TRIGGER_ONE)
		result = bit != 0;
	else {
		/* Edge matches. */
		if (stl->count == 1)
			/* First sample, don't have enough for an edge match yet. */
			return FALSE;
		prev_bit = *(stl->prev_sample )	& (1 << (match->channel->index ));
		if (match->match == SR_TRIGGER_RISING)
			result = prev_bit == 0 && bit != 0;
		else if (match->match == SR_TRIGGER_FALLING)
			result = prev_bit != 0 && bit == 0;
		else if (match->match == SR_TRIGGER_EDGE)
			result = prev_bit != bit;
	}

	return result;
}
/* Returns the offset (in samples) within buf of where the trigger
 * occurred, or -1 if not triggered. */
/*
 * Fast soft-trigger scanning for libsigrok (drop-in replacement for
 * soft_trigger_logic_check / soft_trigger_logic_check8bit /
 * soft_trigger_logic_check_sipeed in src/soft-trigger.c).
 *
 * How it works
 * ------------
 * 1. The stage's match list is compiled ONCE per call into five bit masks.
 *    For a sample s and its predecessor p, the stage matches iff
 *        ((s ^ VS) & MS) | ((p ^ VP) & MP) | (~(s ^ p) & ME)  == 0
 *    ZERO/ONE/RISING/FALLING/EDGE all map onto these masks, so the per-sample
 *    linked-list walk and switch() disappear.
 * 2. Samples are processed as a little-endian bit stream, 64 bits at a time
 *    (SWAR). A sample is W bits wide (W = 1,2,4 for packed Sipeed data,
 *    8/16/32/64 for unitsize 1/2/4/8), so one 64-bit word holds 64/W samples.
 *    The "previous sample" of every sample in the word is just the word
 *    shifted left by W bits, with the carry coming from the previous word.
 *    => Sipeed packed data is scanned in place: no malloc, no unpacking.
 * 3. The hot loop runs 256-bit AVX2 or 512-bit AVX-512 (runtime-detected),
 *    32/64 bytes per iteration, with no branches except "block has a hit".
 *    The exact hit position is then found with tzcnt on a single word.
 *
 * Paste into src/soft-trigger.c (it uses the file's existing includes,
 * pre_trigger_append(), pre_trigger_send(), logic_check_match()).
 * Build with -O2 or -O3; -march=native is optional (dispatch is at runtime).
 */



/* ---------- helpers ---------- */

STRIG_ALWAYS_INLINE uint64_t strig_ld64(const uint8_t *p)
{
	uint64_t v;
	memcpy(&v, p, 8);
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	v = __builtin_bswap64(v);
#endif
	return v;
}

STRIG_ALWAYS_INLINE uint64_t strig_ldn(const uint8_t *p, size_t n)
{
	uint64_t v = 0;
	for (size_t i = 0; i < n; i++)
		v |= (uint64_t)p[i] << (8 * i);
	return v;
}

static uint64_t strig_rep(uint64_t x, unsigned W)
{
	uint64_t r = 0;
	for (unsigned s = 0; s < 64; s += W)
		r |= x << s;
	return r;
}

/* Require bit b of (*m,*v) to be val; returns 0 on a contradiction. */
static int strig_req(uint64_t *m, uint64_t *v, uint64_t b, int val)
{
	if ((*m & b) && (((*v & b) != 0) != val))
		return 0;
	*m |= b;
	if (val)
		*v |= b;
	return 1;
}

/* Compile the match list of one stage for W-bit samples. */
static void trig_compile(const struct sr_trigger_stage *stage, unsigned W,
		trig_prog_t *t)
{
	const GSList *l;
	int ok = 1;

	memset(t, 0, sizeof(*t));
	t->W = W;
	for (l = stage->matches; l; l = l->next) {
		const struct sr_trigger_match *m = l->data;
		unsigned idx;
		uint64_t b;

		if (!m->channel->enabled)
			continue; /* Ignore disabled channels with a trigger. */
		idx = (unsigned)m->channel->index;
		if (idx >= W) {
			/* Channel not present in the sample: the old code read it
			 * as a constant 0, so only ZERO could ever be satisfied. */
			if (m->match != SR_TRIGGER_ZERO)
				ok = 0;
			continue;
		}
		b = 1ULL << idx;
		switch (m->match) {
		case SR_TRIGGER_ZERO:
			ok &= strig_req(&t->ms, &t->vs, b, 0);
			break;
		case SR_TRIGGER_ONE:
			ok &= strig_req(&t->ms, &t->vs, b, 1);
			break;
		case SR_TRIGGER_RISING:
			ok &= strig_req(&t->ms, &t->vs, b, 1);
			ok &= strig_req(&t->mp, &t->vp, b, 0);
			t->has_edge = 1;
			break;
		case SR_TRIGGER_FALLING:
			ok &= strig_req(&t->ms, &t->vs, b, 0);
			ok &= strig_req(&t->mp, &t->vp, b, 1);
			t->has_edge = 1;
			break;
		case SR_TRIGGER_EDGE:
			t->me |= b;
			t->has_edge = 1;
			break;
		default:
			ok = 0; /* analog / unknown match types never matched */
			break;
		}
	}
	t->impossible = !ok;
	t->ms = strig_rep(t->ms, W);
	t->vs = strig_rep(t->vs, W);
	t->mp = strig_rep(t->mp, W);
	t->vp = strig_rep(t->vp, W);
	t->me = strig_rep(t->me, W);
	t->lsb = strig_rep(1, W);
}

/* Previous-sample word for word s, given the preceding word pw. */
STRIG_ALWAYS_INLINE uint64_t strig_prev(uint64_t s, uint64_t pw, const unsigned W)
{
	return W == 64 ? pw : (s << (W & 63)) | (pw >> ((64 - W) & 63));
}

/* Bit k*W is set in the result iff sample k of the word matches. */
STRIG_ALWAYS_INLINE uint64_t strig_hits(const trig_prog_t *t, uint64_t s,
		uint64_t p, const unsigned W)
{
	uint64_t v = ((s ^ t->vs) & t->ms) | ((p ^ t->vp) & t->mp)
			| (~(s ^ p) & t->me);
	if (W > 1)  v |= v >> 1;
	if (W > 2)  v |= v >> 2;
	if (W > 4)  v |= v >> 4;
	if (W > 8)  v |= v >> 8;
	if (W > 16) v |= v >> 16;
	if (W > 32) v |= v >> 32;
	return ~v & t->lsb;
}

/* ---------- SIMD block scanners ----------
 * Scan whole words starting at word k (k >= 1, so word k-1 is readable).
 * Return the index of the first block that contains a hit, or the first
 * word not processed. The scalar loop then pinpoints the hit. */

typedef size_t (*trig_vec_fn)(const trig_prog_t *, const uint8_t *, size_t, size_t);

#if STRIG_X86_SIMD
STRIG_ALWAYS_INLINE __attribute__((target("avx2")))
size_t strig_avx2_body(const trig_prog_t *t, const uint8_t *buf,
		size_t k, size_t nwords, const unsigned W)
{
	const __m256i ms = _mm256_set1_epi64x((long long)t->ms);
	const __m256i vs = _mm256_set1_epi64x((long long)t->vs);
	const __m256i mp = _mm256_set1_epi64x((long long)t->mp);
	const __m256i vp = _mm256_set1_epi64x((long long)t->vp);
	const __m256i me = _mm256_set1_epi64x((long long)t->me);
	const __m256i lsb = _mm256_set1_epi64x((long long)t->lsb);

	for (; k + 4 <= nwords; k += 4) {
		__m256i s = _mm256_loadu_si256((const __m256i *)(buf + 8 * k));
		__m256i pw = _mm256_loadu_si256((const __m256i *)(buf + 8 * k - 8));
		__m256i p = (W == 64) ? pw : _mm256_or_si256(
			_mm256_slli_epi64(s, W & 63), _mm256_srli_epi64(pw, (64 - W) & 63));
		__m256i v = _mm256_or_si256(
			_mm256_or_si256(
				_mm256_and_si256(_mm256_xor_si256(s, vs), ms),
				_mm256_and_si256(_mm256_xor_si256(p, vp), mp)),
			_mm256_andnot_si256(_mm256_xor_si256(s, p), me));
		if (W > 1)  v = _mm256_or_si256(v, _mm256_srli_epi64(v, 1));
		if (W > 2)  v = _mm256_or_si256(v, _mm256_srli_epi64(v, 2));
		if (W > 4)  v = _mm256_or_si256(v, _mm256_srli_epi64(v, 4));
		if (W > 8)  v = _mm256_or_si256(v, _mm256_srli_epi64(v, 8));
		if (W > 16) v = _mm256_or_si256(v, _mm256_srli_epi64(v, 16));
		if (W > 32) v = _mm256_or_si256(v, _mm256_srli_epi64(v, 32));
		/* testc == 1  <=>  (~v & lsb) == 0  <=>  no sample matched */
		if (!_mm256_testc_si256(v, lsb))
			return k;
	}
	return k;
}

__attribute__((target("avx2")))
static size_t strig_scan_avx2(const trig_prog_t *t, const uint8_t *buf,
		size_t k, size_t nwords)
{
	switch (t->W) {
	case 1:  return strig_avx2_body(t, buf, k, nwords, 1);
	case 2:  return strig_avx2_body(t, buf, k, nwords, 2);
	case 4:  return strig_avx2_body(t, buf, k, nwords, 4);
	case 8:  return strig_avx2_body(t, buf, k, nwords, 8);
	case 16: return strig_avx2_body(t, buf, k, nwords, 16);
	case 32: return strig_avx2_body(t, buf, k, nwords, 32);
	default: return strig_avx2_body(t, buf, k, nwords, 64);
	}
}

#ifndef SOFT_TRIG_NO_AVX512
STRIG_ALWAYS_INLINE __attribute__((target("avx512f")))
size_t strig_avx512_body(const trig_prog_t *t, const uint8_t *buf,
		size_t k, size_t nwords, const unsigned W)
{
	const __m512i ms = _mm512_set1_epi64((long long)t->ms);
	const __m512i vs = _mm512_set1_epi64((long long)t->vs);
	const __m512i mp = _mm512_set1_epi64((long long)t->mp);
	const __m512i vp = _mm512_set1_epi64((long long)t->vp);
	const __m512i me = _mm512_set1_epi64((long long)t->me);
	const __m512i lsb = _mm512_set1_epi64((long long)t->lsb);

	for (; k + 8 <= nwords; k += 8) {
		__m512i s = _mm512_loadu_si512((const void *)(buf + 8 * k));
		__m512i pw = _mm512_loadu_si512((const void *)(buf + 8 * k - 8));
		__m512i p = (W == 64) ? pw : _mm512_or_si512(
			_mm512_slli_epi64(s, W & 63), _mm512_srli_epi64(pw, (64 - W) & 63));
		/* 0x28 = (a ^ b) & c   (ternary logic, one uop) */
		__m512i v = _mm512_or_si512(
			_mm512_ternarylogic_epi64(s, vs, ms, 0x28),
			_mm512_ternarylogic_epi64(p, vp, mp, 0x28));
		/* 0x82 = ~(a ^ b) & c */
		v = _mm512_or_si512(v, _mm512_ternarylogic_epi64(s, p, me, 0x82));
		if (W > 1)  v = _mm512_or_si512(v, _mm512_srli_epi64(v, 1));
		if (W > 2)  v = _mm512_or_si512(v, _mm512_srli_epi64(v, 2));
		if (W > 4)  v = _mm512_or_si512(v, _mm512_srli_epi64(v, 4));
		if (W > 8)  v = _mm512_or_si512(v, _mm512_srli_epi64(v, 8));
		if (W > 16) v = _mm512_or_si512(v, _mm512_srli_epi64(v, 16));
		if (W > 32) v = _mm512_or_si512(v, _mm512_srli_epi64(v, 32));
		/* any lane with (~v & lsb) != 0 has a hit */
		if (_mm512_test_epi64_mask(_mm512_andnot_si512(v, lsb),
				_mm512_andnot_si512(v, lsb)))
			return k;
	}
	return k;
}

__attribute__((target("avx512f")))
static size_t strig_scan_avx512(const trig_prog_t *t, const uint8_t *buf,
		size_t k, size_t nwords)
{
	switch (t->W) {
	case 1:  return strig_avx512_body(t, buf, k, nwords, 1);
	case 2:  return strig_avx512_body(t, buf, k, nwords, 2);
	case 4:  return strig_avx512_body(t, buf, k, nwords, 4);
	case 8:  return strig_avx512_body(t, buf, k, nwords, 8);
	case 16: return strig_avx512_body(t, buf, k, nwords, 16);
	case 32: return strig_avx512_body(t, buf, k, nwords, 32);
	default: return strig_avx512_body(t, buf, k, nwords, 64);
	}
}
#endif /* SOFT_TRIG_NO_AVX512 */
#endif /* STRIG_X86_SIMD */

static trig_vec_fn strig_vec_select(void)
{
#if STRIG_X86_SIMD && !defined(SOFT_TRIG_SCALAR_ONLY)
	__builtin_cpu_init();
#ifndef SOFT_TRIG_NO_AVX512
	if (__builtin_cpu_supports("avx512f"))
		return strig_scan_avx512;
#endif
	if (__builtin_cpu_supports("avx2"))
		return strig_scan_avx2;
#endif
	return NULL;
}

static trig_vec_fn strig_vec(void)
{
	static int init; /* 0 = unresolved, 1 = resolved */
	static trig_vec_fn fn;
	if (!__atomic_load_n(&init, __ATOMIC_ACQUIRE)) {
		__atomic_store_n(&fn, strig_vec_select(), __ATOMIC_RELAXED);
		__atomic_store_n(&init, 1, __ATOMIC_RELEASE);
	}
	return __atomic_load_n(&fn, __ATOMIC_RELAXED);
}

/* ---------- core scan ---------- */

/* Returns index (in W-bit samples) of the first matching sample, or -1. */
STRIG_ALWAYS_INLINE int64_t strig_scan_w(const trig_prog_t *t,
		const uint8_t *buf, size_t nbytes, uint64_t pw, int skip0,
		const unsigned W)
{
	const size_t nwords = nbytes / 8, rem = nbytes % 8, spw = 64 / W;
	size_t k = 0;
	uint64_t s, h;

	if (nwords) {
		s = strig_ld64(buf);
		h = strig_hits(t, s, strig_prev(s, pw, W), W);
		if (skip0)
			h &= ~1ULL;
		if (h)
			return (int64_t)(__builtin_ctzll(h) / W);
		pw = s;
		k = 1;

		trig_vec_fn vec = strig_vec();
		if (vec && nwords - k >= 8) {
			k = vec(t, buf, k, nwords);
			pw = strig_ld64(buf + 8 * k - 8);
		}
		for (; k < nwords; k++) {
			s = strig_ld64(buf + 8 * k);
			h = strig_hits(t, s, strig_prev(s, pw, W), W);
			if (h)
				return (int64_t)(k * spw + __builtin_ctzll(h) / W);
			pw = s;
		}
	}
	if (rem) {
		size_t valid_bits = (rem * 8 / W) * W; /* whole samples only, < 64 */
		s = strig_ldn(buf + 8 * nwords, rem);
		h = strig_hits(t, s, strig_prev(s, pw, W), W);
		h &= (1ULL << valid_bits) - 1;
		if (!nwords && skip0)
			h &= ~1ULL;
		if (h)
			return (int64_t)(nwords * spw + __builtin_ctzll(h) / W);
	}
	return -1;
}

static int64_t strig_scan(const trig_prog_t *t, const uint8_t *buf,
		size_t nbytes, uint64_t pw, int skip0)
{
	if (t->impossible)
		return -1;
	switch (t->W) {
	case 1:  return strig_scan_w(t, buf, nbytes, pw, skip0, 1);
	case 2:  return strig_scan_w(t, buf, nbytes, pw, skip0, 2);
	case 4:  return strig_scan_w(t, buf, nbytes, pw, skip0, 4);
	case 8:  return strig_scan_w(t, buf, nbytes, pw, skip0, 8);
	case 16: return strig_scan_w(t, buf, nbytes, pw, skip0, 16);
	case 32: return strig_scan_w(t, buf, nbytes, pw, skip0, 32);
	default: return strig_scan_w(t, buf, nbytes, pw, skip0, 64);
	}
}

/* prev_sample layout: W >= 8 -> the last sample's bytes (as before);
 * W < 8 -> the last sample stored in the top W bits of prev_sample[0]. */
static uint64_t strig_prev_word(const struct soft_trigger_logic *stl, unsigned W)
{
	size_t nb = W >= 8 ? W / 8 : 1;
	return strig_ldn(stl->prev_sample, nb) << (64 - 8 * nb);
}

static void strig_save_prev(struct soft_trigger_logic *stl,
		const uint8_t *buf, size_t idx, unsigned W)
{
	if (W >= 8) {
		memcpy(stl->prev_sample, buf + idx * (W / 8), W / 8);
	} else {
		size_t bit = idx * W;
		uint8_t v = (buf[bit / 8] >> (bit % 8)) & ((1u << W) - 1);
		stl->prev_sample[0] = (uint8_t)(v << (8 - W));
	}
}

static int strig_skip0(const struct soft_trigger_logic *stl,
		const trig_prog_t *t, int across_buffers)
{
	if (!t->has_edge)
		return 0;
	return across_buffers ? stl->count == 0 : 1;
}

static void strig_count_add(struct soft_trigger_logic *stl, uint64_t n)
{
	uint64_t c = (uint64_t)(stl->count < 0 ? 0 : stl->count) + n;
	stl->count = c > INT_MAX ? INT_MAX : (int)c;
}

static const struct sr_trigger_stage *strig_stage(struct soft_trigger_logic *stl)
{
	GSList *l_stage = g_slist_nth(stl->trigger->stages, stl->cur_stage);
	const struct sr_trigger_stage *stage = l_stage ? l_stage->data : NULL;
	return (stage && stage->matches) ? stage : NULL;
}

/*
 * The original upstream per-sample loop, for what the fast scanner does not
 * cover: multi-stage triggers (it advances through the stages, and falls
 * back to stage 0 on a mismatch) and uncommon unit sizes.
 */
static int soft_trigger_logic_check_generic(struct soft_trigger_logic *stl,
		uint8_t *buf, int len, int *pre_trigger_samples)
{
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	GSList *l, *l_stage;
	int offset;
	int i;
	gboolean match_found;

	offset = -1;
	for (i = 0; i < len; i += stl->unitsize) {
		l_stage = g_slist_nth(stl->trigger->stages, stl->cur_stage);
		stage = l_stage->data;
		if (!stage->matches)
			/* No matches supplied, client error. */
			return SR_ERR_ARG;

		match_found = TRUE;
		for (l = stage->matches; l; l = l->next) {
			match = l->data;
			if (!match->channel->enabled)
				/* Ignore disabled channels with a trigger. */
				continue;
			if (!logic_check_match(stl, buf + i, match)) {
				match_found = FALSE;
				break;
			}
		}
		memcpy(stl->prev_sample, buf + i, stl->unitsize);
		if (match_found) {
			/* Matched on the current stage. */
			if (l_stage->next) {
				/* Advance to next stage. */
				stl->cur_stage++;
			} else {
				/* Matched on last stage, send pre-trigger data. */
				pre_trigger_append(stl, buf, i);
				pre_trigger_send(stl, pre_trigger_samples);

				/* Fire trigger. */
				offset = i / stl->unitsize;

				std_session_send_df_trigger(stl->sdi);
				break;
			}
		} else if (stl->cur_stage > 0) {
			/*
			 * We had a match at an earlier stage, but failed on the
			 * current stage. However, we may have a match on this
			 * stage in the next bit -- trigger on 0001 will fail on
			 * seeing 00001, so we need to go back to stage 0 -- but
			 * at the next sample from the one that matched originally,
			 * which the counter increment at the end of the loop
			 * takes care of.
			 */
			i -= stl->cur_stage * stl->unitsize;
			if (i < -1)
				i = -1; /* Oops, went back past this buffer. */
			/* Reset trigger stage. */
			stl->cur_stage = 0;
		}
	}

	if (offset == -1)
		pre_trigger_append(stl, buf, len);

	return offset;
}

/* ---------- public API (same signatures and return values) ---------- */

SR_PRIV int soft_trigger_logic_check(struct soft_trigger_logic *stl,
		uint8_t *buf, int len, int *pre_trigger_samples)
{
	const struct sr_trigger_stage *stage;
	const int u = stl->unitsize;
	trig_prog_t t;
	int64_t idx;
	size_t nsamples;

	/* Multi-stage triggers and uncommon unit sizes: per-sample loop. */
	if ((u != 1 && u != 2 && u != 4 && u != 8) || stl->cur_stage != 0 ||
	    (stl->trigger->stages && stl->trigger->stages->next))
		return soft_trigger_logic_check_generic(stl, buf, len,
				pre_trigger_samples);

	stage = strig_stage(stl);
	if (!stage)
		return SR_ERR_ARG; /* No matches supplied, client error. */

	nsamples = len > 0 ? (size_t)len / u : 0;
	if (nsamples == 0) {
		pre_trigger_append(stl, buf, len);
		return -1;
	}
	trig_compile(stage, 8 * u, &t);
	idx = strig_scan(&t, buf, nsamples * u, strig_prev_word(stl, 8 * u),
			strig_skip0(stl, &t, 1));
	strig_count_add(stl, idx >= 0 ? (uint64_t)idx + 1 : nsamples);

	if (idx >= 0) {
		strig_save_prev(stl, buf, (size_t)idx, 8 * u);
		pre_trigger_append(stl, buf, (int)(idx * u));
		pre_trigger_send(stl, pre_trigger_samples);
		std_session_send_df_trigger(stl->sdi);
		return (int)idx;
	}
	strig_save_prev(stl, buf, nsamples - 1, 8 * u);
	pre_trigger_append(stl, buf, len);
	return -1;
}

SR_PRIV int soft_trigger_logic_check8bit(struct soft_trigger_logic *stl,
		uint8_t *buf, int len, int *pre_trigger_samples)
{
	const struct sr_trigger_stage *stage = strig_stage(stl);
	trig_prog_t t;
	int64_t idx;

	if (!stage)
		return SR_ERR_ARG;
	if (len <= 0)
		return -1;

	trig_compile(stage, 8, &t);
	idx = strig_scan(&t, buf, (size_t)len, strig_prev_word(stl, 8),
			strig_skip0(stl, &t, SOFT_TRIG_EDGE_ACROSS_BUFFERS));
	strig_count_add(stl, idx >= 0 ? (uint64_t)idx + 1 : (uint64_t)len);

	if (idx >= 0) {
		strig_save_prev(stl, buf, (size_t)idx, 8);
		pre_trigger_append(stl, buf, (int)idx);
		pre_trigger_send(stl, pre_trigger_samples);
		std_session_send_df_trigger(stl->sdi);
		return (int)idx;
	}
	strig_save_prev(stl, buf, (size_t)len - 1, 8);
	pre_trigger_append(stl, buf, len);
	return -1;
}

SR_PRIV int soft_trigger_logic_check_sipeed(struct soft_trigger_logic *stl,
		uint8_t *buf, int length, int *pre_trigger_samples, int nChannel)
{
	const struct sr_trigger_stage *stage = strig_stage(stl);
	unsigned W;
	trig_prog_t t;
	int64_t idx;
	size_t nsamples;
	(void)pre_trigger_samples;
	if (!stage)
		return SR_ERR_ARG;
	if (nChannel >= 8)
		W = 8;
	else if (nChannel == 1 || nChannel == 2 || nChannel == 4)
		W = (unsigned)nChannel;
	else
		return SR_ERR_ARG; /* 3/5/6/7 channels cannot be byte-packed */
	if (length <= 0)
		return -1;

	nsamples = (size_t)length * (8 / W);
	trig_compile(stage, W, &t);
	idx = strig_scan(&t, buf, (size_t)length, strig_prev_word(stl, W),
			strig_skip0(stl, &t, SOFT_TRIG_EDGE_ACROSS_BUFFERS));
	strig_count_add(stl, idx >= 0 ? (uint64_t)idx + 1 : nsamples);

	if (idx >= 0) {
		strig_save_prev(stl, buf, (size_t)idx, W);
/*
		if (W < 8) {

			nsp_in_bytes = 8 / W; // NOW must be 2 and 4
			ptr = malloc(idx * nsp_in_bytes);
			for (size_t i = 0; i < idx; i += W) {
				for (size_t j = 0; j < 8; j++) {
					ptr[i * nsp_in_bytes + j] =
						(((uint8_t*)
							buf)[i + j / nsp_in_bytes] >>
							(j % nsp_in_bytes * W)) &
						((1 << W) - 1);
				}
			}
		}

			//pre_trigger_append(stl, ptr, (int)idx * nsp_in_bytes);
			//pre_trigger_send(stl, pre_trigger_samples); */
			//std_session_send_df_trigger(stl->sdi);
//			if (W < 8)
//				free(ptr);

			return (int)(idx / (8 / W)); /* byte offset, as before */
		
	}
	strig_save_prev(stl, buf, nsamples - 1, W);
	return -1;
}
