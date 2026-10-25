/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 Do Van Dung <do_van_dzung@yahoo.com>
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
#include "protocol.h"
#include "vth_lut.h"

/* ------------------------------------------------------------ commands */

SR_PRIV int ch32_cmd(const struct sr_dev_inst *sdi, uint8_t cmd,
		const uint8_t *payload, uint8_t plen, uint8_t *rsp, int rsp_size)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t buf[8], rbuf[64];
	int xfer, ret;

	if (!usb || !usb->devhdl)
		return SR_ERR;
	if (plen > 5)
		plen = 5;

	memset(buf, 0, sizeof(buf));
	buf[0] = cmd;
	buf[1] = plen;
	if (plen)
		memcpy(buf + 2, payload, plen);

	ret = libusb_bulk_transfer(usb->devhdl, CH32_EP_CMD_OUT, buf, 2 + plen,
			&xfer, CH32_CMD_TIMEOUT_MS);
	if (ret != 0) {
		sr_err("Command 0x%02x: %s.", cmd, libusb_error_name(ret));
		return SR_ERR_IO;
	}

	/* START / STOP have no reply */
	if (cmd == CMD_SET_START || cmd == CMD_SET_STOP ||
	    cmd == CMD_SET_ADC_START || cmd == CMD_SET_ADC_STOP)
		return SR_OK;

	memset(rbuf, 0, sizeof(rbuf));
	ret = libusb_bulk_transfer(usb->devhdl, CH32_EP_CMD_IN, rbuf,
			sizeof(rbuf), &xfer, CH32_CMD_TIMEOUT_MS);
	if (ret != 0) {
		sr_err("Command 0x%02x reply: %s.", cmd, libusb_error_name(ret));
		return SR_ERR_IO;
	}
	if (xfer < 3 || rbuf[0] != (cmd | CMD_RETURN_PARA)) {
		sr_err("Command 0x%02x: unexpected reply 0x%02x (%d bytes).",
			cmd, rbuf[0], xfer);
		return SR_ERR_DATA;
	}
	if (rbuf[2] != 0) {
		sr_err("Command 0x%02x failed, status %u.", cmd, rbuf[2]);
		return SR_ERR;
	}
	if (rsp && rsp_size > 0)
		memcpy(rsp, rbuf, MIN(rsp_size, xfer));

	return SR_OK;
}

SR_PRIV int ch32_get_version(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint8_t rsp[8];
	int ret;

	memset(rsp, 0, sizeof(rsp));
	if ((ret = ch32_cmd(sdi, CMD_GET_VERSION, NULL, 0, rsp, sizeof(rsp))) != SR_OK)
		return ret;
	devc->sw_ver = rsp[3];
	devc->hw_ver = rsp[4];
	return SR_OK;
}

/*
 * Comparator threshold DAC: measured inverse table (non-linear, inverted:
 * DAC 0 is about 3.44 V, DAC 1023 about 0.43 V).
 */
SR_PRIV uint16_t ch32_vth_to_dac(double v)
{
	int mv;

	if (v >= 3.44)
		return 0;
	mv = (int)(v * 1000.0 + 0.5);
	if (mv < 0)
		mv = 0;
	if (mv > CH32_VTH_LUT_MV_MAX)
		mv = CH32_VTH_LUT_MV_MAX;
	return ch32_vth_mv_to_dac[mv];
}

SR_PRIV int ch32_apply_vth(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint16_t dac = ch32_vth_to_dac(devc->vth);
	uint8_t p[2] = { dac >> 8, dac & 0xff };

	sr_dbg("Threshold %.2f V -> DAC %u.", devc->vth, dac);
	return ch32_cmd(sdi, CMD_SET_LOGIC_LEVEL, p, 2, NULL, 0);
}

/* --------------------------------------------------------- sample rates */

/*
 * Firmware Set_Logic_Para_Fun(), USB3: index -> rate, descending.
 *
 * fast8: 8 channel rates above 200 MS/s, firmware version 3 and later
 * (logic_sys 0x18..0x1B: PLL 300 / 275 / 250 / 225 MHz with the UHSIF clock
 * undivided). Older firmware runs these codes at its 100 MS/s default, so
 * they are only used when the firmware reports version 3 or later, the
 * link is USB3 and only channels D0-D7 are enabled.
 */
static const struct {
	uint8_t sys;
	uint64_t rate;
	gboolean fast8;
} hw_rates[] = {
	{ 0x18, SR_MHZ(300), TRUE },
	{ 0x19, SR_MHZ(275), TRUE },
	{ 0x1a, SR_MHZ(250), TRUE },
	{ 0x1b, SR_MHZ(225), TRUE },
	{ 0x00, SR_MHZ(200), FALSE },
	{ 0x01, SR_KHZ(187500), FALSE },
	{ 0x02, SR_MHZ(175), FALSE },
	{ 0x03, SR_KHZ(162500), FALSE },
	{ 0x04, SR_KHZ(156250), FALSE },
	{ 0x05, SR_MHZ(150), FALSE },
	{ 0x06, SR_KHZ(137500), FALSE },
	{ 0x07, SR_MHZ(125), FALSE },
	{ 0x08, SR_KHZ(112500), FALSE },
	{ 0x09, SR_MHZ(100), FALSE },
	{ 0x0a, SR_KHZ(93750), FALSE },
	{ 0x0b, SR_KHZ(87500), FALSE },
	{ 0x0c, SR_KHZ(81250), FALSE },
	{ 0x0d, SR_HZ(78125000), FALSE },
	{ 0x0e, SR_MHZ(75), FALSE },
	{ 0x0f, SR_KHZ(68750), FALSE },
	{ 0x10, SR_MHZ(50), FALSE },
	{ 0x11, SR_MHZ(40), FALSE },
	{ 0x12, SR_MHZ(25), FALSE },
	{ 0x13, SR_MHZ(20), FALSE },
	{ 0x14, SR_KHZ(12500), FALSE },
	{ 0x15, SR_MHZ(10), FALSE },
	{ 0x16, SR_MHZ(8), FALSE },
	{ 0x17, SR_KHZ(6250), FALSE },	/* hardware minimum */
};

/* rates below the hardware minimum (USB3) or the fixed USB2 clock, made by
 * decimation on the PC */
static const uint64_t soft_rates[] = {
	SR_MHZ(5), SR_MHZ(2), SR_MHZ(1), SR_KHZ(500), SR_KHZ(200), SR_KHZ(100),
};

/* firmware version with the 8 channel rates above 200 MS/s */
#define CH32_FW_FAST8_VERSION	3

/* USB2: the firmware ignores the rate index and runs at a fixed clock */
#define CH32_USB2_RATE_8BIT	SR_MHZ(40)
#define CH32_USB2_RATE_16BIT	SR_MHZ(20)

static size_t rate_add(uint64_t *out, size_t n, size_t max, uint64_t r)
{
	size_t i, j;

	for (i = 0; i < n; i++) {
		if (out[i] == r)
			return n;
		if (out[i] > r)
			break;
	}
	if (n >= max)
		return n;
	for (j = n; j > i; j--)
		out[j] = out[j - 1];
	out[i] = r;
	return n + 1;
}

/* the device has the 8 channel rates above 200 MS/s (USB3, firmware >= 3) */
SR_PRIV gboolean ch32_has_fast8(const struct dev_context *devc)
{
	return devc->usb3 && devc->sw_ver >= CH32_FW_FAST8_VERSION;
}

/*
 * All selectable rates for the current link, ascending. Like 40 MS/s on
 * USB2, the rates above 200 MS/s are listed whatever channels are enabled;
 * starting an acquisition with one of them and a channel above D7 fails
 * with an explanation. The firmware version is known once the device is
 * open, so these rates appear from then on.
 */
SR_PRIV size_t ch32_samplerate_list(const struct dev_context *devc,
		uint64_t *out, size_t max)
{
	size_t i, n = 0;

	if (devc->usb3) {
		for (i = 0; i < ARRAY_SIZE(hw_rates); i++)
			if (!hw_rates[i].fast8 || ch32_has_fast8(devc))
				n = rate_add(out, n, max, hw_rates[i].rate);
	} else {
		/* 40 MS/s only with channels 0..7 enabled */
		n = rate_add(out, n, max, CH32_USB2_RATE_8BIT);
		n = rate_add(out, n, max, CH32_USB2_RATE_16BIT);
		n = rate_add(out, n, max, SR_MHZ(10));
	}
	for (i = 0; i < ARRAY_SIZE(soft_rates); i++)
		n = rate_add(out, n, max, soft_rates[i]);
	return n;
}

/*
 * Rate index, device rate and decimation for 'rate' with 'logic_bit' wide
 * samples. Exact rates only: an exact device rate if there is one, else the
 * lowest device rate that is an integer multiple (least USB / CPU load).
 */
SR_PRIV int ch32_map_samplerate(struct dev_context *devc, uint64_t rate,
		unsigned int logic_bit)
{
	gboolean fast8;
	int i;

	if (!rate)
		return SR_ERR_ARG;

	if (!devc->usb3) {
		uint64_t base = (logic_bit == 8) ? CH32_USB2_RATE_8BIT
				: CH32_USB2_RATE_16BIT;
		if (rate > base || base % rate)
			return SR_ERR_SAMPLERATE;
		devc->logic_sys = 0x17;
		devc->hw_rate = base;
		devc->decim = (unsigned int)(base / rate);
		return SR_OK;
	}

	fast8 = logic_bit == 8 && ch32_has_fast8(devc);
	for (i = 0; i < (int)ARRAY_SIZE(hw_rates); i++) {
		if (hw_rates[i].fast8 && !fast8)
			continue;
		if (hw_rates[i].rate == rate) {
			devc->logic_sys = hw_rates[i].sys;
			devc->hw_rate = rate;
			devc->decim = 1;
			return SR_OK;
		}
	}
	/* table is descending: search from the lowest rate up */
	for (i = ARRAY_SIZE(hw_rates) - 1; i >= 0; i--) {
		if (hw_rates[i].fast8 && !fast8)
			continue;
		if (hw_rates[i].rate > rate && hw_rates[i].rate % rate == 0) {
			devc->logic_sys = hw_rates[i].sys;
			devc->hw_rate = hw_rates[i].rate;
			devc->decim = (unsigned int)(hw_rates[i].rate / rate);
			return SR_OK;
		}
	}
	return SR_ERR_SAMPLERATE;
}

/* HSADC divider for the nearest rate (rate = 40 MHz / (div + 1)). */
SR_PRIV unsigned int ch32_adc_div(uint64_t rate)
{
	uint64_t div;

	if (!rate)
		return CH32_ADC_MAX_DIV;
	div = (CH32_ADC_CLOCK + rate / 2) / rate;
	div = div ? div - 1 : 0;
	if (div < CH32_ADC_MIN_DIV)
		div = CH32_ADC_MIN_DIV;
	if (div > CH32_ADC_MAX_DIV)
		div = CH32_ADC_MAX_DIV;
	return (unsigned int)div;
}

/* ------------------------------------------------------------- trigger */

/* First trigger stage -> condition masks. Only enabled logic channels. */
SR_PRIV int ch32_build_trigger(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	GSList *l;
	uint16_t m;

	devc->have_trigger = FALSE;
	devc->trig_zero = devc->trig_one = 0;
	devc->trig_rise = devc->trig_fall = devc->trig_edge = 0;

	if (!(trigger = sr_session_trigger_get(sdi->session)) || !trigger->stages)
		return SR_OK;
	if (g_slist_length(trigger->stages) > 1)
		sr_warn("Only one trigger stage is supported, using the first.");

	stage = trigger->stages->data;
	for (l = stage->matches; l; l = l->next) {
		match = l->data;
		if (!match->channel->enabled ||
		    match->channel->type != SR_CHANNEL_LOGIC ||
		    match->channel->index < 0 ||
		    match->channel->index >= CH32_NUM_LOGIC_CHANNELS)
			continue;
		m = 1u << match->channel->index;
		switch (match->match) {
		case SR_TRIGGER_ZERO:    devc->trig_zero |= m; break;
		case SR_TRIGGER_ONE:     devc->trig_one |= m; break;
		case SR_TRIGGER_RISING:  devc->trig_rise |= m; break;
		case SR_TRIGGER_FALLING: devc->trig_fall |= m; break;
		case SR_TRIGGER_EDGE:    devc->trig_edge |= m; break;
		default:
			sr_err("Unsupported trigger match %d.", match->match);
			return SR_ERR_ARG;
		}
	}

	devc->have_trigger = (devc->trig_zero | devc->trig_one | devc->trig_rise |
			devc->trig_fall | devc->trig_edge) != 0;
	return SR_OK;
}

static inline unsigned int sample_at(unsigned int unit, const uint8_t *s, size_t j)
{
	return unit == 2 ? (unsigned int)s[2 * j] | (unsigned int)s[2 * j + 1] << 8
			 : s[j];
}

/*
 * Bits of sample c (after p) that violate the trigger; 0 = match. Works on
 * one sample, or lane by lane on several samples packed in one word.
 */
#define TRIG_BAD(c, p, z, o, r, f, e) \
	(((c) & (z)) | ((o) & ~(c)) | ((r) & ((p) | ~(c))) | \
	 ((f) & (~(p) | (c))) | ((e) & ~((p) ^ (c))))

static inline uint64_t load_u64(const uint8_t *p)
{
	uint64_t v;

	memcpy(&v, p, sizeof(v));	/* unaligned load */
	return v;
}

static inline unsigned int ctz64(uint64_t v)
{
#if defined(__GNUC__) || defined(__clang__)
	return (unsigned int)__builtin_ctzll(v);
#else
	unsigned int n = 0;

	while (!(v & 1)) {
		v >>= 1;
		n++;
	}
	return n;
#endif
}

/*
 * Index of the first sample in s[from..n) that meets the trigger, or n.
 *
 * From the libsigrok4DSL capture core (ALL LOGIC fast-trigger branch): the
 * condition is evaluated bitwise on 64-bit words holding 8 one-byte or 4
 * two-byte samples, the previous samples taken from the same buffer one
 * sample earlier. A sample matches when its lane of 'bad' is zero;
 * (bad - 0x01..) & ~bad & 0x80.. marks zero lanes and is exact for the
 * lowest one, the earliest sample (little endian hosts; others take the
 * plain loop).
 */
SR_PRIV size_t ch32_find_trigger(const struct dev_context *devc,
		const uint8_t *s, size_t from, size_t n)
{
	const unsigned int z = devc->trig_zero, o = devc->trig_one,
		r = devc->trig_rise, f = devc->trig_fall, e = devc->trig_edge;
	const unsigned int u = devc->unit;
	const unsigned int mask = (u == 2) ? 0xffff : 0xff;
	unsigned int p;
	size_t j;

	if (from >= n)
		return n;
	/* the sample before s[from]; without history the first sample cannot
	 * be an edge: use itself */
	if (from)
		p = sample_at(u, s, from - 1);
	else
		p = devc->have_prev ? devc->prev : sample_at(u, s, 0);
	j = from;
	if (!(TRIG_BAD(sample_at(u, s, j), p, z, o, r, f, e) & mask))
		return j;
	j++;

#ifndef WORDS_BIGENDIAN
	{
		/* s[j - 1] exists from here on */
		const uint64_t rep = (u == 2) ? UINT64_C(0x0001000100010001)
					      : UINT64_C(0x0101010101010101);
		const uint64_t Z = (z & mask) * rep, O = (o & mask) * rep,
			R = (r & mask) * rep, F = (f & mask) * rep,
			E = (e & mask) * rep;
		const uint64_t HIGHS = rep << (u * 8 - 1);
		const size_t per = 8 / u;	/* samples per word */

		for (; j + per <= n; j += per) {
			uint64_t cv = load_u64(s + j * u);
			uint64_t pv = load_u64(s + j * u - u);
			uint64_t bad = TRIG_BAD(cv, pv, Z, O, R, F, E);
			uint64_t hit = (bad - rep) & ~bad & HIGHS;
			if (hit)
				return j + ctz64(hit) / (u * 8);
		}
	}
#endif
	for (; j < n; j++)
		if (!(TRIG_BAD(sample_at(u, s, j), sample_at(u, s, j - 1),
			       z, o, r, f, e) & mask))
			return j;
	return n;
}

/* ----------------------------------------------------------- decimation */

/*
 * Keep 1 of every 'decim' samples as a per-channel majority vote over the
 * window (ties: the last sample's bit), which keeps short pulses from
 * turning into aliasing spikes. Windows that hold one value only, the
 * usual case for logic signals, skip the per-bit counting.
 * Returns the number of output samples written to out.
 */
SR_PRIV size_t ch32_decimate(struct dev_context *devc, const uint8_t *in,
		size_t n, uint8_t *out)
{
	const unsigned int u = devc->unit, N = devc->decim;
	const unsigned int bits = u * 8;
	size_t i, m = 0;
	unsigned int b, s, v;

	for (i = 0; i < n; i++) {
		s = sample_at(u, in, i);
		if (devc->dec_fill == 0) {
			devc->dec_first = s;
			devc->dec_mixed = FALSE;
		} else if (!devc->dec_mixed && s != devc->dec_first) {
			/* first different sample: count the ones so far */
			devc->dec_mixed = TRUE;
			for (b = 0; b < bits; b++)
				devc->dec_ones[b] = ((devc->dec_first >> b) & 1) ?
					devc->dec_fill : 0;
		}
		if (devc->dec_mixed)
			for (b = 0; b < bits; b++)
				devc->dec_ones[b] += (s >> b) & 1;
		devc->dec_last = s;

		if (++devc->dec_fill < N)
			continue;

		if (!devc->dec_mixed) {
			v = devc->dec_first;
		} else {
			v = 0;
			for (b = 0; b < bits; b++) {
				unsigned int ones2 = 2 * devc->dec_ones[b];
				if (ones2 > N || (ones2 == N && ((devc->dec_last >> b) & 1)))
					v |= 1u << b;
			}
		}
		out[m * u] = v & 0xff;
		if (u == 2)
			out[m * u + 1] = v >> 8;
		m++;
		devc->dec_fill = 0;
	}
	return m;
}

/* ------------------------------------------------------- sample stream */

static void send_logic(const struct sr_dev_inst *sdi, const uint8_t *data,
		uint64_t n)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	while (n) {
		uint64_t k = MIN(n, (uint64_t)CH32_SEND_CHUNK);
		logic.length = k * devc->unit;
		logic.unitsize = devc->unit;
		logic.data = (void *)data;
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		sr_session_send(sdi, &packet);
		devc->sent += k;
		data += k * devc->unit;
		n -= k;
	}
}

/* pre-trigger ring buffer: keep the newest 'pre' samples */
static void hist_push(struct dev_context *devc, const uint8_t *s, size_t n)
{
	const uint64_t cap = devc->pre;
	const unsigned int u = devc->unit;

	if (!cap)
		return;
	if (n >= cap) {
		memcpy(devc->hist, s + (n - cap) * u, (size_t)(cap * u));
		devc->hist_pos = 0;
		devc->hist_len = cap;
		return;
	}
	while (n) {
		size_t k = (size_t)MIN((uint64_t)n, cap - devc->hist_pos);
		memcpy(devc->hist + devc->hist_pos * u, s, k * u);
		devc->hist_pos = (devc->hist_pos + k) % cap;
		devc->hist_len = MIN(devc->hist_len + k, cap);
		s += k * u;
		n -= k;
	}
}

/* send the newest 'want' history samples, oldest first */
static void hist_send(const struct sr_dev_inst *sdi, uint64_t want)
{
	struct dev_context *devc = sdi->priv;
	const uint64_t cap = devc->pre;
	const unsigned int u = devc->unit;
	uint64_t n = MIN(want, devc->hist_len), start, first;

	if (!n)
		return;
	start = (devc->hist_pos + cap - n) % cap;
	first = MIN(n, cap - start);
	send_logic(sdi, devc->hist + start * u, first);
	send_logic(sdi, devc->hist, n - first);
}

/* Samples (devc->unit bytes each, after decimation) in stream order. */
static void process_samples(const struct sr_dev_inst *sdi, const uint8_t *s,
		size_t n)
{
	struct dev_context *devc = sdi->priv;
	const unsigned int u = devc->unit;
	uint64_t k;

	if (!n)
		return;

	if (devc->state == CH32_ARMED) {
		size_t from = 0, at;
		uint64_t from_buf;

		/*
		 * Like other logic analyzers: a trigger only counts once the
		 * pre-trigger part is complete, else a busy signal triggers at
		 * once with (almost) nothing before the trigger.
		 */
		if (devc->hist_len < devc->pre) {
			uint64_t need = devc->pre - devc->hist_len;
			from = need < n ? (size_t)need : n;
		}
		at = ch32_find_trigger(devc, s, from, n);
		if (at >= n) {
			hist_push(devc, s, n);
			devc->prev = sample_at(u, s, n - 1);
			devc->have_prev = TRUE;
			return;
		}

		/* pre-trigger: history, then this buffer up to the trigger */
		from_buf = MIN((uint64_t)at, devc->pre);
		hist_send(sdi, devc->pre - from_buf);
		send_logic(sdi, s + (at - from_buf) * u, from_buf);
		std_session_send_df_trigger(sdi);
		sr_dbg("Trigger after %" PRIu64 " pre-trigger samples.", devc->sent);
		devc->state = CH32_STREAMING;
		s += at * u;
		n -= at;
	}

	if (devc->state != CH32_STREAMING)
		return;

	k = n;
	if (devc->limit_samples)
		k = MIN(k, devc->limit_samples > devc->sent ?
			devc->limit_samples - devc->sent : 0);
	send_logic(sdi, s, k);
	if (devc->limit_samples && devc->sent >= devc->limit_samples) {
		devc->state = CH32_DONE;
		devc->stop_req = TRUE;
	}
}

/* raw device samples -> decimation (if any) -> process_samples() */
static int process_raw(const struct sr_dev_inst *sdi, const uint8_t *s, size_t n)
{
	struct dev_context *devc = sdi->priv;
	size_t need, m;

	if (devc->decim <= 1) {
		process_samples(sdi, s, n);
		return SR_OK;
	}
	need = (n / devc->decim + 1) * devc->unit;
	if (need > devc->work_size) {
		g_free(devc->work);
		devc->work = g_try_malloc(need);
		devc->work_size = devc->work ? need : 0;
		if (!devc->work)
			return SR_ERR_MALLOC;
	}
	m = ch32_decimate(devc, s, n, devc->work);
	process_samples(sdi, devc->work, m);
	return SR_OK;
}

static int feed_logic(const struct sr_dev_inst *sdi, const uint8_t *buf, size_t len)
{
	struct dev_context *devc = sdi->priv;
	int ret;

	if (devc->unit == 1)
		return process_raw(sdi, buf, len);

	/* 16-bit samples: complete a sample split between two transfers */
	if (devc->carry_len && len) {
		uint8_t pair[2] = { devc->carry[0], buf[0] };
		devc->carry_len = 0;
		buf++;
		len--;
		if ((ret = process_raw(sdi, pair, 1)) != SR_OK)
			return ret;
	}
	if (len / 2 && devc->state != CH32_DONE)
		if ((ret = process_raw(sdi, buf, len / 2)) != SR_OK)
			return ret;
	if (len & 1) {
		devc->carry[0] = buf[len - 1];
		devc->carry_len = 1;
	}
	return SR_OK;
}

/* HSADC: little-endian 16-bit words, 10 significant bits -> volts */
static int feed_analog(const struct sr_dev_inst *sdi, const uint8_t *buf, size_t len)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	size_t n = 0, i, need;
	uint64_t k;

	need = (len / 2 + 1) * sizeof(float);
	if (need > devc->fbuf_size) {
		g_free(devc->fbuf);
		devc->fbuf = g_try_malloc(need);
		devc->fbuf_size = devc->fbuf ? need : 0;
		if (!devc->fbuf)
			return SR_ERR_MALLOC;
	}

#define ADC_VOLTS(lo, hi) \
	((float)((((unsigned int)(hi) << 8) | (lo)) & 0x3ff) * \
	 (float)(CH32_ADC_VREF / 1023.0))

	if (devc->carry_len && len) {
		devc->fbuf[n++] = ADC_VOLTS(devc->carry[0], buf[0]);
		devc->carry_len = 0;
		buf++;
		len--;
	}
	for (i = 0; i + 1 < len; i += 2)
		devc->fbuf[n++] = ADC_VOLTS(buf[i], buf[i + 1]);
	if (len & 1) {
		devc->carry[0] = buf[len - 1];
		devc->carry_len = 1;
	}
#undef ADC_VOLTS

	k = n;
	if (devc->limit_samples)
		k = MIN(k, devc->limit_samples > devc->sent ?
			devc->limit_samples - devc->sent : 0);
	if (k) {
		sr_analog_init(&analog, &encoding, &meaning, &spec, 3);
		analog.meaning->channels = g_slist_append(NULL, devc->analog_ch);
		analog.meaning->mq = SR_MQ_VOLTAGE;
		analog.meaning->unit = SR_UNIT_VOLT;
		analog.meaning->mqflags = SR_MQFLAG_DC;
		analog.num_samples = (uint32_t)k;
		analog.data = devc->fbuf;
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		sr_session_send(sdi, &packet);
		g_slist_free(analog.meaning->channels);
		devc->sent += k;
	}
	if (devc->limit_samples && devc->sent >= devc->limit_samples) {
		devc->state = CH32_DONE;
		devc->stop_req = TRUE;
	}
	return SR_OK;
}

/* ------------------------------------------------------------ transfers */

static void free_transfer(struct libusb_transfer *transfer)
{
	const struct sr_dev_inst *sdi = transfer->user_data;
	struct dev_context *devc = sdi->priv;
	unsigned int i;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}
	libusb_free_transfer(transfer);
	if (devc->submitted > 0)
		devc->submitted--;
	if (devc->submitted == 0)
		devc->finished = TRUE;	/* ended in receive_data() */
}

static void cancel_transfers(struct dev_context *devc)
{
	unsigned int i;

	for (i = 0; i < devc->num_transfers; i++)
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
}

/*
 * Transfer callback. No synchronous USB commands here (libusb does not
 * allow them inside event handling): a stop is only flagged, the STOP
 * command is sent by receive_data().
 */
static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	const struct sr_dev_inst *sdi = transfer->user_data;
	struct dev_context *devc = sdi->priv;
	gboolean ok;
	int ret;

	/* a timeout can still carry data at low rates */
	ok = transfer->status == LIBUSB_TRANSFER_COMPLETED ||
	     transfer->status == LIBUSB_TRANSFER_TIMED_OUT;
	if (!ok && transfer->status != LIBUSB_TRANSFER_CANCELLED) {
		sr_err("Data transfer failed: %s.",
			libusb_error_name(transfer->status == LIBUSB_TRANSFER_NO_DEVICE
				? LIBUSB_ERROR_NO_DEVICE : LIBUSB_ERROR_IO));
		devc->stop_req = TRUE;
	}

	if (ok && transfer->actual_length > 0 && !devc->stop_req &&
	    devc->state != CH32_DONE) {
		ret = devc->analog
			? feed_analog(sdi, transfer->buffer, transfer->actual_length)
			: feed_logic(sdi, transfer->buffer, transfer->actual_length);
		if (ret != SR_OK) {
			sr_err("Out of memory processing samples.");
			devc->stop_req = TRUE;
		}
	}

	if (ok && !devc->stop_req && devc->state != CH32_DONE) {
		if ((ret = libusb_submit_transfer(transfer)) == 0)
			return;
		sr_err("Resubmitting a transfer failed: %s.", libusb_error_name(ret));
		devc->stop_req = TRUE;
	}
	free_transfer(transfer);
}

static void hw_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (!devc->hw_running)
		return;
	devc->hw_running = FALSE;
	if (ch32_cmd(sdi, devc->analog ? CMD_SET_ADC_STOP : CMD_SET_STOP,
			NULL, 0, NULL, 0) != SR_OK)
		sr_warn("Stopping the device failed.");
}

static void free_acq_buffers(struct dev_context *devc)
{
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	g_free(devc->hist);
	devc->hist = NULL;
	g_free(devc->work);
	devc->work = NULL;
	devc->work_size = 0;
	g_free(devc->fbuf);
	devc->fbuf = NULL;
	devc->fbuf_size = 0;
}

/* Session source: runs after libusb events were handled, outside of the
 * transfer callbacks. */
static int receive_data(int fd, int revents, void *cb_data)
{
	const struct sr_dev_inst *sdi = cb_data;
	struct dev_context *devc;
	struct sr_dev_driver *di;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	if (!sdi || !(devc = sdi->priv))
		return TRUE;

	if (devc->stop_req) {
		hw_stop(sdi);
		cancel_transfers(devc);
	}

	if (devc->finished && devc->state != CH32_IDLE) {
		hw_stop(sdi);
		di = sdi->driver;
		drvc = di->context;
		std_session_send_df_end(sdi);
		usb_source_remove(sdi->session, drvc->sr_ctx);
		free_acq_buffers(devc);
		devc->state = CH32_IDLE;
		sr_dbg("Acquisition ended, %" PRIu64 " samples sent.", devc->sent);
	}

	return TRUE;
}

/* ---------------------------------------------------------- acquisition */

/*
 * Bytes per transfer: about 10 ms of data, a multiple of 16 KiB, at most
 * 1 MiB. A multiple of 16 KiB divides by the USB3 packet size (1024), as
 * WinUSB RAW_IO requires, and by 512, so sigrok's patched libusb (MXE) also
 * puts these reads in RAW_IO mode. 1 MiB stays under WinUSB's RAW_IO maximum
 * (about 2 MiB) where it cannot be queried (libusb < 1.0.30); where it can,
 * transfers are kept under the reported value.
 */
static unsigned int transfer_size(const struct dev_context *devc,
		uint64_t bytes_per_sec)
{
	uint64_t s = bytes_per_sec / 100, max = 1024 * 1024;

	if (devc->raw_io_max && devc->raw_io_max < max)
		max = devc->raw_io_max & ~16383u;
	s = (s + 16383) & ~UINT64_C(16383);
	return (unsigned int)MAX(MIN(s, max), (uint64_t)16384);
}

/*
 * libusb >= 1.0.30, Windows / WinUSB: RAW_IO on the data endpoints. Without
 * it WinUSB keeps a single bulk IN read in flight (libusb #490), too slow
 * for 400 MB/s; with it the queued reads stay in flight. Endpoints set up
 * here are also left alone by sigrok's per-read RAW_IO policy in the
 * patched MXE libusb. Elsewhere libusb returns an error and nothing changes.
 */
SR_PRIV void ch32_setup_raw_io(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x0100010C)
	struct sr_usb_dev_inst *usb = sdi->conn;
	const uint8_t eps[] = { CH32_EP_LOGIC_IN, CH32_EP_ADC_IN };
	unsigned int i;
	int max;

	devc->raw_io_max = 0;
	for (i = 0; i < ARRAY_SIZE(eps); i++) {
		if (libusb_endpoint_set_raw_io(usb->devhdl, eps[i], 1) != LIBUSB_SUCCESS)
			continue;
		max = libusb_get_max_raw_io_transfer_size(usb->devhdl, eps[i]);
		sr_info("RAW_IO enabled on ep 0x%02x, max_raw_io=%d.", eps[i], max);
		if (max > 0 && (!devc->raw_io_max || (unsigned int)max < devc->raw_io_max))
			devc->raw_io_max = (unsigned int)max;
	}
#else
	devc->raw_io_max = 0;
#endif
}

static void send_samplerate_meta(const struct sr_dev_inst *sdi, uint64_t rate)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_meta meta;
	struct sr_config *src;

	src = sr_config_new(SR_CONF_SAMPLERATE, g_variant_new_uint64(rate));
	meta.config = g_slist_append(NULL, src);
	packet.type = SR_DF_META;
	packet.payload = &meta;
	sr_session_send(sdi, &packet);
	g_slist_free(meta.config);
	sr_config_free(src);
}

/* Logic or analog, sample width, rate settings, trigger and buffers. */
static int setup_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	GSList *l;
	unsigned int n_logic = 0, n_analog = 0, max_index = 0;
	uint8_t p[2];
	int ret;

	devc->analog_ch = NULL;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->type == SR_CHANNEL_LOGIC) {
			n_logic++;
			max_index = MAX(max_index, (unsigned int)ch->index);
		} else if (ch->type == SR_CHANNEL_ANALOG) {
			n_analog++;
			devc->analog_ch = ch;
		}
	}
	if (n_logic && n_analog) {
		sr_err("Logic and analog channels cannot be used at the same "
			"time, disable one kind.");
		return SR_ERR_ARG;
	}
	if (n_analog > 1) {
		sr_err("Only one analog channel can be used at a time.");
		return SR_ERR_ARG;
	}
	if (!n_logic && !n_analog) {
		sr_err("No channels enabled.");
		return SR_ERR_ARG;
	}

	devc->analog = n_analog > 0;
	devc->have_trigger = FALSE;
	devc->pre = 0;

	if (devc->analog) {
		devc->unit = 2;
		devc->decim = 1;
		devc->adc_div = ch32_adc_div(devc->samplerate);
		devc->hw_rate = CH32_ADC_CLOCK / (devc->adc_div + 1);
		p[0] = CH32_ADC_BITS;
		p[1] = devc->adc_div;
		if ((ret = ch32_cmd(sdi, CMD_SET_ADC_PARA, p, 2, NULL, 0)) != SR_OK)
			return ret;
		p[0] = devc->analog_ch->index - CH32_NUM_LOGIC_CHANNELS;
		if ((ret = ch32_cmd(sdi, CMD_SET_ADC_CHANNEL, p, 1, NULL, 0)) != SR_OK)
			return ret;
		sr_info("Analog: channel %s, %" PRIu64 " S/s (divider %u).",
			devc->analog_ch->name, devc->hw_rate, devc->adc_div);
		return SR_OK;
	}

	/* channels 8..15 need 16-bit samples: half the rate on USB2, and no
	 * rates above 200 MS/s on USB3 */
	devc->logic_bit = (max_index >= 8) ? 16 : 8;
	devc->unit = devc->logic_bit / 8;
	if (ch32_map_samplerate(devc, devc->samplerate, devc->logic_bit) != SR_OK) {
		if (devc->usb3 && devc->samplerate > SR_MHZ(200) &&
		    !ch32_has_fast8(devc))
			sr_err("%" PRIu64 " S/s needs firmware version %d or "
				"later (this device has %u).", devc->samplerate,
				CH32_FW_FAST8_VERSION, devc->sw_ver);
		else if (devc->usb3)
			sr_err("%" PRIu64 " S/s works with channels D0-D7 only; "
				"disable D8-D15 or use 200 MS/s or less.",
				devc->samplerate);
		else
			sr_err("%" PRIu64 " S/s is not available with channels above "
				"D7 on a USB 2 connection; use channels D0-D7 only, "
				"a lower rate, or a USB 3 port.", devc->samplerate);
		return SR_ERR_SAMPLERATE;
	}

	if ((ret = ch32_build_trigger(sdi)) != SR_OK)
		return ret;
	if (devc->have_trigger && devc->limit_samples) {
		devc->pre = devc->limit_samples * devc->capture_ratio / 100;
		if (devc->pre >= devc->limit_samples)
			devc->pre = devc->limit_samples - 1;
	}
	if (devc->pre) {
		devc->hist = g_try_malloc((size_t)(devc->pre * devc->unit));
		if (!devc->hist) {
			sr_err("Not enough memory for %" PRIu64 " pre-trigger samples.",
				devc->pre);
			return SR_ERR_MALLOC;
		}
	}

	p[0] = devc->logic_bit;
	p[1] = devc->logic_sys;
	if ((ret = ch32_cmd(sdi, CMD_SET_LOGIC_PARA, p, 2, NULL, 0)) != SR_OK)
		return ret;
	if ((ret = ch32_apply_vth(sdi)) != SR_OK)
		return ret;

	sr_info("Logic: %u-bit, %" PRIu64 " S/s (device %" PRIu64 " S/s / %u), "
		"trigger %s, %" PRIu64 " pre-trigger samples.", devc->logic_bit,
		devc->samplerate, devc->hw_rate, devc->decim,
		devc->have_trigger ? "on" : "off", devc->pre);
	return SR_OK;
}

SR_PRIV int ch32_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_dev_driver *di = sdi->driver;
	struct drv_context *drvc = di->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct libusb_transfer *transfer;
	unsigned int i, size;
	uint8_t *buf;
	int ret;

	devc->transfers = NULL;
	devc->num_transfers = 0;
	devc->submitted = 0;
	devc->stop_req = FALSE;
	devc->finished = FALSE;
	devc->hw_running = FALSE;
	devc->sent = 0;
	devc->carry_len = 0;
	devc->dec_fill = 0;
	devc->hist = NULL;
	devc->hist_pos = devc->hist_len = 0;
	devc->have_prev = FALSE;

	if ((ret = setup_acquisition(sdi)) != SR_OK) {
		free_acq_buffers(devc);
		return ret;
	}

	devc->state = devc->have_trigger ? CH32_ARMED : CH32_STREAMING;

	size = transfer_size(devc, devc->hw_rate * devc->unit);
	devc->transfers = g_malloc0(sizeof(*devc->transfers) * CH32_NUM_TRANSFERS);
	devc->num_transfers = CH32_NUM_TRANSFERS;

	std_session_send_df_header(sdi);
	if (devc->analog && devc->hw_rate != devc->samplerate)
		send_samplerate_meta(sdi, devc->hw_rate);

	usb_source_add(sdi->session, drvc->sr_ctx, 100, receive_data, (void *)sdi);

	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc(size);
		transfer = buf ? libusb_alloc_transfer(0) : NULL;
		if (!transfer) {
			g_free(buf);
			sr_err("Out of memory for USB transfers.");
			ret = SR_ERR_MALLOC;
			break;
		}
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
			devc->analog ? CH32_EP_ADC_IN : CH32_EP_LOGIC_IN,
			buf, size, receive_transfer, (void *)sdi,
			CH32_BULK_TIMEOUT_MS);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Submitting a transfer failed: %s.",
				libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			ret = SR_ERR_IO;
			break;
		}
		devc->transfers[i] = transfer;
		devc->submitted++;
		ret = SR_OK;
	}

	if (ret == SR_OK) {
		ret = ch32_cmd(sdi, devc->analog ? CMD_SET_ADC_START : CMD_SET_START,
			NULL, 0, NULL, 0);
		if (ret == SR_OK)
			devc->hw_running = TRUE;
	}

	if (ret != SR_OK) {
		/* the header is out: let receive_data() collect the transfers
		 * and end the data feed */
		devc->stop_req = TRUE;
		if (!devc->submitted)
			devc->finished = TRUE;
		cancel_transfers(devc);
	}

	return SR_OK;
}

/*
 * Only flags the stop and cancels the transfers (allowed anywhere); the
 * STOP command and the end of the data feed come from receive_data(), so
 * this is safe even when reached from inside libusb event handling.
 */
SR_PRIV int ch32_acquisition_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (devc->state == CH32_IDLE)
		return SR_OK;
	devc->stop_req = TRUE;
	cancel_transfers(devc);
	if (!devc->submitted)
		devc->finished = TRUE;
	return SR_OK;
}

/* Stop the device if an acquisition left it running (dev_close()). */
SR_PRIV void ch32_hw_stop(const struct sr_dev_inst *sdi)
{
	hw_stop(sdi);
}
