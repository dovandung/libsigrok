/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 Do Van Dzung
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

/*
 * Up to 100 MHz the device streams; faster rates use buffer mode (capture
 * into on-board memory, then upload).
 */
SR_PRIV const uint64_t dl32_samplerates[] = {
	SR_MHZ(1), SR_MHZ(2), SR_MHZ(4), SR_MHZ(5), SR_MHZ(10),
	SR_MHZ(20), SR_MHZ(25), SR_MHZ(40), SR_MHZ(50), SR_MHZ(100),
	SR_MHZ(200), SR_MHZ(250), SR_MHZ(400), SR_MHZ(500), SR_MHZ(800),
	SR_GHZ(1),
};
SR_PRIV const unsigned int dl32_num_samplerates = ARRAY_SIZE(dl32_samplerates);

/* Sample rate -> index used by the 0x11 parameter command. */
static const struct {
	uint64_t rate;
	int index;
} hz_index_table[] = {
	{ SR_MHZ(1), 1 }, { SR_MHZ(2), 2 }, { SR_MHZ(4), 3 }, { SR_MHZ(5), 4 },
	{ SR_MHZ(10), 5 }, { SR_MHZ(20), 6 }, { SR_MHZ(25), 7 },
	{ SR_MHZ(40), 8 }, { SR_MHZ(50), 9 }, { SR_MHZ(100), 10 },
	{ SR_MHZ(200), 11 }, { SR_MHZ(250), 12 }, { SR_MHZ(500), 13 },
	{ SR_GHZ(1), 14 }, { SR_MHZ(400), 15 }, { SR_MHZ(800), 16 },
};

/* Logic family name and the matching input threshold in 0.1 V units. */
SR_PRIV const char *dl32_thresholds[] = {
	"5.0V", "3.3V", "3.0V", "2.5V", "1.8V", "1.5V", "1.2V",
};
SR_PRIV const unsigned int dl32_num_thresholds = ARRAY_SIZE(dl32_thresholds);
static const int threshold_tenths[] = { 25, 16, 15, 12, 9, 7, 6 };

/* Reflected CRC32 (0xEDB88320), init 0, final inversion. */
SR_PRIV uint32_t dl32_crc32(const uint8_t *buf, size_t len)
{
	static uint32_t table[256];
	static gboolean table_ready;
	uint32_t crc, c;
	size_t i;
	int k;

	if (!table_ready) {
		for (i = 0; i < 256; i++) {
			c = i;
			for (k = 0; k < 8; k++)
				c = (c & 1) ? (0xedb88320 ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		table_ready = TRUE;
	}

	crc = 0;
	for (i = 0; i < len; i++)
		crc = table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);

	return ~crc;
}

SR_PRIV void dl32_build_raw(uint8_t out[DL32_RAW_PACKET_LEN], uint8_t cmd,
	const uint8_t *payload, size_t len)
{
	memset(out, 0, DL32_RAW_PACKET_LEN);
	if (len > DL32_RAW_PACKET_LEN - 3)
		len = DL32_RAW_PACKET_LEN - 3;
	out[0] = 0x0a;
	out[1] = cmd;
	if (len)
		memcpy(&out[2], payload, len);
	out[2 + len] = 0x0b;
}

SR_PRIV size_t dl32_build_framed(uint8_t *out, uint8_t cmd,
	const uint8_t *data, size_t len)
{
	uint32_t crc;

	memset(out, 0, len + DL32_FRAME_OVERHEAD);
	out[8] = 0x0a;
	out[9] = cmd;
	out[10] = (len + 1) & 0xff;
	if (len)
		memcpy(&out[11], data, len);
	out[11 + len] = 0x0b;
	/* CRC covers cmd, length and data. */
	crc = dl32_crc32(&out[9], len + 2);
	WL32(&out[12 + len], crc);

	return len + DL32_FRAME_OVERHEAD;
}

SR_PRIV int dl32_samplerate_index(uint64_t samplerate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hz_index_table); i++) {
		if (hz_index_table[i].rate == samplerate)
			return hz_index_table[i].index;
	}

	return -1;
}

static void write_le40(uint8_t *p, uint64_t v)
{
	int i;

	for (i = 0; i < 5; i++)
		p[i] = (v >> (8 * i)) & 0xff;
}

SR_PRIV void dl32_build_params(uint8_t out[DL32_PARAMS_LEN],
	gboolean buffer_mode, gboolean rle, int threshold_tenths,
	int hz_index, uint64_t samples, uint64_t trigger_pos)
{
	out[0] = (buffer_mode ? 0x80 : 0) | (rle ? 0x40 : 0);
	out[1] = (threshold_tenths < 0 ? 0x80 : 0) |
		(ABS(threshold_tenths) & 0x7f);
	out[2] = hz_index;
	write_le40(&out[3], samples);
	write_le40(&out[8], trigger_pos);
}

/*
 * Simple trigger command: one nibble per channel position (even position in
 * the high nibble). DL32_TRIG_ENABLE marks a captured channel, the low three
 * bits hold its trigger condition (DL32_TRIG_NONE = don't care).
 * codes[] has one entry per channel; 0 means the channel is not captured.
 */
SR_PRIV void dl32_build_trigger(uint8_t out[DL32_TRIGGER_LEN],
	const uint8_t *codes, unsigned int num_channels, unsigned int ch_offset)
{
	unsigned int ch, pos;
	uint8_t nib;

	memset(out, 0, DL32_TRIGGER_LEN);
	for (ch = 0; ch < num_channels; ch++) {
		pos = ch_offset + ch;
		if (pos >= DL32_TRIGGER_POSITIONS)
			break;
		nib = codes[ch] & 0x0f;
		out[pos / 2] |= (pos & 1) ? nib : (uint8_t)(nib << 4);
	}
	/*
	 * out[16]: loop capture, out[17]: instant capture (no trigger wait).
	 * Both 0 = single capture that waits for the trigger.
	 */
}

/*
 * Translate a sigrok trigger into per-channel hardware codes. The device
 * supports one condition per channel, all of which must hold at once, so
 * only single-stage triggers with at most one match per channel fit.
 * Returns SR_OK and fills codes[] (condition only, no enable bit), or
 * SR_ERR_NA if the trigger needs the software fallback.
 */
SR_PRIV int dl32_trigger_codes(const struct sr_trigger *trigger,
	unsigned int num_channels, uint8_t *codes)
{
	const struct sr_trigger_stage *stage;
	const struct sr_trigger_match *match;
	const GSList *l;
	unsigned int ch;
	uint8_t code;

	for (ch = 0; ch < num_channels; ch++)
		codes[ch] = DL32_TRIG_NONE;
	if (!trigger || !trigger->stages)
		return SR_OK;
	if (g_slist_length(trigger->stages) != 1)
		return SR_ERR_NA;

	stage = trigger->stages->data;
	for (l = stage->matches; l; l = l->next) {
		match = l->data;
		if (!match->channel || !match->channel->enabled)
			continue;
		ch = match->channel->index;
		if (ch >= num_channels || codes[ch] != DL32_TRIG_NONE)
			return SR_ERR_NA;
		switch (match->match) {
		case SR_TRIGGER_ZERO:	 code = DL32_TRIG_LOW; break;
		case SR_TRIGGER_ONE:	 code = DL32_TRIG_HIGH; break;
		case SR_TRIGGER_RISING:	 code = DL32_TRIG_RISING; break;
		case SR_TRIGGER_FALLING: code = DL32_TRIG_FALLING; break;
		case SR_TRIGGER_EDGE:	 code = DL32_TRIG_EDGE; break;
		default:
			return SR_ERR_NA;
		}
		codes[ch] = code;
	}

	return SR_OK;
}

/* Reply to raw 0x81: [0A][81][01][mode][mcu_hi][mcu_lo][variant][?][plus]. */
SR_PRIV gboolean dl32_parse_hello(const uint8_t *buf, size_t len,
	struct dl32_info *info)
{
	if (len < 9 || buf[0] != 0x0a || buf[1] != DL32_RAW_CMD_HELLO ||
			buf[2] != 0x01)
		return FALSE;

	info->mode = buf[3];
	info->mcu_version = buf[4] * 10 + buf[5];
	info->variant = buf[6];
	info->plus = buf[8] == 1;

	return TRUE;
}

/* The device data reply carries a NUL terminated name such as "DL32". */
SR_PRIV void dl32_parse_name(const uint8_t *buf, size_t len,
	struct dl32_info *info)
{
	size_t i, j;

	for (i = 0; i + 1 < len; i++) {
		if (buf[i] != 'D' || buf[i + 1] != 'L')
			continue;
		for (j = 0; i + j < len && j < sizeof(info->name) - 1; j++) {
			if (buf[i + j] < 0x20 || buf[i + j] > 0x7e)
				break;
			info->name[j] = buf[i + j];
		}
		info->name[j] = '\0';
		return;
	}
}

/*
 * Convert one group (num_channels blocks of DL32_BLOCK_BYTES) into
 * DL32_BLOCK_SAMPLES samples of num_channels / 8 bytes each.
 *
 * spread[b] puts bit k of b into the lowest bit of byte k, so OR-ing
 * spread[b] << c for 8 channels gives 8 samples of 8 channels at once.
 */
/* spread[b] puts bit k of b into the lowest bit of byte k. */
static const uint64_t *spread_table(void)
{
	static uint64_t spread[256];
	static gboolean spread_ready;
	unsigned int i, k;

	if (!spread_ready) {
		for (i = 0; i < 256; i++) {
			spread[i] = 0;
			for (k = 0; k < 8; k++)
				spread[i] |= (uint64_t)((i >> k) & 1) << (8 * k);
		}
		spread_ready = TRUE;
	}

	return spread;
}

SR_PRIV void dl32_deinterleave(const uint8_t *group, unsigned int num_channels,
	uint8_t *out)
{
	const uint64_t *spread;
	unsigned int unitsize, i, c, j, k;
	uint64_t acc[4];

	spread = spread_table();
	unitsize = num_channels / 8;
	for (i = 0; i < DL32_BLOCK_BYTES; i++) {
		for (j = 0; j < unitsize; j++) {
			acc[j] = 0;
			for (c = 0; c < 8; c++) {
				acc[j] |= spread[group[(j * 8 + c) *
					DL32_BLOCK_BYTES + i]] << c;
			}
		}
		for (k = 0; k < 8; k++) {
			for (j = 0; j < unitsize; j++)
				out[k * unitsize + j] = (acc[j] >> (8 * k)) & 0xff;
		}
		out += 8 * unitsize;
	}
}

/*
 * --- Framed device data (status, buffer mode, RLE).
 *
 * Layout and meaning follow the DL16 code in ATK-Logic (pv/data/analysis.cpp,
 * pv/thread/thread_work.cpp): [0A][type][len16 LE][payload][00][0B], with
 * zero padding allowed between frames.
 */

static uint64_t read_le40(const uint8_t *p)
{
	uint64_t v;
	int i;

	v = 0;
	for (i = 4; i >= 0; i--)
		v = (v << 8) | p[i];

	return v;
}

/*
 * Try to read a frame at buf[0] with 'gap' bytes between header and payload.
 * Returns 1 = frame, 0 = not a frame, -1 = need more data.
 */
static int frame_at(const uint8_t *buf, size_t len, size_t gap,
	struct dl32_frame *frame)
{
	size_t plen, i;

	plen = buf[2] | (buf[3] << 8);
	if (4 + gap + plen + 2 > len)
		return -1;
	for (i = 0; i < gap; i++) {
		if (buf[4 + i] != 0x00)
			return 0;
	}
	if (buf[4 + gap + plen] != 0x00 || buf[5 + gap + plen] != 0x0b)
		return 0;
	frame->type = buf[1];
	frame->payload = &buf[4 + gap];
	frame->len = plen;
	frame->size = 4 + gap + plen + 2;

	return 1;
}

/*
 * Frames are [0A][type][len16 LE][gap][payload (len)][00][0B]. The DL16 has
 * no gap; the DL32 FPGA puts DL32_FRAME_GAP zero bytes after the header and
 * pads data frames to 2 KiB. Both layouts are accepted, DL32 first.
 */
SR_PRIV enum dl32_parse_result dl32_parse_frame(const uint8_t *buf,
	size_t len, size_t *skip, struct dl32_frame *frame)
{
	size_t i;
	int r32, r16;

	for (i = 0; i < len; i++) {
		if (buf[i] != 0x0a)
			continue;
		if (i + 1 >= len) {
			*skip = i;
			return DL32_PARSE_NEED_MORE;
		}
		if (buf[i + 1] < DL32_FRAME_DATA || buf[i + 1] > DL32_FRAME_DONE)
			continue;
		if (i + 4 > len) {
			*skip = i;
			return DL32_PARSE_NEED_MORE;
		}
		r32 = frame_at(buf + i, len - i, DL32_FRAME_GAP, frame);
		if (r32 == 1) {
			*skip = i;
			return DL32_PARSE_FRAME;
		}
		r16 = frame_at(buf + i, len - i, 0, frame);
		if (r16 == 1) {
			*skip = i;
			return DL32_PARSE_FRAME;
		}
		if (r32 < 0 || r16 < 0) {
			*skip = i;
			return DL32_PARSE_NEED_MORE;
		}
	}
	*skip = len;

	return DL32_PARSE_NONE;
}

SR_PRIV struct dl32_capture *dl32_capture_new(gboolean rle)
{
	struct dl32_capture *cap;

	cap = g_malloc0(sizeof(*cap));
	cap->rle = rle;

	return cap;
}

SR_PRIV void dl32_capture_free(struct dl32_capture *cap)
{
	unsigned int i;

	if (!cap)
		return;
	for (i = 0; i < DL32_TRIGGER_POSITIONS; i++) {
		if (cap->chan[i])
			g_byte_array_free(cap->chan[i], TRUE);
	}
	g_free(cap);
}

/* Returns the frame type that was handled. */
SR_PRIV int dl32_capture_feed(struct dl32_capture *cap,
	const struct dl32_frame *frame)
{
	const uint8_t *p, *data;
	size_t n, rest, i, k;
	unsigned int id;

	p = frame->payload;
	n = frame->len;

	switch (frame->type) {
	case DL32_FRAME_DATA:
		/* [channel id][?][samples...], 8 samples per byte, LSB first. */
		if (n < 2)
			break;
		id = p[0];
		if (id >= DL32_TRIGGER_POSITIONS) {
			sr_dbg("Data for unknown channel id %u.", id);
			break;
		}
		if (!cap->chan[id])
			cap->chan[id] = g_byte_array_new();
		data = p + 2;
		rest = n - 2;
		if (!cap->rle) {
			g_byte_array_append(cap->chan[id], data, rest);
		} else {
			/* (count, value) pairs: count copies of the value byte. */
			size_t total = 0, at;
			for (i = 0; i + 1 < rest; i += 2)
				total += data[i];
			at = cap->chan[id]->len;
			g_byte_array_set_size(cap->chan[id], at + total);
			for (i = 0; i + 1 < rest; i += 2) {
				memset(cap->chan[id]->data + at, data[i + 1], data[i]);
				at += data[i];
			}
		}
		cap->frames++;
		break;
	case DL32_FRAME_OFFSET:
		/*
		 * [2 bytes][trigger offset, 5 bytes][per-channel byte totals,
		 * 5 bytes each][RLE only: flags, bit 0 over capacity,
		 * bit 1 over bandwidth].
		 */
		if (n < 7)
			break;
		cap->have_offset = TRUE;
		cap->trigger_offset = read_le40(p + 2);
		rest = n - 7;
		if (cap->rle && rest % 5 == 1) {
			cap->exceed_capacity = p[n - 1] & 1;
			cap->exceed_bandwidth = (p[n - 1] >> 1) & 1;
			rest--;
		}
		cap->num_totals = MIN(rest / 5, DL32_TRIGGER_POSITIONS);
		for (k = 0; k < cap->num_totals; k++)
			cap->totals[k] = read_le40(p + 7 + 5 * k);
		sr_info("Trigger offset %" PRIu64 ", %u channel totals%s%s.",
			cap->trigger_offset, cap->num_totals,
			cap->exceed_capacity ? ", over capacity" : "",
			cap->exceed_bandwidth ? ", over bandwidth" : "");
		break;
	case DL32_FRAME_ACK:
		/* [2 bytes][command][result]; a start (0x12) should report 3. */
		if (n < 3)
			break;
		sr_dbg("Ack for command 0x%02x: 0x%02x.", p[2], n > 3 ? p[3] : 0);
		if (p[2] == DL32_CMD_TRIGGER && n > 3 && p[3] != 3) {
			sr_dbg("Start acknowledged with code %u (DL16 uses 3).",
				p[3]);
			cap->ack_error = TRUE;
		}
		if (p[2] == DL32_CMD_STOP)
			cap->done = TRUE;
		break;
	case DL32_FRAME_PROGRESS:
		if (n >= 7)
			cap->progress = read_le40(p + 2);
		break;
	case DL32_FRAME_DONE:
		cap->done = TRUE;
		if (n >= 3 && p[2] == 1) {
			sr_warn("Device reported a stream overflow.");
			cap->exceed_capacity = TRUE;
		}
		break;
	default:
		break;
	}

	return frame->type;
}

static void log_frame(const struct dl32_frame *f)
{
	GString *s;

	if (f->type == DL32_FRAME_DATA || sr_log_loglevel_get() < SR_LOG_DBG)
		return;
	s = sr_hexdump_new(f->payload, MIN(f->len, 24));
	sr_dbg("Frame type %u, %zu bytes: %s", f->type, f->len, s->str);
	sr_hexdump_free(s);
}

/*
 * Parse all complete frames in buf into cap. Returns how many bytes were
 * consumed; the rest (a partial frame) must be kept for the next call.
 */
SR_PRIV size_t dl32_consume_frames(struct dl32_capture *cap,
	const uint8_t *buf, size_t len)
{
	struct dl32_frame frame;
	size_t pos, skip;
	enum dl32_parse_result r;

	pos = 0;
	while (pos < len) {
		r = dl32_parse_frame(buf + pos, len - pos, &skip, &frame);
		if (r != DL32_PARSE_FRAME) {
			pos += skip;
			break;
		}
		log_frame(&frame);
		dl32_capture_feed(cap, &frame);
		pos += skip + frame.size;
	}

	return pos;
}

/*
 * Add one USB read to the pending bytes and parse what is complete.
 *
 * Status frames (including "done") always arrive in their own short read.
 * If such a read holds a complete frame at its start while bytes are still
 * pending, those bytes are leftovers that look like an unfinished frame
 * (seen after RLE data); drop them so the status frame is not stuck
 * behind them.
 */
SR_PRIV void dl32_feed_bytes(struct dl32_capture *cap, GByteArray *pending,
	const uint8_t *p, size_t len)
{
	struct dl32_frame frame;
	size_t skip, used;

	if (pending->len && len < DL32_STATUS_MAX_LEN && p[0] == 0x0a &&
			dl32_parse_frame(p, len, &skip, &frame) == DL32_PARSE_FRAME &&
			skip == 0) {
		if (sr_log_loglevel_get() >= SR_LOG_DBG) {
			GString *str = sr_hexdump_new(pending->data,
				MIN(pending->len, 16));
			sr_dbg("Dropping %u stale bytes before a status frame: %s",
				pending->len, str->str);
			sr_hexdump_free(str);
		}
		g_byte_array_set_size(pending, 0);
	}

	g_byte_array_append(pending, p, len);
	used = dl32_consume_frames(cap, pending->data, pending->len);
	if (used)
		g_byte_array_remove_range(pending, 0, used);
}

/*
 * Work out which device channel feeds each sigrok channel and where the
 * requested window starts, mirroring ATK-Logic's DL16 trimming: the device
 * may send more data than asked for, and the trigger offset plus per-channel
 * totals tell where the pre-trigger window begins.
 */
SR_PRIV void dl32_capture_plan(const struct dl32_capture *cap,
	unsigned int num_channels, unsigned int ch_offset,
	uint32_t enabled_mask, gboolean buffer_mode, uint64_t depth,
	uint64_t pre_trigger, struct dl32_plan *plan)
{
	unsigned int ch, id, shift, tidx;
	uint64_t t, min_total, avail, count;
	gboolean any;

	/* Device ids are either 0-based or include the channel offset. */
	shift = 0;
	for (id = num_channels; id < DL32_TRIGGER_POSITIONS; id++) {
		if (cap->chan[id]) {
			shift = ch_offset;
			break;
		}
	}

	min_total = UINT64_MAX;
	for (ch = 0; ch < num_channels; ch++) {
		id = ch + shift;
		plan->src[ch] = -1;
		plan->start[ch] = 0;
		if (!(enabled_mask & (1u << ch)) || id >= DL32_TRIGGER_POSITIONS ||
				!cap->chan[id])
			continue;
		plan->src[ch] = id;

		if (!buffer_mode || !cap->have_offset)
			continue;
		tidx = (cap->num_totals > num_channels &&
			cap->num_totals >= ch_offset + num_channels) ?
			ch + ch_offset : ch;
		t = tidx < cap->num_totals ? cap->totals[tidx] : 0;
		/* A total larger than what arrived is not usable; use the data. */
		if (t > cap->chan[id]->len) {
			sr_dbg("D%u: reported %" PRIu64 " bytes, received %u.",
				ch, t, cap->chan[id]->len);
			t = cap->chan[id]->len;
		}
		if (t && t * 8 > depth && t * 8 > pre_trigger + cap->trigger_offset)
			plan->start[ch] = t * 8 - pre_trigger - cap->trigger_offset;
		/*
		 * Keep a full window: if the reported offset would leave fewer
		 * than 'depth' samples (e.g. an instant capture reporting 0),
		 * use the last 'depth' samples instead.
		 */
		if (t && depth && t * 8 >= depth && plan->start[ch] > t * 8 - depth)
			plan->start[ch] = t * 8 - depth;
		if (t && t < min_total)
			min_total = t;
	}

	/* Over capacity (RLE): align all channels to the shortest one. */
	if (buffer_mode && cap->rle && cap->exceed_capacity &&
			min_total != UINT64_MAX) {
		for (ch = 0; ch < num_channels; ch++) {
			if (plan->src[ch] < 0)
				continue;
			tidx = (cap->num_totals > num_channels &&
				cap->num_totals >= ch_offset + num_channels) ?
				ch + ch_offset : ch;
			t = tidx < cap->num_totals ? cap->totals[tidx] : 0;
			plan->start[ch] = t ? (t - min_total) * 8 : 0;
		}
	}

	/* Output as many samples as every captured channel can provide. */
	any = FALSE;
	count = UINT64_MAX;
	for (ch = 0; ch < num_channels; ch++) {
		if (plan->src[ch] < 0)
			continue;
		avail = (uint64_t)cap->chan[plan->src[ch]]->len * 8;
		avail = avail > plan->start[ch] ? avail - plan->start[ch] : 0;
		count = MIN(count, avail);
		any = TRUE;
	}
	if (!any)
		count = 0;
	if (depth)
		count = MIN(count, depth);
	plan->count = count;
}

/* Interleave n output samples starting at sample 'first' of the window. */
SR_PRIV void dl32_capture_render(const struct dl32_capture *cap,
	const struct dl32_plan *plan, unsigned int num_channels,
	uint64_t first, uint64_t n, uint8_t *out)
{
	const uint64_t *spread;
	const uint8_t *src[DL32_TRIGGER_POSITIONS];
	unsigned int unitsize, ch, j, c, k, b;
	uint64_t s, bit, q, blocks, acc;
	uint8_t v;

	spread = spread_table();
	unitsize = num_channels / 8;
	for (ch = 0; ch < num_channels; ch++)
		src[ch] = plan->src[ch] < 0 ? NULL :
			cap->chan[plan->src[ch]]->data;

	/* Whole blocks of 8 samples: build each output byte lane at once. */
	blocks = n / 8;
	for (s = 0; s < blocks; s++) {
		for (j = 0; j < unitsize; j++) {
			acc = 0;
			for (c = 0; c < 8; c++) {
				ch = j * 8 + c;
				if (!src[ch])
					continue;
				bit = plan->start[ch] + first + s * 8;
				q = bit >> 3;
				b = bit & 7;
				v = src[ch][q] >> b;
				if (b)
					v |= src[ch][q + 1] << (8 - b);
				acc |= spread[v] << c;
			}
			for (k = 0; k < 8; k++)
				out[(s * 8 + k) * unitsize + j] = (acc >> (8 * k)) & 0xff;
		}
	}

	/* Remaining samples one at a time. */
	memset(out + blocks * 8 * unitsize, 0, (n - blocks * 8) * unitsize);
	for (ch = 0; ch < num_channels; ch++) {
		if (!src[ch])
			continue;
		for (s = blocks * 8; s < n; s++) {
			bit = plan->start[ch] + first + s;
			if (src[ch][bit >> 3] & (1 << (bit & 7)))
				out[s * unitsize + ch / 8] |= 1 << (ch % 8);
		}
	}
}

/* --- Synchronous helpers (scan, start, stop; never in transfer callbacks). */

static int usb_write(libusb_device_handle *hdl, const uint8_t *buf, int len)
{
	int ret, done;

	ret = libusb_bulk_transfer(hdl, DL32_EP_OUT, (uint8_t *)buf, len,
		&done, 300);
	if (ret != 0 || done != len) {
		sr_dbg("Write of %d bytes failed: %s.", len,
			ret ? libusb_error_name(ret) : "short write");
		return SR_ERR;
	}

	return SR_OK;
}

/*
 * Bulk IN read. Always read into buffers of whole packets (DL32_READ_LEN):
 * with WinUSB, a buffer smaller than the device's packet (1024 bytes on
 * USB 3) makes the transfer fail instead of returning the data.
 * Returns the byte count (0 on timeout) or -1 on error.
 */
static int usb_read(libusb_device_handle *hdl, uint8_t *buf, int len,
	unsigned int timeout_ms)
{
	int ret, done;

	done = 0;
	ret = libusb_bulk_transfer(hdl, DL32_EP_IN, buf, len, &done, timeout_ms);
	if (ret == 0 || ret == LIBUSB_ERROR_TIMEOUT)
		return done;
	if (ret == LIBUSB_ERROR_OVERFLOW && done > 0) {
		sr_dbg("Read overflow after %d bytes.", done);
		return done;
	}

	sr_warn("USB read failed: %s.", libusb_error_name(ret));
	if (ret == LIBUSB_ERROR_PIPE)
		libusb_clear_halt(hdl, DL32_EP_IN);

	return -1;
}

/*
 * Read until a reply starting with prefix shows up (anywhere in a read, so
 * stale bytes in front of it do not matter) or timeout_ms passes.
 * On success the reply is moved to the start of buf and its length returned.
 */
static int read_reply(libusb_device_handle *hdl, uint8_t *buf,
	const uint8_t *prefix, size_t prefix_len, unsigned int timeout_ms)
{
	gint64 deadline;
	int n, i;

	deadline = g_get_monotonic_time() + (gint64)timeout_ms * 1000;
	do {
		n = usb_read(hdl, buf, DL32_READ_LEN, 100);
		if (n < 0)
			return -1;
		for (i = 0; i + (int)prefix_len <= n; i++) {
			if (!memcmp(buf + i, prefix, prefix_len)) {
				memmove(buf, buf + i, n - i);
				return n - i;
			}
		}
		if (n > 0 && sr_log_loglevel_get() >= SR_LOG_DBG) {
			GString *str = sr_hexdump_new(buf, MIN(n, 16));
			sr_dbg("Skipping %d unexpected bytes: %s", n, str->str);
			sr_hexdump_free(str);
		}
	} while (g_get_monotonic_time() < deadline);

	return 0;
}

/* Discard anything still queued in the device (e.g. a previous stream). */
static void usb_drain(libusb_device_handle *hdl)
{
	uint8_t *buf;
	int i, n;
	size_t total;

	buf = g_malloc(DL32_READ_LEN);
	total = 0;
	for (i = 0; i < 4096; i++) {
		n = usb_read(hdl, buf, DL32_READ_LEN, 50);
		if (n <= 0)
			break;
		total += n;
	}
	g_free(buf);
	if (total)
		sr_dbg("Drained %zu stale bytes.", total);
}

static int send_raw(libusb_device_handle *hdl, uint8_t cmd,
	const uint8_t *payload, size_t len)
{
	uint8_t pkt[DL32_RAW_PACKET_LEN];

	dl32_build_raw(pkt, cmd, payload, len);

	return usb_write(hdl, pkt, sizeof(pkt));
}

static int send_framed(libusb_device_handle *hdl, uint8_t cmd,
	const uint8_t *data, size_t len)
{
	uint8_t pkt[64 + DL32_FRAME_OVERHEAD];
	size_t n;

	if (len > 64)
		return SR_ERR_ARG;
	n = dl32_build_framed(pkt, cmd, data, len);

	return usb_write(hdl, pkt, n);
}

SR_PRIV int dl32_identify(libusb_device_handle *hdl, struct dl32_info *info)
{
	static const uint8_t hello_prefix[] = { 0x0a, DL32_RAW_CMD_HELLO, 0x01 };
	static const uint8_t info_prefix[] = { 0x0a, DL32_FRAME_INFO };
	uint8_t *buf;
	int n, attempt, ret;

	memset(info, 0, sizeof(*info));
	buf = g_malloc(DL32_READ_LEN);
	ret = SR_ERR;

	/* The first exchange after opening sometimes gets lost; retry. */
	n = 0;
	for (attempt = 1; attempt <= 3; attempt++) {
		usb_drain(hdl);
		if (send_raw(hdl, DL32_RAW_CMD_HELLO, NULL, 0) != SR_OK) {
			sr_warn("Could not send the hello command (attempt %d).",
				attempt);
			continue;
		}
		n = read_reply(hdl, buf, ARRAY_AND_SIZE(hello_prefix), 500);
		if (n > 0 && dl32_parse_hello(buf, n, info))
			break;
		sr_warn("No valid hello reply (attempt %d, %s).", attempt,
			n < 0 ? "read error" : "timeout");
		n = 0;
	}
	if (n <= 0)
		goto out;

	sr_dbg("Hello: mode '%c', MCU %d, variant %d, plus %d.",
		info->mode, info->mcu_version, info->variant, info->plus);
	if (info->mode != 'a') {
		sr_err("Device is not in application mode (mode 0x%02x); "
			"update its firmware with ATK-Logic first.", info->mode);
		goto out;
	}

	if (send_framed(hdl, DL32_CMD_DEVICE_DATA, NULL, 0) == SR_OK) {
		n = read_reply(hdl, buf, ARRAY_AND_SIZE(info_prefix), 500);
		if (n > 0) {
			dl32_parse_name(buf, n, info);
			if (sr_log_loglevel_get() >= SR_LOG_SPEW) {
				GString *s = sr_hexdump_new(buf, MIN(n, 48));
				sr_spew("Device data: %s", s->str);
				sr_hexdump_free(s);
			}
		}
	}
	if (!info->name[0])
		g_strlcpy(info->name, "DL32", sizeof(info->name));

	usb_drain(hdl);
	ret = SR_OK;
out:
	g_free(buf);

	return ret;
}

SR_PRIV void dl32_send_stop(libusb_device_handle *hdl)
{
	const uint8_t zero = 0;

	send_framed(hdl, DL32_CMD_STOP, &zero, 1);
	send_raw(hdl, DL32_RAW_CMD_RESET, &zero, 1);
	usb_drain(hdl);
}


/* --- Asynchronous acquisition. */

static void send_logic(const struct sr_dev_inst *sdi, uint8_t *data,
	uint64_t num_samples)
{
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	devc = sdi->priv;
	if (!num_samples)
		return;

	logic.length = num_samples * devc->unitsize;
	logic.unitsize = devc->unitsize;
	logic.data = data;
	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;
	sr_session_send(sdi, &packet);
}

static void send_trigger_marker(const struct sr_dev_inst *sdi)
{
	struct sr_datafeed_packet packet;

	packet.type = SR_DF_TRIGGER;
	packet.payload = NULL;
	sr_session_send(sdi, &packet);
}

/*
 * Send samples to the session, honouring the software trigger, the sample
 * limit and the hardware trigger position (marker_pos, in output samples).
 */
static void process_samples(const struct sr_dev_inst *sdi, uint8_t *buf,
	uint64_t num_samples)
{
	struct dev_context *devc;
	uint64_t n, before, marker;
	int trigger_offset, pre_trigger_samples;

	devc = sdi->priv;

	if (!devc->trigger_fired) {
		trigger_offset = soft_trigger_logic_check(devc->stl, buf,
			num_samples * devc->unitsize, &pre_trigger_samples);
		if (trigger_offset < 0)
			return;
		/* Pre-trigger samples and the marker were sent by the checker. */
		devc->trigger_fired = TRUE;
		devc->sent_samples += pre_trigger_samples;
		buf += trigger_offset * devc->unitsize;
		num_samples -= trigger_offset;
	}

	n = num_samples;
	if (devc->acq_limit && devc->sent_samples + n > devc->acq_limit)
		n = devc->acq_limit - devc->sent_samples;

	/*
	 * Hardware trigger position: in buffer mode the window is trimmed so
	 * the trigger sits at the pre-trigger point; in stream mode the device
	 * reports where it fired (as ATK-Logic shows it).
	 */
	marker = UINT64_MAX;
	if (devc->hw_trigger && !devc->marker_sent && devc->capture &&
			devc->capture->have_offset)
		marker = devc->buffer_mode ? devc->pre_trigger :
			devc->capture->trigger_offset;
	if (marker >= devc->sent_samples && marker < devc->sent_samples + n) {
		before = marker - devc->sent_samples;
		send_logic(sdi, buf, before);
		send_trigger_marker(sdi);
		send_logic(sdi, buf + before * devc->unitsize, n - before);
		devc->marker_sent = TRUE;
	} else {
		send_logic(sdi, buf, n);
	}
	devc->sent_samples += n;
}

/* Turn the collected buffer-mode / RLE data into session packets. */
static void flush_capture(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct dl32_plan plan;
	uint64_t pos, n;
	const uint64_t chunk = 64 * 1024;
	uint8_t *buf;
	unsigned int ch;

	devc = sdi->priv;
	if (!devc->capture)
		return;

	dl32_capture_plan(devc->capture, devc->num_channels, devc->ch_offset,
		devc->enabled_mask, devc->buffer_mode, devc->acq_limit,
		devc->pre_trigger, &plan);
	for (ch = 0; ch < devc->num_channels; ch++) {
		if (plan.src[ch] >= 0)
			sr_dbg("D%u: device id %d, %u bytes, start %" PRIu64 ".",
				ch, plan.src[ch],
				devc->capture->chan[plan.src[ch]]->len,
				plan.start[ch]);
	}
	sr_dbg("Sending %" PRIu64 " collected samples (%" PRIu64
		" data frames).", plan.count, devc->capture->frames);
	if (!plan.count) {
		if (devc->capture->frames)
			sr_err("Collected data does not cover the requested window.");
		return;
	}

	buf = g_malloc(chunk * devc->unitsize);
	for (pos = 0; pos < plan.count; pos += n) {
		n = MIN(chunk, plan.count - pos);
		dl32_capture_render(devc->capture, &plan, devc->num_channels,
			pos, n, buf);
		process_samples(sdi, buf, n);
	}
	g_free(buf);
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb = sdi->conn;

	usb_source_remove(sdi->session, devc->ctx);
	dl32_send_stop(usb->devhdl);

	if (devc->framed)
		flush_capture(sdi);

	if (devc->frame_begun)
		std_session_send_df_frame_end(sdi);
	std_session_send_df_end(sdi);

	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	g_free(devc->group_buf);
	devc->group_buf = NULL;
	g_free(devc->conv_buf);
	devc->conv_buf = NULL;
	if (devc->pending) {
		g_byte_array_free(devc->pending, TRUE);
		devc->pending = NULL;
	}
	dl32_capture_free(devc->capture);
	devc->capture = NULL;
	if (devc->dump) {
		fclose(devc->dump);
		devc->dump = NULL;
	}
	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	/* Finish from the event source, outside of libusb's callback. */
	if (devc->submitted_transfers == 0)
		devc->acq_done = TRUE;
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("Failed to resubmit transfer: %s.", libusb_error_name(ret));
	free_transfer(transfer);
}

SR_PRIV void dl32_abort_acquisition(struct dev_context *devc)
{
	int i;

	devc->acq_aborted = TRUE;
	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

/* Stream mode: group the raw channel blocks and send them out. */
static void feed_raw(const struct sr_dev_inst *sdi, const uint8_t *p,
	size_t left)
{
	struct dev_context *devc;
	size_t take;

	devc = sdi->priv;
	while (left > 0 && !devc->acq_aborted) {
		take = MIN(left, devc->group_bytes - devc->group_fill);
		memcpy(devc->group_buf + devc->group_fill, p, take);
		devc->group_fill += take;
		p += take;
		left -= take;
		if (devc->group_fill < devc->group_bytes)
			break;
		devc->group_fill = 0;

		dl32_deinterleave(devc->group_buf, devc->num_channels,
			devc->conv_buf);
		process_samples(sdi, devc->conv_buf, DL32_BLOCK_SAMPLES);

		if (devc->acq_limit && devc->sent_samples >= devc->acq_limit) {
			dl32_abort_acquisition(devc);
			break;
		}
	}
}

/* Buffer / RLE mode: collect frames until the device says it is done. */
static void feed_framed(const struct sr_dev_inst *sdi, const uint8_t *p,
	size_t len)
{
	struct dev_context *devc;

	devc = sdi->priv;
	dl32_feed_bytes(devc->capture, devc->pending, p, len);

	if (devc->capture->frames)
		devc->data_started = TRUE;

	/*
	 * If lots of data arrives but none of it is framed, the device sends
	 * raw channel blocks like in stream mode. Switch to that format.
	 */
	if (!devc->capture->frames && !devc->capture->done &&
			devc->pending->len > 4 * devc->group_bytes) {
		sr_warn("No data frames in %u bytes; treating the data as raw "
			"channel blocks.", devc->pending->len);
		devc->framed = FALSE;
		devc->data_started = TRUE;
		feed_raw(sdi, devc->pending->data, devc->pending->len);
		g_byte_array_set_size(devc->pending, 0);
		return;
	}

	if (devc->capture->done)
		dl32_abort_acquisition(devc);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	const uint8_t *p;
	size_t len;

	sdi = transfer->user_data;
	devc = sdi->priv;

	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		sr_err("Device disconnected.");
		dl32_abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* May still carry data. */
		break;
	default:
		sr_err("Transfer error: %s.", libusb_error_name(transfer->status));
		dl32_abort_acquisition(devc);
		free_transfer(transfer);
		return;
	}

	if (transfer->actual_length <= 0) {
		/* Waiting for a trigger or a buffer capture is not an error. */
		if (devc->data_started &&
				++devc->empty_transfer_count > DL32_MAX_EMPTY_XFERS) {
			sr_err("Device stopped sending data.");
			dl32_abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	}
	devc->empty_transfer_count = 0;

	p = transfer->buffer;
	len = transfer->actual_length;
	if (devc->dump)
		fwrite(p, 1, len, devc->dump);
	if (sr_log_loglevel_get() >= SR_LOG_SPEW) {
		GString *str = sr_hexdump_new(p, MIN(len, 16));
		sr_spew("Read %zu bytes: %s", len, str->str);
		sr_hexdump_free(str);
	}

	if (devc->framed) {
		feed_framed(sdi, p, len);
	} else if (!devc->data_started && p[0] == 0x0a &&
			len < DL32_STATUS_MAX_LEN) {
		/* Status frames arrive as their own short reads. */
		dl32_consume_frames(devc->capture, p, len);
	} else {
		devc->data_started = TRUE;
		feed_raw(sdi, p, len);
	}

	if (devc->acq_aborted)
		free_transfer(transfer);
	else
		resubmit_transfer(transfer);
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct timeval tv;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout_completed(devc->ctx->libusb_ctx, &tv, NULL);

	if (devc->acq_done) {
		devc->acq_done = FALSE;
		finish_acquisition(sdi);
	}

	return TRUE;
}

static void reset_acquisition_state(struct dev_context *devc)
{
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
	g_free(devc->group_buf);
	devc->group_buf = NULL;
	g_free(devc->conv_buf);
	devc->conv_buf = NULL;
	if (devc->pending) {
		g_byte_array_free(devc->pending, TRUE);
		devc->pending = NULL;
	}
	dl32_capture_free(devc->capture);
	devc->capture = NULL;
	if (devc->dump) {
		fclose(devc->dump);
		devc->dump = NULL;
	}
}

SR_PRIV int dl32_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_trigger *trigger;
	struct libusb_transfer *transfer;
	struct sr_channel *ch;
	const GSList *l;
	uint8_t params[DL32_PARAMS_LEN], trig[DL32_TRIGGER_LEN];
	uint8_t codes[DL32_TRIGGER_POSITIONS];
	uint8_t one = 1, *reply;
	uint64_t limit, device_samples, trigger_pos, groups;
	const char *dump_path;
	size_t xfer_size;
	unsigned int i;
	int hz_index, ret;

	devc = sdi->priv;
	usb = sdi->conn;

	hz_index = dl32_samplerate_index(devc->samplerate);
	if (hz_index < 0) {
		sr_err("Unsupported samplerate %" PRIu64 ".", devc->samplerate);
		return SR_ERR_SAMPLERATE;
	}

	limit = devc->limit_samples;
	if (devc->limit_msec) {
		uint64_t from_msec = devc->limit_msec * devc->samplerate / 1000;
		if (!limit || from_msec < limit)
			limit = from_msec;
	}
	devc->acq_limit = limit;

	devc->buffer_mode = !devc->stream_mode ||
		devc->samplerate > DL32_MAX_STREAM_RATE;
	if (devc->buffer_mode && !limit) {
		sr_err("Buffer mode (rates above 100 MHz or continuous=0) "
			"needs a sample or time limit.");
		return SR_ERR_ARG;
	}
	devc->framed = devc->buffer_mode || devc->rle;

	/* Captured channels. Stream mode without RLE always sends all of them. */
	devc->enabled_mask = 0;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->enabled && ch->index < (int)devc->num_channels)
			devc->enabled_mask |= 1u << ch->index;
	}
	if (!devc->framed || !devc->enabled_mask)
		devc->enabled_mask = (devc->num_channels == 32) ?
			0xffffffff : (1u << devc->num_channels) - 1;

	/* Hardware trigger when the device can express it, else software. */
	trigger = sr_session_trigger_get(sdi->session);
	devc->hw_trigger = FALSE;
	devc->stl = NULL;
	devc->trigger_fired = TRUE;
	if (dl32_trigger_codes(trigger, devc->num_channels, codes) == SR_OK) {
		devc->hw_trigger = trigger != NULL;
	} else {
		sr_info("Trigger needs more than one stage or condition per "
			"channel; using a software trigger.");
		for (i = 0; i < devc->num_channels; i++)
			codes[i] = DL32_TRIG_NONE;
	}
	for (i = 0; i < devc->num_channels; i++) {
		if (codes[i] != DL32_TRIG_NONE)
			devc->enabled_mask |= 1u << i;
		codes[i] = (devc->enabled_mask & (1u << i)) ?
			(codes[i] | DL32_TRIG_ENABLE) : 0;
	}

	/*
	 * Buffer mode bandwidth: rate x captured channels must stay within the
	 * device's budget, or it uploads unusable data. Measured on a DL32
	 * Plus: 32 ch works at 250 MHz but not 400 MHz; 16 ch works at 400 MHz.
	 */
	if (devc->buffer_mode) {
		unsigned int nen = 0;
		uint64_t budget, max_rate;

		for (i = 0; i < devc->num_channels; i++)
			nen += (devc->enabled_mask >> i) & 1;
		budget = devc->info.plus ? DL32_PLUS_BUDGET : DL32_BUDGET;
		if (nen && devc->samplerate * nen > budget) {
			max_rate = budget / nen;
			sr_err("At %" PRIu64 " MHz the %s can capture at most %"
				PRIu64 " channels in buffer mode (%u enabled). "
				"Disable channels or use %" PRIu64 " MHz or less.",
				devc->samplerate / SR_MHZ(1),
				devc->info.plus ? "DL32 Plus" : "DL32",
				budget / devc->samplerate, nen,
				max_rate / SR_MHZ(1));
			return SR_ERR_SAMPLERATE;
		}
	}

	/* Samples requested from the device, and the pre-trigger position. */
	trigger_pos = limit ? (limit / 100) * devc->capture_ratio : 0;
	if (devc->buffer_mode) {
		device_samples = limit;
	} else if (limit && !(trigger && !devc->hw_trigger)) {
		/* Whole groups; the extra samples are dropped. */
		groups = (limit + DL32_BLOCK_SAMPLES - 1) / DL32_BLOCK_SAMPLES;
		device_samples = groups * DL32_BLOCK_SAMPLES;
	} else {
		/* Continuous, or a software trigger at an unknown point. */
		device_samples = devc->samplerate * 3600;
	}
	device_samples = MIN(device_samples, DL32_MAX_DEVICE_SAMPLES);
	devc->pre_trigger = trigger_pos;
	devc->marker_sent = FALSE;

	devc->unitsize = devc->num_channels / 8;
	devc->group_bytes = (size_t)devc->num_channels * DL32_BLOCK_BYTES;
	devc->group_fill = 0;
	devc->group_buf = g_malloc(devc->group_bytes);
	devc->conv_buf = g_malloc((size_t)DL32_BLOCK_SAMPLES * devc->unitsize);
	devc->pending = g_byte_array_new();
	devc->capture = dl32_capture_new(devc->rle);
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	devc->acq_done = FALSE;
	devc->data_started = FALSE;
	devc->frame_begun = FALSE;
	devc->ctx = ((struct drv_context *)sdi->driver->context)->sr_ctx;

	if (trigger && !devc->hw_trigger) {
		devc->stl = soft_trigger_logic_new(sdi, trigger,
			limit ? (devc->capture_ratio * limit) / 100 : 0);
		if (!devc->stl)
			goto err_free;
		devc->trigger_fired = FALSE;
	}

	/* Diagnostics: ALIENTEK_DL32_DUMP=<file> saves everything received. */
	dump_path = g_getenv("ALIENTEK_DL32_DUMP");
	devc->dump = dump_path ? fopen(dump_path, "wb") : NULL;
	if (dump_path && !devc->dump)
		sr_warn("Cannot open dump file %s.", dump_path);

	dl32_build_params(params, devc->buffer_mode, devc->rle,
		threshold_tenths[devc->threshold_idx], hz_index,
		device_samples, trigger_pos);
	dl32_build_trigger(trig, codes, devc->num_channels, devc->ch_offset);
	/*
	 * Without a trigger, buffer mode must be told to capture at once
	 * ("instant" in ATK-Logic); otherwise the device arms and waits.
	 * Stream mode ignores this (verified with the flag clear).
	 */
	if (devc->buffer_mode && !devc->hw_trigger)
		trig[DL32_TRIGGER_INSTANT] = 1;

	if (sr_log_loglevel_get() >= SR_LOG_DBG) {
		GString *ps = sr_hexdump_new(params, sizeof(params));
		GString *ts = sr_hexdump_new(trig, sizeof(trig));
		sr_dbg("Starting %s mode%s: %" PRIu64 " Hz, %" PRIu64
			" device samples, limit %" PRIu64 ", %s trigger.",
			devc->buffer_mode ? "buffer" : "stream",
			devc->rle ? " (RLE)" : "", devc->samplerate,
			device_samples, limit, devc->hw_trigger ? "hardware" :
			devc->stl ? "software" : "no");
		sr_dbg("Params: %s", ps->str);
		sr_dbg("Trigger: %s", ts->str);
		sr_hexdump_free(ps);
		sr_hexdump_free(ts);
	}

	/* Wake the FPGA, then send parameters and the start (trigger) command. */
	usb_drain(usb->devhdl);
	if (send_raw(usb->devhdl, DL32_RAW_CMD_RESET, &one, 1) != SR_OK)
		goto err_free;
	reply = g_malloc(DL32_READ_LEN);
	usb_read(usb->devhdl, reply, DL32_READ_LEN, 200);
	g_free(reply);
	usb_drain(usb->devhdl);

	/* Queue reads before starting so no data is lost at high rates. */
	xfer_size = devc->group_bytes * DL32_GROUPS_PER_XFER;
	devc->transfers = g_malloc0(sizeof(*devc->transfers) * DL32_NUM_TRANSFERS);
	devc->num_transfers = DL32_NUM_TRANSFERS;
	devc->submitted_transfers = 0;
	for (i = 0; i < DL32_NUM_TRANSFERS; i++) {
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl, DL32_EP_IN,
			g_malloc(xfer_size), xfer_size, receive_transfer,
			(void *)sdi, DL32_USB_TIMEOUT_MS);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
				libusb_error_name(ret));
			g_free(transfer->buffer);
			libusb_free_transfer(transfer);
			goto err_cancel;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	usb_source_add(sdi->session, devc->ctx, 100, receive_data, (void *)sdi);
	std_session_send_df_header(sdi);
	std_session_send_df_frame_begin(sdi);
	devc->frame_begun = TRUE;

	if (send_framed(usb->devhdl, DL32_CMD_PARAMS, params, sizeof(params)) != SR_OK) {
		sr_err("Failed to send capture parameters.");
		dl32_abort_acquisition(devc);
		return SR_OK;
	}
	/* ATK-Logic waits between the two commands so the FPGA can settle. */
	g_usleep(30 * 1000);
	if (send_framed(usb->devhdl, DL32_CMD_TRIGGER, trig, sizeof(trig)) != SR_OK) {
		sr_err("Failed to send the start command.");
		dl32_abort_acquisition(devc);
	}

	return SR_OK;

err_cancel:
	/* Nothing has been handed to the session yet: tear down directly. */
	devc->acq_aborted = TRUE;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
	while (devc->submitted_transfers) {
		struct timeval tv = { 0, 100000 };
		libusb_handle_events_timeout_completed(devc->ctx->libusb_ctx,
			&tv, NULL);
	}
err_free:
	reset_acquisition_state(devc);

	return SR_ERR;
}
