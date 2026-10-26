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

/* Stream mode rates. Higher rates need buffer mode, not supported yet. */
SR_PRIV const uint64_t dl32_samplerates[] = {
	SR_MHZ(1), SR_MHZ(2), SR_MHZ(4), SR_MHZ(5), SR_MHZ(10),
	SR_MHZ(20), SR_MHZ(25), SR_MHZ(40), SR_MHZ(50), SR_MHZ(100),
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
 * the high nibble), 0x8 = enabled, low 3 bits = trigger code, 0x7 = none.
 * Hardware triggers are not used: every captured channel is "enabled, no
 * trigger" and triggering is done in software.
 */
SR_PRIV void dl32_build_trigger(uint8_t out[DL32_TRIGGER_LEN],
	unsigned int num_channels, unsigned int ch_offset)
{
	unsigned int ch, pos;

	memset(out, 0, DL32_TRIGGER_LEN);
	for (ch = 0; ch < num_channels; ch++) {
		pos = ch_offset + ch;
		if (pos >= DL32_TRIGGER_POSITIONS)
			break;
		out[pos / 2] |= (pos & 1) ? 0x0f : 0xf0;
	}
	/* out[16], out[17]: collect type flags, both 0 for a normal run. */
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
SR_PRIV void dl32_deinterleave(const uint8_t *group, unsigned int num_channels,
	uint8_t *out)
{
	static uint64_t spread[256];
	static gboolean spread_ready;
	unsigned int unitsize, i, c, j, k;
	uint64_t acc[4];

	if (!spread_ready) {
		for (i = 0; i < 256; i++) {
			spread[i] = 0;
			for (k = 0; k < 8; k++)
				spread[i] |= (uint64_t)((i >> k) & 1) << (8 * k);
		}
		spread_ready = TRUE;
	}

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

static int usb_read(libusb_device_handle *hdl, uint8_t *buf, int len,
	unsigned int timeout_ms)
{
	int ret, done;

	done = 0;
	ret = libusb_bulk_transfer(hdl, DL32_EP_IN, buf, len, &done, timeout_ms);
	if (ret != 0 && ret != LIBUSB_ERROR_TIMEOUT)
		return -1;

	return done;
}

/* Discard anything still queued in the device (e.g. a previous stream). */
static void usb_drain(libusb_device_handle *hdl)
{
	uint8_t *buf;
	int i, n;
	size_t total;

	buf = g_malloc(64 * 1024);
	total = 0;
	for (i = 0; i < 4096; i++) {
		n = usb_read(hdl, buf, 64 * 1024, 50);
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
	uint8_t buf[0x800];
	int n;

	memset(info, 0, sizeof(*info));
	usb_drain(hdl);

	if (send_raw(hdl, DL32_RAW_CMD_HELLO, NULL, 0) != SR_OK)
		return SR_ERR;
	n = usb_read(hdl, buf, 512, 300);
	if (n <= 0 || !dl32_parse_hello(buf, n, info)) {
		sr_dbg("No valid hello reply (%d bytes).", n);
		return SR_ERR;
	}
	sr_dbg("Hello: mode '%c', MCU %d, variant %d, plus %d.",
		info->mode, info->mcu_version, info->variant, info->plus);
	if (info->mode != 'a') {
		sr_err("Device is not in application mode (mode 0x%02x); "
			"update its firmware with ATK-Logic first.", info->mode);
		return SR_ERR;
	}

	if (send_framed(hdl, DL32_CMD_DEVICE_DATA, NULL, 0) == SR_OK) {
		n = usb_read(hdl, buf, sizeof(buf), 300);
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

	return SR_OK;
}

SR_PRIV void dl32_send_stop(libusb_device_handle *hdl)
{
	const uint8_t zero = 0;

	send_framed(hdl, DL32_CMD_STOP, &zero, 1);
	send_raw(hdl, DL32_RAW_CMD_RESET, &zero, 1);
	usb_drain(hdl);
}

/* --- Asynchronous acquisition. */

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	devc = sdi->priv;
	usb = sdi->conn;

	usb_source_remove(sdi->session, devc->ctx);
	dl32_send_stop(usb->devhdl);

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

/* Feed one de-interleaved group into the session, honouring trigger/limit. */
static void process_samples(const struct sr_dev_inst *sdi, uint8_t *buf,
	uint64_t num_samples)
{
	struct dev_context *devc;
	uint64_t n;
	int trigger_offset, pre_trigger_samples;

	devc = sdi->priv;

	if (!devc->trigger_fired) {
		trigger_offset = soft_trigger_logic_check(devc->stl, buf,
			num_samples * devc->unitsize, &pre_trigger_samples);
		if (trigger_offset < 0)
			return;
		/* Pre-trigger samples were already sent by the checker. */
		devc->trigger_fired = TRUE;
		devc->sent_samples += pre_trigger_samples;
		buf += trigger_offset * devc->unitsize;
		num_samples -= trigger_offset;
	}

	n = num_samples;
	if (devc->acq_limit && devc->sent_samples + n > devc->acq_limit)
		n = devc->acq_limit - devc->sent_samples;
	send_logic(sdi, buf, n);
	devc->sent_samples += n;
}

static void handle_status(const uint8_t *buf, int len)
{
	if (sr_log_loglevel_get() >= SR_LOG_DBG) {
		GString *s = sr_hexdump_new(buf, MIN(len, 24));
		sr_dbg("Status packet type 0x%02x: %s", buf[1], s->str);
		sr_hexdump_free(s);
	}
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	const uint8_t *p;
	size_t left, take;

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
		if (++devc->empty_transfer_count > DL32_MAX_EMPTY_XFERS) {
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
	left = transfer->actual_length;

	/* Status packets ([0A][type]...) arrive as their own short reads. */
	if (!devc->data_started) {
		if (p[0] == 0x0a && left < DL32_STATUS_MAX_LEN) {
			handle_status(p, left);
			resubmit_transfer(transfer);
			return;
		}
		devc->data_started = TRUE;
	}

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

SR_PRIV int dl32_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_trigger *trigger;
	struct libusb_transfer *transfer;
	uint8_t params[DL32_PARAMS_LEN], trig[DL32_TRIGGER_LEN];
	uint8_t one = 1, reply[512];
	uint64_t limit, device_samples, groups;
	size_t xfer_size;
	unsigned int i;
	int hz_index, pre_trigger, ret;

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

	/*
	 * The device streams a fixed number of samples. Round up to whole
	 * groups; for continuous mode ask for an hour (or the 40 bit max).
	 * With a software trigger the trigger point is unknown, so keep
	 * streaming until the post-trigger limit is reached or stopped.
	 */
	trigger = sr_session_trigger_get(sdi->session);
	if (limit && !trigger)
		device_samples = limit;
	else
		device_samples = MIN(devc->samplerate * 3600,
			DL32_MAX_DEVICE_SAMPLES);
	groups = (device_samples + DL32_BLOCK_SAMPLES - 1) / DL32_BLOCK_SAMPLES;
	device_samples = MIN(groups * DL32_BLOCK_SAMPLES, DL32_MAX_DEVICE_SAMPLES);

	devc->unitsize = devc->num_channels / 8;
	devc->group_bytes = (size_t)devc->num_channels * DL32_BLOCK_BYTES;
	devc->group_fill = 0;
	devc->group_buf = g_malloc(devc->group_bytes);
	devc->conv_buf = g_malloc((size_t)DL32_BLOCK_SAMPLES * devc->unitsize);
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	devc->acq_done = FALSE;
	devc->data_started = FALSE;
	devc->frame_begun = FALSE;
	devc->ctx = sdi->driver->context ?
		((struct drv_context *)sdi->driver->context)->sr_ctx : NULL;

	if (trigger) {
		pre_trigger = limit ? (devc->capture_ratio * limit) / 100 : 0;
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger);
		if (!devc->stl)
			goto err_free;
		devc->trigger_fired = FALSE;
	} else {
		devc->trigger_fired = TRUE;
	}

	dl32_build_params(params, FALSE, FALSE,
		threshold_tenths[devc->threshold_idx], hz_index,
		device_samples, 0);
	dl32_build_trigger(trig, devc->num_channels, devc->ch_offset);

	sr_dbg("Starting: %" PRIu64 " Hz, %u channels, %" PRIu64
		" device samples, limit %" PRIu64 ".", devc->samplerate,
		devc->num_channels, device_samples, limit);

	/* Wake the FPGA, then send parameters and the start (trigger) command. */
	usb_drain(usb->devhdl);
	if (send_raw(usb->devhdl, DL32_RAW_CMD_RESET, &one, 1) != SR_OK)
		goto err_free;
	usb_read(usb->devhdl, reply, sizeof(reply), 200);
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

	if (send_framed(usb->devhdl, DL32_CMD_PARAMS, params, sizeof(params)) != SR_OK ||
	    send_framed(usb->devhdl, DL32_CMD_TRIGGER, trig, sizeof(trig)) != SR_OK) {
		sr_err("Failed to send start commands.");
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
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
err_free:
	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
	g_free(devc->group_buf);
	devc->group_buf = NULL;
	g_free(devc->conv_buf);
	devc->conv_buf = NULL;

	return SR_ERR;
}
