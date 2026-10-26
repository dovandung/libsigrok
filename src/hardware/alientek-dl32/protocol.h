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

/*
 * ALIENTEK (ATK) DL32 / DL32 Plus logic analyzer.
 *
 * Protocol knowledge comes from the GPL ATK-Logic sources (DL16 framing,
 * which the DL32 shares) and from the reverse engineering notes of the
 * atk-dl32-mcp project (https://github.com/apiwat59/atk-dl32-mcp), which
 * verified stream mode capture on real hardware.
 */

#ifndef LIBSIGROK_HARDWARE_ALIENTEK_DL32_PROTOCOL_H
#define LIBSIGROK_HARDWARE_ALIENTEK_DL32_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "alientek-dl32"

#define DL32_VID		0x04b4
#define DL32_PID		0x6a6a
#define DL32_USB_INTERFACE	0
#define DL32_EP_OUT		0x01
#define DL32_EP_IN		0x81

/* Raw "system" packets: [0x0A][cmd][payload][0x0B], padded to 510 bytes. */
#define DL32_RAW_PACKET_LEN	0x1fe
#define DL32_RAW_CMD_HELLO	0x81
#define DL32_RAW_CMD_RESET	0x87

/* Framed commands: 00x8 | 0A | cmd | len+1 | data | 0B | crc32 LE | 00. */
#define DL32_FRAME_OVERHEAD	17
#define DL32_CMD_DEVICE_DATA	0x10
#define DL32_CMD_PARAMS		0x11
#define DL32_CMD_TRIGGER	0x12
#define DL32_CMD_STOP		0x15

#define DL32_PARAMS_LEN		13
#define DL32_TRIGGER_LEN	18
#define DL32_TRIGGER_POSITIONS	32

/*
 * Stream data arrives as groups of one 2016 byte block per channel.
 * Each byte holds 8 consecutive samples of that channel, LSB first.
 */
#define DL32_BLOCK_BYTES	2016
#define DL32_BLOCK_SAMPLES	(DL32_BLOCK_BYTES * 8)

/* Short reads that start with 0x0A before any data are status packets. */
#define DL32_STATUS_MAX_LEN	4096

#define DL32_NUM_TRANSFERS	16
#define DL32_GROUPS_PER_XFER	8
#define DL32_USB_TIMEOUT_MS	1000
#define DL32_MAX_EMPTY_XFERS	(DL32_NUM_TRANSFERS * 4)

/* The device takes a 40 bit sample count. */
#define DL32_MAX_DEVICE_SAMPLES	((UINT64_C(1) << 40) - 1)

struct dl32_info {
	char mode;
	int mcu_version;
	int variant;
	gboolean plus;
	char name[32];
};

struct dev_context {
	/* Configuration. */
	uint64_t samplerate;
	uint64_t limit_samples;
	uint64_t limit_msec;
	uint64_t capture_ratio;
	int threshold_idx;
	unsigned int num_channels;
	unsigned int ch_offset;
	struct dl32_info info;

	/* Acquisition state. */
	struct sr_context *ctx;
	struct libusb_transfer **transfers;
	unsigned int num_transfers;
	unsigned int submitted_transfers;
	unsigned int empty_transfer_count;
	gboolean acq_aborted;
	gboolean acq_done;
	gboolean data_started;
	gboolean frame_begun;
	uint64_t acq_limit;
	uint64_t sent_samples;

	uint8_t *group_buf;
	size_t group_bytes;
	size_t group_fill;
	uint8_t *conv_buf;
	unsigned int unitsize;

	struct soft_trigger_logic *stl;
	gboolean trigger_fired;
};

SR_PRIV extern const uint64_t dl32_samplerates[];
SR_PRIV extern const unsigned int dl32_num_samplerates;
SR_PRIV extern const char *dl32_thresholds[];
SR_PRIV extern const unsigned int dl32_num_thresholds;

/* Pure helpers, kept free of USB access so they can be unit tested. */
SR_PRIV uint32_t dl32_crc32(const uint8_t *buf, size_t len);
SR_PRIV void dl32_build_raw(uint8_t out[DL32_RAW_PACKET_LEN], uint8_t cmd,
	const uint8_t *payload, size_t len);
SR_PRIV size_t dl32_build_framed(uint8_t *out, uint8_t cmd,
	const uint8_t *data, size_t len);
SR_PRIV int dl32_samplerate_index(uint64_t samplerate);
SR_PRIV void dl32_build_params(uint8_t out[DL32_PARAMS_LEN],
	gboolean buffer_mode, gboolean rle, int threshold_tenths,
	int hz_index, uint64_t samples, uint64_t trigger_pos);
SR_PRIV void dl32_build_trigger(uint8_t out[DL32_TRIGGER_LEN],
	unsigned int num_channels, unsigned int ch_offset);
SR_PRIV gboolean dl32_parse_hello(const uint8_t *buf, size_t len,
	struct dl32_info *info);
SR_PRIV void dl32_parse_name(const uint8_t *buf, size_t len,
	struct dl32_info *info);
SR_PRIV void dl32_deinterleave(const uint8_t *group, unsigned int num_channels,
	uint8_t *out);

/* Device access. */
SR_PRIV int dl32_identify(libusb_device_handle *hdl, struct dl32_info *info);
SR_PRIV int dl32_start_acquisition(const struct sr_dev_inst *sdi);
SR_PRIV void dl32_abort_acquisition(struct dev_context *devc);
SR_PRIV void dl32_send_stop(libusb_device_handle *hdl);

#endif
