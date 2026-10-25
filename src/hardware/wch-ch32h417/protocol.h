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

/*
 * WCH CH32H417 USB3 logic analyzer (16 channels, up to 200 MS/s, plus a
 * 2 channel 10-bit 20 MS/s HSADC), running the OpenSourceLogic-CH32H417
 * APP firmware. Ported from the ALL LOGIC (libsigrok4DSL) "ch32" driver,
 * fast-trigger branch.
 *
 * USB:
 *   VID 1A86, PID 5537 (APP enumerated at USB2 high speed) or
 *   PID 5538 (APP enumerated at USB3 super speed)
 *   EP1 OUT / EP1 IN  commands: {cmd, len, payload[5]} ->
 *                     {cmd | 0x10, len, status, payload[5]}
 *                     (START / STOP commands have no reply)
 *   EP2 IN            logic stream: 1 byte per sample (8 channels) or
 *                     little-endian 16 bits per sample (16 channels),
 *                     bit k = channel k
 *   EP3 IN            HSADC stream: little-endian 16 bits per sample,
 *                     10 significant bits, one channel at a time
 *
 * Acquisition model: the device streams every sample, there is no trigger
 * or sample memory on the device. The trigger is searched on the PC
 * (8 or 4 samples per step on 64-bit words, see find_trigger()); the
 * pre-trigger part is kept in a ring buffer, and from the trigger on the
 * samples go to the session as they arrive.
 *
 * Sample rates: on USB3 the firmware has 24 PLL / divider settings from
 * 200 MS/s down to 6.25 MS/s. On USB2 the firmware forces a fixed clock
 * (40 MS/s with 8 channels, 20 MS/s with 16). Lower rates are made on the
 * PC by decimation, a per-channel majority vote over N samples.
 */

#ifndef LIBSIGROK_HARDWARE_WCH_CH32H417_PROTOCOL_H
#define LIBSIGROK_HARDWARE_WCH_CH32H417_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "wch-ch32h417"

#define CH32_VID		0x1a86
#define CH32_PID_USB2		0x5537
#define CH32_PID_USB3		0x5538

#define CH32_EP_CMD_OUT		0x01
#define CH32_EP_CMD_IN		0x81
#define CH32_EP_LOGIC_IN	0x82
#define CH32_EP_ADC_IN		0x83

#define CMD_SET_LOGIC_PARA	0xa0	/* {bit width 8/16, rate index} */
#define CMD_SET_LOGIC_LEVEL	0xa1	/* {DAC hi, DAC lo} */
#define CMD_SET_START		0xa2
#define CMD_SET_STOP		0xa3
#define CMD_SET_ADC_PARA	0xa6	/* {bits (0x0a), divider} */
#define CMD_SET_ADC_CHANNEL	0xa7	/* {0 or 1} */
#define CMD_SET_ADC_START	0xa8
#define CMD_SET_ADC_STOP	0xa9
#define CMD_GET_VERSION		0xac
#define CMD_RETURN_PARA		0x10

#define CH32_NUM_LOGIC_CHANNELS	16
#define CH32_NUM_ANALOG_CHANNELS 2

/* HSADC: rate = 40 MHz / (div + 1), div >= 1 (calibrated on hardware) */
#define CH32_ADC_CLOCK		SR_MHZ(40)
#define CH32_ADC_MIN_DIV	1
#define CH32_ADC_MAX_DIV	39	/* 1 MS/s */
#define CH32_ADC_BITS		0x0a
#define CH32_ADC_VREF		3.3

/* comparator threshold range covered by the calibration table */
#define CH32_VTH_MIN		0.45
#define CH32_VTH_MAX		3.40
#define CH32_VTH_STEP		0.05
#define CH32_VTH_DEFAULT	1.65

#define CH32_CMD_TIMEOUT_MS	1000
#define CH32_NUM_TRANSFERS	16
#define CH32_BULK_TIMEOUT_MS	500

/* largest single logic packet sent to the session, in samples */
#define CH32_SEND_CHUNK		(1024 * 1024)

enum ch32_acq_state {
	CH32_IDLE,
	CH32_ARMED,		/* waiting for the trigger */
	CH32_STREAMING,		/* sending samples to the session */
	CH32_DONE,		/* sample limit reached */
};

struct dev_context {
	/* device */
	gboolean usb3;		/* firmware runs at super speed */
	uint8_t sw_ver, hw_ver;
	/* largest WinUSB RAW_IO read on the data endpoints, 0 = unknown or
	 * no RAW_IO (libusb >= 1.0.30 on Windows only) */
	unsigned int raw_io_max;

	/* settings */
	uint64_t samplerate;
	uint64_t limit_samples;	/* 0 = continuous */
	uint64_t capture_ratio;
	double vth;

	/* acquisition setup */
	gboolean analog;	/* HSADC acquisition instead of logic */
	struct sr_channel *analog_ch;
	uint8_t logic_bit;	/* 8 or 16 */
	unsigned int unit;	/* bytes per sample: 1 or 2 */
	uint8_t logic_sys;	/* firmware rate index */
	uint64_t hw_rate;	/* rate the device samples at */
	unsigned int decim;	/* host decimation factor, 1 = none */
	uint8_t adc_div;

	/* USB transfers */
	struct libusb_transfer **transfers;
	unsigned int num_transfers;
	unsigned int submitted;
	gboolean hw_running;	/* START sent, STOP not yet */
	gboolean stop_req;	/* stop the device from receive_data() */
	gboolean finished;	/* all transfers back: end in receive_data() */

	/* sample stream */
	int state;
	uint64_t sent;		/* samples sent to the session */
	uint8_t carry[2];	/* half of a 16-bit sample left by a transfer */
	unsigned int carry_len;
	uint8_t *work;		/* decimated / joined samples of one transfer */
	size_t work_size;
	unsigned int dec_fill;	/* decimation window: samples so far */
	gboolean dec_mixed;	/* window holds more than one value */
	uint16_t dec_first, dec_last;
	uint32_t dec_ones[CH32_NUM_LOGIC_CHANNELS];
	float *fbuf;		/* analog samples in volts */
	size_t fbuf_size;

	/* trigger (all masks ANDed, bit n = channel n) */
	gboolean have_trigger;
	uint16_t trig_zero, trig_one, trig_rise, trig_fall, trig_edge;
	uint64_t pre;		/* samples before the trigger */
	uint8_t *hist;		/* pre-trigger ring buffer, pre samples */
	uint64_t hist_pos, hist_len;
	gboolean have_prev;
	uint16_t prev;
};

SR_PRIV int ch32_cmd(const struct sr_dev_inst *sdi, uint8_t cmd,
		const uint8_t *payload, uint8_t plen, uint8_t *rsp, int rsp_size);
SR_PRIV int ch32_get_version(const struct sr_dev_inst *sdi);
SR_PRIV int ch32_apply_vth(const struct sr_dev_inst *sdi);
SR_PRIV uint16_t ch32_vth_to_dac(double v);

SR_PRIV gboolean ch32_has_fast8(const struct dev_context *devc);
SR_PRIV size_t ch32_samplerate_list(const struct dev_context *devc,
		uint64_t *out, size_t max);
SR_PRIV int ch32_map_samplerate(struct dev_context *devc, uint64_t rate,
		unsigned int logic_bit);
SR_PRIV unsigned int ch32_adc_div(uint64_t rate);

SR_PRIV int ch32_build_trigger(const struct sr_dev_inst *sdi);
SR_PRIV size_t ch32_find_trigger(const struct dev_context *devc,
		const uint8_t *s, size_t from, size_t n);
SR_PRIV size_t ch32_decimate(struct dev_context *devc, const uint8_t *in,
		size_t n, uint8_t *out);

SR_PRIV int ch32_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int ch32_acquisition_stop(const struct sr_dev_inst *sdi);
SR_PRIV void ch32_hw_stop(const struct sr_dev_inst *sdi);
SR_PRIV void ch32_setup_raw_io(const struct sr_dev_inst *sdi);

#endif
