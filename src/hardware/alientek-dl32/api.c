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
#include <stdlib.h>
#include <string.h>
#include "protocol.h"

/*
 * Channel layouts. The trigger/enable positions are the physical inputs:
 * position 0 is CH0 (confirmed on a DL32 Plus, where data channel ids
 * follow the enabled positions). By default the DL32 Plus shows all 32
 * inputs (D0-D31 = CH0-CH31; tested in stream and buffer mode) and the
 * plain DL32 shows CH0-CH15. "logic_channels=16|32" overrides that, and
 * ALIENTEK_DL32_OFFSET=<n> selects a 16-channel window starting at CH<n>
 * (ATK-Logic uses 8 for some DL32 models).
 */
#define DEFAULT_CHANNELS	16
#define PLUS_CHANNELS		32
#define DEFAULT_CH_OFFSET	0

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_NUM_LOGIC_CHANNELS,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_RLE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LOGIC_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct dl32_info info;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	libusb_device_handle *hdl;
	GSList *devices, *conn_devices, *l;
	const char *conn;
	char connection_id[64], name[8];
	unsigned int num_channels, nch, ch_offset, i, j;
	int ret;

	drvc = di->context;
	devices = NULL;
	conn = NULL;
	num_channels = 0; /* Automatic: depends on the model. */
	ch_offset = DEFAULT_CH_OFFSET;
	if (g_getenv("ALIENTEK_DL32_OFFSET")) {
		num_channels = DEFAULT_CHANNELS;
		ch_offset = atoi(g_getenv("ALIENTEK_DL32_OFFSET"));
		if (ch_offset > DL32_TRIGGER_POSITIONS - DEFAULT_CHANNELS ||
				ch_offset % 2) {
			sr_err("ALIENTEK_DL32_OFFSET must be even and at most %d.",
				DL32_TRIGGER_POSITIONS - DEFAULT_CHANNELS);
			return NULL;
		}
		sr_info("Capturing CH%u-CH%u.", ch_offset,
			ch_offset + DEFAULT_CHANNELS - 1);
	}

	for (l = options; l; l = l->next) {
		struct sr_config *src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		case SR_CONF_NUM_LOGIC_CHANNELS:
			num_channels = g_variant_get_int32(src->data);
			break;
		}
	}
	if (num_channels && num_channels != 16 && num_channels != 32) {
		sr_err("logic_channels must be 16 or 32.");
		return NULL;
	}

	conn_devices = conn ? sr_usb_find(drvc->sr_ctx->libusb_ctx, conn) : NULL;
	if (conn && !conn_devices)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		libusb_get_device_descriptor(devlist[i], &des);
		if (des.idVendor != DL32_VID || des.idProduct != DL32_PID)
			continue;

		if (conn_devices) {
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i]) &&
				    usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				continue;
		}

		if (usb_get_port_path(devlist[i], connection_id,
				sizeof(connection_id)) < 0) {
			sr_err("Cannot get the USB port path of a %04x:%04x "
				"device.", des.idVendor, des.idProduct);
			continue;
		}
		sr_info("Checking %04x:%04x at %s.", des.idVendor,
			des.idProduct, connection_id);

		/* 04b4:6a6a is a Cypress FX3 ID; confirm it is a DL32. */
		if ((ret = libusb_open(devlist[i], &hdl)) != 0) {
			if (ret == LIBUSB_ERROR_NOT_SUPPORTED)
				sr_err("Found a DL32 at %s, but it uses a driver "
					"libusb cannot open (probably Cypress "
					"cyusb3 from ATK-Logic). Switch it to "
					"WinUSB with Zadig.", connection_id);
			else
				sr_err("Found a DL32 at %s, but cannot open it: %s.",
					connection_id, libusb_error_name(ret));
			continue;
		}
		libusb_set_auto_detach_kernel_driver(hdl, 1);
		if ((ret = libusb_claim_interface(hdl, DL32_USB_INTERFACE)) != 0) {
			sr_err("Cannot claim the DL32 at %s: %s (close ATK-Logic "
				"or any other program using it).",
				connection_id, libusb_error_name(ret));
			libusb_close(hdl);
			continue;
		}
		ret = dl32_identify(hdl, &info);
		libusb_release_interface(hdl, DL32_USB_INTERFACE);
		libusb_close(hdl);
		if (ret != SR_OK) {
			sr_err("Device %04x:%04x at %s did not answer as a DL32; "
				"run with -l 5 for details.", des.idVendor,
				des.idProduct, connection_id);
			continue;
		}

		sdi = g_malloc0(sizeof(*sdi));
		sdi->status = SR_ST_INACTIVE;
		sdi->vendor = g_strdup("ALIENTEK");
		sdi->model = g_strdup_printf("%s%s", info.name,
			info.plus ? " Plus" : "");
		sdi->version = g_strdup_printf("MCU %d.%d", info.mcu_version / 10,
			info.mcu_version % 10);
		sdi->connection_id = g_strdup(connection_id);
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
			libusb_get_device_address(devlist[i]), NULL);

		nch = num_channels ? num_channels :
			info.plus ? PLUS_CHANNELS : DEFAULT_CHANNELS;
		for (j = 0; j < nch; j++) {
			g_snprintf(name, sizeof(name), "D%u", j);
			sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE, name);
		}

		devc = g_malloc0(sizeof(*devc));
		devc->info = info;
		devc->num_channels = nch;
		devc->ch_offset = nch == 32 ? 0 : ch_offset;
		devc->samplerate = SR_MHZ(10);
		devc->limit_samples = SR_MHZ(1);
		devc->capture_ratio = 10;
		devc->threshold_idx = 1; /* 3.3V */
		devc->stream_mode = TRUE;
		devc->rle = FALSE;
		sdi->priv = devc;

		sr_info("Found %s %s at %s.", sdi->vendor, sdi->model,
			connection_id);
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;
	int ret;

	drvc = sdi->driver->context;
	usb = sdi->conn;

	if (sr_usb_open(drvc->sr_ctx->libusb_ctx, usb) != SR_OK)
		return SR_ERR;

	libusb_set_auto_detach_kernel_driver(usb->devhdl, 1);
	if ((ret = libusb_claim_interface(usb->devhdl, DL32_USB_INTERFACE)) != 0) {
		sr_err("Failed to claim interface: %s.", libusb_error_name(ret));
		sr_usb_close(usb);
		return SR_ERR;
	}

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;
	if (!usb->devhdl)
		return SR_ERR_BUG;

	dl32_send_stop(usb->devhdl);
	libusb_release_interface(usb->devhdl, DL32_USB_INTERFACE);
	sr_usb_close(usb);

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (key) {
	case SR_CONF_CONN:
		usb = sdi->conn;
		if (!usb)
			return SR_ERR_ARG;
		*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_LIMIT_MSEC:
		*data = g_variant_new_uint64(devc->limit_msec);
		break;
	case SR_CONF_LOGIC_THRESHOLD:
		*data = g_variant_new_string(dl32_thresholds[devc->threshold_idx]);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_CONTINUOUS:
		*data = g_variant_new_boolean(devc->stream_mode);
		break;
	case SR_CONF_RLE:
		*data = g_variant_new_boolean(devc->rle);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	int idx;

	(void)cg;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		if (std_u64_idx(data, dl32_samplerates, dl32_num_samplerates) < 0)
			return SR_ERR_ARG;
		devc->samplerate = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_LIMIT_MSEC:
		devc->limit_msec = g_variant_get_uint64(data);
		break;
	case SR_CONF_LOGIC_THRESHOLD:
		if ((idx = std_str_idx(data, dl32_thresholds, dl32_num_thresholds)) < 0)
			return SR_ERR_ARG;
		devc->threshold_idx = idx;
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		break;
	case SR_CONF_CONTINUOUS:
		/* TRUE: stream to the host. FALSE: capture into device memory. */
		devc->stream_mode = g_variant_get_boolean(data);
		break;
	case SR_CONF_RLE:
		devc->rle = g_variant_get_boolean(data);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(dl32_samplerates, dl32_num_samplerates);
		break;
	case SR_CONF_LOGIC_THRESHOLD:
		*data = std_gvar_array_str(dl32_thresholds, dl32_num_thresholds);
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	return dl32_start_acquisition(sdi);
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	dl32_abort_acquisition(sdi->priv);

	return SR_OK;
}

static struct sr_dev_driver alientek_dl32_driver_info = {
	.name = "alientek-dl32",
	.longname = "ALIENTEK DL32 / DL32 Plus",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(alientek_dl32_driver_info);
