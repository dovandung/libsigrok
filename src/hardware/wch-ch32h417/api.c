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

#define USB_INTERFACE		0
#define DEFAULT_SAMPLERATE	SR_MHZ(20)
#define DEFAULT_LIMIT		SR_MHZ(1)	/* samples */
#define MAX_SAMPLERATES		48

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
	SR_CONF_OSCILLOSCOPE,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

static gboolean is_ch32(const struct libusb_device_descriptor *des)
{
	return des->idVendor == CH32_VID &&
		(des->idProduct == CH32_PID_USB2 || des->idProduct == CH32_PID_USB3);
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc = di->context;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_config *src;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	GSList *l, *devices = NULL, *conn_devices = NULL;
	const char *conn = NULL;
	char connection_id[64], name[8];
	int i, j, speed;

	for (l = options; l; l = l->next) {
		src = l->data;
		if (src->key == SR_CONF_CONN)
			conn = g_variant_get_string(src->data, NULL);
	}
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		if (conn) {
			struct sr_usb_dev_inst *usb = NULL;
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i]) &&
				    usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				continue;
		}

		if (libusb_get_device_descriptor(devlist[i], &des) != 0 || !is_ch32(&des))
			continue;
		if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
			continue;

		devc = g_malloc0(sizeof(*devc));
		speed = libusb_get_device_speed(devlist[i]);
		/* the APP firmware tells the link speed by its PID */
		devc->usb3 = des.idProduct == CH32_PID_USB3 ||
			speed == LIBUSB_SPEED_SUPER || speed == LIBUSB_SPEED_SUPER_PLUS;
		devc->samplerate = DEFAULT_SAMPLERATE;
		devc->limit_samples = DEFAULT_LIMIT;
		devc->capture_ratio = 10;
		devc->vth = CH32_VTH_DEFAULT;
		devc->state = CH32_IDLE;

		sdi = g_malloc0(sizeof(*sdi));
		sdi->status = SR_ST_INACTIVE;
		sdi->vendor = g_strdup("WCH");
		sdi->model = g_strdup(devc->usb3 ? "CH32H417 (USB3)" : "CH32H417 (USB2)");
		sdi->connection_id = g_strdup(connection_id);
		sdi->inst_type = SR_INST_USB;
		sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i]), NULL);
		sdi->priv = devc;

		for (j = 0; j < CH32_NUM_LOGIC_CHANNELS; j++) {
			snprintf(name, sizeof(name), "D%d", j);
			sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE, name);
		}
		/* HSADC: used instead of the logic channels, one at a time */
		for (j = 0; j < CH32_NUM_ANALOG_CHANNELS; j++) {
			snprintf(name, sizeof(name), "A%d", j);
			sr_channel_new(sdi, CH32_NUM_LOGIC_CHANNELS + j,
				SR_CHANNEL_ANALOG, FALSE, name);
		}

		sr_dbg("Found %s at %s.", sdi->model, connection_id);
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di = sdi->driver;
	struct drv_context *drvc = di->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct dev_context *devc = sdi->priv;
	int ret;

	if (sr_usb_open(drvc->sr_ctx->libusb_ctx, usb) != SR_OK)
		return SR_ERR;

	if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1)
		libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE);

	if ((ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE)) != 0) {
		if (ret == LIBUSB_ERROR_BUSY)
			sr_err("Unable to claim the USB interface: another program "
				"or driver has it.");
		else
			sr_err("Unable to claim the USB interface: %s.",
				libusb_error_name(ret));
		sr_usb_close(usb);
		return SR_ERR;
	}

	/* also checks that the APP firmware answers */
	if (ch32_get_version(sdi) != SR_OK) {
		sr_err("No answer from the device; is the OpenSourceLogic-CH32H417 "
			"APP firmware running?");
		libusb_release_interface(usb->devhdl, USB_INTERFACE);
		sr_usb_close(usb);
		return SR_ERR;
	}
	if (!sdi->version)
		sdi->version = g_strdup_printf("sw %u hw %u", devc->sw_ver, devc->hw_ver);
	sr_info("Firmware version: software %u, hardware %u.", devc->sw_ver, devc->hw_ver);

	/* in case an earlier program left a capture running */
	ch32_cmd(sdi, CMD_SET_STOP, NULL, 0, NULL, 0);
	ch32_apply_vth(sdi);

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;

	if (!usb->devhdl)
		return SR_ERR_BUG;

	ch32_hw_stop(sdi);
	libusb_release_interface(usb->devhdl, USB_INTERFACE);
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
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		*data = std_gvar_tuple_double(devc->vth, devc->vth);
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
	uint64_t rates[MAX_SAMPLERATES], rate;
	size_t i, n;
	double low, high;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		rate = g_variant_get_uint64(data);
		n = ch32_samplerate_list(devc, rates, ARRAY_SIZE(rates));
		for (i = 0; i < n && rates[i] != rate; i++)
			;
		if (i == n) {
			sr_err("Sample rate %" PRIu64 " is not supported.", rate);
			return SR_ERR_SAMPLERATE;
		}
		devc->samplerate = rate;
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		if (devc->capture_ratio > 100)
			devc->capture_ratio = 100;
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		g_variant_get(data, "(dd)", &low, &high);
		devc->vth = (low + high) / 2.0;
		if (devc->vth < CH32_VTH_MIN)
			devc->vth = CH32_VTH_MIN;
		if (devc->vth > CH32_VTH_MAX)
			devc->vth = CH32_VTH_MAX;
		if (sdi->status == SR_ST_ACTIVE)
			return ch32_apply_vth(sdi);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t rates[MAX_SAMPLERATES];
	size_t n;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
		if (!sdi || !(devc = sdi->priv))
			return SR_ERR_ARG;
		n = ch32_samplerate_list(devc, rates, ARRAY_SIZE(rates));
		*data = std_gvar_samplerates(rates, n);
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		*data = std_gvar_min_max_step_thresholds(CH32_VTH_MIN, CH32_VTH_MAX,
			CH32_VTH_STEP);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	return ch32_acquisition_start(sdi);
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	return ch32_acquisition_stop(sdi);
}

static struct sr_dev_driver wch_ch32h417_driver_info = {
	.name = "wch-ch32h417",
	.longname = "WCH CH32H417 USB3 logic analyzer",
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
SR_REGISTER_DEV_DRIVER(wch_ch32h417_driver_info);
