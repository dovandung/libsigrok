/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2010-2012 Bert Vermeulen <bert@biot.com>
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
 * Helper functions for the Cypress EZ-USB / FX2 series chips.
 */

#include <config.h>
#include <libusb.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "ezusb"

#define FW_CHUNKSIZE (4 * 1024)

#define GET_LSW(v)  ((unsigned short)((v) & 0xFFFF))    // Get Least Significant Word part of an integer.
#define GET_MSW(v)  ((unsigned short)((v) >> 16))       // Get Most Significant Word part of an integer.

SR_PRIV int ezusb_reset(struct libusb_device_handle *hdl, int set_clear)
{
	int ret;
	unsigned char buf[1];

	sr_info("setting CPU reset mode %s...",
		set_clear ? "on" : "off");
	buf[0] = set_clear ? 1 : 0;
	ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR, 0xa0,
				      0xe600, 0x0000, buf, 1, 100);
	if (ret < 0)
		sr_err("Unable to send control request: %s.",
				libusb_error_name(ret));

	return ret;
}

SR_PRIV int ezusb_install_firmware(struct sr_context *ctx,
				   libusb_device_handle *hdl,
				   const char *name)
{
	unsigned char *firmware;
	size_t length, offset, chunksize;
	int ret, result;

	/* Max size is 64 kiB since the value field of the setup packet,
	 * which holds the firmware offset, is only 16 bit wide.
	 */
	firmware = sr_resource_load(ctx, SR_RESOURCE_FIRMWARE,
			name, &length, 1 << 16);
	if (!firmware)
		return SR_ERR;

	sr_info("Uploading firmware '%s'.", name);

	result = SR_OK;
	offset = 0;
	while (offset < length) {
		chunksize = MIN(length - offset, FW_CHUNKSIZE);

		ret = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
					      LIBUSB_ENDPOINT_OUT, 0xa0, offset,
					      0x0000, firmware + offset,
					      chunksize, 100);
		if (ret < 0) {
			sr_err("Unable to send firmware to device: %s.",
					libusb_error_name(ret));
			g_free(firmware);
			return SR_ERR;
		}
		sr_info("Uploaded %zu bytes.", chunksize);
		offset += chunksize;
	}
	g_free(firmware);

	sr_info("Firmware upload done.");

	return result;
}

SR_PRIV int ezusb_upload_firmware(struct sr_context *ctx, libusb_device *dev,
				  int configuration, const char *name)
{
	struct libusb_device_handle *hdl;
	int ret;

	sr_info("uploading firmware to device on %d.%d",
		libusb_get_bus_number(dev), libusb_get_device_address(dev));

	if ((ret = libusb_open(dev, &hdl)) < 0) {
		sr_err("failed to open device: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

/*
 * The libusb Darwin backend is broken: it can report a kernel driver being
 * active, but detaching it always returns an error.
 */
#if !defined(__APPLE__)
	if (libusb_kernel_driver_active(hdl, 0) == 1) {
		if ((ret = libusb_detach_kernel_driver(hdl, 0)) < 0) {
			sr_err("failed to detach kernel driver: %s",
					libusb_error_name(ret));
			return SR_ERR;
		}
	}
#endif

	if ((ret = libusb_set_configuration(hdl, configuration)) < 0) {
		sr_err("Unable to set configuration: %s",
				libusb_error_name(ret));
		return SR_ERR;
	}

	if ((ezusb_reset(hdl, 1)) < 0)
		return SR_ERR;

	if (ezusb_install_firmware(ctx, hdl, name) < 0)
		return SR_ERR;

	if ((ezusb_reset(hdl, 0)) < 0)
		return SR_ERR;

	libusb_close(hdl);

	return SR_OK;
}

SR_PRIV int ezusb_fx3_ram_write(libusb_device_handle *hdl,
		unsigned char *buf, unsigned int ramAddress, int len)
{
	int r, size, index;

	index = 0;
	while (len > 0) {
		size = MIN(len, FW_CHUNKSIZE);

		r = libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
				LIBUSB_ENDPOINT_OUT, 0xa0,
				GET_LSW(ramAddress), GET_MSW(ramAddress),
				&buf[index], size, 100);
		if (r != size) {
			sr_err("Vendor write to FX3 RAM at 0x%08x failed: %s.",
				ramAddress, r < 0 ? libusb_error_name(r) :
				"short write");
			return SR_ERR;
		}
		ramAddress += size;
		index += size;
		len -= size;
	}

	return SR_OK;
}

/*
 * Download an FX3 boot image (.img, as produced by elf2img) to the FX3
 * bootloader's RAM and start it.
 *
 * Image layout, all words little-endian:
 *   'C' 'Y' bImageCTL bImageType(0xB0 = normal firmware)
 *   { dLength (in 32-bit words), dAddress, dLength words of data } ...
 *   dLength = 0, dAddress = program entry, checksum (sum of data words)
 */
SR_PRIV int ezusb_install_firmware_fx3(struct sr_context *ctx,
		libusb_device_handle *hdl, const char *name)
{
	unsigned char *firmware;
	size_t filesize, index, length, i;
	uint32_t address, checksum;
	int r, result;
	gboolean reported;

	/* Max size of an FX3 firmware image is 256 kiB (1 << 18). */
	firmware = sr_resource_load(ctx, SR_RESOURCE_FIRMWARE,
			name, &filesize, 1 << 18);
	if (!firmware)
		return SR_ERR;

	if (filesize < 4 || firmware[0] != 'C' || firmware[1] != 'Y' ||
			firmware[3] != 0xb0) {
		sr_err("'%s' is not an FX3 firmware image.", name);
		g_free(firmware);
		return SR_ERR;
	}

	sr_info("Uploading FX3 firmware '%s'.", name);

	/*
	 * Run through each section of code, and use vendor commands to
	 * download them to RAM. Every read is bounds checked against the
	 * file size, and words are read with RL32() (the image buffer has
	 * no alignment guarantee and the host may not be little-endian).
	 */
	result = SR_ERR;
	reported = FALSE;
	index = 4;
	checksum = 0;
	while (filesize - index >= 8) {
		length = RL32(firmware + index);
		address = RL32(firmware + index + 4);
		index += 8;

		if (length == 0) {
			/* Final section: checksum, then jump to entry point. */
			if (filesize - index < 4) {
				sr_err("FX3 firmware image is truncated.");
				reported = TRUE;
				break;
			}
			if (checksum != RL32(firmware + index)) {
				sr_err("Checksum error in FX3 firmware image.");
				reported = TRUE;
				break;
			}
			/*
			 * The device leaves the bootloader as soon as it
			 * gets this, so the transfer often reports an error.
			 */
			r = libusb_control_transfer(hdl,
					LIBUSB_REQUEST_TYPE_VENDOR |
					LIBUSB_ENDPOINT_OUT, 0xa0,
					GET_LSW(address), GET_MSW(address),
					NULL, 0, 100);
			if (r != 0)
				sr_dbg("Ignored error in FX3 start command: %s.",
					libusb_error_name(r));
			result = SR_OK;
			break;
		}

		if (length > (filesize - index) / 4) {
			sr_err("FX3 firmware image is truncated.");
			reported = TRUE;
			break;
		}
		for (i = 0; i < length; i++)
			checksum += RL32(firmware + index + 4 * i);
		if (ezusb_fx3_ram_write(hdl, firmware + index, address,
				(int)(length * 4)) != SR_OK) {
			sr_err("Failed to download data to FX3 RAM.");
			reported = TRUE;
			break;
		}
		index += length * 4;
	}
	if (result != SR_OK && !reported)
		sr_err("FX3 firmware image has no entry-point section.");
	g_free(firmware);

	if (result == SR_OK)
		sr_info("FX3 firmware upload done.");
	return result;
}

SR_PRIV int ezusb_upload_firmware_fx3(struct sr_context *ctx,
		libusb_device *dev, int configuration, const char *name)
{
	struct libusb_device_handle *hdl;
	int ret;

	sr_info("uploading firmware to FX3 device on %d.%d",
		libusb_get_bus_number(dev), libusb_get_device_address(dev));

	if ((ret = libusb_open(dev, &hdl)) < 0) {
		sr_err("failed to open FX3 device: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

/*
 * The libusb Darwin backend is broken: it can report a kernel driver being
 * active, but detaching it always returns an error.
 */
#if !defined(__APPLE__)
	if (libusb_kernel_driver_active(hdl, 0) == 1) {
		if ((ret = libusb_detach_kernel_driver(hdl, 0)) < 0) {
			sr_err("failed to detach kernel driver: %s",
					libusb_error_name(ret));
			libusb_close(hdl);
			return SR_ERR;
		}
	}
#endif

	if ((ret = libusb_set_configuration(hdl, configuration)) < 0) {
		sr_err("Unable to set configuration: %s",
				libusb_error_name(ret));
		libusb_close(hdl);
		return SR_ERR;
	}

	ret = ezusb_install_firmware_fx3(ctx, hdl, name);
	libusb_close(hdl);

	return ret == SR_OK ? SR_OK : SR_ERR;
}
