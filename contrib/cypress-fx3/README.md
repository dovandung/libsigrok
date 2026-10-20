# Cypress FX3 logic analyzer drivers

Two libsigrok drivers for a Cypress FX3 (CYUSB301x) based logic analyzer:

| Driver (`-d`)    | Source                         | Firmware file       | Sample width |
|------------------|--------------------------------|---------------------|--------------|
| `cypress-fx3`    | `src/hardware/cypress-fx3-16/` | `cypress-fx3.fw`    | 16 bit       |
| `cypress-fx3-8`  | `src/hardware/cypress-fx3-8/`  | `cypress-fx3-8.fw`  | 8 bit        |

Both are enabled by default; `./configure --disable-cypress-fx3-16` or
`--disable-cypress-fx3-8` turns one off.

## Firmware

The firmware images are not in this repository. Put `cypress-fx3.fw`
and/or `cypress-fx3-8.fw` where libsigrok looks for firmware, for example
`<prefix>/share/sigrok-firmware/`, or next to `pulseview.exe` in a
`share/sigrok-firmware` folder on Windows (set `SIGROK_FIRMWARE_DIR` to
override).

The files must be FX3 boot images (`.img` from `elf2img`, starting with
`CY` and image type `0xB0`). `ezusb_upload_firmware_fx3()` checks the
header, the section lengths and the checksum before it starts the
firmware, and refuses a damaged file.

After the upload the device re-enumerates. The driver re-scans the bus
(up to 4 uploads per scan) and then finds the device by its strings
`sigrok` / `cypress-fx3`.

## Known limits

- **Both drivers match the same device.** They use the same VID:PID
  (`04b4:1234`) and the same post-firmware strings (`sigrok` /
  `cypress-fx3`). A scan of all drivers (for example `sigrok-cli --scan`)
  lists the device twice, and a device already running the 16-bit
  firmware is accepted by the 8-bit driver (and the other way round).
  Always pick the driver explicitly (`-d cypress-fx3` or
  `-d cypress-fx3-8`, or the driver drop-down in PulseView). The real fix
  is a different USB product string (or PID) in the 8-bit firmware, then
  matching on it in `cypress-fx3-8/api.c`.
- Windows: the device needs the WinUSB driver (Zadig) both before and
  after the firmware upload, because the PID stays the same.
