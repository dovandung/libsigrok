# wch-ch32h417

libsigrok driver for the CH32H417 USB 3.0 logic analyzer
([oshwhub q2h2 project](https://oshwhub.com/q2h2/project_bszkxrnf)):
16 logic channels up to 200 MS/s and a 2 channel 10-bit HSADC up to 20 MS/s.

Ported from the ALL LOGIC (libsigrok4DSL) `ch32` driver, `fast-trigger`
branch. Needs the [OpenSourceLogic-CH32H417](https://github.com/Doukeyi-X/OpenSourceLogic-CH32H417)
IAP + APP firmware (the original U3LogicAnalyzer firmware speaks a
different protocol). The APP enumerates as `1a86:5537` (USB 2) or
`1a86:5538` (USB 3). On Windows, install WinUSB for it with Zadig.

## Features

- Sample rates: USB 3 — the 24 firmware rates from 200 MS/s down to
  6.25 MS/s, plus 5 MS/s ... 100 kS/s made on the PC. USB 2 — the firmware
  runs at a fixed 40 MS/s (channels D0-D7 only) or 20 MS/s (any channel
  above D7), lower rates are made on the PC. Rates made on the PC use a
  per-channel majority vote over the decimation window, not 1-of-N picking.
- Only D0-D7 enabled: 1 byte per sample (half the USB bandwidth).
- Trigger on the PC: one stage of 0 / 1 / rising / falling / either edge
  matches. The stream is scanned 8 (8-bit) or 4 (16-bit) samples per step
  on 64-bit words, well above the device's 200 MS/s. The pre-trigger part
  (capture ratio) is kept in a ring buffer; from the trigger on, samples go
  to the session as they arrive. A trigger only counts once the
  pre-trigger part is complete.
- Continuous acquisition: sample limit 0.
- Voltage threshold 0.45 ... 3.40 V, through the measured DAC calibration
  table of the ALL LOGIC driver.
- HSADC: enable A0 or A1 (one at a time) and disable all logic channels;
  samples are sent in volts (0 ... 3.3 V). Rate 40 MHz / (div + 1), the
  nearest to the selected sample rate, 1 ... 20 MS/s.

## Not ported

- DSView's 1T glitch filter (no libsigrok config key for it).
- Logic and analog in the same acquisition (the firmware streams one or
  the other).

## Examples

    sigrok-cli -d wch-ch32h417 --scan
    sigrok-cli -d wch-ch32h417 -c samplerate=100M --samples 10M \
        -t D0=r -w -o capture.sr
    sigrok-cli -d wch-ch32h417 -C D0,D1,D2,D3 -c samplerate=40M \
        -c captureratio=20 -t D2=f --samples 1M -o capture.sr
    sigrok-cli -d wch-ch32h417 -C A0 -c samplerate=10M --samples 100k \
        -O analog
