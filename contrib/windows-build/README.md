# Windows build notes (MXE, libusb 1.0.30, SLogic)

Notes for building PulseView / sigrok-cli for Windows with this tree, using
sigrok-util's `cross-compile/mingw/sigrok-cross-mingw` on Linux with MXE.

## What changed and why

| Area | File | Change |
|---|---|---|
| Build | `configure.ac`, `Makefile.am` | The SLogic driver is now registered, so it is actually compiled. |
| Build | `src/std.c` | Adds `std_i32_idx()`, which the SLogic driver needs (from sipeed/libsigrok slogic-dev). |
| USB events | `src/usb.c`, `src/session.c`, `src/libsigrok-internal.h` | Works with **stock libusb** on Windows (see below). |
| SLogic driver | `slogic/slogic_data.c` | Faster 2/4-channel unpack (SSE2/NEON) and trigger scan; fixes a level trigger that never fired. |
| SLogic driver | `slogic/slogic_usb.c` | `%zu` logging fix for MinGW; 1 MiB transfer cap with libusb older than 1.0.30. |
| Soft trigger | `src/soft-trigger.c` | Fast single-stage trigger scanner (30-80x); edges on buffer boundaries and multi-stage triggers handled as upstream. |
| MXE | `mxe-libusb1-1.0.30.patch` | libusb 1.0.27 -> 1.0.30 for MXE, keeping sigrok's RAW_IO policy. |
| PulseView | `pulseview-slogic-channels.patch` | Channel-count option for SLogic; sample-rate list follows enabled channels. |

### Stock libusb on Windows

Upstream libsigrok's USB event source needs `libusb_get_pollfds()`, which
stock libusb does not provide on Windows. That is why sigrok's Windows
builds used Uwe Hermann's old libusb fork (1.0.20-rc3 "event-abstraction").
With stock libusb, `usb_source_add()` failed and drivers such as fx2lafw,
DSLogic, Saleae, Kingst and Hantek never received data.

`src/usb.c` now services libusb from the session main loop when no poll
handles are available. Linux and macOS keep the poll-handle path. Drivers
written for the "idle USB" libsigrok variant, which pass no callback to
`usb_source_add()`, also work.

### Why libusb 1.0.30

1.0.30 adds a real RAW_IO API for WinUSB (`libusb_endpoint_set_raw_io()`,
`libusb_get_max_raw_io_transfer_size()`). With older libusb, WinUSB rejects
RAW_IO reads larger than its maximum (2 MiB on the test machine), so SLogic
captures larger than about 2 MB received nothing. The SLogic driver now
sizes its transfers under the reported maximum.

## How to build

1. **MXE**, once:

       cd ~/mxe-git
       git checkout src/libusb1.mk && rm -f src/libusb1-1-fixes.patch   # undo sigrok's old libusb patch
       patch -p1 < .../contrib/windows-build/mxe-libusb1-1.0.30.patch
       make update-checksum-libusb1
       make MXE_TARGETS=i686-w64-mingw32.static.posix libusb1

   Keep sigrok-util's `mxe_fixes.patch`. Do not apply its
   `libusb1_upgrade.patch`; this patch replaces it.

2. **sigrok-cross-mingw**: clone this branch instead of upstream libsigrok,
   and patch PulseView after its clone:

       $GIT_CLONE -b <this branch> https://github.com/dovandung/libsigrok.git libsigrok
       ...
       $GIT_CLONE $REPO_BASE/pulseview
       cd pulseview
       patch -p1 < .../contrib/windows-build/pulseview-slogic-channels.patch || exit 1

3. Run `sigrok-cross-mingw` as usual. The build is static, so libsigrok and
   PulseView must be rebuilt whenever MXE's libusb changes.

## Run-time switches

Set these in a command prompt before starting `pulseview.exe` or
`sigrok-cli`. They are for testing only; the defaults are correct.

| Variable | Effect |
|---|---|
| `SIGROK_USB_WAIT_MS=0` | USB event source uses a 5 ms timer instead of waiting in libusb (default: wait up to 10 ms). |
| `LIBUSB_SIGROK_RAWIO=0` | Turns off sigrok's per-read RAW_IO policy in libusb (drivers that set RAW_IO themselves, like SLogic, are unaffected). |

Compile-time: `SOFT_TRIG_WIN64_SIMD` enables AVX2/AVX-512 in the soft
trigger for 64-bit Windows GCC builds. It is off by default because of GCC
bug 54412: 256-bit stack spills assume 32-byte alignment, which Win64 does
not guarantee.

## Test results (Windows, MXE i686, libusb 1.0.30)

| Device | Capture | Result |
|---|---|---|
| fx2lafw | 20 M samples @ 24 MHz | ok, no data loss |
| SLogic Combo 8 | 20 M samples, 8 ch @ 40 MHz | `end=ok`, 40.0 of 40.0 MB/s |
| SLogic16U3 (USB 3 5 Gbps) | 100 M samples, 16 ch @ 200 MHz | `end=ok`, 400.0 of 400.0 MB/s, `rearms=0` |

The log line `RAW_IO enabled on ep 0x82, max_raw_io=2097152` confirms
RAW_IO; it is printed when the device is opened (log level 4 or higher).

The fx2lafw test ran in a tree with the "idle USB" drivers. This tree has
upstream's drivers, which use the same event source code; that path was
tested with a GLib main loop harness, not on hardware.

## Known limits

- **PulseView at 400 MB/s.** It consumes about 360 MB/s on the 32-bit
  build, so the driver's queue grows (20 MB peak in the 100 M sample test).
  Fixed-length captures are fine. Long or continuous captures at
  400 MB/s eventually run out of memory on 32-bit; use a 64-bit build or a
  lower data rate.
- **RAW_IO policy and USB 3 packet size.** sigrok's RAW_IO policy enables
  RAW_IO for reads that divide by 512. A USB 3 device whose reads divide by
  512 but not by 1024 would fail with it on. None of the drivers tested
  does that, and `LIBUSB_SIGROK_RAWIO=0` turns the policy off.
- **Default 32-bit builds.** GCC does not assume SSE2 for i686, so the
  SLogic unpack uses its portable loops unless built with `-msse2`.
