# DeskHop Audio Milestone 1 Design

## Goal

Prove that stock DeskHop keyboard/mouse switching can coexist with USB Audio Host on the existing USB-A host port through a USB 2.0 hub, with no hardware modification and no inter-Pico audio transport yet.

## Pinned dependencies

- DeskHop feature branch: `deskhop-audio`.
- Pico-PIO-USB: `copilotnewb/Pico-PIO-USB` commit `4db0782fe2c621cff5a4f405d02b591ca562007c` from branch `test/audio-host-no-periodic-printf`.
- Pico SDK remains the existing DeskHop SDK 1.5.1 tree.
- TinyUSB is replaced with pinned commit `ad54f72e10859596044918b76ed5a73402ca545b`, matching the modern Audio Host API used by the proven standalone test.

## Architecture

DeskHop continues to use the RP2040 native USB controller as a TinyUSB device toward its PC and Pico-PIO-USB/TinyUSB rhport 1 as the USB host on GP14/GP15. A normal USB 2.0 hub is connected to the existing DeskHop USB-A port. Keyboard, mouse, and the Arctis Nova 7 dongle all share that host bus.

Milestone 1 does not expose a USB Audio device to either PC and does not send PCM over the inter-Pico UART. Instead, it embeds TinyUSB's proven Audio Host example behavior into DeskHop as a diagnostic coexistence test: the Arctis should mount behind the hub, cycle mic-only / 1 kHz speaker tone / mic echo, while normal DeskHop HID handling continues to run.

## Host behavior

- Preserve `CFG_TUH_HUB=1` and HID host support.
- Enable TinyUSB Audio Host for UAC1/UAC2 with one Audio device.
- Build Pico-PIO-USB with `PIO_USB_EP_SIZE=384` so the host remains compatible with the previously tested endpoint sizes.
- Keep `CFG_TUH_ENUMERATION_BUFSIZE=512`; the Nova 7 configuration descriptor is 312 bytes.
- Use the current TinyUSB upstream `examples/host/audio_host/src/audio_app.c` for stream discovery/configuration and test-tone/echo behavior rather than reimplementing UAC parsing.
- Do not call the upstream audio example's LED heartbeat, because DeskHop already owns the LED.
- Keep stdout disabled in release builds; audio code must not require periodic printf calls to function.

## Device behavior

The PC-facing DeskHop USB device remains unchanged in milestone 1: HID/MSC/config behavior and descriptors must remain byte-for-byte functionally equivalent. USB Audio Device descriptors are deferred to milestone 2.

## Task integration

`usb_host_task()` remains the highest-frequency TinyUSB host service. Two lightweight application tasks are added on core 1 immediately after it:

- `audio_host_task()` -> calls TinyUSB Audio Host example `audio_app_task()`.
- `audio_host_defer_task()` -> calls `defer_queue_task()` so delayed stream phase/restart callbacks execute.

These tasks must not replace or delay `packet_receiver_task()` or other DeskHop tasks.

## Dependency layout

Keep DeskHop's existing paths so CMake changes stay small:

- `Pico-PIO-USB/` becomes a pinned git submodule pointing to the proven fork commit.
- `pico-sdk/lib/tinyusb/` becomes a pinned nested git submodule pointing to the chosen TinyUSB commit.

GitHub Actions must use recursive submodule checkout and run on `deskhop-audio` while development is active.

## Success criteria

1. CI builds stock DeskHop firmware with Pico SDK 1.5.1, modern TinyUSB, and the proven Pico-PIO-USB revision.
2. Existing HID host/device code compiles without behavior changes.
3. The firmware enumerates keyboard/mouse through a USB 2.0 hub as before.
4. An Arctis Nova 7 connected to the same hub mounts as a TinyUSB Audio Host device and produces the same 1 kHz tone / mic echo behavior already proven by the standalone Pico-PIO-USB test.
5. No UART PCM or PC-facing Audio Device work is included in this milestone.
