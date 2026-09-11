# DeskHop Audio Milestone 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build stock DeskHop with keyboard/mouse/hub support plus the proven Arctis Nova 7 USB Audio Host test on the same GP14/GP15 host port.

**Architecture:** Keep DeskHop's native USB device/HID behavior unchanged. Modern TinyUSB provides Audio Host; the proven Pico-PIO-USB fork provides isochronous host transport. The existing USB-A port remains the only host port and is expected to connect through a USB 2.0 hub. Milestone 1 intentionally excludes PC-facing Audio Device descriptors and inter-Pico PCM.

**Tech Stack:** RP2040, Pico SDK 1.5.1, TinyUSB pinned `ad54f72e10859596044918b76ed5a73402ca545b`, Pico-PIO-USB pinned `4db0782fe2c621cff5a4f405d02b591ca562007c`, CMake, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-09-11-deskhop-audio-m1-design.md`

## Global Constraints

- Work only on branch `deskhop-audio`.
- Preserve stock DeskHop PC-facing descriptors and HID behavior in milestone 1.
- Use GP14/GP15 as the existing PIO USB host port; no new connector/GPIO pair.
- Preserve hub support and HID host support.
- Use `PIO_USB_EP_SIZE=384`.
- Do not add UART PCM transport in milestone 1.
- Do not add PC-facing USB Audio Device interfaces in milestone 1.

---

### Task 1: Pin proven USB dependencies and keep stock DeskHop building

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `.github/workflows/build.yml`
- Add/update dependency metadata as required for reproducible checkout.

**Interfaces:**
- Consumes: existing `tinyusb_device`, `tinyusb_host`, and `Pico-PIO-USB` CMake targets.
- Produces: same target names, backed by the pinned modern TinyUSB and proven Pico-PIO-USB implementation.

- [ ] Record baseline branch SHA and dependency SHAs.
- [ ] Change CI/dependency wiring to use TinyUSB `ad54f72e10859596044918b76ed5a73402ca545b` and Pico-PIO-USB `4db0782fe2c621cff5a4f405d02b591ca562007c`.
- [ ] Define `PIO_USB_EP_SIZE=384` for DeskHop's Pico-PIO-USB compilation.
- [ ] Make GitHub Actions run for `deskhop-audio`.
- [ ] Run the stock DeskHop build in Actions.
- [ ] Expected result: `deskhop.uf2` builds before any audio application code is added.
- [ ] Commit dependency checkpoint.

### Task 2: Add failing Audio Host build configuration

**Files:**
- Modify: `src/include/tusb_config.h`
- Modify: `CMakeLists.txt`
- Modify: `.github/workflows/build.yml`

**Interfaces:**
- Produces TinyUSB Audio Host configuration: UAC1/UAC2, one audio device, hub + HID still enabled.

- [ ] Enable `CFG_TUH_AUDIO=1`, `CFG_TUH_AUDIO_PROTOCOLS=(TUH_AUDIO_PROTOCOL_UAC1 | TUH_AUDIO_PROTOCOL_UAC2)`, `CFG_TUH_AUDIO_MAX=1` and stream buffers sized for the Nova 7.
- [ ] Add the upstream TinyUSB `examples/host/audio_host/src/audio_app.c` to the build without yet adding DeskHop wrappers/tasks.
- [ ] Build in Actions and verify it fails for missing application integration symbols/tasks rather than dependency absence.
- [ ] Commit the red checkpoint only if useful for debugging; otherwise retain as workflow evidence.

### Task 3: Integrate Audio Host application into DeskHop core1

**Files:**
- Create: `src/audio_host.c`
- Create: `src/include/audio_host.h`
- Modify: `src/main.c`
- Modify: `src/include/main.h`
- Modify: `CMakeLists.txt`

**Interfaces:**
- `void audio_host_task(device_t *state)` calls upstream `audio_app_task()`.
- `void audio_host_defer_task(device_t *state)` calls upstream `defer_queue_task()`.
- TinyUSB Audio Host callbacks remain owned by upstream `audio_app.c` for milestone 1.

- [ ] Add compile-level test/build expecting `audio_host_task` and `audio_host_defer_task` to exist.
- [ ] Verify failure before implementation.
- [ ] Add the two wrappers with no UART or device-audio logic.
- [ ] Schedule both tasks on core1 directly after `usb_host_task()`.
- [ ] Do not call TinyUSB's example LED heartbeat.
- [ ] Build and verify green.
- [ ] Commit audio-host task integration.

### Task 4: Verify hub + HID + Arctis coexistence artifact

**Files:**
- Modify: `.github/workflows/build.yml` only if artifact/debug build changes are needed.

**Interfaces:**
- Produces `deskhop.uf2` suitable for hardware testing with USB hub + keyboard + mouse + Arctis Nova 7.

- [ ] Run final CI from the exact feature-branch head.
- [ ] Confirm release build has no periodic audio diagnostic printf dependency.
- [ ] Confirm HID host, HID device, MSC/config mode, hub, and Audio Host classes all compile together.
- [ ] Publish `deskhop.uf2` artifact.
- [ ] Record the exact commit SHA used for hardware testing.

## Deferred milestones

- Milestone 2: PC-facing 48 kHz / 16-bit stereo speaker + mono microphone USB Audio Device on both DeskHop Picos.
- Milestone 3: DMA-backed inter-Pico PCM framing and routing according to `active_output`.
- Milestone 4: FIFO/feedback clock-domain handling, switching flush/silence behavior, recovery and polish.
