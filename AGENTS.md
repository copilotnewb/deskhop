# AGENTS.md

## Repository map

This repository contains the DeskHop firmware, host-side helper scripts, hardware design assets, and web configuration tooling.

### Top-level build and project files
- `CMakeLists.txt` - Main firmware build definition. Declares firmware version, builds `deskhop` from `src/`, links bundled pico/tinyusb + Pico-PIO-USB, and generates `.uf2` output.
- `README.md` - Project overview, feature documentation, build instructions, flashing/usage guidance, and hardware background.
- `LICENSE` - Project license.
- `.gitignore` - Git ignore rules.

### Firmware source (`src/`)
Core RP2040 firmware (switching logic, USB host/device flow, protocol, config handling):
- `main.c` - Firmware entry point and top-level initialization.
- `setup.c` - Runtime setup/initialization routines.
- `tasks.c` - Core scheduled/background task execution.
- `usb.c` - USB orchestration glue for host/device behavior.
- `usb_descriptors.c` - USB descriptor definitions.
- `keyboard.c` - Keyboard input handling and hotkey/switch behavior.
- `mouse.c` - Mouse processing, including movement/switch logic.
- `hid_parser.c` / `hid_report.c` - HID parsing and report creation/translation utilities.
- `handlers.c` - Event/packet handler implementations.
- `protocol.c` - Inter-board or control protocol handling.
- `uart.c` - UART communication between boards.
- `led.c` - LED state/indication control.
- `ramdisk.c` - Virtual/config drive support used by firmware.
- `defaults.c` / `constants.c` / `utils.c` - Shared defaults, constants, and utility helpers.

Header/interface files for the firmware live in `src/include/` (configuration, structs, protocol, pinout, packet definitions, etc.).

### Web configuration tooling (`webconfig/`)
Generates the packed HTML config UI embedded/served by firmware:
- `render.py` - Build script that renders/minifies output.
- `form.py` - Form/schema helpers used in rendering.
- `templates/` - Source templates for HTML/CSS/JS and packing.
- `config-unpacked.htm` / `config.htm` - Generated config-page artifacts.
- `Makefile`, `requirements.txt` - Tooling and dependency entry points.

### Disk image assets (`disk/`)
- `disk.img` - FAT image payload embedded into firmware.
- `disk.S` - Assembly wrapper for embedding disk image section.
- `create.sh` - Script for rebuilding the disk image.

### Host-side helper scripts
- `deskhop_switch_ctypes.py` - CLI utility for sending DeskHop switch/control HID commands from a PC.
- `kvm_gaze_switch.py` - Automatic switching helper driven by gaze/head tracking.

### Hardware and manufacturing assets
- `schematics/` - PDF schematics by board revision.
- `pcb/` - KiCad projects and Gerbers for v1.0/v1.1.
- `case/` - 3D-printable enclosure models (`.stl`, CAD source files).
- `img/` - README/media assets.

### Tooling and misc (`misc/`)
- `memory_map.ld` - Linker script.
- `crc32.py` - Post-build checksum/metadata helper.
- `docker.yml` / `Dockerfile` - Containerized build environment.
- `.clang-format`, `.editorconfig`, `.markdownlint.yaml` - Formatting/lint config.
- `user-manual.pdf` - End-user documentation.

### Bundled dependencies (vendored)
- `pico-sdk/` - Bundled Raspberry Pi Pico SDK + TinyUSB sources used for reproducible builds.
- `Pico-PIO-USB/` - Bundled PIO USB implementation used by firmware.

## Practical editing guidance for agents
- Prefer changing project-owned code/docs (`src/`, `webconfig/`, scripts, docs) before touching vendored dependencies.
- Treat `pico-sdk/` and `Pico-PIO-USB/` as third-party code unless a task explicitly requires vendor patching.
- If firmware behavior changes, check both build wiring in `CMakeLists.txt` and matching headers in `src/include/`.
