---
applyTo: "**"
---

# Project Instructions for GitHub Copilot

## Project Overview
- Firmware for an ESP32-based laser tag "tagger".
- Built and managed using PlatformIO with configuration in `platformio.ini`.

## Build & Upload
- Always run builds from the project root directory.
- Preferred build command for this workspace (used by Copilot): `/Users/urs/.platformio/penv/bin/platformio run -d /Users/urs/development/arduino/projects/smt32/tagger`.
- Alternative: `make` (runs `./update_git_hash.sh` then `platformio run`).
- Clean build: `make clean` (invokes `platformio run --target clean`).
- Upload firmware: `make upload` (invokes `platformio run -t upload`).

## Tools
- Treat `platformio run` as the canonical way to build.
- If `platformio` is not found in PATH, suggest the user install PlatformIO (https://platformio.org/install) or use their existing IDE integration to build.

## Code Style
- Follow existing formatting and patterns in the `src/` directory.
- Prefer splitting classes into `.h`/`.cpp` pairs, mirroring `motors.h`/`motors.cpp`.
- Avoid adding new dependencies unless strictly necessary; rely on PlatformIO-managed libraries in `platformio.ini`.

## Testing
- There are no dedicated unit tests; validation is primarily via successful build and flashing to hardware.
- When changing firmware behavior, suggest the user test on real hardware.
