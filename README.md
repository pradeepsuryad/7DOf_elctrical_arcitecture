# arm-core

Full electrical architecture for a 7-DoF robotic arm: high-power DC system, motor driver PCBs, power distribution & protection, analog sensor interfaces, real-time MCU firmware, and HIL validation.

Northeastern University.

## Stack & requirements

| Area | Tooling | Install |
|---|---|---|
| **Firmware** | C++17, CMake ≥ 3.16, Ninja, GoogleTest (via FetchContent) | `cmake -S firmware -B build/sim -DBUILD_TARGET=SIM` |
| **Schematics / PCB** | [KiCad](https://www.kicad.org) 8+ (`.kicad_pro` / `.kicad_sch`) | — |
| **SPICE** | ngspice (decks in `sim/spice/*.cir`) | `apt install ngspice` |
| **Analysis** | Python 3.11, `numpy>=1.24`, `matplotlib>=3.7` | `pip install -r sim/analysis/requirements.txt` |
| **HIL** | Python 3.11, `python-can>=4.3` + a USB-CAN adapter (CANable/slcan, PEAK PCAN, Kvaser) | `pip install -r hil/scripts/requirements.txt` |
| **Lint** | ruff (`check` + `format --check`) | `pip install ruff` |

The firmware builds two ways: `-DBUILD_TARGET=SIM` compiles natively (x86-64)
so the unit tests run anywhere, and `-DBUILD_TARGET=MCU` cross-compiles for the
target. CI only exercises SIM — the MCU ELF needs vendor linker scripts and
startup files that live with the BSP, not in this repo.

## Build & test

```bash
# firmware unit tests (native, with ASAN + UBSAN)
cmake -S firmware -B build/sim -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TARGET=SIM
cmake --build build/sim --parallel
ctest --test-dir build/sim --output-on-failure

# overcurrent-protection analytical model → sim/analysis/figures/*.png
cd sim/analysis && python overcurrent_analysis.py
```

## CI

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) runs three jobs on push
and PR: firmware unit tests in SIM mode (covering CAN frame packing, PID
direction and anti-windup, fault latch/clear, and soft position limits), the
SPICE analytical model as a smoke test, and a ruff lint/format gate over the
Python.

## Layout

```
hardware/              KiCad projects
  motor-driver/          DC servo motor driver PCB
  power-distribution/    Bus power, short-circuit / overcurrent protection
  sensor-interface/      Analog sensor signal-conditioning boards
  common/                Shared symbols, footprints, libraries

firmware/              Real-time C++ MCU firmware
  src/                   Application sources
  include/               Public headers
  drivers/               Peripheral & comms drivers (CAN / SPI / I2C)
  tests/                 Unit / on-target tests

sim/                   Pre-silicon validation
  spice/                 LTspice / ngspice decks
  analysis/              Python notebooks, scripts, plots

hil/                   Hardware-in-the-loop
  scripts/               Test runners, fixtures
  logs/                  Captured runs (csv, scope traces)

docs/                  Specs and references
  architecture/          Block diagrams, power tree, comms map
  ieee/                  Published IEEE papers and references

tools/                 Build helpers, flashing, codegen
```

## Communication buses

CAN — high-bandwidth motor command / telemetry bus
SPI — fast peripheral and inter-MCU links
I2C — low-rate config and slow sensors
