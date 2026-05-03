# arm-core

Full electrical architecture for a 7-DoF robotic arm: high-power DC system, motor driver PCBs, power distribution & protection, analog sensor interfaces, real-time MCU firmware, and HIL validation.

Northeastern University.

## Layout

```
hardware/              Altium projects
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
