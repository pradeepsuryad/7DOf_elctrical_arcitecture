# Power Tree — 7-DoF Robotic Arm

## Overview

Two independent power domains share a common ground but are isolated at their
sources.  Logic rails are derived from the motor bus via a non-isolated DC-DC;
the motor bus itself can be killed by the E-stop MOSFET without affecting the
MCU.

```
  ┌───────────────────────────────────────────────────────────────────────────┐
  │  INPUT SOURCE                                                             │
  │                                                                           │
  │  48 V Li-Ion pack   ─── or ───   48 V bench PSU (lab / HIL)              │
  │  (10S, 36–54 V range)            (current-limited to 20 A for safety)    │
  └───────────────────┬───────────────────────────────────────────────────────┘
                      │  48 V (nominal), 10 A continuous, 25 A peak
                      │
              ┌───────▼────────┐
              │  Fuse  5 × 20  │  32 A ceramic fast-blow (Littelfuse 0218032)
              │  + TVS diode   │  P6KE58A (58 V clamp, catches regen spikes)
              └───────┬────────┘
                      │
              ┌───────▼────────┐
              │  E-stop MOSFET │  IRF2907 (75 V, 209 A, Rds_on 4.5 mΩ)
              │  gate driven   │  Gate held HIGH by pull-up; MCU or
              │  by MCU GPIO   │  hardware button pulls LOW to kill bus.
              └───────┬────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
  ┌─────▼──┐    ┌─────▼──┐   ┌─────▼──┐       ... × 7 joints
  │ Motor  │    │ Motor  │   │ Motor  │
  │Driver 0│    │Driver 1│   │Driver 2│
  │(J0)    │    │(J1)    │   │(J2)    │
  └────────┘    └────────┘   └────────┘
        │
  ┌─────▼────────────────────────────────────────────────────────────────────┐
  │  Per-joint motor driver stage (× 7, one board per joint)                │
  │                                                                          │
  │  48 V bus ─── DRV8353 gate driver ─── 3-phase half-bridge (6 × FETs)   │
  │                │                                                         │
  │                ├── 100 nF + 10 µF ceramic decoupling at each FET source  │
  │                ├── 0.005 Ω shunt resistor (phase current sense)          │
  │                └── bootstrap capacitor for high-side gate drive          │
  │                                                                          │
  │  DRV8353 internal LDO → 3.3 V (gate bias, SPI comms to MCU)             │
  │  Per-board protection: OCP (hardware, ~50 A), OVP, UVLO, OTW            │
  └──────────────────────────────────────────────────────────────────────────┘
                      │
              ┌───────▼────────┐
              │  Buck converter │  48 V → 5 V, 3 A
              │  LMR36520       │  non-isolated, synchronous
              │  η ≈ 93 % typ  │
              └───────┬────────┘
                      │  5 V rail (logic supply, CAN transceivers, encoders)
                      │
              ┌───────▼────────┐
              │  LDO           │  5 V → 3.3 V, 800 mA
              │  TLV1117-33    │  for MCU core, SPI sensors, I2C devices
              └───────┬────────┘
                      │  3.3 V rail
                      │
           ┌──────────┴──────────┐
           │                     │
      ┌────▼────┐          ┌─────▼─────┐
      │  MCU    │          │  Sensors  │
      │ STM32   │          │ IMU, Enc, │
      │ (Cortex │          │ Temp ICs  │
      │  -M4F)  │          └───────────┘
      └─────────┘
```

## Voltage Rails Summary

| Rail   | Nominal | Range      | Source        | Max current | Protected by          |
|--------|---------|------------|---------------|-------------|-----------------------|
| VBUS   | 48 V    | 36–54 V    | Battery / PSU | 10 A cont.  | Fuse + TVS + E-stop   |
| VMOT   | 48 V    | 36–54 V    | VBUS post FET | 25 A peak   | DRV8353 OCP / UVLO    |
| 5V     | 5.0 V   | 4.85–5.15V | LMR36520 buck | 3 A         | Buck OCP, UVLO        |
| 3V3    | 3.3 V   | 3.2–3.4 V  | TLV1117 LDO   | 800 mA      | LDO current limit     |

## Protection Hierarchy

```
Fault event           First response         Backup
─────────────────────────────────────────────────────────────────
Phase overcurrent     DRV8353 OCP latches    Fuse blows (32 A)
Bus overvoltage       TVS clamps at 58 V     Fuse blows
Regen voltage spike   TVS absorbs energy     TVS + fuse
Thermal overload      DRV8353 OTW → MCU      DRV8353 shutdown
E-stop signal         E-stop FET opens bus   Independent of MCU
MCU software fault    Watchdog → IDLE cmds   E-stop by operator
```

## Connector / Fusing Per Joint

Each motor-driver PCB has:
- **Xt60 or Anderson SB50** for the 48 V bus input (rated 60 A+)
- **8-pin Molex Micro-Fit** for encoder (SPI), CAN, 5 V encoder supply, GND
- **3-phase wires** (14 AWG minimum) to motor winding

Wire gauge (48 V, 10 A per joint):
- Bus runs: 12 AWG (up to 25 A sustained)
- Per-joint spur: 16 AWG (up to 13 A, adequate for 10 A with 3 A margin)

## Power Budget (full-load estimate)

| Consumer           | Qty | Each (W) | Total (W) |
|--------------------|-----|----------|-----------|
| DC servo motor     | 7   | 40       | 280       |
| Motor driver PCB   | 7   | 1.5      | 10.5      |
| MCU + logic        | 1   | 1.0      | 1.0       |
| Sensors            | 7   | 0.2      | 1.4       |
| CAN transceivers   | 1   | 0.1      | 0.1       |
| **Total**          |     |          | **293 W** |

At 48 V nominal: 293 W / 48 V ≈ **6.1 A** average draw; peak with all joints
stalled simultaneously (not realistic): 25 A × 7 × 48 V = 8.4 kW — but each
joint's DRV8353 OCP trips at ~50 A phase current ≈ 24 V × 50 A / √2 ≈ 850 W
per joint before fuse.  Real worst-case: 3–4 joints stalled at once → fuse
holds at 32 A (1.5 kW).
