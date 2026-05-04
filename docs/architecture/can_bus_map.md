# CAN Bus Map — 7-DoF Arm

## Physical layer

| Parameter       | Value                                  |
|-----------------|----------------------------------------|
| Standard        | CAN 2.0B (standard 11-bit IDs)         |
| Bit-rate        | 1 Mbit/s                               |
| Termination     | 120 Ω at each bus end                  |
| Transceiver     | TJA1050 or SN65HVD230 (3.3 V)          |
| Cable           | 22 AWG twisted-pair, < 4 m total       |
| Max stub length | 0.3 m per node (critical at 1 Mbit/s)  |

## Node list

| Node     | ID range        | Role              |
|----------|-----------------|-------------------|
| MCU      | transmits only  | Command source    |
| Joint 0  | 0x100 / 0x300   | Shoulder roll     |
| Joint 1  | 0x101 / 0x301   | Shoulder pitch    |
| Joint 2  | 0x102 / 0x302   | Elbow             |
| Joint 3  | 0x103 / 0x303   | Wrist pitch       |
| Joint 4  | 0x104 / 0x304   | Wrist roll        |
| Joint 5  | 0x105 / 0x305   | Wrist yaw         |
| Joint 6  | 0x106 / 0x306   | End-effector      |

## Message ID allocation

### Command frames (MCU → joint)  `0x100 + joint_id`

Sent every 1 ms (1 kHz) for every joint.

```
Byte  Width   Field             Units / scaling
────  ─────   ─────────────────────────────────
0–3   float32  position setpoint  rad  (IEEE 754 little-endian)
4–5   int16    velocity limit     rad/s × 100   range ±327.67 rad/s
6–7   int16    torque command     Nm   × 100    range ±327.67 Nm
```

### Feedback frames (joint → MCU)  `0x300 + joint_id`

One joint replies per tick (staggered — see timing budget below).
Effective feedback rate per joint: **143 Hz** (1000 Hz ÷ 7 joints).

```
Byte  Width   Field             Units / scaling
────  ─────   ─────────────────────────────────
0–1   int16    position         rad   × 1000   range ±32.767 rad
2–3   int16    velocity         rad/s × 100    range ±327.67 rad/s
4–5   int16    torque           Nm    × 100    range ±327.67 Nm
6     uint8    fault_flags      bitmask (see JointFault enum)
7     uint8    temperature      °C,  0–255
```

Fault flag bitmask (byte 6):

| Bit | Value | Meaning              |
|-----|-------|----------------------|
| 0   | 0x01  | Overcurrent          |
| 1   | 0x02  | Overtemperature      |
| 2   | 0x04  | Encoder error        |
| 3   | 0x08  | Undervoltage         |
| 4   | 0x10  | Overvoltage          |
| 5   | 0x20  | Comm timeout (watchdog) |

### Diagnostic broadcast  `0x7DF`

Sent by MCU every 1 s.  Any node can reply with its firmware version and
health summary.  Not time-critical; no slot reserved in the timing budget.

## Timing budget

### Why staggered feedback?

A CAN 2.0B frame with 8 data bytes is at minimum 111 bits.
With worst-case bit stuffing (+20%) that is **~133 bits per frame**.

```
At 500 kbit/s:   133 bits ÷ 500 000 bit/s = 266 µs / frame
At 1 Mbit/s:     133 bits ÷ 1 000 000 bit/s = 133 µs / frame
```

If all 7 joints sent feedback every tick (7 cmd + 7 feedback = 14 frames):

```
14 frames × 133 µs = 1 862 µs  >  1 000 µs tick period  ✗  (overloaded)
```

Staggered scheme: only **one joint's feedback** arrives per tick.
Joint `j` replies during tick `t` when `t mod 7 == j`.

```
7 cmd + 1 feedback = 8 frames × 133 µs = 1 064 µs  >  1 000 µs  ✗
```

Still tight. Solution: reduce command frame interval from 1 ms to 800 µs
**or** use 1 Mbit/s and keep the minimum-stuffing case:

```
8 frames × 111 µs (min, no stuffing) = 888 µs  <  1 000 µs  ✓
```

In practice, average stuffing is ~10 %, giving ~122 µs/frame:

```
8 frames × 122 µs = 976 µs  →  ~98% bus utilisation  (tight but feasible)
```

**Chosen operating point:**

| Parameter              | Value                         |
|------------------------|-------------------------------|
| Bit-rate               | 1 Mbit/s                      |
| Tick period            | 1 ms                          |
| Commands per tick      | 7 (all joints, every tick)    |
| Feedback per tick      | 1 (staggered, round-robin)    |
| Feedback rate per joint| ~143 Hz                       |
| Worst-case utilisation | ~98 %                         |
| Budget for diagnostics | < 2 % (1 frame per second)    |

### Tick-by-tick schedule

```
Tick t    Commands sent       Feedback expected
────────  ──────────────────  ────────────────────
t mod 7 = 0   J0..J6 cmd     J0 feedback (0x300)
t mod 7 = 1   J0..J6 cmd     J1 feedback (0x301)
t mod 7 = 2   J0..J6 cmd     J2 feedback (0x302)
t mod 7 = 3   J0..J6 cmd     J3 feedback (0x303)
t mod 7 = 4   J0..J6 cmd     J4 feedback (0x304)
t mod 7 = 5   J0..J6 cmd     J5 feedback (0x305)
t mod 7 = 6   J0..J6 cmd     J6 feedback (0x306)
```

The MCU checks for the expected feedback frame within the tick.
If absent for 5 consecutive ticks (35 ms), the COMM_TIMEOUT fault is raised.

## Fixed-point encoding notes

All int16 values use the same sign convention as standard two's complement.
The MCU always decodes: `float_value = int16_value / scale_factor`.

| Field    | Scale | Min representable | Max representable |
|----------|-------|-------------------|-------------------|
| position | 1000  | −32.767 rad       | +32.767 rad       |
| velocity | 100   | −327.67 rad/s     | +327.67 rad/s     |
| torque   | 100   | −327.67 Nm        | +327.67 Nm        |

Position scale (×1000) gives 1 mrad resolution ≈ 0.057° — adequate for a 14-bit
encoder (AS5047P 0.022° resolution) since quantisation error is dominated by
the encoder, not the CAN encoding.
