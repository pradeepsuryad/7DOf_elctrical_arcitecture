"""
CAN protocol encoding / decoding for the 7-DoF arm.

Byte layout mirrors firmware/src/joint_controller.cpp exactly.
Any change here must also be made there (and vice versa).

CMD frame   (host → joint)  ID = 0x100 + joint_id
  [0..3]  float32  position setpoint  (rad, little-endian IEEE 754)
  [4..5]  int16    velocity limit     (rad/s × 100)
  [6..7]  int16    torque command     (Nm  × 100)

FEEDBACK frame  (joint → host)  ID = 0x300 + joint_id
  [0..1]  int16   position   (rad   × 1000)
  [2..3]  int16   velocity   (rad/s × 100)
  [4..5]  int16   torque     (Nm    × 100)
  [6]     uint8   fault_flags
  [7]     uint8   temperature (°C)
"""

import struct
from dataclasses import dataclass, field
from enum import IntFlag
from typing import Optional


# ── ID allocation ─────────────────────────────────────────────────────────────

NUM_JOINTS = 7

def cmd_id(joint_id: int) -> int:
    return 0x100 + joint_id

def feedback_id(joint_id: int) -> int:
    return 0x300 + joint_id

def joint_id_from_feedback(can_id: int) -> Optional[int]:
    """Return joint index if `can_id` is a feedback frame, else None."""
    if 0x300 <= can_id <= 0x300 + NUM_JOINTS - 1:
        return can_id - 0x300
    return None


# ── Fault flags (matches JointFault enum in joint_state.hpp) ──────────────────

class JointFault(IntFlag):
    NONE            = 0x00
    OVERCURRENT     = 0x01
    OVERTEMPERATURE = 0x02
    ENCODER_ERROR   = 0x04
    UNDERVOLTAGE    = 0x08
    OVERVOLTAGE     = 0x10
    COMM_TIMEOUT    = 0x20


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class JointCommand:
    joint_id:    int
    position:    float          # rad
    vel_limit:   float = 6.28   # rad/s
    torque_cmd:  float = 0.0    # Nm


@dataclass
class JointFeedback:
    joint_id:    int
    position:    float          # rad
    velocity:    float          # rad/s
    torque:      float          # Nm
    faults:      JointFault = JointFault.NONE
    temp_c:      float = 0.0
    timestamp_s: float = 0.0    # filled in by runner from host clock


# ── Encoding ──────────────────────────────────────────────────────────────────

def encode_command(cmd: JointCommand) -> bytes:
    """
    Pack a JointCommand into an 8-byte CAN payload.

    struct layout (little-endian):
      offset 0: float32  position
      offset 4: int16    vel_limit  × 100
      offset 6: int16    torque_cmd × 100
    """
    pos_bytes    = struct.pack('<f', cmd.position)
    vel_i16      = int(round(cmd.vel_limit  * 100.0))
    torque_i16   = int(round(cmd.torque_cmd * 100.0))

    # Clamp to int16 range to avoid silent truncation.
    vel_i16    = max(-32768, min(32767, vel_i16))
    torque_i16 = max(-32768, min(32767, torque_i16))

    return pos_bytes + struct.pack('<hh', vel_i16, torque_i16)


def decode_feedback(can_id: int, payload: bytes, timestamp_s: float = 0.0) -> Optional[JointFeedback]:
    """
    Unpack an 8-byte feedback payload into a JointFeedback.
    Returns None if can_id is not a feedback frame or payload is short.
    """
    jid = joint_id_from_feedback(can_id)
    if jid is None or len(payload) < 8:
        return None

    pos_raw, vel_raw, torque_raw = struct.unpack_from('<hhh', payload, 0)
    fault_byte = payload[6]
    temp_byte  = payload[7]

    return JointFeedback(
        joint_id    = jid,
        position    = pos_raw    / 1000.0,
        velocity    = vel_raw    / 100.0,
        torque      = torque_raw / 100.0,
        faults      = JointFault(fault_byte),
        temp_c      = float(temp_byte),
        timestamp_s = timestamp_s,
    )


# ── Helpers for logging ───────────────────────────────────────────────────────

FEEDBACK_CSV_HEADER = (
    "timestamp_s,joint_id,"
    "position_rad,velocity_rads,torque_nm,"
    "fault_flags,temp_c"
)

def feedback_to_csv_row(fb: JointFeedback) -> str:
    return (
        f"{fb.timestamp_s:.6f},{fb.joint_id},"
        f"{fb.position:.6f},{fb.velocity:.4f},{fb.torque:.4f},"
        f"{int(fb.faults)},{fb.temp_c:.1f}"
    )
