#pragma once
#include "hal.hpp"
#include "pid.hpp"
#include "joint_state.hpp"

// ── JointController ───────────────────────────────────────────────────────────
//
// Manages one joint of the 7-DoF arm.
//
// Control architecture — three cascaded PID loops:
//
//   Setpoint.position
//       │
//       ▼
//   ┌──────────┐  vel_cmd  ┌──────────┐  torque_cmd  ┌──────────┐
//   │ Position │ ────────▶ │ Velocity │ ────────────▶ │  Torque  │ ──▶ CAN
//   │   PID    │           │   PID    │               │   PID    │
//   └──────────┘           └──────────┘               └──────────┘
//       ▲                      ▲                           ▲
//   measured pos           measured vel               measured torque
//
// Why cascade?
//   Each inner loop is much faster than the outer one:
//     torque  PID: 5–10 kHz   (innermost, fastest — limited by current sensing)
//     velocity PID: 1–2 kHz
//     position PID: 200–500 Hz (outermost, slowest)
//   This separation of timescales makes each loop easier to tune independently
//   and achieves tighter tracking than a single-loop approach.
//
// CAN frame packing (see can_driver.hpp for ID allocation):
//   CMD (0x100+id, 8 bytes):
//     [0..3] float32  position setpoint (rad)
//     [4..5] int16    velocity limit (rad/s × 100)
//     [6..7] int16    torque limit   (Nm  × 100)
//   FEEDBACK (0x300+id, 8 bytes, received from joint):
//     [0..1] int16    position  (rad   × 1000)
//     [2..3] int16    velocity  (rad/s × 100)
//     [4..5] int16    torque    (Nm    × 100)
//     [6]    uint8    fault_flags
//     [7]    uint8    temp (°C)
// ─────────────────────────────────────────────────────────────────────────────

namespace arm {

class JointController {
public:
    struct PidGainSet {
        control::Pid<float>::Gains position;
        control::Pid<float>::Gains velocity;
        control::Pid<float>::Gains torque;
    };

    JointController(const JointConfig& cfg,
                    const PidGainSet&  gains,
                    hal::ICanBus&      can_bus);

    // ── Called every control tick ─────────────────────────────────────────────

    // Update from a CAN feedback frame received from the joint motor driver.
    // Returns false if the frame ID does not belong to this joint.
    bool ingest_feedback(const hal::CanFrame& frame);

    // Run one control step, then transmit the CAN command frame.
    // dt_s: seconds since last call (must match the inner-loop rate).
    void tick(const JointSetpoint& setpoint, float dt_s);

    // ── Accessors ─────────────────────────────────────────────────────────────
    const JointState&  state()  const { return state_; }
    const JointConfig& config() const { return cfg_; }

    // Clears a latched fault (only if root cause is gone).
    void clear_fault();

private:
    hal::CanFrame pack_command(const JointSetpoint& sp,
                               float torque_cmd) const;

    void unpack_feedback(const hal::CanFrame& f);

    JointConfig cfg_;
    JointState  state_{};

    control::Pid<float> pid_pos_;
    control::Pid<float> pid_vel_;
    control::Pid<float> pid_tor_;

    hal::ICanBus& can_;
};

}  // namespace arm
