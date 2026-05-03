#pragma once
#include "joint_controller.hpp"
#include <array>
#include <cstdint>

// ── ArmController ─────────────────────────────────────────────────────────────
//
// Top-level orchestrator for the full 7-DoF arm.
//
// Responsibilities:
//  1. Hold one JointController per joint (joints 0–6).
//  2. Route incoming CAN feedback frames to the correct joint.
//  3. Accept a full-arm setpoint and dispatch per-joint setpoints.
//  4. Enforce global safety rules:
//       - E-stop: zeros all torques immediately.
//       - Watchdog: if tick() is not called within WATCHDOG_US, go to IDLE.
//  5. Expose arm-level state for telemetry.
//
// Timing expectations:
//   tick() is called at 1 kHz from a hardware timer ISR or RTOS task.
//   ingest_can_frame() is called from the CAN RX ISR (interrupt context —
//   keep it short and lock-free, or copy to a ring buffer and call from tick).
// ─────────────────────────────────────────────────────────────────────────────

namespace arm {

static constexpr uint8_t NUM_JOINTS = 7;

// Full-arm setpoint passed into tick() every control cycle.
struct ArmSetpoint {
    std::array<JointSetpoint, NUM_JOINTS> joints{};
};

// Snapshot of all joint states for telemetry or HIL logging.
struct ArmState {
    std::array<JointState, NUM_JOINTS> joints{};
    bool estop_active{false};
    bool watchdog_expired{false};
};

class ArmController {
public:
    // Per-joint config and gains arrays must have exactly NUM_JOINTS elements.
    ArmController(
        const std::array<JointConfig,                    NUM_JOINTS>& configs,
        const std::array<JointController::PidGainSet,   NUM_JOINTS>& gains,
        hal::ICanBus&  can_bus,
        hal::ISysClock& clock);

    // ── Called from CAN RX ISR or a ring-buffer consumer ─────────────────────
    void ingest_can_frame(const hal::CanFrame& frame);

    // ── Called at 1 kHz from timer ISR / RTOS task ────────────────────────────
    // Returns false if e-stop or watchdog is active (caller should log/alert).
    bool tick(const ArmSetpoint& sp);

    // ── Safety controls ───────────────────────────────────────────────────────
    void estop();               // latch e-stop (requires explicit release)
    void release_estop();       // clear e-stop if all joints fault-free
    void clear_joint_fault(uint8_t joint_id);

    // ── State access ──────────────────────────────────────────────────────────
    ArmState state() const;

private:
    static constexpr uint32_t WATCHDOG_US = 5000;  // 5 ms — 5 missed ticks at 1 kHz

    std::array<JointController, NUM_JOINTS> joints_;
    hal::ISysClock& clock_;

    bool     estop_active_    {false};
    uint32_t last_tick_us_    {0};
    bool     watchdog_expired_{false};
    float    dt_s_            {0.001f};  // updated every tick from real elapsed time
};

}  // namespace arm
