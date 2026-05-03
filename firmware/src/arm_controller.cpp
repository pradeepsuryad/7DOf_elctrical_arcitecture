#include "arm_controller.hpp"
#include <algorithm>

namespace arm {

// ── Constructor ───────────────────────────────────────────────────────────────
// std::array doesn't support in-place construction via index, so we use an
// immediately-invoked lambda to initialise joints_ from the two parameter arrays.

ArmController::ArmController(
    const std::array<JointConfig,                  NUM_JOINTS>& configs,
    const std::array<JointController::PidGainSet,  NUM_JOINTS>& gains,
    hal::ICanBus&   can_bus,
    hal::ISysClock& clock)
    : joints_{[&]() {
          std::array<JointController, NUM_JOINTS> arr{
              JointController(configs[0], gains[0], can_bus),
              JointController(configs[1], gains[1], can_bus),
              JointController(configs[2], gains[2], can_bus),
              JointController(configs[3], gains[3], can_bus),
              JointController(configs[4], gains[4], can_bus),
              JointController(configs[5], gains[5], can_bus),
              JointController(configs[6], gains[6], can_bus),
          };
          return arr;
      }()}
    , clock_(clock)
{
    last_tick_us_ = clock_.micros();
}

// ── ingest_can_frame ──────────────────────────────────────────────────────────
// Cheap fan-out: each JointController checks if the frame belongs to it.
// For 7 joints this is 7 comparisons — negligible in an ISR.
// If traffic grows, replace with a lookup table keyed on (frame.id >> 4).

void ArmController::ingest_can_frame(const hal::CanFrame& frame) {
    for (auto& j : joints_) {
        if (j.ingest_feedback(frame)) return;  // matched — done
    }
    // Unknown ID — could log/count here for diagnostics.
}

// ── tick ──────────────────────────────────────────────────────────────────────

bool ArmController::tick(const ArmSetpoint& sp) {
    const uint32_t now_us = clock_.micros();

    // Real elapsed time — handles wrap-around for uint32_t timestamps.
    const uint32_t elapsed_us = now_us - last_tick_us_;
    dt_s_ = static_cast<float>(elapsed_us) * 1e-6f;
    last_tick_us_ = now_us;

    // Watchdog: if we somehow skip more than WATCHDOG_US since last tick, latch.
    if (elapsed_us > WATCHDOG_US) {
        watchdog_expired_ = true;
        estop_active_     = true;
    }

    if (estop_active_) {
        // Override all setpoints with IDLE — JointController will send zero torque.
        ArmSetpoint idle_sp{};
        for (uint8_t i = 0; i < NUM_JOINTS; ++i)
            joints_[i].tick(idle_sp.joints[i], dt_s_);
        return false;
    }

    // Normal path: dispatch per-joint setpoints.
    for (uint8_t i = 0; i < NUM_JOINTS; ++i)
        joints_[i].tick(sp.joints[i], dt_s_);

    return true;
}

// ── Safety ────────────────────────────────────────────────────────────────────

void ArmController::estop() {
    estop_active_ = true;
}

void ArmController::release_estop() {
    // Only release if no joint is currently in FAULT state.
    bool any_fault = false;
    for (const auto& j : joints_) {
        if (j.state().faults != JointFault::NONE) { any_fault = true; break; }
    }
    if (!any_fault) {
        estop_active_     = false;
        watchdog_expired_ = false;
    }
}

void ArmController::clear_joint_fault(uint8_t joint_id) {
    if (joint_id < NUM_JOINTS)
        joints_[joint_id].clear_fault();
}

// ── State snapshot ────────────────────────────────────────────────────────────

ArmState ArmController::state() const {
    ArmState s;
    s.estop_active     = estop_active_;
    s.watchdog_expired = watchdog_expired_;
    for (uint8_t i = 0; i < NUM_JOINTS; ++i)
        s.joints[i] = joints_[i].state();
    return s;
}

}  // namespace arm
