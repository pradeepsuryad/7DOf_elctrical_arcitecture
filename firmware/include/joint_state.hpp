#pragma once
#include <cstdint>

// ── Joint data types ──────────────────────────────────────────────────────────
//
// SI units throughout:
//   position  → radians
//   velocity  → rad/s
//   torque    → Nm
//   current   → A
//   temp      → °C
// ─────────────────────────────────────────────────────────────────────────────

namespace arm {

// Bitmask of faults that a joint motor driver can report back over CAN.
// Each bit maps to a byte in the 0x300+id feedback frame (byte 6: fault_flags).
enum class JointFault : uint8_t {
    NONE            = 0x00,
    OVERCURRENT     = 0x01,   // phase current exceeded limit
    OVERTEMPERATURE = 0x02,   // motor or FET junction temp too high
    ENCODER_ERROR   = 0x04,   // encoder checksum or signal fault
    UNDERVOLTAGE    = 0x08,   // bus voltage dropped below threshold
    OVERVOLTAGE     = 0x10,   // regenerative braking spike not clamped
    COMM_TIMEOUT    = 0x20,   // MCU stopped sending commands (watchdog)
};

inline JointFault operator|(JointFault a, JointFault b) {
    return static_cast<JointFault>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
inline bool has_fault(JointFault flags, JointFault bit) {
    return (static_cast<uint8_t>(flags) & static_cast<uint8_t>(bit)) != 0;
}

// What the motor driver is currently doing.
enum class JointMode : uint8_t {
    IDLE      = 0,   // no current, holding position passively (or free)
    POSITION  = 1,   // outer PID: position → velocity setpoint
    VELOCITY  = 2,   // middle PID: velocity → torque/current setpoint
    TORQUE    = 3,   // inner loop: direct torque command (current control)
    FAULT     = 4,   // latched fault — must be cleared before re-enabling
};

// Snapshot of one joint at one control tick.
struct JointState {
    float position {0.0f};   // rad, from absolute encoder
    float velocity {0.0f};   // rad/s, differentiated or from observer
    float torque   {0.0f};   // Nm, estimated from phase current
    float current  {0.0f};   // A, measured phase current (RMS or peak)
    float temp_c   {0.0f};   // °C, motor winding or FET junction

    JointFault faults{JointFault::NONE};
    JointMode  mode  {JointMode::IDLE};

    uint32_t timestamp_us{0};   // when this state was captured
};

// Setpoint sent from the arm controller down to a joint controller each tick.
struct JointSetpoint {
    float    position{0.0f};        // rad — used in POSITION mode
    float    velocity{0.0f};        // rad/s — used in VELOCITY mode
    float    torque  {0.0f};        // Nm — used in TORQUE mode
    float    vel_limit  {6.28f};    // rad/s — safety ceiling regardless of mode
    float    torque_limit{5.0f};    // Nm — safety ceiling
    JointMode mode   {JointMode::IDLE};
};

// Fixed physical limits per joint — populated from a config file at startup.
struct JointConfig {
    uint8_t  id{0};                     // 0–6
    float    pos_min{-3.14159f};        // rad, soft limit
    float    pos_max{ 3.14159f};        // rad, soft limit
    float    vel_max{6.28f};            // rad/s, absolute limit
    float    torque_max{10.0f};         // Nm, absolute limit
    float    gear_ratio{1.0f};          // output/motor (e.g. 50 for 50:1 gearbox)
    float    encoder_offset{0.0f};      // rad, zero-position calibration
    bool     direction_inverted{false}; // flip if wiring reverses positive direction
};

}  // namespace arm
