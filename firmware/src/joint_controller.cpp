#include "joint_controller.hpp"
#include <cstring>
#include <algorithm>
#include <cmath>

namespace arm {

// Helper: copy 4 bytes of a float into a byte buffer at offset (little-endian).
static void pack_f32(uint8_t* buf, int offset, float val) {
    std::memcpy(buf + offset, &val, 4);
}

// Helper: pack a signed int16 (value already scaled by caller).
static void pack_i16(uint8_t* buf, int offset, int16_t val) {
    buf[offset]   = static_cast<uint8_t>(val & 0xFF);
    buf[offset+1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

// Helper: unpack signed int16 from byte buffer at offset.
static int16_t unpack_i16(const uint8_t* buf, int offset) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(buf[offset]) |
        (static_cast<uint16_t>(buf[offset+1]) << 8));
}

// ── Constructor ───────────────────────────────────────────────────────────────

JointController::JointController(const JointConfig& cfg,
                                 const PidGainSet&  gains,
                                 hal::ICanBus&      can_bus)
    : cfg_(cfg)
    , pid_pos_(gains.position)
    , pid_vel_(gains.velocity)
    , pid_tor_(gains.torque)
    , can_(can_bus)
{
    state_.mode = JointMode::IDLE;
}

// ── ingest_feedback ───────────────────────────────────────────────────────────

bool JointController::ingest_feedback(const hal::CanFrame& frame) {
    // Feedback frames have IDs in the 0x300..0x306 range.
    if (frame.id != static_cast<uint32_t>(0x300 + cfg_.id)) return false;
    if (frame.dlc < 8) return false;

    unpack_feedback(frame);
    return true;
}

void JointController::unpack_feedback(const hal::CanFrame& f) {
    // Fixed-point decoding — reverse of the motor driver's encoding:
    //   position:  int16 = actual_rad * 1000
    //   velocity:  int16 = actual_rad_s * 100
    //   torque:    int16 = actual_Nm  * 100
    state_.position = unpack_i16(f.data, 0) / 1000.0f;
    state_.velocity = unpack_i16(f.data, 2) / 100.0f;
    state_.torque   = unpack_i16(f.data, 4) / 100.0f;
    state_.faults   = static_cast<JointFault>(f.data[6]);
    state_.temp_c   = static_cast<float>(f.data[7]);

    // Apply calibration offset and direction.
    state_.position -= cfg_.encoder_offset;
    if (cfg_.direction_inverted) {
        state_.position = -state_.position;
        state_.velocity = -state_.velocity;
        state_.torque   = -state_.torque;
    }
}

// ── tick ──────────────────────────────────────────────────────────────────────

void JointController::tick(const JointSetpoint& sp, float dt_s) {
    // If faulted, send an idle command and do nothing else.
    if (state_.faults != JointFault::NONE) {
        state_.mode = JointMode::FAULT;
        JointSetpoint idle{};
        auto frame = pack_command(idle, 0.0f);
        can_.transmit(frame);
        return;
    }

    state_.mode = sp.mode;
    float torque_cmd = 0.0f;

    switch (sp.mode) {
    case JointMode::POSITION: {
        // Outer loop: position error → velocity command
        float vel_cmd = pid_pos_.update(sp.position, state_.position, dt_s);
        vel_cmd = std::clamp(vel_cmd, -sp.vel_limit, sp.vel_limit);
        // Middle loop: velocity error → torque command
        torque_cmd = pid_vel_.update(vel_cmd, state_.velocity, dt_s);
        torque_cmd = std::clamp(torque_cmd, -sp.torque_limit, sp.torque_limit);
        break;
    }
    case JointMode::VELOCITY: {
        // Skip position loop, drive velocity directly.
        torque_cmd = pid_vel_.update(sp.velocity, state_.velocity, dt_s);
        torque_cmd = std::clamp(torque_cmd, -sp.torque_limit, sp.torque_limit);
        break;
    }
    case JointMode::TORQUE:
        // Direct torque command — bypass all PID loops.
        torque_cmd = std::clamp(sp.torque, -sp.torque_limit, sp.torque_limit);
        break;
    case JointMode::IDLE:
    case JointMode::FAULT:
    default:
        pid_pos_.reset();
        pid_vel_.reset();
        pid_tor_.reset();
        torque_cmd = 0.0f;
        break;
    }

    // Apply absolute joint limits.
    torque_cmd = std::clamp(torque_cmd, -cfg_.torque_max, cfg_.torque_max);

    // Soft position limits: if outside range, override with a restoring torque.
    if (state_.position < cfg_.pos_min)
        torque_cmd = std::fabs(torque_cmd);   // force positive (away from limit)
    else if (state_.position > cfg_.pos_max)
        torque_cmd = -std::fabs(torque_cmd);  // force negative

    auto frame = pack_command(sp, torque_cmd);
    can_.transmit(frame);
}

// ── pack_command ──────────────────────────────────────────────────────────────

hal::CanFrame JointController::pack_command(const JointSetpoint& sp,
                                            float torque_cmd) const {
    hal::CanFrame f{};
    f.id  = 0x100 + cfg_.id;
    f.dlc = 8;
    f.is_extended = false;

    pack_f32(f.data, 0, sp.position);
    pack_i16(f.data, 4, static_cast<int16_t>(sp.vel_limit   * 100.0f));
    pack_i16(f.data, 6, static_cast<int16_t>(torque_cmd     * 100.0f));
    return f;
}

// ── clear_fault ───────────────────────────────────────────────────────────────

void JointController::clear_fault() {
    if (state_.faults == JointFault::NONE) {
        state_.mode = JointMode::IDLE;
        pid_pos_.reset();
        pid_vel_.reset();
        pid_tor_.reset();
    }
    // If faults are still non-zero, refuse to clear — root cause must be fixed.
}

}  // namespace arm
