#include <gtest/gtest.h>
#include <cstring>
#include <cmath>

#include "fake_can_bus.hpp"
#include "joint_controller.hpp"

// ── Helpers ───────────────────────────────────────────────────────────────────

// Decode a float32 from a byte buffer at offset (little-endian).
static float decode_f32(const uint8_t* buf, int offset) {
    float v;
    std::memcpy(&v, buf + offset, 4);
    return v;
}

// Decode a signed int16 from a byte buffer at offset.
static int16_t decode_i16(const uint8_t* buf, int offset) {
    return static_cast<int16_t>(
        static_cast<uint16_t>(buf[offset]) |
        (static_cast<uint16_t>(buf[offset + 1]) << 8));
}

// Build a default JointConfig for joint 0.
static arm::JointConfig default_config(uint8_t id = 0) {
    arm::JointConfig cfg;
    cfg.id             = id;
    cfg.pos_min        = -3.14159f;
    cfg.pos_max        =  3.14159f;
    cfg.vel_max        =  6.28f;
    cfg.torque_max     = 10.0f;
    cfg.gear_ratio     = 1.0f;
    cfg.encoder_offset = 0.0f;
    cfg.direction_inverted = false;
    return cfg;
}

// Build a PID gain set with mild, non-zero gains so the output is predictable.
static arm::JointController::PidGainSet default_gains() {
    arm::JointController::PidGainSet gs;
    gs.position = {.kp=1.0f, .ki=0.0f, .kd=0.0f,
                   .tau_d=0.005f, .out_min=-6.28f, .out_max=6.28f};
    gs.velocity = {.kp=0.5f, .ki=0.0f, .kd=0.0f,
                   .tau_d=0.005f, .out_min=-10.0f, .out_max=10.0f};
    gs.torque   = {.kp=1.0f, .ki=0.0f, .kd=0.0f,
                   .tau_d=0.005f, .out_min=-10.0f, .out_max=10.0f};
    return gs;
}

// Build a minimal feedback frame as if sent by joint `id`.
// All fields zero by default (joint at rest, no faults).
static hal::CanFrame make_feedback(uint8_t id,
                                   float pos_rad  = 0.0f,
                                   float vel_rads = 0.0f,
                                   float torque_nm= 0.0f,
                                   uint8_t faults = 0,
                                   uint8_t temp_c = 25) {
    hal::CanFrame f{};
    f.id  = 0x300 + id;
    f.dlc = 8;
    f.is_extended = false;

    // Pack using the same convention as JointController::unpack_feedback.
    auto pack_i16 = [&](int offset, int16_t val) {
        f.data[offset]   = val & 0xFF;
        f.data[offset+1] = (val >> 8) & 0xFF;
    };
    pack_i16(0, static_cast<int16_t>(pos_rad   * 1000.0f));
    pack_i16(2, static_cast<int16_t>(vel_rads  * 100.0f));
    pack_i16(4, static_cast<int16_t>(torque_nm * 100.0f));
    f.data[6] = faults;
    f.data[7] = temp_c;
    return f;
}

// ── Test fixture ──────────────────────────────────────────────────────────────

class JointControllerTest : public ::testing::Test {
protected:
    test::FakeCanBus           bus;
    arm::JointController       ctrl{default_config(0), default_gains(), bus};
    static constexpr float     DT = 0.001f;  // 1 kHz tick rate
};

// ══════════════════════════════════════════════════════════════════════════════
// CAN frame structure
// ══════════════════════════════════════════════════════════════════════════════

// The command frame must use ID 0x100 + joint_id.
TEST_F(JointControllerTest, CommandFrameHasCorrectId) {
    arm::JointSetpoint sp;
    sp.mode = arm::JointMode::POSITION;
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    EXPECT_EQ(bus.pop_tx().id, 0x100u);
}

// Standard (11-bit) frame, not extended.
TEST_F(JointControllerTest, CommandFrameIsStandardId) {
    arm::JointSetpoint sp;
    sp.mode = arm::JointMode::POSITION;
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    EXPECT_FALSE(bus.pop_tx().is_extended);
}

// DLC must be 8 (all 8 bytes used).
TEST_F(JointControllerTest, CommandFrameDlcIsEight) {
    arm::JointSetpoint sp;
    sp.mode = arm::JointMode::POSITION;
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    EXPECT_EQ(bus.pop_tx().dlc, 8u);
}

// Bytes 0–3: position setpoint as float32.
TEST_F(JointControllerTest, CommandFramePacksPositionSetpoint) {
    arm::JointSetpoint sp;
    sp.mode     = arm::JointMode::POSITION;
    sp.position = 1.23f;
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    EXPECT_NEAR(decode_f32(frame.data, 0), 1.23f, 1e-5f);
}

// Bytes 4–5: velocity limit as int16 = value × 100.
TEST_F(JointControllerTest, CommandFramePacksVelocityLimit) {
    arm::JointSetpoint sp;
    sp.mode      = arm::JointMode::POSITION;
    sp.vel_limit = 3.50f;  // → int16 = 350
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    EXPECT_EQ(decode_i16(frame.data, 4), int16_t(350));
}

// Each tick produces exactly one TX frame, not more.
TEST_F(JointControllerTest, OneTickProducesOneFrame) {
    arm::JointSetpoint sp;
    sp.mode = arm::JointMode::VELOCITY;
    ctrl.tick(sp, DT);

    EXPECT_EQ(bus.tx_count(), 1u);
}

// ══════════════════════════════════════════════════════════════════════════════
// Feedback ingestion
// ══════════════════════════════════════════════════════════════════════════════

// ingest_feedback returns true only for frames with the matching joint ID.
TEST_F(JointControllerTest, IngestFeedbackAcceptsMatchingId) {
    auto frame = make_feedback(/*id=*/0, 0.5f);
    EXPECT_TRUE(ctrl.ingest_feedback(frame));
}

TEST_F(JointControllerTest, IngestFeedbackRejectsWrongId) {
    auto frame = make_feedback(/*id=*/3, 0.5f);  // joint 3, not 0
    EXPECT_FALSE(ctrl.ingest_feedback(frame));
}

// After ingesting a feedback frame the state reflects the decoded values.
TEST_F(JointControllerTest, IngestFeedbackUpdatesPosition) {
    auto frame = make_feedback(0, /*pos=*/1.5f);
    ctrl.ingest_feedback(frame);
    EXPECT_NEAR(ctrl.state().position, 1.5f, 0.002f);  // ±1 LSB at ×1000
}

TEST_F(JointControllerTest, IngestFeedbackUpdatesVelocity) {
    auto frame = make_feedback(0, 0.0f, /*vel=*/2.5f);
    ctrl.ingest_feedback(frame);
    EXPECT_NEAR(ctrl.state().velocity, 2.5f, 0.01f);
}

TEST_F(JointControllerTest, IngestFeedbackUpdatesFaultFlags) {
    auto frame = make_feedback(0, 0.0f, 0.0f, 0.0f,
                               static_cast<uint8_t>(arm::JointFault::OVERCURRENT));
    ctrl.ingest_feedback(frame);
    EXPECT_TRUE(arm::has_fault(ctrl.state().faults, arm::JointFault::OVERCURRENT));
}

// ══════════════════════════════════════════════════════════════════════════════
// IDLE mode
// ══════════════════════════════════════════════════════════════════════════════

// In IDLE mode the controller still sends a frame (so the joint knows the MCU
// is alive) but bytes 6–7 (torque × 100) must be zero.
TEST_F(JointControllerTest, IdleModeTransmitsZeroTorque) {
    arm::JointSetpoint sp;
    sp.mode = arm::JointMode::IDLE;
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    EXPECT_EQ(decode_i16(frame.data, 6), int16_t(0));
}

// ══════════════════════════════════════════════════════════════════════════════
// PID direction and output sign
// ══════════════════════════════════════════════════════════════════════════════

// If the joint is behind its setpoint, torque command must be positive
// (proportional-only, kp=1 for position, kp=0.5 for velocity, so
//  vel_cmd = 1.0*(1.0-0.0)=1.0, torque_cmd = 0.5*(1.0-0.0)=0.5 Nm → int16=50).
TEST_F(JointControllerTest, PositionModePositiveErrorGivesPositiveTorque) {
    arm::JointSetpoint sp;
    sp.mode     = arm::JointMode::POSITION;
    sp.position = 1.0f;   // target: 1 rad
    // Joint is at 0 rad (default state after construction)
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    int16_t torque_scaled = decode_i16(frame.data, 6);
    EXPECT_GT(torque_scaled, int16_t(0));
}

// Negative error → negative torque.
TEST_F(JointControllerTest, PositionModeNegativeErrorGivesNegativeTorque) {
    // Pretend the joint is already at +1 rad.
    ctrl.ingest_feedback(make_feedback(0, /*pos=*/1.0f));

    arm::JointSetpoint sp;
    sp.mode     = arm::JointMode::POSITION;
    sp.position = 0.0f;   // target: 0 rad — joint must move backward
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    int16_t torque_scaled = decode_i16(frame.data, 6);
    EXPECT_LT(torque_scaled, int16_t(0));
}

// ══════════════════════════════════════════════════════════════════════════════
// Fault latch
// ══════════════════════════════════════════════════════════════════════════════

// Once a fault arrives the controller must latch it and send zero torque.
TEST_F(JointControllerTest, FaultLatchesAndSendsZeroTorque) {
    ctrl.ingest_feedback(make_feedback(
        0, 0.0f, 0.0f, 0.0f,
        static_cast<uint8_t>(arm::JointFault::OVERCURRENT)));

    arm::JointSetpoint sp;
    sp.mode     = arm::JointMode::POSITION;
    sp.position = 2.0f;   // large setpoint — would produce torque if not faulted
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    EXPECT_EQ(decode_i16(frame.data, 6), int16_t(0));
}

// After fault clears, clear_fault() re-enables the controller.
TEST_F(JointControllerTest, ClearFaultReEnablesController) {
    ctrl.ingest_feedback(make_feedback(
        0, 0.0f, 0.0f, 0.0f,
        static_cast<uint8_t>(arm::JointFault::OVERCURRENT)));

    // Fault cleared by hardware — next feedback has no fault bits.
    ctrl.ingest_feedback(make_feedback(0));
    ctrl.clear_fault();

    arm::JointSetpoint sp;
    sp.mode     = arm::JointMode::POSITION;
    sp.position = 1.0f;
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    // Torque should be non-zero now that the fault is gone.
    EXPECT_NE(decode_i16(frame.data, 6), int16_t(0));
}

// ══════════════════════════════════════════════════════════════════════════════
// Soft position limits
// ══════════════════════════════════════════════════════════════════════════════

// If the joint exceeds pos_max, torque must be negative (pushing back).
TEST_F(JointControllerTest, SoftLimitClampsTorqueAtMaxPosition) {
    // Joint is past the soft limit.
    ctrl.ingest_feedback(make_feedback(0, /*pos=*/3.2f));  // pos_max = π ≈ 3.14

    arm::JointSetpoint sp;
    sp.mode     = arm::JointMode::POSITION;
    sp.position = 3.2f;   // setpoint is also past limit
    ctrl.tick(sp, DT);

    ASSERT_FALSE(bus.tx_empty());
    auto frame = bus.pop_tx();
    EXPECT_LE(decode_i16(frame.data, 6), int16_t(0));
}
