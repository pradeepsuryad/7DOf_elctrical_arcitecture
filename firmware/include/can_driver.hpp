#pragma once
#include "hal.hpp"
#include <cstdint>

// ── CAN driver ────────────────────────────────────────────────────────────────
//
// Wraps the STM32 HAL_CAN API and implements hal::ICanBus.
//
// CAN bus topology for the 7-DoF arm:
//   MCU  ─── TJA1050/SN65HVD23x transceiver ─── 120 Ω termination ─── joints
//
// Bit-rate choices (all joints must agree):
//   500 kbit/s  →  safe for runs up to ~4 m at low EMI
//   1 Mbit/s    →  max per CAN 2.0B; requires tight stub lengths
//
// Message ID allocation (11-bit standard frame):
//   0x100 + joint_id  →  position / velocity command  (MCU → joint)
//   0x200 + joint_id  →  torque command                (MCU → joint)
//   0x300 + joint_id  →  state feedback                (joint → MCU)
//   0x7DF             →  broadcast diagnostic request
// ─────────────────────────────────────────────────────────────────────────────

// Forward-declare the opaque STM32 CAN handle so this header doesn't drag in
// the full stm32fxxx_hal.h when building in SIM mode.
struct CAN_HandleTypeDef;

namespace drivers {

class CanDriver : public hal::ICanBus {
public:
    // `hcan` is the STM32 peripheral handle initialised by CubeMX / your BSP.
    // Pass nullptr in SIM builds — transmit/receive return false gracefully.
    explicit CanDriver(CAN_HandleTypeDef* hcan);

    bool transmit(const hal::CanFrame& frame) override;
    bool receive(hal::CanFrame& frame) override;
    bool is_healthy() const override;

    // Start the peripheral and activate RX FIFO 0 interrupt.
    // Call once after HAL_CAN_Init().
    bool start();

private:
    CAN_HandleTypeDef* hcan_;
};

}  // namespace drivers
