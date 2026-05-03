#pragma once
#include <cstdint>

// ── Hardware abstraction layer ────────────────────────────────────────────────
//
// Why pure virtual interfaces?
//   Real MCU peripherals (STM32 HAL, ASF, etc.) differ per vendor.
//   By coding higher layers against these interfaces, the same motor-control
//   logic compiles and runs on the MCU *and* on a desktop SIM/HIL host.
//   On the MCU you pass an Stm32CanBus; in unit tests you pass a FakeCanBus.
//
// Rule: no peripheral register access above this layer.
// ─────────────────────────────────────────────────────────────────────────────

namespace hal {

// ── CAN ───────────────────────────────────────────────────────────────────────
struct CanFrame {
    uint32_t id;           // 11-bit standard or 29-bit extended ID
    uint8_t  data[8];
    uint8_t  dlc;          // data-length code (0–8)
    bool     is_extended;  // false → standard 11-bit ID
};

class ICanBus {
public:
    virtual ~ICanBus() = default;

    // Blocking transmit — returns false on arbitration loss or bus-off.
    virtual bool transmit(const CanFrame& frame) = 0;

    // Non-blocking receive — returns false if RX FIFO is empty.
    virtual bool receive(CanFrame& frame) = 0;

    // True while the bus is error-active or error-passive (not bus-off).
    virtual bool is_healthy() const = 0;
};

// ── SPI ───────────────────────────────────────────────────────────────────────
class ISpi {
public:
    virtual ~ISpi() = default;

    // Full-duplex transfer: write tx_len bytes, capture rx_len bytes.
    // CS assert/deassert is the caller's responsibility (allows multi-byte bursts).
    virtual bool transfer(const uint8_t* tx, uint8_t* rx, uint16_t len) = 0;

    virtual void cs_assert()   = 0;
    virtual void cs_deassert() = 0;
};

// ── I2C ───────────────────────────────────────────────────────────────────────
class II2c {
public:
    virtual ~II2c() = default;

    // Write `len` bytes to `device_addr` starting at `reg`.
    virtual bool write(uint8_t device_addr, uint8_t reg,
                       const uint8_t* data, uint16_t len) = 0;

    // Read `len` bytes from `device_addr` starting at `reg`.
    virtual bool read(uint8_t device_addr, uint8_t reg,
                      uint8_t* data, uint16_t len) = 0;
};

// ── System clock / time ───────────────────────────────────────────────────────
class ISysClock {
public:
    virtual ~ISysClock() = default;

    // Microsecond timestamp — wraps at ~71 minutes for uint32_t.
    // Use uint64_t overload for long-running sessions.
    virtual uint32_t micros() const = 0;
    virtual uint64_t micros64() const = 0;

    virtual void delay_us(uint32_t us) = 0;
};

}  // namespace hal
