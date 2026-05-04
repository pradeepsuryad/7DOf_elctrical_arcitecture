#pragma once
#include "hal.hpp"
#include <cstdint>

// ── AS5047P absolute magnetic rotary encoder ──────────────────────────────────
//
// Manufacturer: ams-OSRAM  (formerly AMS)
// Resolution:   14-bit → 16384 counts/rev → 0.022° per count
// Interface:    SPI mode 3 (CPOL=1, CPHA=1), up to 10 MHz, MSB first
// Supply:       3.3 V or 5 V (with level shifter on SPI lines if MCU is 3.3 V)
//
// Frame format (16 bits, MSB first):
//   Bit 15     : parity (even parity over bits 14:0)
//   Bit 14     : error flag — 1 if any fault is latched in ERRFL register
//   Bits 13:0  : data (register address on write, angle/register value on read)
//
// Register map (relevant subset):
//   0x3FFF  ANGLEC   — 14-bit compensated angle (use this for position)
//   0x3FFE  ANGLEUNC — 14-bit uncompensated angle (raw)
//   0x0001  ERRFL    — error flags: FRERR, INVCOMM, PARERR
//   0x0003  DIAAGC   — diagnostics: MAGL (weak field), MAGH (too strong),
//                      COF (CORDIC overflow), LF (offset comp not finished)
//
// Wiring (per joint):
//   MCU SPI SCK  → AS5047P CLK
//   MCU SPI MOSI → AS5047P MOSI
//   MCU SPI MISO ← AS5047P MISO
//   MCU GPIO     → AS5047P CS  (active-low, one pin per encoder)
//
// Usage:
//   drivers::As5047p enc(spi_driver, joint_id);
//   float angle_rad;
//   if (enc.read_angle(angle_rad)) {
//       // use angle_rad ...
//   }
// ─────────────────────────────────────────────────────────────────────────────

namespace drivers {

class As5047p : public hal::IEncoder {
public:
    // Error flags from the ERRFL register (may be OR-ed together).
    enum class Error : uint8_t {
        NONE    = 0x00,
        FRERR   = 0x01,   // framing error (SPI clock count mismatch)
        INVCOMM = 0x02,   // invalid command (unrecognised register address)
        PARERR  = 0x04,   // parity error on the received frame
    };

    // Diagnostic flags from the DIAAGC register.
    enum class Diag : uint8_t {
        NONE  = 0x00,
        MAGL  = 0x10,   // magnetic field too weak (magnet too far)
        MAGH  = 0x08,   // magnetic field too strong (magnet too close)
        COF   = 0x04,   // CORDIC overflow — angle output invalid
        LF    = 0x02,   // offset compensation loop not finished (startup only)
    };

    explicit As5047p(hal::ISpi& spi);

    // ── hal::IEncoder ─────────────────────────────────────────────────────────

    // Read the compensated angle.  Returns false on SPI failure or parity error.
    // On success `angle_rad` is in [0, 2π).
    bool read_angle(float& angle_rad) override;

    // True if the last SPI transaction flagged any hardware error.
    bool has_error() const override { return last_error_ != Error::NONE; }

    // ── Extended diagnostics ──────────────────────────────────────────────────

    // Read the raw 14-bit count (0–16383).  Useful for calibration.
    bool read_raw(uint16_t& counts);

    // Read the ERRFL register and return accumulated error bits.
    Error read_errors();

    // Read DIAAGC register — use to check magnetic field strength at startup.
    Diag read_diag();

    Error last_error() const { return last_error_; }

private:
    // Send one 16-bit SPI frame and receive one 16-bit reply.
    // Returns false if the reply has a parity error.
    bool spi_transfer(uint16_t tx, uint16_t& rx);

    // Compute even parity over bits 14:0.
    static uint16_t even_parity(uint16_t v);

    // Build a read-command word for the given register address.
    static uint16_t read_cmd(uint16_t reg_addr);

    hal::ISpi& spi_;
    Error      last_error_{Error::NONE};

    static constexpr float COUNTS_TO_RAD = 6.28318530718f / 16384.0f;

    // Register addresses
    static constexpr uint16_t REG_ERRFL   = 0x0001;
    static constexpr uint16_t REG_DIAAGC  = 0x0003;
    static constexpr uint16_t REG_ANGLEC  = 0x3FFF;
    static constexpr uint16_t REG_ANGLEUNC= 0x3FFE;
};

}  // namespace drivers
