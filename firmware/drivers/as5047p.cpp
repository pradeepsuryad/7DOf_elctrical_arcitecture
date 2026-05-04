#include "as5047p.hpp"
#include <cstdint>

namespace drivers {

// ── SPI frame helpers ─────────────────────────────────────────────────────────

// AS5047P SPI frame (16-bit, MSB first):
//   Bit 15  : parity (even parity over bits 14:0, computed and checked here)
//   Bit 14  : R/!W — 1 for read, 0 for write
//   Bits 13:0 : register address (on command) or data (on reply)
//
// A read takes two SPI transactions:
//   Transaction 1: send read-command frame, reply is the NOP response (discard)
//   Transaction 2: send NOP (0xC000), reply contains the register value
//
// Why two transactions?  The AS5047P pipeline is one cycle deep — it captures
// the address in transaction N and returns the data in transaction N+1.
// This is documented in Section 2.5 of the datasheet.

uint16_t As5047p::even_parity(uint16_t v) {
    // XOR all 15 bits together; result is 1 if an odd number of bits are set.
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1u;
}

uint16_t As5047p::read_cmd(uint16_t reg_addr) {
    // Bit 14 = 1 → read; bit 15 = parity.
    uint16_t frame = (1u << 14) | (reg_addr & 0x3FFF);
    frame |= (even_parity(frame) << 15);
    return frame;
}

bool As5047p::spi_transfer(uint16_t tx, uint16_t& rx) {
    uint8_t tx_buf[2] = { static_cast<uint8_t>(tx >> 8),
                          static_cast<uint8_t>(tx & 0xFF) };
    uint8_t rx_buf[2] = {0, 0};

    spi_.cs_assert();
    bool ok = spi_.transfer(tx_buf, rx_buf, 2);
    spi_.cs_deassert();

    if (!ok) {
        last_error_ = Error::FRERR;
        return false;
    }

    rx = static_cast<uint16_t>((rx_buf[0] << 8) | rx_buf[1]);

    // Verify parity on the reply (covers bits 14:0).
    uint16_t expected_parity = even_parity(rx & 0x7FFF);
    uint16_t received_parity = (rx >> 15) & 1u;
    if (expected_parity != received_parity) {
        last_error_ = Error::PARERR;
        return false;
    }

    // Bit 14 of reply is the error flag from the AS5047P.
    // If set, the ERRFL register has details — caller should call read_errors().
    if (rx & (1u << 14)) {
        // Don't overwrite a harder fault; just note the flag.
        if (last_error_ == Error::NONE)
            last_error_ = Error::INVCOMM;  // placeholder until ERRFL is read
    }

    return true;
}

// ── Constructor ───────────────────────────────────────────────────────────────

As5047p::As5047p(hal::ISpi& spi) : spi_(spi) {}

// ── read_raw ──────────────────────────────────────────────────────────────────

bool As5047p::read_raw(uint16_t& counts) {
    last_error_ = Error::NONE;

    uint16_t dummy;
    // Transaction 1: send read command for ANGLEC, discard reply.
    if (!spi_transfer(read_cmd(REG_ANGLEC), dummy)) return false;

    // Transaction 2: send NOP, receive ANGLEC data.
    uint16_t reply;
    if (!spi_transfer(read_cmd(REG_ANGLEC), reply)) return false;

    counts = reply & 0x3FFF;   // bits 13:0 are the 14-bit angle
    return true;
}

// ── read_angle ────────────────────────────────────────────────────────────────

bool As5047p::read_angle(float& angle_rad) {
    uint16_t counts;
    if (!read_raw(counts)) return false;
    angle_rad = static_cast<float>(counts) * COUNTS_TO_RAD;
    return true;
}

// ── read_errors ───────────────────────────────────────────────────────────────

As5047p::Error As5047p::read_errors() {
    uint16_t dummy, reply;
    spi_transfer(read_cmd(REG_ERRFL), dummy);
    spi_transfer(read_cmd(REG_ERRFL), reply);
    last_error_ = static_cast<Error>(reply & 0x07);
    return last_error_;
}

// ── read_diag ─────────────────────────────────────────────────────────────────

As5047p::Diag As5047p::read_diag() {
    uint16_t dummy, reply;
    spi_transfer(read_cmd(REG_DIAAGC), dummy);
    spi_transfer(read_cmd(REG_DIAAGC), reply);
    return static_cast<Diag>(reply & 0x1E);   // bits 4:1 are MAGL/MAGH/COF/LF
}

}  // namespace drivers
