#pragma once
#include "hal.hpp"

// SPI is used for fast peripheral links (e.g. absolute encoder readback via
// AS5047P, motor driver gate-driver config via DRV8353 registers).
// CS line is GPIO-controlled separately to allow back-to-back burst transfers.

struct SPI_HandleTypeDef;

namespace drivers {

class SpiDriver : public hal::ISpi {
public:
    // `gpio_port` / `gpio_pin` identify the CS GPIO on STM32.
    // In SIM mode pass nullptr / 0 — all calls return false gracefully.
    SpiDriver(SPI_HandleTypeDef* hspi, void* gpio_port, uint16_t gpio_pin);

    bool transfer(const uint8_t* tx, uint8_t* rx, uint16_t len) override;
    void cs_assert()   override;
    void cs_deassert() override;

private:
    SPI_HandleTypeDef* hspi_;
    void*    cs_port_;
    uint16_t cs_pin_;
};

}  // namespace drivers
