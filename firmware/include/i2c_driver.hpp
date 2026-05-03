#pragma once
#include "hal.hpp"

// I2C is used for low-rate peripherals: IMU (MPU-6050 / BNO055),
// temperature sensors on the power stage, and EEPROM config storage.

struct I2C_HandleTypeDef;

namespace drivers {

class I2cDriver : public hal::II2c {
public:
    explicit I2cDriver(I2C_HandleTypeDef* hi2c);

    bool write(uint8_t device_addr, uint8_t reg,
               const uint8_t* data, uint16_t len) override;

    bool read(uint8_t device_addr, uint8_t reg,
              uint8_t* data, uint16_t len) override;

private:
    I2C_HandleTypeDef* hi2c_;
    static constexpr uint32_t TIMEOUT_MS = 10;
};

}  // namespace drivers
