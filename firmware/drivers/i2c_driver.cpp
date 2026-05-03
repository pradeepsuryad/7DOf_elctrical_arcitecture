#include "i2c_driver.hpp"

#ifdef STM32_TARGET
#  include "stm32f4xx_hal.h"
#endif

namespace drivers {

I2cDriver::I2cDriver(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {}

bool I2cDriver::write(uint8_t device_addr, uint8_t reg,
                      const uint8_t* data, uint16_t len) {
#ifdef STM32_TARGET
    if (!hi2c_) return false;
    // STM32 HAL expects 8-bit address shifted left by 1 (R/W bit handled internally).
    return HAL_I2C_Mem_Write(hi2c_,
                             static_cast<uint16_t>(device_addr << 1),
                             reg, I2C_MEMADD_SIZE_8BIT,
                             const_cast<uint8_t*>(data), len,
                             TIMEOUT_MS) == HAL_OK;
#else
    (void)device_addr; (void)reg; (void)data; (void)len;
    return false;
#endif
}

bool I2cDriver::read(uint8_t device_addr, uint8_t reg,
                     uint8_t* data, uint16_t len) {
#ifdef STM32_TARGET
    if (!hi2c_) return false;
    return HAL_I2C_Mem_Read(hi2c_,
                            static_cast<uint16_t>(device_addr << 1),
                            reg, I2C_MEMADD_SIZE_8BIT,
                            data, len,
                            TIMEOUT_MS) == HAL_OK;
#else
    (void)device_addr; (void)reg; (void)data; (void)len;
    return false;
#endif
}

}  // namespace drivers
