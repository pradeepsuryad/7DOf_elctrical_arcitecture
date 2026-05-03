#include "spi_driver.hpp"

#ifdef STM32_TARGET
#  include "stm32f4xx_hal.h"
#endif

namespace drivers {

SpiDriver::SpiDriver(SPI_HandleTypeDef* hspi, void* gpio_port, uint16_t gpio_pin)
    : hspi_(hspi), cs_port_(gpio_port), cs_pin_(gpio_pin) {}

void SpiDriver::cs_assert() {
#ifdef STM32_TARGET
    if (cs_port_)
        HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(cs_port_), cs_pin_, GPIO_PIN_RESET);
#endif
}

void SpiDriver::cs_deassert() {
#ifdef STM32_TARGET
    if (cs_port_)
        HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(cs_port_), cs_pin_, GPIO_PIN_SET);
#endif
}

bool SpiDriver::transfer(const uint8_t* tx, uint8_t* rx, uint16_t len) {
#ifdef STM32_TARGET
    if (!hspi_) return false;
    // TransmitReceive is full-duplex — both buffers must be len bytes.
    return HAL_SPI_TransmitReceive(hspi_,
                                   const_cast<uint8_t*>(tx), rx, len,
                                   /*timeout_ms=*/5) == HAL_OK;
#else
    (void)tx; (void)rx; (void)len;
    return false;
#endif
}

}  // namespace drivers
