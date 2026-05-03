#include "can_driver.hpp"

// Pull in STM32 HAL only when cross-compiling for the MCU.
// In SIM mode the handle is nullptr and all calls short-circuit below.
#ifdef STM32_TARGET
#  include "stm32f4xx_hal.h"   // adjust to your exact STM32 family
#endif

namespace drivers {

CanDriver::CanDriver(CAN_HandleTypeDef* hcan) : hcan_(hcan) {}

bool CanDriver::start() {
#ifdef STM32_TARGET
    if (!hcan_) return false;
    if (HAL_CAN_Start(hcan_) != HAL_OK) return false;
    // Activate FIFO 0 message-pending interrupt so RX doesn't need polling.
    return HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING)
           == HAL_OK;
#else
    return false;  // SIM: no hardware to start
#endif
}

bool CanDriver::transmit(const hal::CanFrame& frame) {
#ifdef STM32_TARGET
    if (!hcan_) return false;

    CAN_TxHeaderTypeDef hdr{};
    hdr.DLC = frame.dlc;

    if (frame.is_extended) {
        hdr.IDE   = CAN_ID_EXT;
        hdr.ExtId = frame.id;
    } else {
        hdr.IDE   = CAN_ID_STD;
        hdr.StdId = frame.id;
    }
    hdr.RTR = CAN_RTR_DATA;

    uint32_t mailbox;
    // HAL_CAN_AddTxMessage copies data into the hardware TX mailbox.
    // Returns HAL_OK if a mailbox was free; HAL_ERROR if all three are busy.
    return HAL_CAN_AddTxMessage(hcan_, &hdr,
                                const_cast<uint8_t*>(frame.data),
                                &mailbox) == HAL_OK;
#else
    (void)frame;
    return false;
#endif
}

bool CanDriver::receive(hal::CanFrame& frame) {
#ifdef STM32_TARGET
    if (!hcan_) return false;

    // Check FIFO 0 fill level — don't block if empty.
    if (HAL_CAN_GetRxFifoFillLevel(hcan_, CAN_RX_FIFO0) == 0) return false;

    CAN_RxHeaderTypeDef hdr{};
    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &hdr, frame.data) != HAL_OK)
        return false;

    frame.dlc         = static_cast<uint8_t>(hdr.DLC);
    frame.is_extended = (hdr.IDE == CAN_ID_EXT);
    frame.id          = frame.is_extended ? hdr.ExtId : hdr.StdId;
    return true;
#else
    (void)frame;
    return false;
#endif
}

bool CanDriver::is_healthy() const {
#ifdef STM32_TARGET
    if (!hcan_) return false;
    // HAL_CAN_GetError returns a bitmask; 0 means no fault.
    // CAN_ESR bus-off flag is 0x00000004 — anything non-zero is worth flagging.
    return HAL_CAN_GetError(hcan_) == HAL_CAN_ERROR_NONE;
#else
    return false;
#endif
}

}  // namespace drivers
