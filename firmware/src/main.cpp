#include "can_driver.hpp"
#include "spi_driver.hpp"
#include "i2c_driver.hpp"

// ── Entry point ───────────────────────────────────────────────────────────────
//
// Real-time loop structure for a 7-DoF arm:
//
//   1 kHz  outer loop  → trajectory interpolation, safety watchdog
//   5 kHz  inner loop  → current / torque PID, CAN command dispatch
//   async              → I2C sensor polling, CAN RX ISR
//
// On STM32, main() is called after the vendor BSP sets up clocks and runs
// global constructors.  In SIM mode it's a normal desktop process entry.
// ─────────────────────────────────────────────────────────────────────────────

#ifdef STM32_TARGET
// CubeMX generates these extern handles in main.c / stm32fxxx_it.c.
extern CAN_HandleTypeDef  hcan1;
extern SPI_HandleTypeDef  hspi1;
extern I2C_HandleTypeDef  hi2c1;
#endif

int main() {
#ifdef STM32_TARGET
    // Construct drivers around the BSP-owned peripheral handles.
    drivers::CanDriver can(&hcan1);
    drivers::SpiDriver spi(&hspi1, GPIOA, GPIO_PIN_4);  // PA4 = SPI1_NSS
    drivers::I2cDriver i2c(&hi2c1);

    can.start();
#else
    // SIM mode: pass nullptr — drivers will short-circuit all calls.
    drivers::CanDriver can(nullptr);
    drivers::SpiDriver spi(nullptr, nullptr, 0);
    drivers::I2cDriver i2c(nullptr);
#endif

    // ── Main loop (bare-metal super-loop; replace with RTOS tasks later) ─────
    while (true) {
        // 1. Receive any pending CAN frames from joints.
        hal::CanFrame rx{};
        while (can.receive(rx)) {
            // TODO: dispatch rx.id to the correct joint state object.
            (void)rx;
        }

        // 2. Run control loop tick (motor_controller.tick()).
        //    Not yet implemented — placeholder.

        // 3. Watchdog kick (HAL_IWDG_Refresh or equivalent).
    }

    return 0;  // never reached on bare-metal
}
