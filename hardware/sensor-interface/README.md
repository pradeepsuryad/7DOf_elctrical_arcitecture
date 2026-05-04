# Sensor Interface

Reference material for analog and digital sensors connected to the MCU.

- Absolute encoder: AS5047P (14-bit, SPI mode 3) — one per joint
  Driver: firmware/drivers/as5047p.cpp
- IMU: BNO055 or MPU-6050 (I2C) — arm base
- Temperature: NTC thermistor on each motor winding, read via MCU ADC

Drop datasheets, wiring pinouts, or calibration notes here.
