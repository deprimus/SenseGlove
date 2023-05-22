# Sense Glove

This is a smart glove that uses an accelerometer + gyroscope sensor to do stuff (e.g. act as a mouse).

# Hardware

- Raspberry Pi Pico
- MPU-6050

## Wiring

- Pico Pin 36 (3V3 OUT) -> MPU Pin 1 (VCC)
- Pico Pin 38 (GND) -> MPU Pin 2 (GND)
- Pico Pin 6 (GP4 I2C0 SDA) -> MPU Pin 4 (SDA)
- Pico Pin 7 (GP5 I2C0 SCL) -> MPU Pin 3 (SCL)

# Comms protocol

- SYNC (`0xFF`): syncs with the Pico which sends `0xFF` back
- POLL (`0x0`): retrieves data from the sensor, receives `3 int16` for acceleration XYZ, `3 int16` for gyro XYZ, `1 int8` which is `1` if the BOOTSEL button is pressed, `0` otherwise
- CALIBRATE (`0x1`): tells the Pico to calibrate the sensor

For more details, see the included driver.