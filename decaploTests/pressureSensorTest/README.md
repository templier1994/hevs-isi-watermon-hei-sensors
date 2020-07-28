# PressureSensorTest

This project tests the I2C M32 pressure detector. 



## CubeMX configuration

The I2C for the sensor and LPUART for the serial console are configured with CubeMX GUI.

*  I2C1:
  * 100 kHz
  * SCL: PB8
  * SDA: PB9
* LPUART1:
  * Asynchronous
  * 9600 Bits/s
  * 8 Bits
  * No parity
  * 1 stop Bit



## Project files

* **main** : Each 5 seconds, wakes up the device, reads the value and suspends it
* **pressureSensorDecaplo**: Methods to wakeup/suspend the sensor and read the pressure and temperature from the sensor (with m32_api)
* **m32_api** : Method to read the pressure and temperature from the m32 sensor