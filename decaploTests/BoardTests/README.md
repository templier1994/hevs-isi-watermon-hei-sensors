# BoardTests

This project tests the DeCapLo board: 

* The bistable relay
* The RESET and ACT1 buttons, plus the reed relays
  * Tested by polling
* The green LED
* The pulse input 
  * Tested by polling and with the  low power counter
* The Battery measure
  * Tested with the ADC in 12-bit
  * 3.3V => 1365 (4096/3)
  * Under 1.5V, the TPS22945 stops driving 
* Reading/Writing into the Flash (TODO)



## CubeMX configuration

These configurations are done from CubeMX GUI:

* **LPUART1** for the serial console:
  * Asynchronous
  * 9600 Bits/s
  * 8 Bits
  * No parity
  * 1 stop Bit
* **ADC** for the battery measurement:
  * IN4
  * 12-bit resolution
  * 39.5 or 79.5 cycles is OK for the precision
* **LPTIM1** for the pulse counting
  * Mode: Standalone: counts external Clock events (PB5 used for input count)
  * Clock: Prescaler Div1
  * Clock Polarity: Rising edge
  * Clock sample time: Direct Transition
  * Update mode: Update immediate
  * Trigger source: Software trigger
  * NVIC Settings: LPTIM1 wake-up interrupt through EXTI line 29 
    * only if we want to wake-up the device when the counter reaches the period value

* **RTC** to wake-up the device from STOP or STANDBY mode after a while
  * Activate Clock Source
  * WakeUp: Internal WakeUp
  * Asynchronous Predivider value: 127
  * Synchronous Predivider value: 255
  * Wake Up Clock: 1 Hz
  * NVIC Settings: RTC global interrupt through EXTI lines 



## Consumption measurements 

Consumption measures were made either in normal, stop and standby modes. The board was powered in 3.3 [V] and the Agilent U1252A was used to measure the current. 

The JP1 of the board connects VDD_RF with V_SENSOR and not V_CPU. By doing so, the RF circuit can be powered or not.



### Stop mode:

TODO: The STOP mode couldn't be correctly configured (directly goes out of the stop mode).

* The StopModeTest project works. Must find what causes this behavior



### Standby mode:

Several measurements were done, either in Debug or Release configurations, with/without the ST-Link connected and with/without  the VCC_RF powered.

 * Debug + ST-Link connected + VCC_RF powered:
    * 4.5 [mA] in normal mode
    * 3.8 [mA] in standby mode
 * Debug + ST-Link connected + VCC_RF powered:
    *      1.1 [mA] in normal mode
    *      540 [uA] in standby mode
 * Release + ST-Link disconnected + VCC_RF not powered:
    *      700 [uA] in normal mode
    *      10-15 [uA] in standby mode (measure not stable with multimeter)