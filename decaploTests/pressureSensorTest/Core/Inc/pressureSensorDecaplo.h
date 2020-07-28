/*
 * pressureSensorDecaplo.h
 *
 *  Created on: Jul 16, 2020
 *      Author: joachim
 */

#ifndef INC_PRESSURESENSORDECAPLO_H_
#define INC_PRESSURESENSORDECAPLO_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/**
 * @brief   Reads the value of the i2c pressure/temperature sensor.
 *
 * @param   hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *          the configuration information for the specified I2C.
 * @param   pressure Pointer to the output pressure, in millibar
 * @param   temp Pointer to the output temperature, in Degrees Celsius
 * @return  0 if read OK
 */
int read_pressure(I2C_HandleTypeDef *hi2c, uint32_t *pressure, uint8_t *temp);

/**
 * @brief Wake up the device
 */
void wakeup();

/**
 * @brief Suspend the sensor to reduce consumption
 *
 * - Disable vcc_sensor
 * - Set the I2C pins in input to avoid consumptions
 */
void suspend();

#endif /* INC_PRESSURESENSORDECAPLO_H_ */
