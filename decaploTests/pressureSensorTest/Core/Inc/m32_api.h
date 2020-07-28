/*
 * m32.h
 *
 *  Created on: Jul 16, 2020
 *      Author: joachim
 */

#ifndef INC_M32_API_H_
#define INC_M32_API_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/**
 * @brief   Reads the value of the m32 pressure transducer.
 *
 * The method sends a measurement request to the sensor throught the i2c bus,
 * and then reads 3 bytes (2 for the pressure and 1 for the temperature).
 *
 * @param   hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *          the configuration information for the specified I2C.
 * @param   pressure Pointer to the output pressure, in millibar
 * @param   temp Pointer to the output temperature, in Degrees Celsius
 * @return  0 if read OK
 */
int m32_read_value(I2C_HandleTypeDef *hi2c, uint32_t *pressure, uint8_t *temp);

#endif /* INC_M32_API_H_ */
