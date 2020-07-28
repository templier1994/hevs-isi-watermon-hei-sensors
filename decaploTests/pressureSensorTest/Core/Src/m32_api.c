/*
 * m32_api.c
 *
 *  Created on: Jul 16, 2020
 *      Author: joachim
 */

/* Includes ------------------------------------------------------------------*/
#include "m32_api.h"

/* Defines -------------------------------------------------------------------*/
#define M32_ADDR              0x28 << 1
#define MAX_PRESSURE_BAR      7
#define I2C_BUF_SIZE          4


/* Private function prototypes -----------------------------------------------*/
uint32_t extract_pressure(const uint8_t *i2cBuf);
uint8_t extract_temp(const uint8_t *i2cBuf);


int m32_read_value(I2C_HandleTypeDef *hi2c, uint32_t *pressure, uint8_t *temp)
{
  uint8_t i2cBuf[I2C_BUF_SIZE];
  HAL_StatusTypeDef ret;
  uint8_t status;

  // Measurement Request
  ret = HAL_I2C_Master_Receive(hi2c, M32_ADDR, i2cBuf, 0, HAL_MAX_DELAY);
  if(ret != HAL_OK) {
    return -1;
  }
  HAL_Delay(3);   // 3ms delay after request

  // Read pressure and temperature
  ret = HAL_I2C_Master_Receive(hi2c, M32_ADDR, i2cBuf, 3, HAL_MAX_DELAY);
  if(ret != HAL_OK) {
      return -1;
  }

  // Extract measure status:
  //    0x00 = Normal operation: Good Data Packet
  //    0x10 = Stale Data: Data has been fetched since last measurement cycle
  //    0x11 = Fault Detected
  status = ((uint8_t) i2cBuf[0] >> 6);
  if(status != 0) {
    // Stale data or fault detected
    return -1;
  }

  // Extract pressure
  *pressure = extract_pressure(i2cBuf);

  // Extract temperature
  *temp = extract_temp(i2cBuf);

  return 0;
}

/**
 * Extracts the pressure (in millibar) from the data given by the device
 */
uint32_t extract_pressure(const uint8_t *i2cBuf)
{
  uint16_t pressureCounts = ((uint16_t)(i2cBuf[0] & 0x3F) << 8);  // mask 2 status MSB and shift others
  pressureCounts |= ((uint16_t)i2cBuf[1]);

  int32_t pressureTmp = (int32_t) ((pressureCounts - 1000) * MAX_PRESSURE_BAR / (float)(15000 - 1000) * 1000);    // formula from the datasheet
  return ((pressureTmp < 0) ? 0 : (uint32_t) pressureTmp);
}

/**
 * Returns the temperature (in Degrees Celcius) from the data given by the device
 */
uint8_t extract_temp(const uint8_t *i2cBuf)
{
  uint16_t tempCounts = ((uint16_t)i2cBuf[2] << 3);
  return (tempCounts * (150 - (-50)) / 2048) - 50;
}
