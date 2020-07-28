/*
 * pressureSensorDecaplo.c
 *
 *  Created on: Jul 16, 2020
 *      Author: joachim
 */

/* Includes ------------------------------------------------------------------*/
#include "pressureSensorDecaplo.h"
#include "m32_api.h"

/* Defines -------------------------------------------------------------------*/
#define I2C_GPIO            GPIOB
#define SCL_PIN             GPIO_PIN_8
#define SDA_PIN             GPIO_PIN_9

#define VCC_SENSOR_GPIO     GPIOA
#define VCC_SENSOR_PIN      GPIO_PIN_11

/* Private function prototypes -----------------------------------------------*/
static void enable_i2c();
static void disable_i2c();
static void enable_vcc_sensor();
static void disable_vcc_sensor();


int read_pressure(I2C_HandleTypeDef *hi2c, uint32_t *pressure, uint8_t *temp)
{
  enable_vcc_sensor();
  int value = m32_read_value(hi2c, pressure, temp);
  disable_vcc_sensor();
  return value;
}

void wakeup()
{
  enable_vcc_sensor();
  enable_i2c();
  HAL_Delay(9); // wait 9ms after wakeup
}

void suspend()
{
  disable_vcc_sensor();
  disable_i2c();
}

static void enable_i2c()
{
  GPIO_InitTypeDef gp = {
    .Pin        = SCL_PIN | SDA_PIN,
    .Mode       = GPIO_MODE_AF_OD,
    .Pull       = GPIO_NOPULL,
    .Speed      = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate  = GPIO_AF4_I2C1,
  };
  HAL_GPIO_Init(I2C_GPIO, &gp);
}

static void disable_i2c()
{
  GPIO_InitTypeDef gp = {
    .Pin  = SCL_PIN | SDA_PIN,
    .Mode = GPIO_MODE_INPUT,
    .Pull = GPIO_PULLDOWN,
  };
  HAL_GPIO_Init(I2C_GPIO, &gp);
}

static void enable_vcc_sensor()
{
  GPIO_InitTypeDef gp = {
      .Pin  = VCC_SENSOR_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
  };
  HAL_GPIO_Init(VCC_SENSOR_GPIO, &gp);
  HAL_GPIO_WritePin(VCC_SENSOR_GPIO, gp.Pin, 1);
}

static void disable_vcc_sensor()
{
  GPIO_InitTypeDef gp = {
      .Pin  = VCC_SENSOR_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
  };
  HAL_GPIO_Init(VCC_SENSOR_GPIO, &gp);
  HAL_GPIO_WritePin(VCC_SENSOR_GPIO, gp.Pin, 0);
}
