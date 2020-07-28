/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VCC_SENSOR_PIN      GPIO_PIN_11 // PA11
#define RELAY_OPEN_PIN      GPIO_PIN_8  // PA8
#define LORA_VDD_TCXO       GPIO_PIN_12 // PA12
#define BUTTON_PIN          GPIO_PIN_0  // PA0
#define GREEN_LED_PIN       GPIO_PIN_7  // PB7
#define PULSE_COUNTER_PIN   GPIO_PIN_5  // PB6 FIXME: Pulse counter should be on PB5 (LPTIM1 IN1): swap with SONAR_CMD
#define BAT_MES_NOW_PIN     GPIO_PIN_2  // PB2
#define VBAT_DIV3_PIN       GPIO_PIN_4  // PA4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

LPTIM_HandleTypeDef hlptim1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
static const uint8_t  UART_BUF_SIZE = 64;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void test_relay();
static void enable_vcc_sensor();
static void disable_vcc_sensor();
static void test_button_poll();
static void test_leds();
static void test_pulse_poll();
static void test_pulse_lp_timer();
static void test_battery_measure();
static void test_stop_mode();
static void test_standby_mode();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t uartBuf[UART_BUF_SIZE];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_LPTIM1_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  snprintf((char*)uartBuf, UART_BUF_SIZE, "Code started..\n\r");
  HAL_UART_Transmit(&hlpuart1, uartBuf, strlen((char*)uartBuf), HAL_MAX_DELAY);

  /**Set a pin to 1 at begining, the 0 when stop, the 1 when wake up**/
	 GPIO_InitTypeDef x;
	 x.Pin   = PULSE_COUNTER_PIN;
	 x.Mode  = GPIO_MODE_OUTPUT_PP;

	 HAL_GPIO_Init(GPIOB, &x);
	 HAL_GPIO_WritePin(GPIOB, x.Pin, 1);




  /**
   * TODO: Uncomment these functions to test the functionalities
   */
  //test_relay();
  //test_button_poll();
  //test_leds();
  //test_pulse_poll();
  //test_pulse_lp_timer();
  //test_battery_measure();
  test_stop_mode();
 // test_standby_mode();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOB, x.Pin, 0);
    // Print out buffer (data or error message)
    snprintf((char*)uartBuf, UART_BUF_SIZE, "Hello world\n\r");
    HAL_UART_Transmit(&hlpuart1, uartBuf, strlen((char*)uartBuf), HAL_MAX_DELAY);

    // Wait
    HAL_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_ULPTIM;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * Changes the relay state two times. The v_sensor is enabled while driving the relay
 */
static void test_relay()
{
  int relay_state = 0;

  GPIO_InitTypeDef gp = {
      .Pin  = RELAY_OPEN_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
  };
  HAL_GPIO_Init(GPIOA, &gp);

  for(int i = 0; i < 2; i++) {
    enable_vcc_sensor();

    relay_state = (relay_state ? 0 : 1);
    HAL_GPIO_WritePin(GPIOA, gp.Pin, relay_state);
    HAL_Delay(4);   // max commutating time is 4ms

    disable_vcc_sensor();

    HAL_Delay(1500);
  }
}

static void enable_vcc_sensor()
{
  GPIO_InitTypeDef gp = {
      .Pin  = VCC_SENSOR_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
  };
  HAL_GPIO_Init(GPIOA, &gp);
  HAL_GPIO_WritePin(GPIOA, gp.Pin, 1);
}

static void disable_vcc_sensor()
{
  GPIO_InitTypeDef gp = {
      .Pin  = VCC_SENSOR_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
  };
  HAL_GPIO_Init(GPIOA, &gp);
  HAL_GPIO_WritePin(GPIOA, gp.Pin, 0);
}

/**
 * Reads the BUTTON 'ACT1' value doing polling
 */
static void test_button_poll()
{
  uint8_t buf[32];
  uint8_t button_value = 0;

  // init button as input
  GPIO_InitTypeDef gp = {
      .Pin  = BUTTON_PIN,
      .Mode = GPIO_MODE_INPUT,
  };
  HAL_GPIO_Init(GPIOA, &gp);

  for(int i = 0; i < 50; i++) {
    button_value = HAL_GPIO_ReadPin(GPIOA, BUTTON_PIN);
    snprintf((char*)buf, 32, "Button value = %d\n\r", button_value);
    HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
    HAL_Delay(100);
  }
}

/**
 * Blinking the LD_GREEN to test the LED
 *
 * TODO: Couldn't initialize the LED GPIO here and not from the MX_GPIO_Init() (the LED doesn't blink))
 */
static void test_leds()
{
  int led_state = 0;

  // Init LED GPIO (FIXME: Don't know why when initializing the LED GPIO here and not in the MX_GPIO_Init(), the LED is not blinking)
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  for(int i = 0; i < 20; i++) {
      led_state = (led_state ? 0 : 1);
      HAL_GPIO_WritePin(GPIOB, GREEN_LED_PIN, led_state);
      HAL_Delay(500);
    }
}

/**
 * Reads PULSE_COUNTER state (high or low) doing polling to validate the optocoupler is working
 *
 * Note: The PULSE_COUNTER input is now on PB6 (LPTIM1_ETR) and will be swap with PB5 (LPTIM1_IN1)
 */
static void test_pulse_poll()
{
  uint8_t buf[32];
  uint8_t input_value = 0;

  // init pulse as input
  GPIO_InitTypeDef gp = {
      .Pin  = PULSE_COUNTER_PIN,
      .Mode = GPIO_MODE_INPUT,
  };
  HAL_GPIO_Init(GPIOB, &gp);

  for(int i = 0; i < 50; i++) {
    input_value = HAL_GPIO_ReadPin(GPIOB, PULSE_COUNTER_PIN);
    snprintf((char*)buf, 32, "Pulse input value = %d\n\r", input_value);
    HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
    HAL_Delay(100);
  }
}

/**
 * Counts the pulses from the PULSE_COUNTER input using the Low-Power Timer. The CPU is set in STOP
 * mode and wake up by the timer when the counter reaches 50.
 *
 * TODO: When the CPU enters in stop mode, it continues instead of stopping there until the interrupt occurs
 *
 * Note: The PULSE_COUNTER input is now on PB6 (LPTIM1_ETR) and will be swap with PB5 (LPTIM1_IN1)
 */
static void test_pulse_lp_timer()
{
  uint8_t buf[32];
  uint16_t ticks = 0;

  // start counter
  HAL_LPTIM_Counter_Start_IT(&hlptim1, 10);    // period of the counting up to 65535

  for(int i = 0; i < 50; i++) {
      ticks = HAL_LPTIM_ReadCounter(&hlptim1);
      snprintf((char*)buf, 32, "Pulse input counter=%d\n\r", ticks);
      HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
      HAL_Delay(200);
    }

  snprintf((char*)buf, 32, "I'm going to sleep..\n\r");
  HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

  // Go in stop mode
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);  // clear wakeup flag
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // go in stop mode

  snprintf((char*)buf, 32, "I'm waking up\n\r");
  HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
}

/**
  * @brief  Autoreload match callback in non blocking mode. Interrupt called when the LPTIM reaches its period
  *
  * @param  hlptim : LPTIM handle
  * @retval None
  */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  uint8_t buf[32];
  snprintf((char*)buf, 32, "AutoReloadMatchCallback\n\r");
  HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
}

/**
 * Tests the battery measurement. Put the BAT_MES_NOW pin to HIGH at start and
 * then do one measure each 500ms. In the end, put BAT_MES_NOW to LOW.
 * The ADC measure range is from 0 (for 0v) to 4096 (for 3.3V) and v_bat is
 * divided by 3 (so meas=1365 when v_bat=3.3V).
 *
 * Note: The TPS22945 stops driving when v_bat is under 1.5V
 */
static void test_battery_measure()
{
  uint8_t buf[32];
  uint16_t battery_value = 0;

  // init bat_mes_now as output
  GPIO_InitTypeDef gp = {
      .Pin  = BAT_MES_NOW_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
  };
  HAL_GPIO_Init(GPIOB, &gp);

  // enable bat_mes_now
  HAL_GPIO_WritePin(GPIOB, BAT_MES_NOW_PIN, 1);
  HAL_Delay(5);

  for(int i = 0; i < 100; i++) {
    // get ADC value
    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    battery_value = HAL_ADC_GetValue(&hadc);

    snprintf((char*)buf, 32, "Battery value = %u\n\r", battery_value);
    HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
    HAL_Delay(500);
  }

  // disable bat_mes_now
  HAL_GPIO_WritePin(GPIOB, BAT_MES_NOW_PIN, 0);
}

/**
 * First runs for a while in standard mode, and then pass in stop mode. The rtc wakes up
 * the device after a while and the code should continue where it was.
 *
 * TODO: The EnterSTOPMode function doesn't stops or is woke up directly after
 *
 * TODO: Enable the LPTIM counter (uncomment the related lines) and check the value still increments in stop mode
 */
static void test_stop_mode()
{
  uint8_t buf[32];
  uint32_t ticks = 0;

  //while(1) {
    // start counting pulses
    //HAL_LPTIM_Counter_Start_IT(&hlptim1, 30);

    // run for a while
   // for(int i = 0; i < 50; i++) {
      //ticks = HAL_LPTIM_ReadCounter(&hlptim1);
   //   snprintf((char*)buf, 32, "Normal mode, i=%u, pulse=%lu\n\r", i, ticks);
   //   HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
   // }

    // set RTC wakeup
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 30, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

    snprintf((char*)buf, 32, "Go to stop mode...\n\r");
    HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

    // go to stop mode
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);  // clear wakeup flag
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // go in stop mode

    // ticks after wake up
    //ticks = HAL_LPTIM_ReadCounter(&hlptim1);
   // snprintf((char*)buf, 32, "Waking up, pulse=%lu\n\r", ticks);
   // HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
  //}
}

/**
 * First runs for a while in standard mode, and then pass in standby mode. The rtc wakes up
 * the device after a while and the main() function is called.
 * A message is printed to indicate that the system was resumed from standby mode and not
 * from a reset.
 *
 *
 * Note: The LPTIM1 can't be used in standby mode to count the pulses
 */
static void test_standby_mode()
{
  uint8_t buf[32];

  // Check and handle if the system was resumed from StandBy mode
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
    snprintf((char*)buf, 32, "System resumed from standy\n\r");
    HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

    // Clear Standby flag
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  }

  // run for a while
  for(int i = 0; i < 200; i++) {
    snprintf((char*)buf, 32, "Normal mode, i=%u\n\r", i);
    HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
  }

  // disable vcc sensor (and also RF if JP1 connects V_SENSOR with VDD_FR)
  disable_vcc_sensor();

  // set RTC wakeup
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 15, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

  snprintf((char*)buf, 32, "Go to standby mode...\n\r");
  HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);

  // go to standby
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);  // clear wakeup flag
  HAL_PWR_EnterSTANDBYMode();

  // after wake up (should not arrive in standby mode because the main() function is called after wake up)
  snprintf((char*)buf, 32, "Waking up, pulse=%lu\n\r", HAL_LPTIM_ReadCounter(&hlptim1));
  HAL_UART_Transmit(&hlpuart1, buf, strlen((char*)buf), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
