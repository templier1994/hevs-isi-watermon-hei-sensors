/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
Description: Generic lora driver implementation
License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "timeServer.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "lora.h"
#include "stdio.h"
#include "stm32l0xx_hal.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LORAWAN_MAX_BAT   254

/**
 * @brief LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/**
 * When fast wake up is enabled, the mcu wakes up in ~20us and
 * does not wait for the VREFINT to be settled. THis is ok for
 * most of the case except when adc must be used in this case before
 * starting the adc, you must make sure VREFINT is settled
 */
#define ENABLE_FAST_WAKEUP

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);
/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined(void);
/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass(DeviceClass_t Class);

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded(void);

/* callback to get the battery level in % of full charge (254 full charge, 0 no charge)*/
static uint8_t LORA_GetBatteryLevel(void);

/* tx timer callback function*/
static void LoraMacProcessNotify(void);

static void LORA_McpsDataConfirm(void);

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks = { LORA_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LoraRxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify,
                                                LORA_McpsDataConfirm
                                              };
LoraFlagStatus LoraMacProcessRequest = LORA_RESET;
/**
 * Initialises the Lora Parameters
 */

#define LORAWAN_APP_PORT	 								2
#define LORAWAN_APP_DATA_BUFF_SIZE                          64
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           		LORAWAN_UNCONFIRMED_MSG
#define LORAWAN_DEFAULT_CLASS                       		CLASS_A
#define LORAWAN_DEFAULT_DATA_RATE 							DR_0
#define LORAWAN_ADR_STATE 									LORAWAN_ADR_ON
#define APP_TX_DUTYCYCLE                            		240000 //2m

static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE, LORAWAN_DEFAULT_DATA_RATE, LORAWAN_PUBLIC_NETWORK };

static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];
lora_AppData_t AppData = { AppDataBuff,  0, 0 };
static void sendMsg(void *context, uint8_t BufToSend[]);

/*Timer*/
static TimerEvent_t TxTimer;
/* tx timer callback function*/
static void OnTxTimerEvent(void *context);
/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded(void);
/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);
LoraFlagStatus AppProcessRequest = LORA_RESET;



/* Private functions ---------------------------------------------------------*/

/* UART ultrasound sensors*/
UART_HandleTypeDef huart1;

static void MX_USART1_UART_Init(void);

#define rxBuf_size 		102 	//5
uint8_t rxBuf[rxBuf_size];
HAL_StatusTypeDef UART1status;



/*RTC------------------*/
#define SLEEPTIME 				15
RTC_HandleTypeDef hrtc;
static void MX_RTC_Init(void);
static void test_stop_mode();
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);


/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  /* STM32 HAL library initialization*/
  HAL_Init();

  /* Configure the system clock*/
  SystemClock_Config();

  /* Configure the hardware*/
  HW_Init();
  MX_RTC_Init();

  /*Pins enable : 11 relais, 12 radio*/
 GPIO_InitTypeDef x;
 x.Pin   = GPIO_PIN_11 | GPIO_PIN_12;
 x.Mode  = GPIO_MODE_OUTPUT_PP;

 RCC_GPIO_CLK_ENABLE((uint32_t)GPIOA);
 HAL_GPIO_Init(GPIOA, &x);

 HAL_GPIO_WritePin(GPIOA, x.Pin, 0);

  CMD_Init();

  PPRINTF("ATtention command interface\n\r");

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

//  PRINTF("LORA_JOIN()... wait 10s \n\r");
//  LORA_Join(); //this function take ~10s,
//  LoraStartTx(TX_ON_TIMER) ;

//  MX_USART1_UART_Init();

  /* main loop*/
  while (1)
  {
	  PRINTF("wait 5s before sleep \n\r");

	  HAL_Delay(5000);




	  test_stop_mode();

//-----------------------------------------------------------------------------
//	  if (AppProcessRequest == LORA_SET && LORA_JoinStatus() == LORA_SET)
//	  {
//		PRINTF("LoRa routine \n\r");

		/*get uart msg (ultrasonic sensor)*/

//		 HAL_GPIO_WritePin(GPIOA, x.Pin, 1);


//		 while(rxBuf[4]==0){//
//		   UART1status = HAL_UART_Receive(&huart1, (uint8_t *)rxBuf, rxBuf_size, HAL_MAX_DELAY); //HAL_MAX_DELAY
//		 }
//		   PRINTF("%s",&rxBuf);
//		 HAL_GPIO_WritePin(GPIOA, x.Pin, 0);

		/*get i2c msg (pressure sensor)*/

		/*reset notification flag*/
//		AppProcessRequest = LORA_RESET;
		/*SendMsg*/
//		sendMsg(NULL, rxBuf);
		//memset(rxBuf, 0 , sizeof(rxBuf));
//		for(uint8_t i = 0; i< rxBuf_size; i++){
//			rxBuf[i]=0;
//		}

//	}
    /* Handle UART commands */
//      CMD_Process();
//    if (LoraMacProcessRequest == LORA_SET)
//    {
      /*reset notification flag*/
//      LoraMacProcessRequest = LORA_RESET;
//      LoRaMacProcess();
//    }


  }
}


static void LoraRxData(lora_AppData_t *AppData)
{
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  Error_Handler();
}
#endif

void LoraMacProcessNotify(void)
{
  LoraMacProcessRequest = LORA_SET;
}

static void LORA_HasJoined(void)
{
  PRINTF("JOINED\n\r");
}

static void LORA_ConfirmClass(DeviceClass_t Class)
{
  PRINTF("switch to class %c done\n\r", "ABC"[Class]);
}


/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t LORA_GetBatteryLevel(void)
{
  uint16_t batteryLevelmV;
  uint8_t batteryLevel = 0;

  batteryLevelmV = HW_GetBatteryLevel();


  /* Convert batterey level from mV to linea scale: 1 (very low) to 254 (fully charged) */
  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }

  return batteryLevel;
}

static void LORA_McpsDataConfirm(void)
{
  PRINTF("Network Server \"ack\" an uplink data confirmed message transmission\n\r");
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/************************ LoRa Part****/
/**
 * Send a message to TTN
 */
static void sendMsg(void *context, uint8_t bufToSend[]){

	//if not joined, rejoind the network
	if(LORA_JoinStatus() != LORA_SET){
		LORA_Join();
		return;
	}

	/*Ultrasound part*/
	int8_t ultrasound = 0;

	for(uint8_t i = 50; i<rxBuf_size-13; i++){ // 50 is ~ the end of the header
			if(bufToSend[i] == 'R' && bufToSend[i+1] == '0'&& bufToSend[i+2] == '0' && bufToSend[i+3] == '0'){
				char hundred[2]={bufToSend[i+11],0};
				char dozen[2]={bufToSend[i+12],0};
				char unit[2]={bufToSend[i+13],0};

				int valHundred;
				int valDozen;
				int valUnit;

				valHundred = atoi(hundred);
				valDozen = atoi(dozen);
				valUnit = atoi(unit);
				ultrasound= valHundred*100 + valDozen * 10 + valUnit;
				break;
			}
		}
	/*End UltraSound*/
	/*Pressure Part*/

	/*End pressure part*/

	uint8_t batteryLevel ;

	batteryLevel = LORA_GetBatteryLevel();

	AppData.Port = LORAWAN_APP_PORT;
	uint32_t i = 0;

	AppData.Buff[i++] = batteryLevel;
	AppData.Buff[i++] = ultrasound;

	AppData.BuffSize = i;

	LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	PRINTF("Main.c : sendMsg() \n\r");
}

static void OnTxTimerEvent(void *context)
{
	PRINTF("OnTxTimerEvent \n\r");
  /*Wait for next tx slot*/
  TimerStart(&TxTimer);

  AppProcessRequest = LORA_SET;
}

static void LoraStartTx(TxEventType_t EventType)
{
	PRINTF("LoraStartTx \n\r");

  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */

    TimerInit(&TxTimer, OnTxTimerEvent);
    TimerSetValue(&TxTimer,  APP_TX_DUTYCYCLE);
    OnTxTimerEvent(NULL);
  }

}
static void LORA_TxNeeded(void)
{
	PRINTF("LORA_TxNeeded \n\r");

  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

/************************ End LoRa Part****/

/************************ Uart 1 Part : ultrasound sensor****/
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart1.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, SLEEPTIME, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}



/**
  * @brief  RTC Wake Up callback
  * @param  None
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}



/**
 *
 */
static void test_stop_mode()
{
	PRINTF("test_stop_mode() \n\r");
	HAL_Delay(100);
	//clear rtc event flag
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

    // set RTC wakeup
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, SLEEPTIME, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

    // go to stop mode
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);  // clear wakeup flag


    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // go in stop mode

    //after wake up
    SystemClock_Config();

    PRINTF("wakeUp  \n\r");
}


