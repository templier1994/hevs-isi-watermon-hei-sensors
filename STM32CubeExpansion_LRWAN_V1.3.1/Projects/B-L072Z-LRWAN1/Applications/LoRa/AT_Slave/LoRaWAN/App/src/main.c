
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

#define LORAWAN_APP_PORT	 								1
#define LORAWAN_APP_DATA_BUFF_SIZE                          64
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           		LORAWAN_UNCONFIRMED_MSG
#define LORAWAN_DEFAULT_CLASS                       		CLASS_A
#define LORAWAN_DEFAULT_DATA_RATE 							DR_0
#define LORAWAN_ADR_STATE 									LORAWAN_ADR_ON
#define APP_TX_DUTYCYCLE                            		120000 //2m

static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE, LORAWAN_DEFAULT_DATA_RATE, LORAWAN_PUBLIC_NETWORK };

static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];
lora_AppData_t AppData = { AppDataBuff,  0, 0 };
static void sendMsg(void *context);

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


  /*pims, enable ultrasound sensor */
	/* enable the relay */
	 GPIO_InitTypeDef x;
	 x.Pin   = GPIO_PIN_11;
	 x.Mode  = GPIO_MODE_OUTPUT_PP;

	 HAL_GPIO_Init(GPIOA, &x);
	 HAL_GPIO_WritePin(GPIOA, x.Pin, 0);
	 /*pims*/


  /* Configure Debug mode */
  DBG_Init();

  /* USER CODE BEGIN 1 */
  CMD_Init();
  /*Disable standby mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

  PPRINTF("ATtention command interface\n\r");
  /* USER CODE END 1 */

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

  //Derectly at the intitialisation join LoRa
  PRINTF("LORA_JOIN()... wait 10s \n\r");
  LORA_Join(); //this function take ~10s, how can i wait till finished

  LoraStartTx(TX_ON_TIMER) ;
  /* main loop*/
  while (1)
  {

	  if (AppProcessRequest == LORA_SET)
	  {
		/*get uart msg (ultrasonic sensor)*/

		/*get i2c msg (pressure sensor)*/

		/*reset notification flag*/
		AppProcessRequest = LORA_RESET;
		/*SendMsg*/
		sendMsg(NULL);
	  }





    /* Handle UART commands */
    CMD_Process();

    if (LoraMacProcessRequest == LORA_SET)
    {
      /*reset notification flag*/
      LoraMacProcessRequest = LORA_RESET;
      LoRaMacProcess();
    }
    /*
     * low power section
     */
    DISABLE_IRQ();
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
    if (LoraMacProcessRequest != LORA_SET)
    {
#ifndef LOW_POWER_DISABLE
      LPM_EnterLowPower();
#endif
    }
    ENABLE_IRQ();

    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
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

//static void LORA_TxNeeded(void)
//{
//  PRINTF("Network Server is asking for an uplink transmission\n\r");
//}

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
/**
 * Send a message to TTN
 */
static void sendMsg(void *context){
	PRINTF("Main.c : sendMsg() \n\r");

	/*Variables to send*/
	int16_t snowHeight = 12;
	uint8_t batteryLevel;

	 if (LORA_JoinStatus() != LORA_SET)
	  {
	    /*Not joined, try again later*/
		// PRINTF("re-join\n\r");
	    LORA_Join();
	    return;
	  }

	batteryLevel = LORA_GetBatteryLevel();

	AppData.Port = LORAWAN_APP_PORT;
	uint32_t i = 0;

	AppData.Buff[i++] = batteryLevel;
	AppData.Buff[i++] = snowHeight;

	AppData.BuffSize = i;

	LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);

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
