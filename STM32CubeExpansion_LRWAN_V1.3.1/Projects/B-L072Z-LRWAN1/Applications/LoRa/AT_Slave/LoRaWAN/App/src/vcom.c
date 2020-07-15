/**
  ******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @brief   manages virtual com port
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

#include "hw.h"
#include "vcom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Uart Handle */
static UART_HandleTypeDef UartHandle;

uint8_t charRx;

static void (*TxCpltCallback)(void);

static void (*RxCpltCallback)(uint8_t *rxChar);
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
void vcom_Init(void (*TxCb)(void))
{

  /*Record Tx complete for DMA*/
  TxCpltCallback = TxCb;
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void vcom_Trace(uint8_t *p_data, uint16_t size)
{
  HAL_UART_Transmit_DMA(&UartHandle, p_data, size);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* buffer transmission complete*/
  if (NULL != TxCpltCallback)
  {
    TxCpltCallback();
  }
}

void vcom_ReceiveInit(void (*RxCb)(uint8_t *rxChar))
{
  UART_WakeUpTypeDef WakeUpSelection;

  /*record call back*/
  RxCpltCallback = RxCb;

  /*Set wakeUp event on start bit*/
  WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
//
  HAL_UARTEx_StopModeWakeUpSourceConfig(&UartHandle, WakeUpSelection);

  /*Enable wakeup from stop mode*/
  HAL_UARTEx_EnableStopMode(&UartHandle);

  /*Start LPUART receive on IT*/
  HAL_UART_Receive_IT(&UartHandle, &charRx, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if ((NULL != RxCpltCallback) && (HAL_UART_ERROR_NONE == UartHandle->ErrorCode))
  {
    RxCpltCallback(&charRx);
  }
  HAL_UART_Receive_IT(UartHandle, &charRx, 1);
}

void vcom_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

void vcom_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);
}

void vcom_DeInit(void)
{
  HAL_UART_DeInit(&UartHandle);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	  {
		  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* USER CODE BEGIN USART1_MspInit 0 */

	  /* USER CODE END USART1_MspInit 0 */
	    /* Peripheral clock enable */
	    __HAL_RCC_USART1_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    /**USART1 GPIO Configuration
	    PA10     ------> USART1_RX
	    PA9     ------> USART1_TX
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* USER CODE BEGIN USART1_MspInit 1 */

	  /* USER CODE END USART1_MspInit 1 */
	  }else{
  static DMA_HandleTypeDef hdma_tx;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  /* Enable USARTx clock */
  USARTx_CLK_ENABLE();
  /* select USARTx clock source*/
  RCC_PeriphCLKInitTypeDef  PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /* Enable DMA clock */
  DMAx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART  pin configuration  */
  vcom_IoInit();

  /*##-3- Configure the DMA ##################################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
#ifndef STM32L152xE
  hdma_tx.Init.Request             = USARTx_TX_DMA_REQUEST;
#endif
  HAL_DMA_Init(&hdma_tx);

  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA(huart, hdmatx, hdma_tx);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt*/
  HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, USARTx_Priority, 1);
  HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);

  /* NVIC for USART, to catch the TX complete */
  HAL_NVIC_SetPriority(USARTx_IRQn, USARTx_DMA_Priority, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
	  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  vcom_IoDeInit();
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-3- Disable the DMA #####################################################*/
  /* De-Initialize the DMA channel associated to reception process */
  if (huart->hdmarx != 0)
  {
    HAL_DMA_DeInit(huart->hdmarx);
  }
  /* De-Initialize the DMA channel associated to transmission process */
  if (huart->hdmatx != 0)
  {
    HAL_DMA_DeInit(huart->hdmatx);
  }

  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
}

void vcom_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}

void vcom_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  USARTx_TX_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  GPIO_InitStructure.Pin =  USARTx_TX_PIN ;
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin =  USARTx_RX_PIN ;
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
