/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM4)
  {
    /* USER CODE BEGIN TIM4_MspDeInit 0 */
 
    /* USER CODE END TIM4_MspDeInit 0 */
    __HAL_RCC_TIM4_CLK_DISABLE();
    /* USER CODE BEGIN TIM4_MspDeInit 1 */
 
    /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM12)
  {
    /* USER CODE BEGIN TIM12_MspDeInit 0 */
 
    /* USER CODE END TIM12_MspDeInit 0 */
    __HAL_RCC_TIM12_CLK_DISABLE();
    /* USER CODE BEGIN TIM12_MspDeInit 1 */
 
    /* USER CODE END TIM12_MspDeInit 1 */
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (huart->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */
 
    /* USER CODE END USART2_MspInit 0 */
 
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
 
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
    /* ===================================================================
     * INI BAGIAN YANG DIBAHAS DI PERTANYAANMU: mengaktifkan interrupt
     * USART2 di NVIC. Tanpa 2 baris ini, HAL_UART_Receive_IT() di main.c
     * tidak akan pernah memicu USART2_IRQHandler(), sehingga
     * HAL_UART_RxCpltCallback() tidak akan pernah terpanggil meskipun
     * ESP32 sudah mengirim data dengan benar ke pin RX (PA3).
     * =================================================================== */
    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspInit 1 */
 
    /* USER CODE END USART2_MspInit 1 */
  }
}
 
/**
  * @brief	UART MSP De-Initialization.
  * @param	huart Pointer ke handle UART.
  * @retval	None
  */
