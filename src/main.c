/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receiveESP[10];
/* USER CODE END PV */
int Selection = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

  /* USER CODE END 2 */
  Inverse_Kinematics_Mecanum(0, 0, 0, 0);
  /* Infinite loop */
  HAL_UART_Receive_IT(&huart2, receiveESP, 1);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Inverse_Kinematics_Omni(400, 0, 0);
    HAL_Delay(1000);
    Inverse_Kinematics_Omni(0, 400, 0);
    HAL_Delay(1000);
    Inverse_Kinematics_Omni(0, -400, 0);
    HAL_Delay(1000);
    Inverse_Kinematics_Omni(-400, 0, 0);
    HAL_Delay(1000);
    Inverse_Kinematics_Omni(0, 0, 0);
    HAL_Delay(10000);

  /**switch (receiveESP[0]) {
		case 1: // MAJU
			Inverse_Kinematics_Mecanum(2000, 0, 0, 0);
			Selection = 1;
			break;

		case 2: // MUNDUR
			Inverse_Kinematics_Mecanum(-2000, 0, 0, 0);
			Selection = 2;
			break;

		case 3: // GESER KIRI
			Inverse_Kinematics_Mecanum(0, 2000, 0, 0);
			Selection = 3;
			break;

		case 4: // GESER KANAN
			Inverse_Kinematics_Mecanum(0, -2000, 0, 0);
			Selection = 4;
			break;

		case 5: // PUTAR KIRI (rotasi CCW di tempat)
			Inverse_Kinematics_Mecanum(0, 0, -40, 0);
			break;

		case 6: // PUTAR KANAN (rotasi CW di tempat)
			Inverse_Kinematics_Mecanum(0, 0, 40, 0);
			break;

		case 7: // DIAGONAL MAJU-KIRI
			Inverse_Kinematics_Mecanum(700, -700, 0, 0);
			break;

		case 8: // DIAGONAL MAJU-KANAN
			Inverse_Kinematics_Mecanum(700, 700, 0, 0);
			break;

		case 9: // DIAGONAL MUNDUR-KIRI
			Inverse_Kinematics_Mecanum(-700, -700, 0, 0);
			break;

		case 10: // DIAGONAL MUNDUR-KANAN
			Inverse_Kinematics_Mecanum(-700, 700, 0, 0);
			break;

		default:
			// Blok ini berjalan saat ESP32 tidak sedang mengirim kode gerak
			// aktif (mis. semua tombol dilepas). Sebelum benar-benar
			// berhenti, robot diberi pulsa singkat ke arah BERLAWANAN dari
			// gerakan terakhirnya (auto-brake) untuk meredam sisa
			// momentum/inersia agar tidak "ngesot".
			if (Selection == 1) {
				Inverse_Kinematics_Mecanum_Decay(-1000, 0, 0, 0);
				HAL_Delay(500);
				Selection = 0;
			}
			if (Selection == 2) {
				Inverse_Kinematics_Mecanum_Decay(1000, 0, 0, 0);
				HAL_Delay(500);
				Selection = 0;
			}
			if (Selection == 3) {
				Inverse_Kinematics_Mecanum_Decay(0, -1000, 0, 0);
				HAL_Delay(500);
				Selection = 0;
			}
			if (Selection == 4) {
				Inverse_Kinematics_Mecanum_Decay(0, 1000, 0, 0);
				HAL_Delay(500);
				Selection = 0;
			}
			Inverse_Kinematics_Mecanum(0, 0, 0, 0);
			break;
	}*/    

  }  
    /* USER CODE BEGIN 3 */
}


  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // receiveESP[0] sudah terisi data baru di sini
        HAL_UART_Receive_IT(&huart2, receiveESP, 1);  // re-arm supaya siap terima byte berikutnya
    }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
