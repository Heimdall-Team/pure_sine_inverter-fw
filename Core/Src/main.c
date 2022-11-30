/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwr.h"
#include "display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define samples 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t sine_lut[samples] =
{
		0, 13, 25, 38, 50, 63, 75, 88, 100, 113,
		 125, 138, 150, 163, 175, 187, 200, 212,
		 224, 236, 249, 261, 273, 285, 297, 309,
		 321, 333, 345, 356, 368, 380, 391, 403,
		 415, 426, 437, 449, 460, 471, 482, 493,
		 504, 515, 526, 537, 547, 558, 568, 579,
		 589, 599, 609, 619, 629, 639, 649, 659,
		 668, 678, 687, 696, 705, 714, 723, 732,
		 741, 750, 758, 766, 775, 783, 791, 799,
		 806, 814, 822, 829, 836, 844, 851, 858,
		 864, 871, 877, 884, 890, 896, 902, 908,
		 914, 919, 925, 930, 935, 940, 945, 950,
		 954, 959, 963, 967, 971, 975, 979, 983,
		 986, 989, 992, 995, 998, 1001, 1003, 1006,
		 1008, 1010, 1012, 1014, 1015, 1017, 1018,
		 1019, 1020, 1021, 1022, 1022, 1023, 1023,
		 1023, 1023, 1023, 1022, 1022, 1021, 1020,
		 1019, 1018, 1017, 1015, 1014, 1012, 1010,
		 1008, 1006, 1003, 1001, 998, 995, 992,
		 989, 986, 983, 979, 975, 971, 967, 963,
		 959, 954, 950, 945, 940, 935, 930, 925,
		 919, 914, 908, 902, 896, 890, 884, 877,
		 871, 864, 858, 851, 844, 836, 829, 822,
		 814, 806, 799, 791, 783, 775, 766, 758,
		 750, 741, 732, 723, 714, 705, 696, 687,
		 678, 668, 659, 649, 639, 629, 619, 609,
		 599, 589, 579, 568, 558, 547, 537, 526,
		 515, 504, 493, 482, 471, 460, 449, 437,
		 426, 415, 403, 391, 380, 368, 356, 345,
		 333, 321, 309, 297, 285, 273, 261, 249,
		 236, 224, 212, 200, 187, 175, 163, 150,
		 138, 125, 113, 100, 88, 75, 63, 50, 38, 25, 13,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
};

uint32_t sine_lut_neg[samples] =
{
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
		0, 13, 25, 38, 50, 63, 75, 88, 100, 113,
		125, 138, 150, 163, 175, 187, 200, 212,
		 224, 236, 249, 261, 273, 285, 297, 309,
		 321, 333, 345, 356, 368, 380, 391, 403,
		 415, 426, 437, 449, 460, 471, 482, 493,
		 504, 515, 526, 537, 547, 558, 568, 579,
		 589, 599, 609, 619, 629, 639, 649, 659,
		 668, 678, 687, 696, 705, 714, 723, 732,
		 741, 750, 758, 766, 775, 783, 791, 799,
		 806, 814, 822, 829, 836, 844, 851, 858,
		 864, 871, 877, 884, 890, 896, 902, 908,
		 914, 919, 925, 930, 935, 940, 945, 950,
		 954, 959, 963, 967, 971, 975, 979, 983,
		 986, 989, 992, 995, 998, 1001, 1003, 1006,
		 1008, 1010, 1012, 1014, 1015, 1017, 1018,
		 1019, 1020, 1021, 1022, 1022, 1023, 1023,
		 1023, 1023, 1023, 1022, 1022, 1021, 1020,
		 1019, 1018, 1017, 1015, 1014, 1012, 1010,
		 1008, 1006, 1003, 1001, 998, 995, 992,
		 989, 986, 983, 979, 975, 971, 967, 963,
		 959, 954, 950, 945, 940, 935, 930, 925,
		 919, 914, 908, 902, 896, 890, 884, 877,
		 871, 864, 858, 851, 844, 836, 829, 822,
		 814, 806, 799, 791, 783, 775, 766, 758,
		 750, 741, 732, 723, 714, 705, 696, 687,
		 678, 668, 659, 649, 639, 629, 619, 609,
		 599, 589, 579, 568, 558, 547, 537, 526,
		 515, 504, 493, 482, 471, 460, 449, 437,
		 426, 415, 403, 391, 380, 368, 356, 345,
		 333, 321, 309, 297, 285, 273, 261, 249,
		 236, 224, 212, 200, 187, 175, 163, 150,
		 138, 125, 113, 100, 88, 75, 63, 50, 38, 25, 13,
};

uint32_t destaddr = (uint32_t) &(TIM1->CCR1);
uint32_t destaddr2 = (uint32_t) &(TIM1->CCR2);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  pwr_set_gate_driver_off();
  //pwr_init();
 // display_init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
  HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)sine_lut, destaddr, samples);
  HAL_DMA_Start_IT(&hdma_tim4_ch2, (uint32_t)sine_lut_neg, destaddr2, samples);
  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
