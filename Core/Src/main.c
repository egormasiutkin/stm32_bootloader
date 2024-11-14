/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define APP1_ADDRESS 0x08002000
#define NEW_FIRMWARE 0x08040000
#define FIRMWARE_LEN 0x08040004
#define FIRMWARE_CRC 0x08040008
#define APP2_ADDRESS 0x08040800
typedef  void (*pFunction)(void);
pFunction JumpToApplication;
uint32_t JumpAddress;
uint32_t crc_orig, crc_calc;
uint32_t idx, len;
uint32_t PageError = 0;
FLASH_EraseInitTypeDef EraseInitStruct = {0};
uint32_t getCRC(uint8_t* data, uint32_t len);
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
  /* USER CODE BEGIN 2 */
  PageError = (*(__IO uint32_t*)NEW_FIRMWARE);
    if ((*(__IO uint32_t*)NEW_FIRMWARE) == 0x1234ABCD)
  	{
    	HAL_FLASH_Unlock();
  		len = (*(__IO uint32_t*)FIRMWARE_LEN);
  		crc_orig = (*(__IO uint32_t*)FIRMWARE_CRC);
  		crc_calc = getCRC((uint8_t*)APP2_ADDRESS, len);
  		if(crc_orig == crc_calc){

			EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
			EraseInitStruct.Banks = FLASH_BANK_1;
			EraseInitStruct.Page        = 4;
			EraseInitStruct.NbPages     = 124;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
			for(idx = 0;idx<len;idx+=8)
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP1_ADDRESS + idx, *(__IO uint64_t*)(APP2_ADDRESS + idx));
			for(idx = 0;idx<len;idx+=4)
			{
				crc_orig = (*(__IO uint32_t*)(APP1_ADDRESS + idx));
				crc_calc = (*(__IO uint32_t*)(APP2_ADDRESS + idx));
				if(crc_orig != crc_calc)
				{
					// validation error
					while(1)
					{
						HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
						HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
						HAL_Delay(100);
					}
				}
			}
  		}
  		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_MASS;
  		EraseInitStruct.Banks = FLASH_BANK_2;
		HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
		HAL_FLASH_Lock();
  	}
	if (((*(__IO uint32_t*)APP1_ADDRESS) & 0x2FF00000 ) == 0x20000000)
	{
		JumpAddress = *(__IO uint32_t*) (APP1_ADDRESS + 4);
		JumpToApplication = (pFunction) JumpAddress;
		__set_MSP(*(__IO uint32_t*) APP1_ADDRESS);
		JumpToApplication();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // There is no app
	  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t getCRC(uint8_t* data, uint32_t len)
{
	static uint32_t crcTable[256];
	static int tableGenerated = 0;

	if (!tableGenerated) {
		// Generate the CRC table
		uint32_t crc;
		for (int i = 0; i < 256; i++) {
			crc = i;
			for (int j = 8; j > 0; j--) {
				if (crc & 1) {
					crc = (crc >> 1) ^ 0xEDB88320;
				} else {
					crc >>= 1;
				}
			}
			crcTable[i] = crc;
		}
		tableGenerated = 1;
	}
	// Calculate the CRC
	uint32_t i = 0;
	uint32_t crc = 0xFFFFFFFF;
	while (len--) {
		crc = (crc >> 8) ^ crcTable[(crc ^ data[i]) & 0xFF];
		i++;
	}
	return crc ^ 0xFFFFFFFF;
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
