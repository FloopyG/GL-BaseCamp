/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
pca9685_handle handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t UART3_rxBuffer[UART3_RX_BUFFER_SIZE] = {0};
uint8_t UART3_currentIndex = 0;

uint8_t doneCommandStr[26] = " - executed successfully!\n";
uint8_t wrongCommandStr[16] = "Invalid command\n";

uint32_t test = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void handleCommand (uint8_t* command);
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  pca9685_handle handle = {
        .i2c_handle = &hi2c1,
        .device_address = PCA9685_I2C_DEFAULT_DEVICE_ADDRESS,
    };

  PCA9685_Init(&handle);
  PCA9685_setDutyCycle(&handle, 90, 0);

  HAL_UART_Receive_IT(&huart3, (uint8_t*)UART3_rxBuffer, 1);

  PCA9685_setDutyCycle(&handle, 30, 0);
  uint32_t tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  tick = HAL_GetTick();
	  if (tick == 5000)
	  {
		  PCA9685_setDutyCycle(&handle, 75, 0);
	  }
	  if (test)
	  {
		  handleCommand(UART3_rxBuffer);
		  test = 0;
	      UART3_currentIndex = 0;
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void handleCommand (uint8_t* command)
{
	char* parts[4];
	uint8_t part_index = 0;
	parts[part_index] = strtok((char*) command, " ");
	while (parts[part_index] != NULL && part_index < 4)
	{
	  parts[++part_index] = strtok(NULL, " ");
	}

	if (strcmp(parts[0], "set") == 0)
	{
		if (strcmp(parts[1], "freq") == 0)
		{
			uint8_t value = atoi(parts[2]);
			if (PCA9685_setFrequency(&handle, (float) value) == true)
			{
				HAL_UART_Transmit(&huart3, command, sizeof(command), 100);
				HAL_UART_Transmit(&huart3, doneCommandStr, sizeof(doneCommandStr), 100);
			}
			else
			{
				HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
			}
		}
		else if (strcmp(parts[1], "duty") == 0)
		{
			uint8_t channel = atoi(parts[2]);
			uint8_t value = atoi(parts[3]);
			if (PCA9685_setDutyCycle(&handle, value, channel) == true)
			{
				HAL_UART_Transmit(&huart3, command, sizeof(command), 100);
				HAL_UART_Transmit(&huart3, doneCommandStr, sizeof(doneCommandStr), 100);
			}
			else
			{
				HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
			}
		}
		else
		{
			HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
		}
	}
	else if (strcmp(parts[0], "sleep") == 0)
	{
		if (strcmp(parts[1], "on") == 0)
		{
			if (PCA9685_sleep(&handle) == true)
			{
				HAL_UART_Transmit(&huart3, command, sizeof(command), 100);
				HAL_UART_Transmit(&huart3, doneCommandStr, sizeof(doneCommandStr), 100);
			}
			else
			{
				HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
			}

		}
		else if (strcmp(parts[1], "off") == 0)
		{
			if (PCA9685_wakeUp(&handle) == true)
			{
				HAL_UART_Transmit(&huart3, command, sizeof(command), 100);
				HAL_UART_Transmit(&huart3, doneCommandStr, sizeof(doneCommandStr), 100);
			}
			else
			{
				HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
			}
		}
		else
		{
			HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
		}
	}
	else
	{
		HAL_UART_Transmit(&huart3, wrongCommandStr, sizeof(wrongCommandStr), 100);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)(UART3_rxBuffer + UART3_currentIndex), 1, 100);
  if (UART3_currentIndex < UART3_RX_BUFFER_SIZE)
  {
    if (UART3_rxBuffer[UART3_currentIndex] == '\r' || UART3_rxBuffer[UART3_currentIndex] == '\n')
    {
      // End of command received
    	UART3_rxBuffer[UART3_currentIndex] = '\0';
    	test = 1;
    }
    else
    {
    	UART3_currentIndex++;
    }
  }

  HAL_UART_Receive_IT(&huart3,(uint8_t*)(UART3_rxBuffer + UART3_currentIndex), 1);
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
