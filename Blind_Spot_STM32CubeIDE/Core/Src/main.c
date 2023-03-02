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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char hex_str[9];
uint8_t rx_buffer[9] = {0}; // Receive buffer
const int RX_BUFFER_SIZE = 9;
extern uint16_t rx_index;
extern uint8_t flag_uart_data_received;
uint8_t start_byte_detected = 0;

//----------Beginning of LiDAR commands----------
//uint8_t getVersionCMD[] = {0x5A, 0x04, 0x01, 0x00}; //Get version (0x01)
uint8_t softResetCMD[] = {0x5A, 0x04, 0x02, 0x00}; //Soft reset (0x02)
uint8_t sampleFreq0HzCMD[] = {0x5A, 0x06, 0x03, 0x00, 0x00, 0x00}; //Sample frequency 0Hz (0x03)
uint8_t sampleFreq10HzCMD[] = {0x5A, 0x06, 0x03, 0x0A, 0x00, 0x00}; //Sample frequency 10Hz (0x03)
//uint8_t sampleFreq250HzCMD[] = {0x5A, 0x06, 0x03, 0xFA, 0x00, 0x00}; //Sample frequency 250Hz (0x03)
uint8_t sampleTriggerCMD[] = {0x5A, 0x04, 0x04, 0x00}; //Sample trigger (0x04)
uint8_t outForm9B_cmCMD[] = {0x5A, 0x05, 0x05, 0x01, 0x00}; //Output format 9-byte/cm (0x05)
uint8_t outForm8B_cmCMD[] = {0x5A, 0x05, 0x05, 0x09, 0x00}; //Output format 8-byte/cm (0x05)
uint8_t baud9600CMD[] = {0x5A, 0x08, 0x06, 0x80, 0x25, 0x00, 0x00, 0x00}; //Baud rate 115200 (0x06)
//uint8_t baud19200CMD[] = {0x5A, 0x08, 0x06, 0x00, 0x4B, 0x00, 0x00, 0x00}; //Baud rate 115200 (0x06)
//uint8_t baud38400CMD[] = {0x5A, 0x08, 0x06, 0x00, 0x96, 0x00, 0x00, 0x00}; //Baud rate 115200 (0x06)
//uint8_t baud57600CMD[] = {0x5A, 0x08, 0x06, 0x00, 0xE1, 0x00, 0x00, 0x00}; //Baud rate 115200 (0x06)
uint8_t baud115200CMD[] = {0x5A, 0x08, 0x06, 0x00, 0xC2, 0x01, 0x00, 0x00}; //Baud rate 115200 (0x06)
//uint8_t outputEnableCMD[] = {0x5A, 0x05, 0x07, 0x01, 0x00}; //Output enable (0x07)
//uint8_t outputDisableCMD[] = {0x5A, 0x05, 0x07, 0x00, 0x00}; //Output disable (0x07)
//uint8_t checksumEnableCMD[] = {0x5A, 0x05, 0x08, 0x01, 0x00}; //Enable checksum comparison (0x08)
//uint8_t checksumDisableCMD[] = {0x5A, 0x05, 0x08, 0x00, 0x67}; //Disable checksum comparison (0x08)
//uint8_t resDefSetCMD[] = {0x5A, 0x04, 0x10, 0x00}; //Restore default settings (0x10)
uint8_t saveSettingsCMD[] = {0x5A, 0x04, 0x11, 0x00}; //Save current settings (0x11)
//uint8_t barcodeCMD[] = {0x5A, 0x04, 0x12, 0x00}; //Output product barcode (0x12)
//uint8_t getFullVersionCMD[] = {0x5A, 0x04, 0x14, 0x00}; //Get full version (0x14)
uint8_t powerSavingEN1HzCMD[] = {0x5A, 0x06, 0x35, 0x01, 0x00, 0x00}; //Enable power saving mode and measure at 1Hz (0x35)
uint8_t powerSavingEN10HzCMD[] = {0x5A, 0x06, 0x35, 0x0A, 0x00, 0x00}; //Enable power saving mode and measure at 10Hz (0x35)
uint8_t ultraLowPwrModeEnableCMD[] = {0x5A, 0x05, 0x58, 0x01, 0x00}; //Enable ultra low power mode (0x58)
uint8_t ultraLowPwrModeDisableCMD[] = {0x5A, 0x05, 0x58, 0x00, 0x00}; //Disable ultra low power mode - Send 5 times repeatedly (0x58)
//----------End of LiDAR commands----------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void lidar_init();
void data_to_hex_str(uint8_t *data, uint8_t data_len, char *hex_str);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
   __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

  lidar_init();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(start_byte_detected == 0){
//		  memset(rx_buffer, 0, RX_BUFFER_SIZE); // Reset the rx_buffer with 0s
		  HAL_UART_Receive(&huart2, (uint8_t*)&rx_buffer[0], 1, 1000); // Receive a single byte in blocking

		  char msg[] = "\n\rChecking byte: ";
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		  // Send the byte to serial terminal
		  HAL_UART_Transmit(&huart2, (uint8_t*)&rx_buffer[0], sizeof(rx_buffer[0]), HAL_MAX_DELAY);

		  // Send the entire rx_buffer to serial terminal
		  data_to_hex_str(rx_buffer, sizeof(rx_buffer), hex_str);
		  HAL_UART_Transmit(&huart2, (uint8_t*)hex_str, 2 * RX_BUFFER_SIZE, 100);

		  // Checks if the byte received is 0x59
		  if(rx_buffer[0] == 0x59){
				  // Turn on LED for a second
				  HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
				  start_byte_detected = 1; // Set the flag for start byte being found
			  }
	  } else {
		  // Receive the remaining 8 bytes
		  for(int i = 1; i < RX_BUFFER_SIZE; i++){
				  HAL_UART_Receive(&huart2, &rx_buffer[i], 1, 100);
			  }

		  // Send the rx_buffer to serial terminal
		  char msg0[] = "\n\rReceived: ";
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg0, strlen(msg0), HAL_MAX_DELAY);
		  data_to_hex_str(rx_buffer, RX_BUFFER_SIZE, hex_str);
		  HAL_UART_Transmit(&huart2, (uint8_t*)hex_str, strlen(hex_str), 100);
		  HAL_Delay(500);
		  start_byte_detected = 0;
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MISC_GPIO3_Pin|MISC_GPIO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ACCEL_SPI1_CS_Pin|MISC_GPIO1_Pin|LED_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MISC_GPIO2_GPIO_Port, MISC_GPIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MISC_GPIO3_Pin MISC_GPIO4_Pin */
  GPIO_InitStruct.Pin = MISC_GPIO3_Pin|MISC_GPIO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ON_OFF_MODE_Pin ACCEL_INT2_Pin ACCEL_INT1_Pin */
  GPIO_InitStruct.Pin = ON_OFF_MODE_Pin|ACCEL_INT2_Pin|ACCEL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ACCEL_SPI1_CS_Pin MISC_GPIO1_Pin */
  GPIO_InitStruct.Pin = ACCEL_SPI1_CS_Pin|MISC_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MISC_GPIO2_Pin */
  GPIO_InitStruct.Pin = MISC_GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MISC_GPIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_OUT_Pin */
  GPIO_InitStruct.Pin = LED_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void data_to_hex_str(uint8_t *data, uint8_t data_len, char *hex_str)
{
    for (int i = 0; i < data_len; i++)
    {
        hex_str[2 * i]     = data[i] >> 4;
        hex_str[2 * i + 1] = data[i] & 0x0f;
        hex_str[2 * i]     += hex_str[2 * i] > 9 ? 'A' - 10 : '0';
        hex_str[2 * i + 1] += hex_str[2 * i + 1] > 9 ? 'A' - 10 : '0';
    }
    hex_str[2 * data_len] = '\0';
}

void lidar_init(){
	//Send soft reset command
	HAL_Delay(10000);
	HAL_UART_Transmit(&huart2, softResetCMD, sizeof(softResetCMD), 100);
	HAL_UART_Receive(&huart2, rx_buffer, 5, 3000);

	// Send message
	char msg1[] = "\n\rChecking return: ";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg1, strlen(msg1), HAL_MAX_DELAY);

	// Send rx_buffer
	HAL_UART_Transmit(&huart2, (uint8_t*)rx_buffer, sizeof(rx_buffer), 100);
	HAL_Delay(500);

	// Send a 1
	HAL_UART_Transmit(&huart2, (uint8_t*)"1", 1, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, baud9600CMD, sizeof(baud9600CMD), 100);
	HAL_Delay(500);

	//Send sample at 0Hz command
	HAL_UART_Transmit(&huart2, sampleFreq10HzCMD, sizeof(sampleFreq10HzCMD), 100);
	HAL_Delay(500);

	//Send output format 9-byte/cm
	HAL_UART_Transmit(&huart2, outForm9B_cmCMD, sizeof(outForm9B_cmCMD), 100);
	HAL_Delay(500);

	// Save
	HAL_UART_Transmit(&huart2, saveSettingsCMD, sizeof(saveSettingsCMD), 100);
	HAL_Delay(500);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == huart2.Instance){
		HAL_Delay(100);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);
	char msg[] = "Sent___\r\n";
	if(huart->Instance == huart2.Instance){
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, 9, HAL_MAX_DELAY);
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
