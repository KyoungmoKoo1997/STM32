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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define MAX_POS 65535
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
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t txData[1000];
uint32_t txData2[1000];
uint32_t txSine[1000];
uint32_t txStep[1000];
uint32_t rxData[1000];
uint32_t rxData2[1000];
uint32_t rxData4[100];
uint8_t buf[12];
char buffer[40];
char flag_buffer[4];
int transmitFlag = 0;

uint32_t PackPositionData (uint16_t xPos, uint16_t yPos){
	return ((uint32_t)xPos << 16) | yPos;
}

void GenerateStepPositionData (uint16_t step, uint32_t* PositionDataArray, uint16_t arraySize){
	for (uint16_t i=1; i<= arraySize; i++){
			uint16_t x = step * i;
			uint16_t y = step * i;

			PositionDataArray[i-1] = PackPositionData(x, y);
		}
}

void GenerateReversePositionData (uint16_t step, uint32_t* PositionDataArray, uint16_t arraySize){
	for (uint16_t i=1; i<= arraySize; i++){
		uint16_t x = step * i;
		uint16_t y = (step * arraySize) - (step  * i);

		PositionDataArray[i-1] = PackPositionData(x, y);
	}
}

void GenerateSinePositionData(uint32_t* positionDataArray, uint16_t arraySize){
	for(uint16_t i = 0; i < arraySize; i++){
		uint16_t x = (uint16_t)(MAX_POS * sin(1.0 * M_PI * i / arraySize));
		uint16_t y = (uint16_t)(MAX_POS * sin(1.0 * M_PI * i / arraySize));

		positionDataArray[i] = PackPositionData(x, y);
	}
}

void SendData_SAI (uint32_t* positionDataArray, uint16_t arraySize){
	for(uint16_t i=0; i < arraySize; i++){
		HAL_SAI_Transmit(&hsai_BlockA1, &positionDataArray[i], 1, HAL_MAX_DELAY);
	}
}

void SAI_TransmitReceive (uint32_t* Data, uint16_t DataSize){
	// uint32_t counter = 0;
	HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint32_t*)txData, DataSize);
	HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint32_t*)rxData, DataSize);
	/*counter++ ;
	if (counter >=100000){
	for(int i=0; i<1; i++){
	uint16_t xPos = (rxData[i] >> 16) & 0xFFFF;
	uint16_t yPos = rxData[i] & 0xFFFF;

	sprintf(buffer, "(%u, %u)\r\n", xPos, yPos);
	HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), HAL_MAX_DELAY);

	}
	counter = 0;
	}*/
}

void UART_SendData(uint32_t* Data, uint16_t DataSize){
	HAL_UART_Transmit(&huart2, (uint32_t*)Data, DataSize, HAL_MAX_DELAY);
}

//void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai){
	/*hsai->XferCount -= hsai->XferSize;
	    if (hsai->XferCount == 0)
	    {
	        hsai->State = HAL_SAI_STATE_READY;
	    }*/
//}

void HAL_SAI_TxCpltCallback (SAI_HandleTypeDef *hsai){
	if(transmitFlag == 2){
		transmitFlag = 3;
	}
	else if (transmitFlag == 6){
		transmitFlag = 7;
	}
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	if(transmitFlag == 0){
		transmitFlag = 1;
	}
	else if (transmitFlag == 4){
		transmitFlag = 5;
	}
}

void HAL_SAI_RxCpltCallback (SAI_HandleTypeDef *hsai){
	for(int i = 0; i < 50; i++){
	uint16_t xPos = (rxData[i] >> 16) & 0xFFFF;
	uint16_t yPos = rxData[i] & 0xFFFF;
	sprintf(buffer, "(%u, %u)\r\n", xPos, yPos);
	//HAL_UART_Transmit_DMA(&huart2, buffer, strlen((char*)buffer));
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	LED_Toggle();
	if(transmitFlag == 1 | transmitFlag == 2 | transmitFlag == 5 | transmitFlag == 6){
		memcpy(txData, txData2, 500 * sizeof(uint32_t));
	}
	else{
		memcpy(&txData[500], txData2, 500 * sizeof(uint32_t));
	}

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
	//LED_Toggle();
}

void LED_Toggle(void){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

bool isArrayEmpty(uint32_t *arr, uint32_t size){
	for(uint32_t i =0; i < size; i++){
		if(arr[i] > 0){
			return false;
		}
	}
	return true;
}

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */

  //GenerateSinePositionData(txData, 1000);
  GenerateStepPositionData(10, txData, 1000);
  GenerateStepPositionData(0, txData2, 1000);
  GenerateSinePositionData(txSine, 1000);
  GenerateStepPositionData(65, txStep, 1000);


  /*uint32_t txData[4] = {(uint32_t)1, (uint32_t)2, (uint32_t)3, (uint32_t)4};*/

  /*for (int i = 0; i< 1000; i ++){
  txData_current[i] = txData[i];
  }

  for(int i = 0; i < 1000; ++i){
	  txData[i] = 0;
  }*/

  void charsToUint32(char charArray[3200], uint32_t uint32Array[100]) {
      for (int i = 0; i < 3200; ++i) {
    	  // Determine the index of the uint32_t element and the specific bit within that element
    	          int uint32Index = i / 32;
    	          int bitPosition = 31 - (i % 32);

    	          // Set or clear the bit based on the character
    	          if (charArray[i] == '1') {
    	             uint32Array[uint32Index] |= (1U << bitPosition);
    	          }
      }
      //HAL_UART_Transmit(&huart2, "\r\n", 2, 1000);
      //HAL_UART_Transmit(&huart2, &uint32Array[0], 4, HAL_MAX_DELAY);
  }
  //sprintf(flag_buffer, "send2\r\n");
  //HAL_UART_Transmit(&huart2, flag_buffer, strlen((char*)flag_buffer), 1000);

  SAI_TransmitReceive(txData, 1000);

  HAL_UART_Receive_DMA(&huart2, txData2, 500 * sizeof(uint32_t));

  //HAL_UART_Transmit_DMA(&huart2, flag_buffer, strlen((char*)flag_buffer));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(transmitFlag == 1){
		  // Sine half transmitted
		  //HAL_UART_Transmit_DMA(&huart2, flag_buffer, strlen((char*)flag_buffer));
		  //memcpy(&txData, txData2, 500 * sizeof(uint32_t));

		  sprintf(flag_buffer, "1\r\n");
		  HAL_UART_Transmit(&huart2, flag_buffer, strlen((char*)flag_buffer), 1000);
		  transmitFlag = 2;
	  }
	  else if (transmitFlag == 3){
		  //Sine Completely transmitted
		  //HAL_UART_Transmit_DMA(&huart2, flag_buffer, strlen((char*)flag_buffer));
		  //memcpy(&txData[500], txData2, 500 * sizeof(uint32_t));

		  sprintf(flag_buffer, "2\r\n");
		  HAL_UART_Transmit(&huart2, flag_buffer, strlen((char*)flag_buffer), 1000);
		  transmitFlag = 4;
	  }
	  else if (transmitFlag == 5){
		  //Step half Transmitted
		  //HAL_UART_Transmit_DMA(&huart2, flag_buffer, strlen((char*)flag_buffer));
		  //memcpy(txData, txData2, 500 * sizeof(uint32_t));

		  sprintf(flag_buffer, "3\r\n");
		  HAL_UART_Transmit(&huart2, flag_buffer, strlen((char*)flag_buffer), 1000);
	  	  transmitFlag = 6;
	  }
	  else if (transmitFlag == 7){
		  //Step Completely Transmitted
		  //HAL_UART_Transmit_DMA(&huart2, flag_buffer, strlen((char*)flag_buffer));
		  //memcpy(&txData[500], txData2, 500 * sizeof(uint32_t));

		  sprintf(flag_buffer, "4\r\n");
		  HAL_UART_Transmit(&huart2, flag_buffer, strlen((char*)flag_buffer), 1000);
	 	  transmitFlag = 0;
	  }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 18;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_8K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 32;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.FrameInit.FrameLength = 32;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 1;
  hsai_BlockB1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

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
  huart2.Init.BaudRate = 2000000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
