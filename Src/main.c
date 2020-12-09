/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
volatile uint8_t pressed1 = 0;//detect if key1 is pressed
volatile uint8_t pressed2 =0;//detect if key2 is pressed
volatile uint8_t state = 0;//record state to control LED blinking; 0: none, 1-4: each row of LED
uint8_t Enter[] = {0x0d,0x0a};
/* Buffer used for reception */
uint8_t uart3_read_buffers[8];
uint8_t uart3_read_cnt=0;
uint8_t uart3_conv;
uint8_t aRxBuffer[20];
uint8_t aTxStartMessage[] = "\r\n****UART-Hyperterminal communication based on IT ****\r\nEnter 10 characters using keyboard :\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_UART_Receive_IT(&huart3, aRxBuffer, 8);
  HAL_UART_Receive_IT(&huart3, &uart3_conv,1);
  //HAL_UART_Transmit_IT(&huart3, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(pressed2==1){//if key2 is pressed
		HAL_TIM_Base_Start_IT(&htim2);//start timer
		pressed2=0;//reset key2
	}

	if(pressed1==1){//if key2 is pressed
		state=0;//reset LED state, turn it off
		pressed1 = 0;//reset key1
	}

	/*detect state to change LED */
	switch(state){
		case 1:
			//LED row1 turn on
			HAL_GPIO_TogglePin(LED8_GPIO_Port, LED8_Pin);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			//LED row4 turn off
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			break;

	  	case 2:
	  		//LED row2 turn on
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_2, GPIO_PIN_SET);
	  		//LED row1 turn off
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_2, GPIO_PIN_RESET);
	  		break;
	  	case 3:
	  		//LED row3 turn on
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_14, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	  		//LED row2 turn off
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_14, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	  		break;
	  	case 4:
	  		//LED row4 turn on
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  		//LED row3 turn off
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  		break;
	  	case 0:
	  		//all LED turn off
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_2, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_14, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  		break;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED13_Pin|LED14_Pin|LED15_Pin|LED6_Pin 
                          |LED7_Pin|LED8_Pin|LED9_Pin|LED10_Pin 
                          |LED11_Pin|LED12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED13_Pin LED14_Pin LED15_Pin LED6_Pin 
                           LED7_Pin LED8_Pin LED9_Pin LED10_Pin 
                           LED11_Pin LED12_Pin */
  GPIO_InitStruct.Pin = LED13_Pin|LED14_Pin|LED15_Pin|LED6_Pin 
                          |LED7_Pin|LED8_Pin|LED9_Pin|LED10_Pin 
                          |LED11_Pin|LED12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin 
                           LED5_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : key1_Pin */
  GPIO_InitStruct.Pin = key1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(key1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED16_Pin */
  GPIO_InitStruct.Pin = LED16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED16_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : key2_Pin */
  GPIO_InitStruct.Pin = key2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(key2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//after pressed, turn on timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){//if timer source is tim2

		state++;//increase state one by one, makes row blink one by one
		if(state>=5)
			state=1;//repeat LED state

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);//reset
/* NOTE : This function should not be modified, when the callback is needed,the HAL_UART_RxCpltCallback can be implemented in the user file*/
	UNUSED(huart);
	uart3_read_buffers[uart3_read_cnt]=uart3_conv;
	uart3_read_cnt++;
	HAL_UART_Receive_IT (&huart3,&uart3_conv,1);
	if (uart3_conv==0x0a){
		HAL_UART_Transmit_IT (&huart3,uart3_read_buffers,uart3_read_cnt);
		 uart3_read_cnt=0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
