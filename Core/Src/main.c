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
#include <stdio.h>

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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
SysVarType SysVar = {
	"100230519", //char ver[16];
	STATE_STANDBY, //uint8_t state;
	0, //uint8_t secFlag;
	0, //uint8_t dsecFlag;
	0, //uint8_t csecFlag;
	0, //uint8_t msecFlag;
	0, //uint32_t secCount;
	0, //uint32_t dsecCount;
	0, //uint32_t csecCount;
	0, //uint32_t msecCount;
	0, //uint32_t time;
	0, //uint32_t error;
	{ {0,}, }, //StpMotorType m[STP_MOTOR_CH_NUM];
};

ParamVarType ParamVar = {
	{
		{
			0, //uint8_t num;
			4.0L, //double vMax;
			0.05L, //double vMin;
			0.0002L, //double r;
			0.05L, //double ta;
		}, // StpMotorParamType mp[0];
		{
			1, //uint8_t num;
			4.0L, //double vMax;
			0.05L, //double vMin;
			0.0002L, //double r;
			0.05L, //double ta;
		}, // StpMotorParamType mp[1];
		{
			2, //uint8_t num;
			4.0L, //double vMax;
			0.05L, //double vMin;
			0.0002L, //double r;
			0.05L, //double ta;
		}, // StpMotorParamType mp[2];
		{
			3, //uint8_t num;
			4.0L, //double vMax;
			0.05L, //double vMin;
			0.0002L, //double r;
			0.05L, //double ta;
		}, // StpMotorParamType mp[3];
	},
	{ 1, 1, 0, 0 },
};

uint32_t ccr_monitor[2][CCR_MONITOR_MAX] = { {0, }, {0, } };
int ccr_monitor_idx[2] = { 0, 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void System_Init(void);


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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  System_Init();

//  COMM_Print(&huart1, "COMMAND Start\r\n");
//  COMM_Print(&huart2, "CONSOLE Start\r\n");
  COMM_Print(&huart3, "***** Stage Controller %s *****\r\n", SysVar.ver);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	/* USER CODE END WHILE */
	  	switch(SysVar.state)
		{
			case STATE_STANDBY:
				if(Op_Standby(0))
					Op_Standby(0);
				break;
			case STATE_COMMAND:
				if(Op_Command(0))
					Op_Standby(1);
				break;
			case STATE_MOTOR_ORIGIN:
				if(Op_MotorOrigin(0))
					if(Op_Command(0))
						Op_Standby(1);
				break;
			case STATE_MOTOR_MOVE_FAST:
				if(Op_MotorMoveFast(0, NULL, NULL))
					if(Op_Command(0))
						Op_Standby(1);
				break;
			case STATE_MOTOR_MOVE_LINE:
				if(Op_MotorMoveLine(0, NULL, NULL, NULL))
					if(Op_Command(0))
						Op_Standby(1);
				break;
			default:
				Op_Standby(1);
		}
	  	if(SysVar.secFlag)
	  	{
	  		SysVar.secFlag = 0;
	  	}
	  	if( (comm_bufIdxH-comm_bufIdxT+MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE > 0 )
	  	{
	  		COMM_G_CMD(&huart1, comm_buf[comm_bufIdxT]);
	  		comm_bufIdxT = (comm_bufIdxT+1) % MAX_BUFFER_SIZE;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_0
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE0
                           LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_0
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 PF6 PF7
                           PF8 PF9 PF10 PF11
                           PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 PD14
                           PD15 PD4 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void System_Init(void)
{
	int i;
	for(i = 0; i < STP_MOTOR_CH_NUM; i++)
	{
		StpMotorType* m = &(SysVar.m[i]);
		m->clk = SystemD2Clock / (htim1.Init.Prescaler+1);
		m->param = &(ParamVar.mp[i]);
		m->fMax = m->param->vMax / m->param->r;
		m->fMin = m->param->vMin / m->param->r;
		m->acc = (m->param->vMax - m->param->vMin) / m->param->ta;
		m->pMax = (uint16_t)(m->clk / m->fMax / 2.0);
		m->pMin = (uint16_t)(m->clk / m->fMin / 2.0);
		m->A = 2*m->acc / (m->param->r * m->clk * m-> clk);
		m->B = (double)m->pMin * m->pMin;
		m->Bp = 4.0L * m->B * m->acc / m->clk / m->clk / m->param->r;

		m->en = 0;
		m->dir = 0;
		m->pos = 0;
		m->targPos = 0;
		m->startPos = 0;
		m->spd = 0.0;
		m->targPulse = 0.0;
		m->pulseD = 0.0;
		m->pulse = 0;
		m->next_ccr = 0;
		m->stop = 1;
		m->mode = STP_MODE_MANUAL;

		Print(">> Motor %d Initialize\r\n", m->param->num);
		Print("- Max Speed: %.4lf [mm/sec]\r\n", m->param->vMax);
		Print("- Min Speed: %.4lf [mm/sec]\r\n", m->param->vMin);
		Print("- Pitch Ratio: %.4lf [mm/pulse]\r\n", m->param->r);
		Print("- Acceleration Time: %.4lf [sec]\r\n", m->param->ta);
		Print("- Timer Clock: %.0lf [kHz]\r\n", m->clk/1000.0);
		Print("- Max Speed Frequency: %.0lf [Hz]\r\n", m->fMax);
		Print("- Min Speed Frequency: %.0lf [Hz]\r\n", m->fMin);
		Print("- Acceleration Slope: %.3lf [mm/sec^2]\r\n", m->acc);
		Print("- Max Speed Timer Period: %d [count/2]\r\n", m->pMax);
		Print("- Min Speed Timer Period: %d [count/2]\r\n", m->pMin);
		Print("- Constant Value A: %.4le\r\n", m->A);
		Print("- Constant Value B: %.4le\r\n", m->B);
		Print("- Constant Value Bp: %.4le\r\n\r\n", m->Bp);

		Motor_PWR(i, 0);
		Motor_CS(i, 0);
		Motor_INH(i, 0);
		Motor_DIR(i, 0);
		Motor_OFF(i, 1);
	}

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim5);

	strcpy(Op.cmd[0], "N1 G28*12");
	strcpy(Op.cmd[1], "N2 G90*12");
	strcpy(Op.cmd[2], "N3 G0 X-5 Y-5*2b");
	strcpy(Op.cmd[3], "N4 G91*15");
	strcpy(Op.cmd[4], "N5 G1 X10*54");
	strcpy(Op.cmd[5], "N6 G1 Y1*66");
	strcpy(Op.cmd[6], "N7 G1 X-10*7b");
	strcpy(Op.cmd[7], "N8 G1 Y1*68");
	strcpy(Op.cmd[8], "N9 G1 X10*58");
	strcpy(Op.cmd[9], "N10 G1 Y1*51");
	strcpy(Op.cmd[10], "N11 G1 X-10*4c");
	strcpy(Op.cmd[11], "N12 G1 Y1*53");
	strcpy(Op.cmd[12], "N13 G1 X10*63");
	strcpy(Op.cmd[13], "N14 G1 Y1*55");
	strcpy(Op.cmd[14], "N15 G1 X-10*48");
	strcpy(Op.cmd[15], "N16 G1 Y1*57");
	strcpy(Op.cmd[16], "N17 G1 X10*67");
	strcpy(Op.cmd[17], "N18 G1 Y1*59");
	strcpy(Op.cmd[18], "N19 G1 X-10*44");
	strcpy(Op.cmd[19], "N20 G1 Y1*52");
	strcpy(Op.cmd[20], "N21 G1 X10*62");
	strcpy(Op.cmd[21], "N22 G1 Y1*50");
	strcpy(Op.cmd[22], "N23 G1 X-10*4d");
	strcpy(Op.cmd[23], "N24 G1 Y1*56");
	Op.cmdLen = 24;

//	  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
//	  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
//	  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
//	  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

}

void Motor_Start(int ch)
{
	SysVar.m[ch].next_ccr = htim1.Instance->CNT + SysVar.m[ch].pulse;
	switch(ch)
	{
	case 0:
		htim1.Instance->CCR1 = SysVar.m[ch].next_ccr;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
	break;
	case 1:
		htim1.Instance->CCR2 = SysVar.m[ch].next_ccr;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
	break;
	case 2:
		htim1.Instance->CCR3 = SysVar.m[ch].next_ccr;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
	break;
	case 3:
		htim1.Instance->CCR4 = SysVar.m[ch].next_ccr;
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
	break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case GPIO_PIN_0:
		if(Op.sens_origin)
			SysVar.m[0].stop = 1;
		Op.sens_lim |= SENS_LIM_HOME_CH1;
		//COMM_Print(&huart3, "SENS HOME CH1\r\n");
	break;
	case GPIO_PIN_1:
		if(Op.sens_origin)
			SysVar.m[1].stop = 1;
		Op.sens_lim |= SENS_LIM_HOME_CH2;
		//COMM_Print(&huart3, "SENS HOME CH2\r\n");
	break;
	case GPIO_PIN_2:
		if(Op.sens_origin)
			SysVar.m[2].stop = 1;
		Op.sens_lim |= SENS_LIM_HOME_CH3;
		//COMM_Print(&huart3, "SENS HOME CH3\r\n");
	break;
	case GPIO_PIN_3:
		if(Op.sens_origin)
			SysVar.m[3].stop = 1;
		Op.sens_lim |= SENS_LIM_HOME_CH4;
		//COMM_Print(&huart3, "SENS HOME CH4\r\n");
	break;
	case GPIO_PIN_4:
		SysVar.m[0].stop = 1;
		Op.sens_lim |= SENS_LIM_CW_CH1;
		//COMM_Print(&huart3, "SENS CW CH1\r\n");
	break;
	case GPIO_PIN_5:
		SysVar.m[1].stop = 1;
		Op.sens_lim |= SENS_LIM_CW_CH2;
		//COMM_Print(&huart3, "SENS CW CH2\r\n");
	break;
	case GPIO_PIN_6:
		SysVar.m[2].stop = 1;
		Op.sens_lim |= SENS_LIM_CW_CH3;
		//COMM_Print(&huart3, "SENS CW CH3\r\n");
	break;
	case GPIO_PIN_7:
		SysVar.m[3].stop = 1;
		Op.sens_lim |= SENS_LIM_CW_CH4;
		//COMM_Print(&huart3, "SENS CW CH4\r\n");
	break;
	case GPIO_PIN_8:
		SysVar.m[0].stop = 1;
		Op.sens_lim |= SENS_LIM_CCW_CH1;
		//COMM_Print(&huart3, "SENS CCW CH1\r\n");
	break;
	case GPIO_PIN_9:
		SysVar.m[1].stop = 1;
		Op.sens_lim |= SENS_LIM_CCW_CH2;
		//COMM_Print(&huart3, "SENS CCW CH2\r\n");
	break;
	case GPIO_PIN_10:
		SysVar.m[2].stop = 1;
		Op.sens_lim |= SENS_LIM_CCW_CH3;
		//COMM_Print(&huart3, "SENS CCW CH3\r\n");
	break;
	case GPIO_PIN_11:
		SysVar.m[3].stop = 1;
		Op.sens_lim |= SENS_LIM_CCW_CH4;
		//COMM_Print(&huart3, "SENS CCW CH4\r\n");
	break;
	case GPIO_PIN_12:
		//COMM_Print(&huart3, "MOTOR TIMING CH1\r\n");
	break;
	case GPIO_PIN_13:
		//COMM_Print(&huart3, "MOTOR TIMING CH2\r\n");
	break;
	case GPIO_PIN_14:
		//COMM_Print(&huart3, "MOTOR TIMING CH3\r\n");
	break;
	case GPIO_PIN_15:
		//COMM_Print(&huart3, "MOTOR TIMING CH4\r\n");
	break;
	}
}

void Motor_PWR(int ch, GPIO_PinState onoff)
{
	switch(ch)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, onoff);
	break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, onoff);
	break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, onoff);
	break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, onoff);
	break;
	}
}

void Motor_CS(int ch, GPIO_PinState onoff)
{
	switch(ch)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, onoff);
	break;
	case 1:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, onoff);
	break;
	case 2:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, onoff);
	break;
	case 3:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, onoff);
	break;
	}
}

void Motor_INH(int ch, GPIO_PinState onoff)
{
	switch(ch)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, onoff);
	break;
	case 1:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, onoff);
	break;
	case 2:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, onoff);
	break;
	case 3:
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, onoff);
	break;
	}
}

void Motor_DIR(int ch, GPIO_PinState onoff)
{
	switch(ch)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, onoff);
	break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, onoff);
	break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, onoff);
	break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, onoff);
	break;
	}
}

void Motor_OFF(int ch, GPIO_PinState onoff)
{
	switch(ch)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, onoff);
	break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, onoff);
	break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, onoff);
	break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, onoff);
	break;
	}
}

void Print(const char *str, ...)
{
	char buffer[MAX_BUFFER_SIZE];
	va_list ap;
	size_t strSize;

	va_start(ap, str);
	vsprintf(buffer, str, ap);
	va_end(ap);

	strSize = strlen(buffer);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strSize, COMM_TIMEOUT);
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
