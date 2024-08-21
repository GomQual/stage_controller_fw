/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm.h"
#include "stp_motor.h"
#include "operation.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define STP_MOTOR_CH_NUM	(4)

typedef enum {
	STATE_STANDBY = 0,
	STATE_COMMAND,
	STATE_MOTOR_ORIGIN,
	STATE_MOTOR_MOVE_FAST,
	STATE_MOTOR_MOVE_LINE,

} StateMode;

typedef struct __SysVarType__
{
	char ver[16];
	StateMode state;
	uint8_t secFlag;
	uint8_t dsecFlag;
	uint8_t csecFlag;
	uint8_t msecFlag;
	uint32_t secCount;
	uint32_t dsecCount;
	uint32_t csecCount;
	uint32_t msecCount;
	uint32_t time;
	uint32_t error;
	StpMotorType m[STP_MOTOR_CH_NUM];
} SysVarType;

typedef struct __ParamVarType__
{
	StpMotorParamType mp[STP_MOTOR_CH_NUM];
	uint8_t m_active[STP_MOTOR_CH_NUM];
} ParamVarType;

extern SysVarType SysVar;
extern ParamVarType ParamVar;
#define CCR_MONITOR_MAX	(50000)
extern uint32_t ccr_monitor[2][CCR_MONITOR_MAX];
extern int ccr_monitor_idx[2];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Motor_PWR(int ch, GPIO_PinState onoff);
void Motor_CS(int ch, GPIO_PinState onoff);
void Motor_INH(int ch, GPIO_PinState onoff);
void Motor_DIR(int ch, GPIO_PinState onoff);
void Motor_OFF(int ch, GPIO_PinState onoff);
void Motor_Start(int ch);
void Print(const char *str, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
