#ifndef INC_OPERATION_H_
#define INC_OPERATION_H_

#include "stp_motor.h"

#define SENS_LIM_HOME_CH1	(0x0001)
#define SENS_LIM_HOME_CH2	(0x0002)
#define SENS_LIM_HOME_CH3	(0x0004)
#define SENS_LIM_HOME_CH4	(0x0008)
#define SENS_LIM_CW_CH1		(0x0010)
#define SENS_LIM_CW_CH2		(0x0020)
#define SENS_LIM_CW_CH3		(0x0040)
#define SENS_LIM_CW_CH4		(0x0080)
#define SENS_LIM_CCW_CH1	(0x0100)
#define SENS_LIM_CCW_CH2	(0x0200)
#define SENS_LIM_CCW_CH3	(0x0400)
#define SENS_LIM_CCW_CH4	(0x0800)


#define SENS_GPIO_HOME_CH1	(!HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0))
#define SENS_GPIO_HOME_CH2	(!HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1))
#define SENS_GPIO_HOME_CH3	(!HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2))
#define SENS_GPIO_HOME_CH4	(!HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3))
#define SENS_GPIO_CW_CH1	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4))
#define SENS_GPIO_CW_CH2	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5))
#define SENS_GPIO_CW_CH3	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6))
#define SENS_GPIO_CW_CH4	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7))
#define SENS_GPIO_CCW_CH1	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8))
#define SENS_GPIO_CCW_CH2	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9))
#define SENS_GPIO_CCW_CH3	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10))
#define SENS_GPIO_CCW_CH4	(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11))

#define CMD_MAX	(5000)
#define CMD_LINE_MAX	(100)

typedef struct __OpVarType__
{
	char cmd[CMD_MAX][CMD_LINE_MAX];
	int cmdLen;
	uint16_t sens_lim;
	uint16_t sens_origin;
	double spd;
	uint8_t	coordRel;
} OpVarType;

extern OpVarType Op;

uint8_t Op_Standby(uint8_t reset);
uint8_t Op_Command(uint8_t reset);
uint8_t Op_MotorMoveFast(uint8_t reset, double targParam[MOTOR_CH_NUM], double* delayParam);
uint8_t Op_MotorMoveLine(uint8_t reset, double targParam[MOTOR_CH_NUM], double* spdParam, double* delayParam);
uint8_t Op_MotorOrigin(uint8_t reset);

#endif /* INC_OPERATION_H_ */
