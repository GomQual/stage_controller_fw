#ifndef INC_STP_MOTOR_H_
#define INC_STP_MOTOR_H_

#define MOTOR_CH_NUM	(4)

typedef struct __StpMotorParamType__
{
	uint8_t num;
	double vMax;
	double vMin;
	double r;
	double ta;
} StpMotorParamType;

typedef enum {
	STP_MODE_MANUAL = 0,
	STP_MODE_CTRL_SPD,
	STP_MODE_CTRL_POS,
	STP_MODE_CTRL_POS_INV,
} StpMotorMode;

typedef struct __StpMotorType__
{
	// saved parameter
	StpMotorParamType* param;
	// temporal setting
	double clk;
	double fMax;
	double fMin;
	double acc;
	uint16_t pMax;
	uint16_t pMin;
	double A;
	double B;
	double Bp;
	// control variables
	uint8_t en;
	uint8_t dir;
	int32_t pos; // position [pulse]
	int32_t targPos; // target position [pulse]
	int32_t startPos; // start position [pulse]
	double spd; // speed [mm/sec]
	double targPulse; // target pulse period
	double pulseD;
	uint16_t pulse; // pulse in period
	uint16_t next_ccr;
	uint8_t stop;
	StpMotorMode mode; // 0: manual, 1: constant speed, 2: position control
} StpMotorType;

uint16_t StpMotor_PulseCtrl(StpMotorType *m);
uint8_t Motor_MoveFast(int32_t targ[MOTOR_CH_NUM]);
uint8_t Motor_MoveLine(double spd, int32_t targ[MOTOR_CH_NUM]);
uint16_t Motor_Speed2Period(StpMotorType *m, double spd);
void Motor_Move(StpMotorType *m, uint8_t dir, uint16_t pulse);
uint8_t Motor_MovePos_ConstSpd(StpMotorType *m, int32_t targPos, uint16_t pulse);
void Motor_MovePos(StpMotorType *m, int32_t targPos);
void Motor_Stop(StpMotorType *m);
void Motor_PosReset(StpMotorType *m);
void Motor_OnOff(StpMotorType *m, uint8_t onoff);

#endif /* INC_STP_MOTOR_H_ */
