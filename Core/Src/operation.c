#include "main.h"

OpVarType Op = {
	{ "", },// char cmd[CMD_MAX][CMD_LINE_MAX];
	0, // int cmdLen;
	0, // uint16_t sens_lim;
	0, // uint16_t sens_origin;
	2.0, // double spd;
	0, // uint8_t coordRel;
};



uint8_t Op_Standby(uint8_t reset)
{
	static uint8_t opStep = 0;

	if(reset)
	{
		opStep = 0;
		SysVar.state = STATE_STANDBY;
		Print(">> Standby\r\n");
	}
	else
	{
		switch(opStep)
		{
		case 0:
			Motor_Stop(NULL);
			opStep++;
		break;
		case 1:
			Motor_OnOff(NULL, 1);
			opStep = 100;
		break;
		case 100:
		break;
		}
	}

	return 0;
}

uint8_t Op_Command(uint8_t reset)
{
	static uint32_t opStep = 0;
	static int cmdIdx = 0;
	static int processing = 0;
	int num, Gcmd;
	double targ[STP_MOTOR_CH_NUM];
	double spd;
	double delay;
	int i;
	char* cmd;

	if(reset)
	{
		opStep = 0;
		SysVar.state = STATE_COMMAND;
		Print(">> Command Operation\r\n");
		cmdIdx = 0;
		processing = 1;
	}
	else if(processing)
	{
		Print("Command Step: %d\r\n", opStep+1);
		// read command
		if(opStep >= Op.cmdLen)
		{
			Print("End of the command\r\n");
			processing = 0;
			return 1;
		}
		if(Op.cmd[opStep] == NULL)
		{
			Print("Empty Command Line\r\n");
			opStep++;
			return 0;
		}

		// initial parameters
		num = 0;
		Gcmd = 100;
		if(Op.coordRel)
		{
			for(i = 0; i < STP_MOTOR_CH_NUM; i++)
				targ[i] = 0.0;
		}
		else
		{
			for(i = 0; i < STP_MOTOR_CH_NUM; i++)
				targ[i] = ((double)SysVar.m[i].pos)*SysVar.m[i].param->r/2.0;
		}
		spd = Op.spd;
		delay = 0.0;

		cmd = Op.cmd[opStep];
		cmdIdx = 0;
		while(cmd[cmdIdx] != '\0')
		{
			switch(cmd[cmdIdx])
			{
			case 'N':
				sscanf(&cmd[cmdIdx+1], "%d", &num);
				break;

			case 'G':
				sscanf(&cmd[cmdIdx+1], "%d", &Gcmd);
				break;

			case 'X':
				sscanf(&cmd[cmdIdx+1], "%lf", &targ[0]);
				break;

			case 'Y':
				sscanf(&cmd[cmdIdx+1], "%lf", &targ[1]);
				break;

			case 'Z':
				sscanf(&cmd[cmdIdx+1], "%lf", &targ[2]);
				break;

			case 'I':
				sscanf(&cmd[cmdIdx+1], "%lf", &targ[3]);
				break;

			case 'F':
				sscanf(&cmd[cmdIdx+1], "%lf", &spd);
				break;

			case 'P':
				sscanf(&cmd[cmdIdx+1], "%lf", &delay);
				break;

			case '*':
				cmdIdx += 2;
				break;
			}
			cmdIdx++;
		}

		// validate
		if(num != opStep+1)
			Print("!!! Command Line Error N%d != %d step\r\n", num, opStep+1);
		if(spd < 0.0)
			spd *= -1.0;
		if(spd != Op.spd)
			Op.spd = spd;

		// run command
		Print("Run G Command: G%d\r\n", Gcmd);
		switch(Gcmd)
		{
		case 0:
			if(Op_MotorMoveFast(1, targ, (delay<0.0)?(NULL):(&delay)) == -1)
				SysVar.state = STATE_COMMAND;
			break;
		case 1:
			if(Op_MotorMoveLine(1, targ, (spd==Op.spd)?(NULL):(&spd), (delay<0.0)?(NULL):(&delay)) == -1)
				SysVar.state = STATE_COMMAND;
			break;
		case 28:
			Op_MotorOrigin(1);
			break;
		case 90:
			Op.coordRel = 0;
			break;
		case 91:
			Op.coordRel = 1;
			break;
		case 92:
			Motor_PosReset(NULL);
			break;
		default:
			Print("!!! Wrong G Command: %d\r\n", Gcmd);
			break;
		}
		opStep++;
	}
	else
		return 1;

	return 0;
}


// G0 Xnnn Ynnn Znnn Innn Pnnn *nn
uint8_t Op_MotorMoveFast(uint8_t reset, double targParam[MOTOR_CH_NUM], double* delayParam)
{
	static uint8_t opStep = 0;
	static uint8_t opStep_ex = 100;
	static int32_t targ[STP_MOTOR_CH_NUM];
	static double delay;
	int i;

	if(reset)
	{
		opStep = 0;
		SysVar.state = STATE_MOTOR_MOVE_FAST;
		Print(">> Motor Move Fast\r\n");
		if(targParam == NULL)
		{
			Print("!!! No target position\r\n");
			return -1;
		}
		for(i = 0; i < STP_MOTOR_CH_NUM; i++)
		{
			targ[i] = (int32_t)(targParam[i]/SysVar.m[i].param->r*2.0);
			if(Op.coordRel)
			{
				targ[i] += SysVar.m[i].pos;
			}
		}
		if(delayParam == NULL)
			delay = 0.0;
		else
			delay = *delayParam;
		Print("Command: G0 X%.3lf Y%.3lf Z%.3lf I%.3lf",
			targParam[0], targParam[1], targParam[2], targParam[3]);
		if(delay >= 1.0)
			Print(" D%.2lf", delay);
		Print("\r\n");
		Motor_Stop(NULL);
	}
	else
	{
		switch(opStep)
		{
		case 0:
			Motor_MoveFast(targ);
			opStep++;
			break;
		case 1:
			if(!SysVar.m[0].en && !SysVar.m[1].en && !SysVar.m[2].en && !SysVar.m[3].en)
			{
				SysVar.msecCount = abs((int)delay);
				opStep++;
			}
			break;
		case 2:
			if(!SysVar.msecCount)
				opStep = 100;
			break;

		case 100:
			return 1;
			break;
		}
	}

	if(opStep_ex != opStep)
	{
		opStep_ex = opStep;
		Print("Motor Move Axis Step: %d\r\n", opStep);
	}

	return 0;
}

// G1 Xnnn Ynnn Znnn Innn Fnnn Pnnn *nn
uint8_t Op_MotorMoveLine(uint8_t reset, double targParam[MOTOR_CH_NUM], double* spdParam, double* delayParam)
{
	static uint8_t opStep = 0;
	static uint8_t opStep_ex = 100;
	static int32_t targ[STP_MOTOR_CH_NUM];
	static double delay;
	int i;

	if(reset)
	{
		opStep = 0;
		SysVar.state = STATE_MOTOR_MOVE_LINE;
		Print(">> Motor Move Line\r\n");
		if(targParam == NULL)
		{
			Print("!!! No target position\r\n");
			return -1;
		}
		for(i = 0; i < STP_MOTOR_CH_NUM; i++)
		{
			targ[i] = (int32_t)(targParam[i]/SysVar.m[i].param->r*2.0);
			if(Op.coordRel)
			{
				targ[i] += SysVar.m[i].pos;
			}
		}
		if(spdParam != NULL)
			Op.spd = *spdParam;
		if(delayParam == NULL)
			delay = 0.0;
		else
			delay = *delayParam;
		Print("Command: G1 X%.3lf Y%.3lf Z%.3lf I%.3lf",
			targParam[0], targParam[1], targParam[2], targParam[3]);
		if(Op.spd >= 0.05)
			Print(" F%.2lf", Op.spd);
		if(delay >= 1.0)
			Print(" D%.2lf", delay);
		Print("\r\n");
		Motor_Stop(NULL);
	}
	else
	{
		switch(opStep)
		{
		case 0:
			Motor_MoveLine(Op.spd, targ);
			opStep++;
			break;
		case 1:
			if(!SysVar.m[0].en && !SysVar.m[1].en && !SysVar.m[2].en && !SysVar.m[3].en)
			{
				SysVar.msecCount = abs((int)delay);
				opStep++;
			}
			break;
		case 2:
			if(!SysVar.msecCount)
				opStep = 100;
			break;

		case 100:
			return 1;
			break;
		}
	}

	if(opStep_ex != opStep)
	{
		opStep_ex = opStep;
		Print("Motor Move Axis Step: %d\r\n", opStep);
	}

	return 0;
}


// G28
uint8_t Op_MotorOrigin(uint8_t reset)
{
	static uint8_t opStep = 0;
	static uint8_t opStep_ex = 100;

	if(reset)
	{
		opStep = 0;
		SysVar.state = STATE_MOTOR_ORIGIN;
		Print(">> Motor Origin\r\n");
		Op.sens_lim = 0;
		Op.sens_origin = 1;
		Motor_Stop(NULL);
	}
	else
	{
		switch(opStep)
		{
		// CH1
		case 0:
			if(ParamVar.m_active[0])
				opStep = 1;
			else
				opStep = 20;
			break;

		// check sensor
		case 1:
			// if home, move down 75000 and move up -> next
			if(SENS_GPIO_HOME_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Motor_MovePos_ConstSpd(&SysVar.m[0], SysVar.m[0].pos-75000, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// if cw, move down and check home -> 8
			else if(SENS_GPIO_CW_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Motor_Move(&SysVar.m[0], 0, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep = 8;
			}
			// if ccw or somewhere, move up and check home -> 6
			else
			{
				Motor_Move(&SysVar.m[0], 1, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep = 6;
			}
			break;

		case 2:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;

		case 3:
			// move up to home
			Motor_Move(&SysVar.m[0], 1, Motor_Speed2Period(&SysVar.m[0], 4.0));
			Op.sens_lim = 0;
			opStep++;
			break;

		case 4:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;

		case 5:
			// if check home sensor, move back and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Motor_MovePos_ConstSpd(&SysVar.m[0], SysVar.m[0].pos-5000, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep = 11;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 6:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;

		case 7:
			if(Op.sens_lim & SENS_LIM_HOME_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Motor_MovePos_ConstSpd(&SysVar.m[0], SysVar.m[0].pos-5000, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep = 11;
			}
			else if(Op.sens_lim & SENS_LIM_CW_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Motor_Move(&SysVar.m[0], 0, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			break;

		case 8:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;

		case 9:
			// home sensor checked in backward. move down 80000 and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Motor_MovePos_ConstSpd(&SysVar.m[0], SysVar.m[0].pos-75000, Motor_Speed2Period(&SysVar.m[0], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 10:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;


		// precise homing
		case 11:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;

		case 12:
			Motor_Move(&SysVar.m[0], 1, Motor_Speed2Period(&SysVar.m[0], 0.05));
				opStep++;
			break;

		case 13:
			if(SysVar.m[0].en == 0)
				opStep++;
			break;

		case 14:
			if(Op.sens_lim & SENS_LIM_HOME_CH1)
			{
				Motor_PosReset(&SysVar.m[0]);
				Op.sens_lim = 0;
				opStep = 20;
			}
			break;

		// CH2
		case 20:
			if(ParamVar.m_active[1])
				opStep = 21;
			else
				opStep = 40;
			break;

		// check sensor
		case 21:
			// if home, move down 75000 and move up -> next
			if(SENS_GPIO_HOME_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Motor_MovePos_ConstSpd(&SysVar.m[1], SysVar.m[1].pos-75000, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// if cw, move down and check home -> 28
			else if(SENS_GPIO_CW_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Motor_Move(&SysVar.m[1], 0, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep = 28;
			}
			// if ccw or somewhere, move up and check home -> 26
			else
			{
				Motor_Move(&SysVar.m[1], 1, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep = 26;
			}
			break;

		case 22:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;

		case 23:
			// move up to home
			Motor_Move(&SysVar.m[1], 1, Motor_Speed2Period(&SysVar.m[1], 4.0));
			Op.sens_lim = 0;
			opStep++;
			break;

		case 24:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;

		case 25:
			// if check home sensor, move back and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Motor_MovePos_ConstSpd(&SysVar.m[1], SysVar.m[1].pos-5000, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep = 31;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 26:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;

		case 27:
			if(Op.sens_lim & SENS_LIM_HOME_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Motor_MovePos_ConstSpd(&SysVar.m[1], SysVar.m[1].pos-5000, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep = 31;
			}
			else if(Op.sens_lim & SENS_LIM_CW_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Motor_Move(&SysVar.m[1], 0, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			break;

		case 28:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;

		case 29:
			// home sensor checked in backward. move down 80000 and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Motor_MovePos_ConstSpd(&SysVar.m[1], SysVar.m[1].pos-75000, Motor_Speed2Period(&SysVar.m[1], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
		break;

		case 30:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;


		// precise homing
		case 31:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;

		case 32:
			Motor_Move(&SysVar.m[1], 1, Motor_Speed2Period(&SysVar.m[1], 0.05));
				opStep++;
				break;

		case 33:
			if(SysVar.m[1].en == 0)
				opStep++;
			break;

		case 34:
			if(Op.sens_lim & SENS_LIM_HOME_CH2)
			{
				Motor_PosReset(&SysVar.m[1]);
				Op.sens_lim = 0;
				opStep = 40;
			}
			break;

		// CH3
		case 40:
			if(ParamVar.m_active[2])
				opStep = 41;
			else
				opStep = 60;
			break;

		// check sensor
		case 41:
			// if home, move down 75000 and move up -> next
			if(SENS_GPIO_HOME_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Motor_MovePos_ConstSpd(&SysVar.m[2], SysVar.m[2].pos-75000, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// if cw, move down and check home -> 48
			else if(SENS_GPIO_CW_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Motor_Move(&SysVar.m[2], 0, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep = 48;
			}
			// if ccw or somewhere, move up and check home -> 46
			else
			{
				Motor_Move(&SysVar.m[2], 1, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep = 46;
			}
			break;

		case 42:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;

		case 43:
			// move up to home
			Motor_Move(&SysVar.m[2], 1, Motor_Speed2Period(&SysVar.m[2], 4.0));
			Op.sens_lim = 0;
			opStep++;
			break;

		case 44:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;

		case 45:
			// if check home sensor, move back and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Motor_MovePos_ConstSpd(&SysVar.m[2], SysVar.m[2].pos-5000, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep = 51;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 46:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;

		case 47:
			if(Op.sens_lim & SENS_LIM_HOME_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Motor_MovePos_ConstSpd(&SysVar.m[2], SysVar.m[2].pos-5000, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep = 51;
			}
			else if(Op.sens_lim & SENS_LIM_CW_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Motor_Move(&SysVar.m[2], 0, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			break;

		case 48:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;

		case 49:
			// home sensor checked in backward. move down 80000 and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Motor_MovePos_ConstSpd(&SysVar.m[2], SysVar.m[2].pos-75000, Motor_Speed2Period(&SysVar.m[2], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 50:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;


		// precise homing
		case 51:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;

		case 52:
			Motor_Move(&SysVar.m[2], 1, Motor_Speed2Period(&SysVar.m[2], 0.05));
				opStep++;
				break;

		case 53:
			if(SysVar.m[2].en == 0)
				opStep++;
			break;

		case 54:
			if(Op.sens_lim & SENS_LIM_HOME_CH3)
			{
				Motor_PosReset(&SysVar.m[2]);
				Op.sens_lim = 0;
				opStep = 60;
			}
			break;

		// CH4
		case 60:
			if(ParamVar.m_active[3])
				opStep = 61;
			else
				opStep = 100;
			break;

		// check sensor
		case 61:
			// if home, move down 75000 and move up -> next
			if(SENS_GPIO_HOME_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Motor_MovePos_ConstSpd(&SysVar.m[3], SysVar.m[3].pos-75000, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// if cw, move down and check home -> 68
			else if(SENS_GPIO_CW_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Motor_Move(&SysVar.m[3], 0, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep = 68;
			}
			// if ccw or somewhere, move up and check home -> 66
			else
			{
				Motor_Move(&SysVar.m[3], 1, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep = 66;
			}
			break;

		case 62:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;

		case 63:
			// move up to home
			Motor_Move(&SysVar.m[3], 1, Motor_Speed2Period(&SysVar.m[3], 4.0));
			Op.sens_lim = 0;
			opStep++;
			break;

		case 64:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;

		case 65:
			// if check home sensor, move back and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Motor_MovePos_ConstSpd(&SysVar.m[3], SysVar.m[3].pos-5000, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep = 71;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 66:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;

		case 67:
			if(Op.sens_lim & SENS_LIM_HOME_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Motor_MovePos_ConstSpd(&SysVar.m[3], SysVar.m[3].pos-5000, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep = 71;
			}
			else if(Op.sens_lim & SENS_LIM_CW_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Motor_Move(&SysVar.m[3], 0, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			break;

		case 68:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;

		case 69:
			// home sensor checked in backward. move down 80000 and start precise homing
			if(Op.sens_lim & SENS_LIM_HOME_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Motor_MovePos_ConstSpd(&SysVar.m[3], SysVar.m[3].pos-75000, Motor_Speed2Period(&SysVar.m[3], 4.0));
				Op.sens_lim = 0;
				opStep++;
			}
			// sensor error - should check home sensor
			else
			{
				Print("Sensor Error. Operation Fault.\r\n");
				opStep = 100;
			}
			break;

		case 70:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;


		// precise homing
		case 71:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;

		case 72:
			Motor_Move(&SysVar.m[3], 1, Motor_Speed2Period(&SysVar.m[3], 0.05));
				opStep++;
				break;

		case 73:
			if(SysVar.m[3].en == 0)
				opStep++;
			break;

		case 74:
			if(Op.sens_lim & SENS_LIM_HOME_CH4)
			{
				Motor_PosReset(&SysVar.m[3]);
				Op.sens_lim = 0;
				opStep = 100;
			}
			break;


		case 100:
			Op.sens_origin = 0;
			return 1;
			break;
		}
	}

	if(opStep_ex != opStep)
	{
		opStep_ex = opStep;
		Print("Motor Origin Step: %d\r\n", opStep);
	}

	return 0;
}
