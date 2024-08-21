#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

char comm_buf[MAX_BUFFER_SIZE];
int comm_bufIdxH = 0;
int comm_bufIdxT = 0;

void COMM_IT_Receive(UART_HandleTypeDef *huart, uint8_t data)
{
	HAL_UART_Transmit(huart, &data, 1, COMM_TIMEOUT);
}

void COMM_ButtonControl(UART_HandleTypeDef *huart, uint8_t data)
{
	static int ch = 0;
	static double spd = 4.0;
	static int pos_n = 1;
	static int pos_o = 1000;
	static int star_n = 0;
	static char* str;
	int32_t x, y, targ[4];
	uint8_t cs;
	int i;
	switch(data)
	{

	case '!':
		ch = 0;
		COMM_Print(huart, "Control: CH%d\r\n", ch+1);
		break;
	case '@':
		ch = 1;
		COMM_Print(huart, "Control: CH%d\r\n", ch+1);
		break;
	case '#':
		ch = 2;
		COMM_Print(huart, "Control: CH%d\r\n", ch+1);
		break;
	case '$':
		ch = 3;
		COMM_Print(huart, "Control: CH%d\r\n", ch+1);
		break;

	case 'q':
		COMM_Print(huart, "CH%d CCW\r\n", ch+1);
		Motor_Move(&SysVar.m[ch], 0, Motor_Speed2Period(&SysVar.m[ch], spd));
		break;
	case 'a':
		COMM_Print(huart, "CH%d CW\r\n", ch+1);
		Motor_Move(&SysVar.m[ch], 1, Motor_Speed2Period(&SysVar.m[ch], spd));
		break;
	case 'w':
		spd = spd + 0.05;
		COMM_Print(huart, "Motor SPD++: %f\r\n", spd);
		break;
	case 's':
		spd = spd - 0.05;
		if(spd < 0.0) spd = 0.05;
		COMM_Print(huart, "Motor SPD--: %f\r\n", spd);
		break;

	case 'e':
		if(pos_n < 0)
			pos_n *= -1;
		pos_n++;
		if(pos_n >= 10)
			pos_n = 1;
		COMM_Print(huart, "Motor POS: %d\r\n", pos_n*pos_o);
		break;
	case 'd':
		if(pos_n > 0)
			pos_n *= -1;
		pos_n--;
		if(pos_n <= -10)
			pos_n = -1;
		COMM_Print(huart, "Motor POS: %d\r\n", pos_n*pos_o);
		break;
	case 'r':
		pos_o *= 10;
		if(pos_o > 100000)
			pos_o = 1;
		COMM_Print(huart, "Motor POS: %d\r\n", pos_n*pos_o);
		break;
	case 'f':
		COMM_Print(huart, "CH%d Move to %d\r\n", ch+1, SysVar.m[ch].pos+pos_n*pos_o);
		Motor_MovePos_ConstSpd(&SysVar.m[ch], SysVar.m[ch].pos+pos_n*pos_o, Motor_Speed2Period(&SysVar.m[ch], spd));
		break;

	case '0':
		COMM_Print(huart, "Motor OFF\r\n");
		Motor_OnOff(NULL, 0);
		break;
	case '1':
		Op_Standby(1);
		break;
	case '2':
		Op_MotorOrigin(1);
		break;
	case '3':
		Op_Command(1);
		break;


	case 'z':
		COMM_Print(huart, "Motor STOP\r\n");
		Motor_Stop(NULL);
		break;
	case 'x':
		COMM_Print(huart, "Current Position\r\nCh1: %d\r\nCh2: %d\r\nCh3: %d\r\nCh4: %d\r\n",
				SysVar.m[0].pos, SysVar.m[1].pos, SysVar.m[2].pos, SysVar.m[3].pos);
		break;
	case 'c':
		switch(star_n)
		{
		case 0:
			x = 100000; y = 0;
			break;
		case 1:
			x = 30902; y = 95106;
			break;
		case 2:
			x = -80902; y = 58779;
			break;
		case 3:
			x = -80902; y = -58779;
			break;
		case 4:
			x = 30902; y = -95106;
			break;
		case 5:
			x = 100000; y = 0;
			break;
		case 6:
			x = 0; y = 100000;
			break;
		case 7:
			x = -100000; y = 0;
			break;
		case 8:
			x = 0; y = -100000;
			break;
		}
		star_n = (star_n+1)%9;
		COMM_Print(huart, "Star Move (%d, %d)\r\n", x, y);
		targ[0] = y;
		targ[1] = x;
		targ[2] = 0;
		targ[3] = 0;
		Motor_MoveLine(spd, targ);
		break;

	case 'v':
		switch(star_n)
		{
		case 0:
			x = 100000; y = 0;
			break;
		case 1:
			x = 30902; y = 95106;
			break;
		case 2:
			x = -80902; y = 58779;
			break;
		case 3:
			x = -80902; y = -58779;
			break;
		case 4:
			x = 30902; y = -95106;
			break;
		case 5:
			x = 100000; y = 0;
			break;
		case 6:
			x = 0; y = 100000;
			break;
		case 7:
			x = -100000; y = 0;
			break;
		case 8:
			x = 0; y = -100000;
			break;
		}
		star_n = (star_n+1)%9;
		COMM_Print(huart, "Star Move Fast (%d, %d)\r\n", x, y);
		targ[0] = y;
		targ[1] = x;
		targ[2] = 0;
		targ[3] = 0;
		Motor_MoveFast(targ);
		break;
//
//	case 'b':
//		COMM_Print(huart, "%s\r\n", Op.cmd);
//		i = 0;
//		while(Op.cmd[i] != '\r' && i < Op.cmdLen)
//		{
//			if(Op.cmd[i] == 'N')
//				cs = 'N';
//			else if(Op.cmd[i] == '*')
//			{
//				Op.cmd[++i] = BYTE2HEX(cs>>4);
//				Op.cmd[++i] = BYTE2HEX(cs);
//			}
//			else if(Op.cmd[i] != '\r' && Op.cmd[i] != '\n')
//			{
//				cs ^= Op.cmd[i];
//			}
//			i++;
//		}
//		COMM_Print(huart, "%s\r\n", Op.cmd);
//		break;

	case 'b':
		COMM_Print(huart, "Show Command List: %d\r\n", Op.cmdLen);
		for(i = 0; i < Op.cmdLen; i++)
		{
			COMM_Print(huart, "%s ", Op.cmd[i]);
			if(COMM_CheckCS(Op.cmd[i], strlen(Op.cmd[i])))
				COMM_Print(huart, "<== Wrong Check Sum");
			COMM_Print(huart, "\r\n");
		}
		break;

	case 'i':
		COMM_Print(huart, "Sensor GPIO\r\nHOME:\t%d\t%d\t%d\t%d\t\r\nCW:\t%d\t%d\t%d\t%d\t\r\nCCW:\t%d\t%d\t%d\t%d\t\r\n",
				SENS_GPIO_HOME_CH1, SENS_GPIO_HOME_CH2, SENS_GPIO_HOME_CH3, SENS_GPIO_HOME_CH4,
				SENS_GPIO_CW_CH1, SENS_GPIO_CW_CH2, SENS_GPIO_CW_CH3, SENS_GPIO_CW_CH4,
				SENS_GPIO_CCW_CH1, SENS_GPIO_CCW_CH2, SENS_GPIO_CCW_CH3, SENS_GPIO_CCW_CH4);
		break;
	}
}

void COMM_G_CMD(UART_HandleTypeDef *huart, uint8_t data)
{
	static int mode = 0;
	static char m_buf[100];
	static int m_idx = 0;
	static int cmd_num = 0;

	// G code mode
	if(mode == 1)
	{
		if(data != '\n' && data != '\r' && m_idx < 100)
		{
			m_buf[m_idx++] = data;
		}
		else
		{
			m_buf[m_idx] = '\0';
			Print("Received Command: %s\r\n", m_buf);
			if(COMM_CheckCS(m_buf, m_idx))
			{
				Print("Check Sum Error: N%d\r\n", cmd_num);
				COMM_Print(huart, "rs N%d\r", cmd_num);
				mode = 0;
				m_idx = 0;
				return;
			}
			sscanf(m_buf, "N%d", &cmd_num);
//			free(Op.cmd[cmd_num-1]);
//			Op.cmd[cmd_num-1] = (char*)malloc(sizeof(char)*(m_idx+1));
//			Print("Allocation: %x\r\n", Op.cmd[cmd_num-1]);
			strcpy(Op.cmd[cmd_num-1], m_buf);

			if(data == '\r')
			{
				COMM_Print(huart, "ok\r");
				Op.cmdLen = cmd_num;
			}
			else if(data == '\n')
			{
				if(Op.cmdLen < cmd_num)
					Op.cmdLen = cmd_num;
			}
			m_idx = 0;
			mode = 0;
		}
	}
	// command mode
	else
	{
		if(m_idx == 0) // first data of new command
		{
			// G code
			if(data == 'N')
			{
				mode = 1;
				m_buf[0] = 'N';
			}
			// M command
			else if(data == 'M')
			{
				mode = 0;
				m_buf[0] = 'M';
			}
			m_idx++;
		}
		else
		{
			if(data != '\r')
			{
				m_buf[m_idx++] = data;
			}
			else
			{
				m_buf[m_idx] = '\0';
				sscanf(m_buf, "M%d", &cmd_num);
				switch(cmd_num)
				{
				// stop current operation and standby
				case 1:
					Print("Command: M1\r\n");
					COMM_Print(huart, "ok\r");
					Op_Standby(1);
					break;
				// motor on
				case 17:
					Print("Command: M17\r\n");
					COMM_Print(huart, "ok\r");
					Motor_OnOff(NULL, 1);
					break;
				// motor off
				case 18:
					Print("Command: M18\r\n");
					COMM_Print(huart, "ok\r");
					Motor_OnOff(NULL, 0);
					break;
				// repeat G code command
				case 41:
					Print("Command: M41\r\n");
					COMM_Print(huart, "ok\r");
					Op_Command(1);
					break;
				// return current position and speed
				case 114:
					Print("Command: M114\r\n");
					COMM_Print(huart, "ok X%.3lf Y%.3lf Z%.3lf I%.3lf F%.3lf\r",
							(double)SysVar.m[0].pos*SysVar.m[0].param->r/2.0,
							(double)SysVar.m[1].pos*SysVar.m[1].param->r/2.0,
							(double)SysVar.m[2].pos*SysVar.m[2].param->r/2.0,
							(double)SysVar.m[3].pos*SysVar.m[3].param->r/2.0,
							Op.spd);
					break;
				// return current version
				case 115:
					Print("Command: M115\r\n");
					COMM_Print(huart, "ok V%s\r", SysVar.ver);
					break;
				default:
					Print("!!! Wrong Command\r\n");
					COMM_Print(huart, "!! Wrong Command\r");
					break;
				}
				m_idx = 0;
			}
		}
	}
}


uint8_t COMM_CheckCS(char str[], int len)
{
	uint8_t cs;
	int i;

	if(str[0] == 'N')
		cs = 'N';
	else
	{
		Print("!!! Check CS: Starting Character Not 'N'\r\n");
		return 1;
	}

	for(i = 1; i < len && str[i] != '\r' && str[i] != '\n' && str[i] != '\0'; i++)
	{
		if(str[i] == '*')
		{
			char cs1 = BYTE2HEX(cs>>4);
			char cs2 = BYTE2HEX(cs);

			if(str[i+1] == cs1 && str[i+2] == cs2)
				return 0;
			else
			{
				Print("!!! Check CS: Miss Match 0x%c%c 0x%02x\r\n", cs1, cs2, cs);
				return 1;
			}
		}
		else
		{
			cs ^= str[i];
		}
	}
	Print("!!! Check CS: End of Command without CS\r\n");
	return 1;
}

void COMM_Print(UART_HandleTypeDef *huart, const char *str, ...)
{
	char buffer[MAX_BUFFER_SIZE];
	va_list ap;
	size_t strSize;

	va_start(ap, str);
	vsprintf(buffer, str, ap);
	va_end(ap);

	strSize = strlen(buffer);
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strSize, COMM_TIMEOUT);
}

char BYTE2HEX(uint8_t byte)
{
	byte &= 0x0F;
	if(byte < 10)
		return byte+'0';
	else
		return byte-10+'a';
}

uint8_t HEX2BYTE(char c)
{
	if('0' <=  c && c <= '9')
		return (uint8_t)(c - '0');
	else if('A' <= c && c <= 'F')
		return (uint8_t)(c - 'A');
	else if('a' <= c && c <= 'f')
		return (uint8_t)(c - 'a');
	else
		return 0xFF;
}
