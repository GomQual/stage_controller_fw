#include "main.h"

uint16_t StpMotor_PulseCtrl(StpMotorType *m)
{
	m->pos += (m->dir)?1:-1;
	if( m->mode == STP_MODE_MANUAL )
	{
		m->next_ccr += m->pulse;
	}
	else if( m->mode == STP_MODE_CTRL_SPD )
	{
		if( m->targPos == m->pos )
		// target reached - stop
			m->stop = 1;
		m->next_ccr += m->pulse;
	}
	else if( m->mode == STP_MODE_CTRL_POS_INV )
	{
		if( m->pulseD >= m->pMin-1 && !(m->pos%2) )
		// deacceleration completed
		{
			m->dir = !m->dir;
			Motor_DIR(m->param->num, m->dir);
			m->startPos = m->pos;
			m->mode = STP_MODE_CTRL_POS;
		}
		else
		// inverse phase
		{
			if( m->pulseD < m->pMin )
				m->pulseD = 1 / (1.0/m->pulseD - m->A*m->pulseD);
			if( m->pulseD >= m->pMin )
				m->pulseD = m->pMin;
			m->pulse = (uint16_t)m->pulseD;
			m->next_ccr += m->pulse;
		}
	}
	else if( m->mode == STP_MODE_CTRL_POS )
	{
		if( m->targPos == m->pos )
		// target reached - stop
			m->stop = 1;
		else if( (m->dir && (m->targPos - m->pos >= m->pos - m->startPos)) ||
			(!m->dir && (m->targPos - m->pos < m->pos - m->startPos)) )
		// acceleration phase
		{
			if( m->pulseD > m->targPulse )
				m->pulseD = 1 / (1.0/m->pulseD + m->A*m->pulseD);
			if( m->pulseD < m->targPulse )
				m->pulseD = m->targPulse;
			m->pulse = (uint16_t)m->pulseD;
			m->next_ccr += m->pulse;
		}
		else
		// deacceleration phase
		{
			if( m->dir )
				m->pulseD = sqrt(m->B / (m->Bp*(double)(m->targPos-m->pos) + 1));
			else
				m->pulseD = sqrt(m->B / (m->Bp*(double)(m->pos-m->targPos) + 1));

			if( m->pulseD < m->targPulse )
				m->pulseD = m->targPulse;
			else if( m->pulseD > m->pMin )
				m->pulseD = m->pMin;
			m->pulse = (uint16_t)m->pulseD;
			m->next_ccr += m->pulse;
		}
	}

	if( m->stop && !(m->pos%2) )
	{
		m->en = 0;
		m->pulse = m->pMin;
		m->pulseD = m->pMin;
	}

	return m->next_ccr;
}

uint8_t Motor_MoveFast(int32_t targ[MOTOR_CH_NUM])
{
	double spd[MOTOR_CH_NUM];
	int i;
	StpMotorType* m;

	for(i = 0; i < MOTOR_CH_NUM; i++)
	{
		// if motor running, return error and abort
		if(SysVar.m[0].en)
			return 1;

		// target position value to even
		if( targ[i] %2 ) targ[i] = (targ[i]/2)*2;

		// if motor is not active, set current position as target to make no movement
	}

	for(i = 0; i < MOTOR_CH_NUM; i++)
	{
		m = &SysVar.m[i];
		// set motor speed for each motor as ratio of vector length
		spd[i] = m->param->vMax;
		// set target position
		m->targPos = targ[i];

		if( m->targPos == m->pos )
		{
			m->stop = 1;
		}
		else
		{
			if( m->targPos-m->pos < 0 )
				m->dir = 0;
			else if( m->targPos-m->pos > 0 )
				m->dir = 1;
			m->mode = STP_MODE_CTRL_SPD;

			m->pulse = Motor_Speed2Period(m, spd[i]);
			if( m->pulse >= m->pMin )
				m->pulse = m->pMin;
			else if( m->pulse <= m->pMax )
				m->pulse = m->pMax;
			m->stop = 0;
			m->en = 1;
			Motor_DIR(i, m->dir);
			Motor_INH(i, 0);
			Motor_OFF(i, 0);
		}
	}

	for(i = 0; i < MOTOR_CH_NUM; i++)
	{
		if(!SysVar.m[i].stop)
			Motor_Start(i);
	}

	return 0;
}


uint8_t Motor_MoveLine(double lineSpd, int32_t targ[MOTOR_CH_NUM])
{
	int32_t curr[MOTOR_CH_NUM];
	int32_t diff[MOTOR_CH_NUM];
	double dist = 0.0, spd[MOTOR_CH_NUM];
	int i;
	StpMotorType* m;

	for(i = 0; i < MOTOR_CH_NUM; i++)
	{
		// if motor running, return error and abort
		if(SysVar.m[0].en)
			return 1;

		// target position value to even
		if( targ[i] %2 ) targ[i] = (targ[i]/2)*2;

		curr[i] = SysVar.m[i].pos;

		// set max speed to lowest active motor max speed
		// if motor is not active, set current position as target to make no movement
		if(ParamVar.m_active[i])
		{
			if(lineSpd > SysVar.m[i].param->vMax)
				lineSpd = SysVar.m[i].param->vMax;
		}
		else
			curr[i] = targ[i];

		// position difference calculation
		// 0 if motor is not active
		diff[i] = targ[i] - curr[i];

		// overall vector length calculation, adding each axis
		dist += ((double)diff[i])*diff[i];
	}
	// overall vector length calculation, square root
	dist = sqrt(dist);

	for(i = 0; i < MOTOR_CH_NUM; i++)
	{
		m = &SysVar.m[i];
		// set motor speed for each motor as ratio of vector length
		spd[i] = fabs((double)diff[i]/dist) * lineSpd;
		// set target position
		m->targPos = targ[i];

		if( m->targPos == m->pos )
		{
			m->stop = 1;
		}
		else
		{
			if( m->targPos-m->pos < 0 )
				m->dir = 0;
			else if( m->targPos-m->pos > 0 )
				m->dir = 1;
			m->mode = STP_MODE_CTRL_SPD;

			m->pulse = Motor_Speed2Period(m, spd[i]);
			if( m->pulse >= m->pMin )
				m->pulse = m->pMin;
			else if( m->pulse <= m->pMax )
				m->pulse = m->pMax;
			m->stop = 0;
			m->en = 1;
			Motor_DIR(i, m->dir);
			Motor_INH(i, 0);
			Motor_OFF(i, 0);
		}
	}

	for(i = 0; i < MOTOR_CH_NUM; i++)
	{
		if(!SysVar.m[i].stop)
			Motor_Start(i);
	}

	return 0;
}


uint16_t Motor_Speed2Period(StpMotorType *m, double spd)
{
	double f;
	if(spd > m->param->vMax)
		spd = m->param->vMax;
	else if(spd < m->param->vMin)
		spd = m->param->vMin;
	f = spd / m->param->r;
	return (uint16_t)(m->clk / f / 2.0);
}

void Motor_Move(StpMotorType *m, uint8_t dir, uint16_t pulse)
{
	m->en = 0;
	m->dir = dir;
	m->mode = STP_MODE_MANUAL;
	m->pulse = pulse;
	if( m->pulse >= m->pMin )
		m->pulse = m->pMin;
	else if( m->pulse <= m->pMax )
		m->pulse = m->pMax;
	m->stop = 0;
	m->en = 1;

	Motor_DIR(m->param->num, m->dir);
	Motor_INH(m->param->num, 0);
	Motor_OFF(m->param->num, 0);
	Motor_Start(m->param->num);
}

uint8_t Motor_MovePos_ConstSpd(StpMotorType *m, int32_t targPos, uint16_t pulse)
{
	m->en = 0;
	if( targPos %2 )
		targPos = (targPos/2)*2;
	m->targPos = targPos;

	if( m->stop == 0 )
		return 1;

	if( m->targPos == m->pos )
	{
		m->stop = 1;
	}
	else
	{
		if( m->targPos-m->pos < 0 )
			m->dir = 0;
		else if( m->targPos-m->pos > 0 )
			m->dir = 1;
		m->mode = STP_MODE_CTRL_SPD;
		m->pulse = pulse;
		if( m->pulse >= m->pMin )
			m->pulse = m->pMin;
		else if( m->pulse <= m->pMax )
			m->pulse = m->pMax;
		m->stop = 0;
		m->en = 1;
		Motor_DIR(m->param->num, m->dir);
		Motor_INH(m->param->num, 0);
		Motor_OFF(m->param->num, 0);
		Motor_Start(m->param->num);
	}
	return 0;
}

void Motor_MovePos(StpMotorType *m, int32_t targPos)
{
	m->en = 0;
	if( targPos %2 )
		targPos = (targPos/2)*2;
	m->targPos = targPos;

	if( !m->stop || (m->stop && m->targPos != m->pos ) )
	{
		if( (m->dir && (m->targPos-m->pos) < 0) ||
			(!m->dir && (m->targPos-m->pos) > 0) )
		{
			if( m->pulseD >= m->pMin-1 && !(m->pos%2) )
			{
				m->dir = !m->dir;
				Motor_DIR(m->param->num, m->dir);
				m->startPos = m->pos;
				m->mode = STP_MODE_CTRL_POS;
			}
			else
				m->mode = STP_MODE_CTRL_POS_INV;
		}
		else
		{
			m->mode = STP_MODE_CTRL_POS;
			if( !m->stop && m->pulseD < m->pMin )
			{
				m->startPos = (2.0 * m->B) / m->pulseD + 2.0 * m->Bp;
				if( m->dir )
					m->startPos = m->pos - m->startPos;
				else
					m->startPos = m->pos + m->startPos;
			}
			else
				m->startPos = m->pos;
		}

		m->targPulse = m->pMax;
		m->pulse = m->pMin;
		m->stop = 0;

		m->en = 1;
		Motor_DIR(m->param->num, m->dir);
		Motor_INH(m->param->num, 0);
		Motor_OFF(m->param->num, 0);
		Motor_Start(m->param->num);
	}
}

void Motor_Stop(StpMotorType *m)
{
	if( m == NULL )
	{
		SysVar.m[0].stop = 1;
		SysVar.m[1].stop = 1;
		SysVar.m[2].stop = 1;
		SysVar.m[3].stop = 1;
	}
	else
	{
		m->stop = 1;
	}
}

void Motor_PosReset(StpMotorType *m)
{
	if( m == NULL )
	{
		SysVar.m[0].pos = 0;
		SysVar.m[1].pos = 0;
		SysVar.m[2].pos = 0;
		SysVar.m[3].pos = 0;
	}
	else
	{
		m->pos = 0;
	}
}

void Motor_OnOff(StpMotorType *m, uint8_t onoff)
{
	if( m == NULL )
	{
		Motor_OFF(0, !onoff);
		Motor_OFF(1, !onoff);
		Motor_OFF(2, !onoff);
		Motor_OFF(3, !onoff);
	}
	else
	{
		Motor_OFF(m->param->num, !onoff);
	}
}
/*

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Trigger) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Trigger);
		if(Status.op == OP_SCAN_TIMER)
		{
			if(!Op.done)
			{
				TIM_Cmd(TIM3, ENABLE);
				TIM_Cmd(TIM4, DISABLE);
			}
		}

		if(Status.op == OP_SCAN_SWEEP)
		{
			if(!Op.done)
			{
				TIM_Cmd(TIM4, DISABLE);
				Op.sweepTrig = 1;
			}
		}

		if(Status.op == OP_SCAN_EXTERN)
		{
			if(Op.dir) // Y first
			{
				Op.iy++;
				if(Op.iy < Op.ny) // move Y
				{
					if(Op.ix % 2) // odd column
						Motor_MovePos(MOTOR_CH_Y, Op.ys + (Op.ny-Op.iy-1)*Op.dy);
					else // even column
						Motor_MovePos(MOTOR_CH_Y, Op.ys + Op.iy*Op.dy);

				}
				else // move X
				{
					Op.iy = 0;
					Op.ix++;
					if(Op.ix < Op.nx) // next column
					{
						Motor_MovePos(MOTOR_CH_X, Op.xs + Op.ix*Op.dx);
					}
					else
					{
						Op.done = 1;
						TIM_Cmd(TIM4, DISABLE);
					}
				}
				Op.idx = Op.ix*Op.nx + Op.iy;
			}
			else // X first
			{
				Op.ix++;
				if(Op.ix < Op.nx) // move X
				{
					if(Op.iy % 2) // odd row
						Motor_MovePos(MOTOR_CH_X, Op.xs + (Op.nx-Op.ix-1)*Op.dx);
					else // even row
						Motor_MovePos(MOTOR_CH_X, Op.xs + Op.ix*Op.dx);

				}
				else // move Y
				{
					Op.ix = 0;
					Op.iy++;
					if(Op.iy < Op.ny) // next row
					{
						Motor_MovePos(MOTOR_CH_Y, Op.ys + Op.iy*Op.dy);
					}
					else
					{
						Op.done = 1;
						TIM_Cmd(TIM4, DISABLE);
					}
				}
				Op.idx = Op.iy*Op.ny + Op.ix;
			}
		}
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		TIM3->CCR1 += Op.period;
		if(Status.op == OP_SCAN_TIMER)
		{
			if(Op.dir) // Y first
			{
				Op.iy++;
				if(Op.iy < Op.ny) // move Y
				{
					if(Op.ix % 2) // odd column
						Motor_MovePos(MOTOR_CH_Y, Op.ys + (Op.ny-Op.iy-1)*Op.dy);
					else // even column
						Motor_MovePos(MOTOR_CH_Y, Op.ys + Op.iy*Op.dy);

				}
				else // move X
				{
					Op.iy = 0;
					Op.ix++;
					if(Op.ix < Op.nx) // next column
					{
						Motor_MovePos(MOTOR_CH_X, Op.xs + Op.ix*Op.dx);
					}
					else
					{
						Op.done = 1;
						TIM_Cmd(TIM3, DISABLE);
					}
				}
				Op.idx = Op.ix*Op.nx + Op.iy;
			}
			else // X first
			{
				Op.ix++;
				if(Op.ix < Op.nx) // move X
				{
					if(Op.iy % 2) // odd row
						Motor_MovePos(MOTOR_CH_X, Op.xs + (Op.nx-Op.ix-1)*Op.dx);
					else // even row
						Motor_MovePos(MOTOR_CH_X, Op.xs + Op.ix*Op.dx);

				}
				else // move Y
				{
					Op.ix = 0;
					Op.iy++;
					if(Op.iy < Op.ny) // next row
					{
						Motor_MovePos(MOTOR_CH_Y, Op.ys + Op.iy*Op.dy);
					}
					else
					{
						Op.done = 1;
						TIM_Cmd(TIM3, DISABLE);
					}
				}
				Op.idx = Op.iy*Op.ny + Op.ix;
			}
		}

	}
}

*/
