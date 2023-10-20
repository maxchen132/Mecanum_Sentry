/**
 * @file Shooting_Control.c
 * @author Leo Liu
 * @brief control shooting
 * @version 1.0
 * @date 2022-07-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "Shooting_Control.h"

uint8_t niuSystick = 0;

Shooting_t Shooting;
Ramp_Calc_t Fric_Wheel_Ramp;
uint16_t counter_1 = 0;
void Shooting_Init(void);
void Turn_Friction_Wheel_On(void);
void Turn_Friction_Wheel_Off(void);
void Trigger_Get_Data(Shooting_t *Shooting);
void Shooting_Processing(Shooting_t *Shooting);
void Shooting_Disabled(void);

Shooting_Func_t Shooting_Func = Shooting_Func_GroundInit;
#undef Shooting_Func_GroundInit

void Shooting_Init(void)
{
	Shooting.Fric_Wheel.Turned_On = 0;
	Ramp_Calc_Func.Clear_Ramp(&Fric_Wheel_Ramp);
	srand(1);
}

void Trigger_Get_Data(Shooting_t *Shooting)
{
	if (State_Machine.Control_Source == Remote_Control)
	{
		// Dial wheel up for single fire
		if (DR16_Export_Data.Remote_Control.Dial_Wheel < -50)
		{
			//			Shooting->Type.Right_Burst_Flag = 0;
			//			Shooting->Type.Left_Burst_Flag = 1;
			Shooting->Type.Right_Single_Fire_Flag = 1;
		}

		// Dial wheel down for burst
		else if (DR16_Export_Data.Remote_Control.Dial_Wheel > 50)
		{
			Shooting->Type.Right_Single_Fire_Flag = 1;
			//			Shooting->Type.Right_Burst_Flag = 1;
			//			Shooting->Type.Left_Burst_Flag = 0;
		}

		// Return to middle for reset
		else
		{
			Shooting->Type.Left_Single_Fire_Flag = 0;
			Shooting->Type.Left_Burst_Flag = 0;
			Shooting->Type.Left_Single_Fired_Flag = 0;
			Shooting->Type.Right_Single_Fire_Flag = 0;
			Shooting->Type.Right_Burst_Flag = 0;
			Shooting->Type.Right_Single_Fired_Flag = 0;
		}
	}
}

void Shooting_Processing(Shooting_t *Shooting)
{
	if (Shooting->Fric_Wheel.Turned_On)
		Turn_Friction_Wheel_On();
	else
		Turn_Friction_Wheel_Off();

	// Friction wheel has to reach maximum speed before it's allowed to fire
	if (Shooting->Fric_Wheel_Ready_Flag)
	{
		if (State_Machine.Control_Source == Remote_Control)
		{
			if (Shooting->Type.Right_Single_Fire_Flag && (Shooting->Type.Right_Single_Fired_Flag == 0))
			{
				PID_Func.Clear_PID_Data(&Trigger_Angle_PID);
				PID_Func.Clear_PID_Data(&Trigger_Speed_PID);
				
                niuSystick = SysTick->VAL;
				if (1)
				{
					Shooting->Trigger.Target_Left_Angle = M2006_Trigger[LEFT_TRIGGER_INDEX].Total_Angle + LEFT_TRIGGER_DIRECTION * M2006_ANGLE_1_BULLET;
					Shooting->Type.Right_Single_Fired_Flag = 1;
				}
				else
				{
					Shooting->Trigger.Target_Right_Angle = M2006_Trigger[RIGHT_TRIGGER_INDEX].Total_Angle + RIGHT_TRIGGER_DIRECTION * M2006_ANGLE_1_BULLET;
					Shooting->Type.Right_Single_Fired_Flag = 1;
				}
                counter_1++;
			}
			// Then fire this
			else if (Shooting->Type.Right_Single_Fire_Flag && Shooting->Type.Right_Single_Fired_Flag)
			{
				/* Left */
				Shooting->Trigger.Target_Left_Speed = PID_Func.Positional_PID(&Trigger_Angle_PID, Shooting->Trigger.Target_Left_Angle, M2006_Trigger[LEFT_TRIGGER_INDEX].Total_Angle);
				M2006_Trigger[LEFT_TRIGGER_INDEX].Output_Current = PID_Func.Positional_PID(&Trigger_Speed_PID, Shooting->Trigger.Target_Left_Speed, M2006_Trigger[LEFT_TRIGGER_INDEX].Actual_Speed);
				/* Right */
				Shooting->Trigger.Target_Right_Speed = PID_Func.Positional_PID(&Trigger_Angle_PID, Shooting->Trigger.Target_Right_Angle, M2006_Trigger[RIGHT_TRIGGER_INDEX].Total_Angle);
				M2006_Trigger[RIGHT_TRIGGER_INDEX].Output_Current = PID_Func.Positional_PID(&Trigger_Speed_PID, Shooting->Trigger.Target_Right_Speed, M2006_Trigger[RIGHT_TRIGGER_INDEX].Actual_Speed);
			}

			else
			{
				M2006_Trigger[LEFT_TRIGGER_INDEX].Output_Current = 0;
				M2006_Trigger[RIGHT_TRIGGER_INDEX].Output_Current = 0;
			}
		}
	}
}

void Turn_Friction_Wheel_On(void)
{
	// Slowly ramp up the target speed to protect the motor and allows reaction time for accidental triggering
	M3508_Fric_Wheel[0].Target_Speed = FRIC_LEFT_DIRECTION * Ramp_Calc_Func.Ramp_Up(&Fric_Wheel_Ramp, FRIC_SPEED_SLOW);
	M3508_Fric_Wheel[1].Target_Speed = FRIC_RIGHT_DIRECTION * Ramp_Calc_Func.Ramp_Up(&Fric_Wheel_Ramp, FRIC_SPEED_SLOW);
	M3508_Fric_Wheel[2].Target_Speed = FRIC_RIGHT_DIRECTION * Ramp_Calc_Func.Ramp_Up(&Fric_Wheel_Ramp, FRIC_SPEED_SLOW);
	M3508_Fric_Wheel[3].Target_Speed = FRIC_LEFT_DIRECTION * Ramp_Calc_Func.Ramp_Up(&Fric_Wheel_Ramp, FRIC_SPEED_SLOW);

	M3508_Fric_Wheel[0].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[0].Target_Speed, M3508_Fric_Wheel[0].Actual_Speed);
	M3508_Fric_Wheel[1].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[1].Target_Speed, M3508_Fric_Wheel[1].Actual_Speed);
	M3508_Fric_Wheel[2].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[2].Target_Speed, M3508_Fric_Wheel[2].Actual_Speed);
	M3508_Fric_Wheel[3].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[3].Target_Speed, M3508_Fric_Wheel[3].Actual_Speed);

	if (Fric_Wheel_Ramp.Ramp_Finished_Flag)
		Shooting.Fric_Wheel_Ready_Flag = 1;
}

void Turn_Friction_Wheel_Off(void)
{
	Shooting.Fric_Wheel_Ready_Flag = 0;

	M3508_Fric_Wheel[0].Target_Speed = FRIC_LEFT_DIRECTION * Ramp_Calc_Func.Ramp_Down(&Fric_Wheel_Ramp, 0);
	M3508_Fric_Wheel[1].Target_Speed = FRIC_RIGHT_DIRECTION * Ramp_Calc_Func.Ramp_Down(&Fric_Wheel_Ramp, 0);
	M3508_Fric_Wheel[2].Target_Speed = FRIC_RIGHT_DIRECTION * Ramp_Calc_Func.Ramp_Down(&Fric_Wheel_Ramp, 0);
	M3508_Fric_Wheel[3].Target_Speed = FRIC_LEFT_DIRECTION * Ramp_Calc_Func.Ramp_Down(&Fric_Wheel_Ramp, 0);

	M3508_Fric_Wheel[0].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[0].Target_Speed, M3508_Fric_Wheel[0].Actual_Speed);
	M3508_Fric_Wheel[1].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[1].Target_Speed, M3508_Fric_Wheel[1].Actual_Speed);
	M3508_Fric_Wheel[2].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[2].Target_Speed, M3508_Fric_Wheel[2].Actual_Speed);
	M3508_Fric_Wheel[3].Output_Current = PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[3].Target_Speed, M3508_Fric_Wheel[3].Actual_Speed);

	M3508_Fric_Wheel[0].Output_Current = abs(M3508_Fric_Wheel[0].Target_Speed) < 600 ? 0 : PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[0].Target_Speed, M3508_Fric_Wheel[0].Actual_Speed);
	M3508_Fric_Wheel[1].Output_Current = abs(M3508_Fric_Wheel[1].Target_Speed) < 600 ? 0 : PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[1].Target_Speed, M3508_Fric_Wheel[1].Actual_Speed);
	M3508_Fric_Wheel[2].Output_Current = abs(M3508_Fric_Wheel[2].Target_Speed) < 600 ? 0 : PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[2].Target_Speed, M3508_Fric_Wheel[2].Actual_Speed);
	M3508_Fric_Wheel[3].Output_Current = abs(M3508_Fric_Wheel[3].Target_Speed) < 600 ? 0 : PID_Func.Positional_PID(&Fric_Wheel_PID, M3508_Fric_Wheel[3].Target_Speed, M3508_Fric_Wheel[3].Actual_Speed);
}

void Shooting_Disabled(void)
{
	Shooting.Fric_Wheel.Turned_On = 0;
	Ramp_Calc_Func.Clear_Ramp(&Fric_Wheel_Ramp);
	PID_Func.Clear_PID_Data(&Fric_Wheel_PID);
	PID_Func.Clear_PID_Data(&Trigger_Angle_PID);
	PID_Func.Clear_PID_Data(&Trigger_Speed_PID);
	DR16_Export_Data.Mouse.Click_Counter = 0;
	Shooting.Type.Left_Single_Fire_Flag = 0;
	Shooting.Type.Left_Burst_Flag = 0;
	Shooting.Type.Right_Single_Fire_Flag = 0;
	Shooting.Type.Right_Burst_Flag = 0;
}
