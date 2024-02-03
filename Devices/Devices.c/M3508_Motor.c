/**
 * @file M3508_Motor.c
 * @author Leo Liu
 * @brief M3508 communication
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "M3508_Motor.h"
#include <stdio.h>

Motor_Init_t M3508_Chassis[4];
Motor_Init_t M3508_Fric_Wheel[4];

void M3508_Get_Data(CAN_Export_Data_t RxMessage, Motor_Init_t *motor);
void M3508_Chassis_Send_Data(int16_t Motor_1_Current, int16_t Motor_2_Current, int16_t Motor_3_Current, int16_t Motor_4_Current);


M3508_Func_t M3508_Func = M3508_Func_GroundInit;
#undef M3508_Func_GroundInit

void M3508_Get_Data(CAN_Export_Data_t RxMessage, Motor_Init_t *motor)
{
	motor->Prev_Angle = motor->Actual_Angle;
	motor->Actual_Angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	motor->Actual_Speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	motor->Actual_Current = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
	motor->Temperature = RxMessage.CANx_Export_RxMessage[6];
	if ((motor->Actual_Angle - motor->Prev_Angle) < -6500)
		motor->Turn_Count++;
	else if ((motor->Actual_Angle - motor->Prev_Angle) > 6500)
		motor->Turn_Count--;
	motor->Total_Angle = motor->Actual_Angle + (M3508_MECH_ANGLE_MAX * motor->\Turn_Count);
	motor->Info_Update_Frame++;
}


// Send data through specified identifier
void M3508_Send_Data(int16_t Motor_1_Current, int16_t Motor_2_Current, int16_t Motor_3_Current, int16_t Motor_4_Current, CAN_HandleTypeDef hcan)
{
	CAN_Func.CAN_0x200_Send_Data(&hcan, Motor_1_Current, Motor_2_Current, Motor_3_Current, Motor_4_Current);
}


void Check_M3508(Motor_Init_t motor*)
{
	for (int i = 0; i < 4; i++)
	{
		if (motor->Info_Update_Frame < 1)
			motor->Offline_Flag = 1;
		else
			motor->Offline_Flag = 0;
		motor->Info_Update_Frame = 0;
	}
}

