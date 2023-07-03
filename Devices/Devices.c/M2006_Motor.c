/**
 * @file M2006_Motor.c
 * @author Leo Liu
 * @brief M2006 communication
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "M2006_Motor.h"
#include <stdio.h>

Motor_Init_t M2006_Trigger[2];

void M2006_Trigger_Get_Data(CAN_Export_Data_t RxMessage);
void M2006_Trigger_Send_Data(int16_t Left_Trigger_Current, int16_t Right_Trigger_Current);
void Check_M2006_Trigger(void);

M2006_Func_t M2006_Func = M2006_Func_GroundInit;
#undef M2006_Func_GroundInit

// Obatin trigger motor data from CAN
void M2006_Trigger_Get_Data(CAN_Export_Data_t RxMessage)
{
	switch (RxMessage.CAN_RxHeader.StdId)
	{
	case M2006_TRIGGER_LEFT_ID:
		M2006_Trigger[0].Prev_Angle = M2006_Trigger[0].Actual_Angle;
		M2006_Trigger[0].Actual_Angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
		M2006_Trigger[0].Actual_Speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
		if ((M2006_Trigger[0].Actual_Angle - M2006_Trigger[0].Prev_Angle) < -6500)
			M2006_Trigger[0].Turn_Count++;
		else if ((M2006_Trigger[0].Actual_Angle - M2006_Trigger[0].Prev_Angle) > 6500)
			M2006_Trigger[0].Turn_Count--;
		M2006_Trigger[0].Total_Angle = M2006_Trigger[0].Actual_Angle + (M2006_MECH_ANGLE_MAX * M2006_Trigger[0].Turn_Count);
		M2006_Trigger[0].Info_Update_Frame++;
		break;

	case M2006_TRIGGER_RIGHT_ID:
		M2006_Trigger[1].Prev_Angle = M2006_Trigger[1].Actual_Angle;
		M2006_Trigger[1].Actual_Angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
		M2006_Trigger[1].Actual_Speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
		if ((M2006_Trigger[1].Actual_Angle - M2006_Trigger[1].Prev_Angle) < -6500)
			M2006_Trigger[1].Turn_Count++;
		else if ((M2006_Trigger[1].Actual_Angle - M2006_Trigger[1].Prev_Angle) > 6500)
			M2006_Trigger[1].Turn_Count--;
		M2006_Trigger[1].Total_Angle = M2006_Trigger[1].Actual_Angle + (M2006_MECH_ANGLE_MAX * M2006_Trigger[1].Turn_Count);
		M2006_Trigger[1].Info_Update_Frame++;
		break;
	}
}

// Send trigger data through specified identifier
void M2006_Trigger_Send_Data(int16_t LEFT_Trigger_Current,
							 int16_t RIGHT_Trigger_Current)
{
	CAN_Func.CAN_0x1FF_Send_Data(&hcan2, LEFT_Trigger_Current,
								 RIGHT_Trigger_Current,
								 0,
								 0);
}

void Check_M2006_Trigger(void)
{
	for (int i = 0; i < 2; i++)
	{
		if (M2006_Trigger[i].Info_Update_Frame < 1)
			M2006_Trigger[i].Offline_Flag = 1;
		else
			M2006_Trigger[i].Offline_Flag = 0;
		M2006_Trigger[i].Info_Update_Frame = 0;
	}
}
