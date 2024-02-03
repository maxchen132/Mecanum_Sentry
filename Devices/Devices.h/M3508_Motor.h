/**
 * @file M3508_Motor.h
 * @author Leo Liu
 * @brief header file for M3508
 * @version 1.0
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "can.h"
#include "CAN_Setup.h"
#include "Motor_Init.h"
#include <stdint.h>
#include <stdio.h>

#define M3508_CHASSIS_FIRST_ID 			0x201 		//First ID for chassis motors
#define M3508_CHASSIS_SECOND_ID 		0x202 		//First ID for chassis motors
#define M3508_CHASSIS_THIRD_ID 			0x203 		//First ID for chassis motors
#define M3508_CHASSIS_FOURTH_ID			0x204 		//Fourth ID for chassis motors
#define M3508_FRIC_WHEEL_LEFT_UP_ID 	0x201 		//Left up friction wheel motor ID
#define M3508_FRIC_WHEEL_LEFT_DOWN_ID 	0x202 		//Left Down friction wheel motor ID
#define M3508_FRIC_WHEEL_RIGHT_UP_ID 	0x203		//Right Up friction wheel motor ID
#define M3508_FRIC_WHEEL_RIGHT_DOWN_ID 	0x204		//Right Downfriction wheel motor ID
#define M3508_SPEED_MAX 				9000.0f 	//M3508 maximum speed
#define M3508_OUTPUT_MAX 				16384.0f	//M3508 maximum output current
#define M3508_MECH_ANGLE_MAX			8192.0f

#define M3508_Func_GroundInit					   \
		{																	   \
				&M3508_Get_Data,			   \
				&M3508_Send_Data,		 \
				&Check_M3508,			   \
		}

typedef struct
{
	void (*M3508_Get_Data)(CAN_Export_Data_t RxMessage, Motor_Init_t *motor);
	void (*M3508_Chassis_Send_Data)(int16_t Motor_1_Current,
										int16_t Motor_2_Current,
										int16_t Motor_3_Current,
										int16_t Motor_4_Current);
	void (*Check_M3508)(void);
}M3508_Func_t;

extern M3508_Func_t M3508_Func;

#endif
