#ifndef __DJI6020_H__
#define __DJI6020_H__

#include "main.h"


#define CANID_PITCH_6020 0x205

#define GIMBAL_YAW_ENCODER_MIDDLE1 2615		//底盘和云台朝向相同1，指向＋y
#define GIMBAL_YAW_ENCODER_MIDDLE2 6750		//底盘和云台朝向相同2，指向-y


//6020电机参数
typedef struct 
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	float	 	speed_rpm;    //转子转速
	float		target_speed_rpm;
	float  	current;  //返回电流
	float		target_current;	//设定电流
	float     tempure;      //温度
	float 	angle;				//机械角度
	float		target_angle;	//目标角度
	uint32_t	Rx_add;			
	uint16_t RX_Frequancy;  //接收频率
} Motor_6020Type;


extern Motor_6020Type Motor_6020;


void Motor_6020_receive(Motor_6020Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_6020_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
