#ifndef __DJI3508_H__
#define __DJI3508_H__

#include "main.h"


#define    CANID_Shoot_l  0x201
#define  	 CANID_Shoot_r  0x202


//3508电机参数
typedef struct
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	float	 	speed_rpm;    //转子转速
	float	 	target_speed_rpm;
	float  	current;  //返回电流
	float		target_current;	//设定电流
	float     tempure;      //温度
	float 	angle;				//机械角度
	float  	target_angle; //目标角度 底盘电机一般速度单环
	uint32_t	Rx_add;			
	uint16_t  Rx_Frequancy;  //接收频率

} Motor_3508Type;

extern Motor_3508Type Motor_3508[2];

void Motor_3508_receive(Motor_3508Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_3508_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
