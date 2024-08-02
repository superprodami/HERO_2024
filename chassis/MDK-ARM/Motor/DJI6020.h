#ifndef __DJI6020_H__
#define __DJI6020_H__

#include "main.h"


#define CANID_Yaw 0x205



//6020电机参数
typedef struct Motor_6020Type
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	int16_t	 	speed_rpm;    //转子转速
	int16_t		target_speed_rpm;
	int16_t  	current;  //返回电流
	int16_t		target_current;	//设定电流
	float     tempure;      //温度
	int16_t 	angle;				//机械角度
	int16_t		target_angle;	//目标角度
	uint32_t	Rx_add;			
	uint16_t RX_Frequancy;  //接收频率
} Motor_6020Type;


extern Motor_6020Type Motor_6020,Motor_encoder_6020;			//yaw轴6020



void Motor_6020_receive(Motor_6020Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_6020_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
