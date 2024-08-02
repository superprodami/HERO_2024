#ifndef __DJI6020_H__
#define __DJI6020_H__

#include "main.h"


#define CANID_Yaw 0x205



//6020�������
typedef struct Motor_6020Type
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	int16_t	 	speed_rpm;    //ת��ת��
	int16_t		target_speed_rpm;
	int16_t  	current;  //���ص���
	int16_t		target_current;	//�趨����
	float     tempure;      //�¶�
	int16_t 	angle;				//��е�Ƕ�
	int16_t		target_angle;	//Ŀ��Ƕ�
	uint32_t	Rx_add;			
	uint16_t RX_Frequancy;  //����Ƶ��
} Motor_6020Type;


extern Motor_6020Type Motor_6020,Motor_encoder_6020;			//yaw��6020



void Motor_6020_receive(Motor_6020Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_6020_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
