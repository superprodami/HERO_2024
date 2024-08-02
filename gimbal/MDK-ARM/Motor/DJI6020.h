#ifndef __DJI6020_H__
#define __DJI6020_H__

#include "main.h"


#define CANID_PITCH_6020 0x205

#define GIMBAL_YAW_ENCODER_MIDDLE1 2615		//���̺���̨������ͬ1��ָ��y
#define GIMBAL_YAW_ENCODER_MIDDLE2 6750		//���̺���̨������ͬ2��ָ��-y


//6020�������
typedef struct 
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	float	 	speed_rpm;    //ת��ת��
	float		target_speed_rpm;
	float  	current;  //���ص���
	float		target_current;	//�趨����
	float     tempure;      //�¶�
	float 	angle;				//��е�Ƕ�
	float		target_angle;	//Ŀ��Ƕ�
	uint32_t	Rx_add;			
	uint16_t RX_Frequancy;  //����Ƶ��
} Motor_6020Type;


extern Motor_6020Type Motor_6020;


void Motor_6020_receive(Motor_6020Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_6020_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
