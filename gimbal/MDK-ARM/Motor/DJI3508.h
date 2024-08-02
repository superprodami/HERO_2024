#ifndef __DJI3508_H__
#define __DJI3508_H__

#include "main.h"


#define    CANID_Shoot_l  0x201
#define  	 CANID_Shoot_r  0x202


//3508�������
typedef struct
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	float	 	speed_rpm;    //ת��ת��
	float	 	target_speed_rpm;
	float  	current;  //���ص���
	float		target_current;	//�趨����
	float     tempure;      //�¶�
	float 	angle;				//��е�Ƕ�
	float  	target_angle; //Ŀ��Ƕ� ���̵��һ���ٶȵ���
	uint32_t	Rx_add;			
	uint16_t  Rx_Frequancy;  //����Ƶ��

} Motor_3508Type;

extern Motor_3508Type Motor_3508[2];

void Motor_3508_receive(Motor_3508Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_3508_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
