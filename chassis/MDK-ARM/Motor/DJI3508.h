#ifndef __DJI3508_H__
#define __DJI3508_H__

#include "main.h"


#define    CANID_Chassis0  0x201
#define  	 CANID_Chassis1  0x202
#define    CANID_Chassis2  0x203
#define    CANID_Chassis3  0x204

//3508�������
typedef struct Motor_3508Type
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	int16_t	 	speed_rpm;    //ת��ת��
	int16_t	 	target_speed_rpm;
	int16_t  	current;  //���ص���
	int16_t		target_current;	//�趨����
	float     tempure;      //�¶�
	int16_t 	angle;				//��е�Ƕ�
	int16_t  	target_angle; //Ŀ��Ƕ� ���̵��һ���ٶȵ���
	uint32_t	Rx_add;			
	uint16_t  RX_Frequancy;  //����Ƶ��
} Motor_3508Type;

extern Motor_3508Type Motor_3508[4];

void Motor_3508_receive(Motor_3508Type* motor, uint8_t* temp, uint8_t CAN_ID);
void Motor_3508_send(CAN_HandleTypeDef* hcan, uint32_t StdID, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
