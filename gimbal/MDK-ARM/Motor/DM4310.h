#ifndef __DM4310_H__
#define __DM4310_H__

#include "main.h"



#define Motar_mode 0	//����ģʽΪ����ģʽ��Ϊ0ΪIMTģʽ��Ϊ1Ϊλ���ٶ�ģʽ��Ϊ2Ϊ�ٶ�ģʽ
#define CANID_PITCH  0x007
#define PITCH_ContrlID 0x107





typedef struct 
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	int16_t 	Error;	      //error���� 
	float	 		speed_rpm;    //ת��ת��
	float			target_speed_rpm;
	float	  	torque;  //��������
	float		target_torque;	//�趨����
	float     tempure;      //�¶�
	float 	angle;				//��е�Ƕ�
	float		target_angle;	//Ŀ��Ƕ�
	uint32_t	Rx_add;			
	uint16_t  RX_Frequancy;  //����Ƶ��
	uint32_t  Tx_add;
	uint16_t  Tx_Frequancy;  //����Ƶ��
	
} Motor_4310Type;




#define P_MIN -3.141593		//λ����Сֵ
#define P_MAX 3.141593		//λ�����ֵ
#define V_MIN -30			//�ٶ���Сֵ
#define V_MAX 30			//�ٶ����ֵ
#define KP_MIN 0.0		//Kp��Сֵ
#define KP_MAX 500.0	//Kp���ֵ
#define KD_MIN 0.0		//Kd��Сֵ
#define KD_MAX 5.0		//Kd���ֵ
#define T_MIN -10			//ת�����ֵ
#define T_MAX 10			//ת����Сֵ



extern Motor_4310Type Motor_4310_PITCH;			
extern Motor_4310Type Motor_4310_Yaw;			


uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
extern void DM_enable(CAN_HandleTypeDef* hcan,Motor_4310Type motor,uint16_t ID);
extern void DM_save_zero(CAN_HandleTypeDef* hcan,uint16_t ID);
extern void Motor_4310_receive(Motor_4310Type* motor, uint8_t* temp, uint16_t CAN_ID);
extern void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
extern void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel);
extern void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);




#endif

