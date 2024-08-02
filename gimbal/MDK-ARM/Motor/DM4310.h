#ifndef __DM4310_H__
#define __DM4310_H__

#include "main.h"



#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式
#define CANID_PITCH  0x007
#define PITCH_ContrlID 0x107





typedef struct 
{
	uint32_t 	ID;           //CAN ID
	uint8_t 	HCAN;         //CAN1 or CAN2
	int16_t 	Error;	      //error反馈 
	float	 		speed_rpm;    //转子转速
	float			target_speed_rpm;
	float	  	torque;  //返回力矩
	float		target_torque;	//设定力矩
	float     tempure;      //温度
	float 	angle;				//机械角度
	float		target_angle;	//目标角度
	uint32_t	Rx_add;			
	uint16_t  RX_Frequancy;  //接收频率
	uint32_t  Tx_add;
	uint16_t  Tx_Frequancy;  //控制频率
	
} Motor_4310Type;




#define P_MIN -3.141593		//位置最小值
#define P_MAX 3.141593		//位置最大值
#define V_MIN -30			//速度最小值
#define V_MAX 30			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -10			//转矩最大值
#define T_MAX 10			//转矩最小值



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

