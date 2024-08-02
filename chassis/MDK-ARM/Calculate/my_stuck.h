#ifndef __MY_STUCK_H__
#define __MY_STUCK_H__



#include "main.h"


typedef enum									//����  or  ���
{
    VISION_AUTO_AIM  =  0,
	VISION_BUFF =  1,
}eVisionMode;

//���������ݽṹ��
typedef struct IMUType
{
	float ol[3];
	float accel[3];
	float quat[4];
	float gyro[3];
	float ol_accel[3];
	float temp;
	float yaw_target;
	float pitch_target;
}IMUType;


//�������ݽṹ��
typedef struct ChassisDataType
{

uint8_t chassismsg; //����д
	
	
}ChassisDataType;


//��������Ϣ��
typedef struct RobotDataType
{

	uint8_t robotmsg; //����д

	
	
}RobotDataType;


//Ħ������Ϣ
typedef struct ShootDataType
{
	float Friction_Motor_Speed_Set;		//����Ħ�������õĵ���
	uint8_t	shoot_mode;								//0������ģʽ		1������ģʽ
	uint8_t	use_vision;								//ʹ���Ӿ�
	uint8_t use_vision_has_shoot;			//�Ӿ�ģʽ���Ƿ��Ѵ�
	uint8_t vision_mode;							//0:������				1������
}ShootDataType;


//��̨��Ϣ
typedef struct GimbalDataType
{
	IMUType						IMUData;
	RobotDataType			RobotData;
	ShootDataType			ShootData;
}GimbalDataType;





#endif

