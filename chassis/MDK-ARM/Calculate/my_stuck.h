#ifndef __MY_STUCK_H__
#define __MY_STUCK_H__



#include "main.h"


typedef enum									//自瞄  or  打符
{
    VISION_AUTO_AIM  =  0,
	VISION_BUFF =  1,
}eVisionMode;

//陀螺仪数据结构体
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


//底盘数据结构体
typedef struct ChassisDataType
{

uint8_t chassismsg; //待会写
	
	
}ChassisDataType;


//机器人信息化
typedef struct RobotDataType
{

	uint8_t robotmsg; //待会写

	
	
}RobotDataType;


//摩擦轮信息
typedef struct ShootDataType
{
	float Friction_Motor_Speed_Set;		//两边摩擦轮设置的弹速
	uint8_t	shoot_mode;								//0：单发模式		1：连发模式
	uint8_t	use_vision;								//使用视觉
	uint8_t use_vision_has_shoot;			//视觉模式下是否已打弹
	uint8_t vision_mode;							//0:非自瞄				1：自瞄
}ShootDataType;


//云台信息
typedef struct GimbalDataType
{
	IMUType						IMUData;
	RobotDataType			RobotData;
	ShootDataType			ShootData;
}GimbalDataType;





#endif

