#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"

/*云台状态机*/
typedef enum
{
	GIMB_Init     = 0x00,
	GIMB_Stop     = 0x01,
	GIMB_Remote   = 0x02,
	GIMB_Keyboard = 0x03
}GIMBALState;


//陀螺仪数据结构体
typedef struct 
{
	float ol[3];     //分别是Yaw，Pitch，Roll
	float accel[3];
	float quat[4];
	float gyro[3];
	float ol_accel[3];
	float temp;
	float pitch_target;
	uint8_t pitch_ok;     //yaw是否到达自瞄指定位置(开火判断)
}IMU_t;


/*吊射模式*/
typedef enum  
{
	
   GIMBAL_NORMAL = 0x00,  //正常
	 GIMBAL_AUTO   = 0x01,  //自瞄
}Gimbal_action_t;

/*吊射模式*/
typedef enum  
{
   vertical = 0x00,  //竖直
	 level    = 0x01,  //水平
}Hanging_Mode_t;


typedef struct Gimbal_flag_t
{

	bool remotelose_flag; //遥控器失联
}Gimbal_flag_t;


//云台信息
typedef struct GimbalDataType
{
	GIMBALState       State;
	Gimbal_flag_t	    flag;
	IMU_t							IMUData;
	Hanging_Mode_t		Hanging_Mode;
	Gimbal_action_t   Action;
	uint8_t           Robot_color;
}Gimbal_t;

extern Gimbal_t Gimbal;

#endif

