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
typedef struct IMUType
{
	float    IMU_Yaw_angle;
	float    yaw_target;
	float    Vision_yaw_target; //自瞄目标值
	char    find_bool; //自瞄是否找到目标
	union{
		uint8_t buff[4];
		float  distance;
	}distance_t;//装甲板距离；
	uint8_t	   pitch_ok;//自瞄到位
	uint8_t		 yaw_ok;
	uint32_t Rx_add;
	uint32_t Rx_add_2;	
	uint16_t Rx_Frequency;
	uint16_t Rx_Frequency_2;	
}IMU_t;




/*吊射模式*/
typedef enum  
{
   vertical = 0x00,  //竖直
	 level    = 0x01,  //水平
}Hanging_Mode_t;


/*吊射模式*/
typedef enum  
{
   GIMBAL_NORMAL 	= 0x00,  //正常模式
	 GIMBAL_AUTO    = 0x01,  //自瞄
}Gimbal_action_t;


#define ENCODER 0
#define IMU     1

typedef struct Gimbal_flag_t
{
  bool     Move_flag;   		//是否移动，用来消除零飘判断
	bool     Fist_encoder; 		//第一次进入编码器模式
	bool     ControlMode;      //判断是编码器模式or陀螺仪模式 0
	uint32_t boardlose_time;//板间通信失联判断
}Gimbal_flag_t;


//云台信息
typedef struct GimbalDataType
{
	GIMBALState       State;
	Gimbal_flag_t	    flag;
	IMU_t							IMUData;
	Hanging_Mode_t		Hanging_Mode;
	Gimbal_action_t   Action;
}Gimbal_t;

extern Gimbal_t Gimbal;


///***********************YAW轴云台编码器的特定值******************/
//#define GIMBAL_YAW_ENCODER_MIDDLE1   2.445     	//底盘和云台朝向相同1，指向＋y
//#define GIMBAL_YAW_ENCODER_MIDDLE2  -0.697   		//底盘和云台朝向相同2，指向-y
//#define GIMBAL_YAW_ENCODER_NINETY1   0.874      //130			//底盘和云台朝向90°，指向+90°
//#define GIMBAL_YAW_ENCODER_NINETY2  -2.267  	  //4205			//底盘和云台朝向90°，指向-90°




#endif

