#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"

/*��̨״̬��*/
typedef enum
{
	GIMB_Init     = 0x00,
	GIMB_Stop     = 0x01,
	GIMB_Remote   = 0x02,
	GIMB_Keyboard = 0x03
}GIMBALState;


//���������ݽṹ��
typedef struct 
{
	float ol[3];     //�ֱ���Yaw��Pitch��Roll
	float accel[3];
	float quat[4];
	float gyro[3];
	float ol_accel[3];
	float temp;
	float pitch_target;
	uint8_t pitch_ok;     //yaw�Ƿ񵽴�����ָ��λ��(�����ж�)
}IMU_t;


/*����ģʽ*/
typedef enum  
{
	
   GIMBAL_NORMAL = 0x00,  //����
	 GIMBAL_AUTO   = 0x01,  //����
}Gimbal_action_t;

/*����ģʽ*/
typedef enum  
{
   vertical = 0x00,  //��ֱ
	 level    = 0x01,  //ˮƽ
}Hanging_Mode_t;


typedef struct Gimbal_flag_t
{

	bool remotelose_flag; //ң����ʧ��
}Gimbal_flag_t;


//��̨��Ϣ
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

