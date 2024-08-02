#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "DJI3508.h"
#include "DM4310.h"
#include "pid.h"


typedef enum
{
	CHASS_Init     = 0x00,
	CHASS_Stop     = 0x01,
	CHASS_Remote   = 0x02,
	CHASS_Keyboard = 0x03
}ChassState;

typedef enum
{
    CHASS_NORMAL        = 0x01,  //���̲�������̨����	
    CHASS_FOLLOW 				= 0x02,	//���̸�����������
    CHASS_GYROSCOPE     = 0x03  //С����ģʽ
}Chassis_Action_t;

typedef struct
{
    float vx;
    float vy;
    float vw;
}Chassis_Speed_t;

typedef enum
{
  GIMBAL_HEAD = 0x00,
	GIMBAL_TAIL = 0x01
} Gimbal_Follow;

typedef struct
{
	bool 						remotelose_flag; //ң����ʧ����־�� missingtime�ڶ�ʱ���ж��ۼӣ��ڽ���ң��������ʱ��0�����ʧ������ʱ�䣬��Chassis�����а�ʧ����־��1or��0
	Gimbal_Follow		gimbal_follow;   //�жϵ�ǰ���淽����ͷ����ƨ��
	
}CHASS_flag_t;

typedef struct
{
	ChassState        State;
	Chassis_Action_t  Action;
	CHASS_flag_t      flag;
	struct
	{
		float MotorCurr[4];
	}CtrlMsg;
	
	
}chass_t;




extern chass_t Chassis;


/***********************������Ϣ****************************************/
#define CHASSIS_DECELE_RATIO  (3591.0f/187.0f)		//���ٱ�  670*715*450
#define LENGTH_A 218         //mm
#define LENGTH_B 232         //mm
#define WHEEL_PERIMETER 152  //mm ֱ��



#endif 


