#ifndef __SHOOTTASK_H__
#define __SHOOTTASK_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
		SHOOT_Init = 0x00,
		SHOOT_Run  = 0x01,
		SHOOT_STOP = 0x02
}Shoot_State_t;

typedef enum
{
		NORMAL    = 0x00,
		SINGLE 		= 0x01,
		BERSERK	  = 0x02,  //����
		STOP      = 0x03   //����or����ϵͳ�µ�
}Shoot_Action_t;

typedef struct
{
	bool      wheel_shoot_flag; 		//�Ƿ�ִ�У�ȷ������
	bool      left_shoot_flag;      //ͬ��
	bool      shoot_finish_flag;   //���Ƿ����
	bool      shoot_lock_flag;     //�Ƿ񿨵�
	bool	    judge_on_flag;			//����ϵͳ���µ�
	uint16_t  shoot_finish_time;   //�ж��Ƿ���ɴ򵯣�����ʱ����Ϊ����
  uint16_t  shoot_stop_time;     //����ʱ��
	uint16_t  R_press_time;
	portTickType ATUO_time;
	portTickType ATUO_time_last;

}Shoot_flag_t;

typedef struct
{
	Shoot_State_t 	State;	
	Shoot_Action_t  Action;
	Shoot_flag_t 		flag;
}Shoot_t;


extern Shoot_t Shoot;

#endif


