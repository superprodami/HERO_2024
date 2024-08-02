#include "ShootFun.h"
#include "dbus.h"
#include "ShootTask.h"
#include "Chassis_Task.h"
#include "pid.h"
#include "judge.h"
#include "DM4310.h"
#include "Board_can.h"
#include "math.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Gimbal_task.h"
#include "board_uart.h"
#include "my_fun.h"

/*��������or����*/
void Shoot_Statemachine_2_Init(Shoot_t* shoot);
void Shoot_Statemachine_2_Run(Shoot_t* shoot);
void Shoot_Statemachine_2_Stop(Shoot_t* shoot);




void Shoot_Init(Shoot_t* shoot)
{
	Shoot_flag_Init(shoot);
	float DM4310_Rammer_angloop[3]	  = {18,0.005,5};		
	float DM4310_Rammer_spdloop[3] 		= {0.35,0.08,1.5};//{0.4,0.1,1.5};

	DM4310_Rammer_angleloop.Slop      = 0.5;
	DM4310_Rammer_speedloop.Slop      = 3;

	
	PID_init(&DM4310_Rammer_angleloop,PID_POSITION,DM4310_Rammer_angloop,10,5);
	PID_init(&DM4310_Rammer_speedloop,PID_POSITION,DM4310_Rammer_spdloop,10,3);

	Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;

}

//flag�Լ�״̬��ʼ��
void Shoot_flag_Init(Shoot_t* shoot)
{
	DM4310_Rammer_angleloop.flag_Slop	= 1;
	DM4310_Rammer_speedloop.flag_Slop = 1;
	shoot->flag.shoot_finish_flag 		= 1;	
	shoot->flag.left_shoot_flag 			= 0;
	shoot->flag.wheel_shoot_flag 			= 0;	
	shoot->flag.shoot_lock_flag  			= 0;
	
	shoot->State 	= SHOOT_Init;
	shoot->Action = NORMAL;
}



//�����������pid
void Clean_Shoot_CtrlMsg(Shoot_t* shoot)
{
		Motor_4310_Rammer.target_torque = 0;
		PID_clear(&DM4310_Rammer_angleloop);
		PID_clear(&DM4310_Rammer_speedloop);
}


/****************************************************************������ģʽѡ��****************************************************************/
void Shoot_Mode_Choose(Shoot_t* shoot)
{
	if(RC_Ctl.rc.s1 == 3 && RC_Ctl.rc.s2 == 3) //����ң��ģʽ�¿���Ħ���ֺ����
		Shoot_Mode_choose_remote(shoot);
	else if(RC_Ctl.rc.s1 == 3 && RC_Ctl.rc.s2 == 1)
	  Shoot_Mode_choose_keyboard(shoot);
}


/*����ģʽѡ��*/
void Shoot_Mode_choose_keyboard(Shoot_t* shoot)
{
	
	if((!IF_KEY_PRESSED_R)&&shoot->Action == BERSERK)  //��ģʽȡ����ʱ�޸�Ŀ��Ƕ�
		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;	    
	
//	if(!(shoot->flag.shoot_lock_flag||Revolver_Heat_Limit()))  //������or����������ֱ������ѡ��
	if(!Revolver_Heat_Limit())  //������or����������ֱ������ѡ��		
	{
		
		if(Gimbal.Action == GIMBAL_NORMAL)
		{		//���&&ȷ������&&������ϴδ�&&(������or�����ҵ�λ)
				if(IF_MOUSE_PRESSED_LEFT&&(!shoot->flag.left_shoot_flag)&&shoot->flag.shoot_finish_flag)
				{
				shoot->Action = SINGLE;	
				}
			else if(!IF_MOUSE_PRESSED_LEFT)
				{
				shoot->flag.left_shoot_flag = 0;
				shoot->Action = NORMAL;		
				}
		}
		else if(Gimbal.Action == GIMBAL_AUTO)
		{				
			shoot->flag.ATUO_time = xTaskGetTickCount();
		
			if((Gimbal.IMUData.find_bool == '1')&&(Gimbal.IMUData.yaw_ok&&Gimbal.IMUData.pitch_ok)&&(shoot->flag.shoot_finish_flag)&&(shoot->flag.ATUO_time-shoot->flag.ATUO_time_last >1500))
				shoot->Action = SINGLE;	
			else 
				shoot->Action = NORMAL;	
		}
	}
	else
	{
//		if(Revolver_Heat_Limit()) //��������
//			shoot->Action = NORMAL;
//		else											//��������
//			shoot->Action = STOP; //����
			shoot->Action = NORMAL;

	}

	
	if(IF_KEY_PRESSED_R)     //����R������ģʽ(��ǰ���if�ֿ�,����R�򸲸�ǰ���״̬)
		shoot->flag.R_press_time++;
	else
		shoot->flag.R_press_time=0;
	if(shoot->flag.R_press_time>250)
		shoot->Action = BERSERK;		
}

/*ң����ģʽѡ��*/
void Shoot_Mode_choose_remote(Shoot_t* shoot)
{
//		if(!shoot->flag.shoot_lock_flag)  //��������ֱ������ѡ��
//			{
				if(RC_Ctl.rc.wheel>500&&shoot->flag.wheel_shoot_flag==0)
				{
					shoot->Action = SINGLE;
				}
				else if(RC_Ctl.rc.wheel<100&&RC_Ctl.rc.wheel>-100)
				{
					shoot->flag.wheel_shoot_flag = 0;
					shoot->Action = NORMAL;
				}
//			}
//		else
//		{
//			shoot->Action = STOP;
//		}
}


/****************************************************************�����̼���****************************************************************/
void Shoot_clc(void)
{
	switch(Shoot.Action)
	{
		case NORMAL:
		{
			Shoor_Ctl_NORMAL(&Shoot);		//���㲢���ֵ�ǰλ��				
			break;
		}
		case SINGLE:
		{
			Shoor_Ctl_SINGLE(&Shoot);		//����
			break;
		}
		case BERSERK:
		{			
			Shoor_Ctl_BERSERK(&Shoot);	//һ����Ƶ
			break;
		}
		case STOP:
		{			
			Shoor_Ctl_STOP(&Shoot);			//�����ڷ������������ƻ���˳���˫��������ȷ�����о������Բ����Ǳ��������ʱ������������
			break;
		}
	}

}
void Shoor_Ctl_NORMAL(Shoot_t* shoot)
{
	AngleLoop_f(&Motor_4310_Rammer.target_angle,2*PI);
	PID_Angle_calc(&DM4310_Rammer_angleloop,Motor_4310_Rammer.angle,Motor_4310_Rammer.target_angle,-PI,PI);
	Motor_4310_Rammer.target_speed_rpm = DM4310_Rammer_angleloop.out;
	PID_calc(&DM4310_Rammer_speedloop,Motor_4310_Rammer.speed_rpm,Motor_4310_Rammer.target_speed_rpm);
	Motor_4310_Rammer.target_torque = DM4310_Rammer_speedloop.out;
	//�����ж�
	if(fabsf(Motor_4310_Rammer.target_angle - Motor_4310_Rammer.angle)<0.035f) //1������
	{
		shoot->flag.shoot_finish_flag = 1;
		shoot->flag.shoot_finish_time = 0;
	}
	if(!shoot->flag.shoot_finish_flag)
	{
		shoot->flag.shoot_finish_time++;
			if(shoot->flag.shoot_finish_time>2500)   
			{	
//				shoot->flag.shoot_lock_flag = 1;
				shoot->flag.shoot_finish_flag = 1;
				Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
			}
	}
	
	//����ϵͳ��������ϵ�󱣳���λ����ֹ�ϵ緢��
	if(!Game_Robot_State.power_management_shooter_output)
		shoot->flag.judge_on_flag = 0;
	else if(Game_Robot_State.power_management_shooter_output == 1&&shoot->flag.judge_on_flag == 0)
	{
		shoot->flag.judge_on_flag = 1;
		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
	}
}




void Shoor_Ctl_SINGLE(Shoot_t* shoot)
{
	if(Gimbal.Action == GIMBAL_NORMAL)
	{
	if((!shoot->flag.left_shoot_flag)||(!shoot->flag.wheel_shoot_flag))  //δִ�й�
	{
		Motor_4310_Rammer.target_angle += 1.04719f;	
		shoot->flag.shoot_finish_flag = 0;		
		shoot->flag.left_shoot_flag  	= 1;
		shoot->flag.wheel_shoot_flag 	= 1;		
		shoot->Action = NORMAL;	
	}
	}
	else if (Gimbal.Action == GIMBAL_AUTO)
	{
				Motor_4310_Rammer.target_angle += 1.04719f;	
				shoot->flag.shoot_finish_flag = 0;	
				shoot->Action = NORMAL;	
				shoot->flag.ATUO_time_last = shoot->flag.ATUO_time;
	}
}

void Shoor_Ctl_BERSERK(Shoot_t* shoot)
{
	Motor_4310_Rammer.target_speed_rpm = 4;
	PID_calc(&DM4310_Rammer_speedloop,Motor_4310_Rammer.speed_rpm,Motor_4310_Rammer.target_speed_rpm);
	Motor_4310_Rammer.target_torque = DM4310_Rammer_speedloop.out;
}

void Shoor_Ctl_STOP(Shoot_t* shoot)
{
	Motor_4310_Rammer.target_torque = 0;
	shoot->flag.shoot_stop_time++;
	if(shoot->flag.shoot_stop_time>500) 		//����1s
	{
		shoot->flag.shoot_stop_time = 0;	
		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
		shoot->flag.shoot_finish_time = 0;
		shoot->flag.shoot_lock_flag = 0;
		shoot->Action = NORMAL;
	}
}

/****************************************************************�����̱���****************************************************************/

/*����������Normalģʽ��*/


/**
  * @brief  ǹ����������
  * @param  void
  * @retval �����Ƿ���
  * @attention  ����Ҫ����һ�²���,����ʣ��ɷ����������ջ�
  *             
  */
bool Revolver_Heat_Limit(void)
{
	static bool Heat_Limit = 0; //�Ƿ񴥷���������
	int16_t Q_max,Q_now,Q_left;
	Q_max = Game_Robot_State.shooter_barrel_heat_limit;
	Q_now = Power_Heat_Data.shooter_id1_42mm_cooling_heat;
	Q_left = Q_max-Q_now;
    if(Q_left>100)
		{ 
        Heat_Limit = 0;
    }
		else
		{
        Heat_Limit = 1;
    }

    if(IF_KEY_PRESSED_R)//�������� һ����Ƶ
	{
		Heat_Limit = 0;
	}
	
	return Heat_Limit;	

}

/****************************************************************״̬������****************************************************************/

/* ״̬�������� */
void ShootSelate(Shoot_t* shoot)
{
		if((RC_Ctl.rc.s1 == 3&&RC_Ctl.rc.s2 == 3)||(RC_Ctl.rc.s1 == 3&&RC_Ctl.rc.s2 == 1))
			Shoot_Statemachine_2_Run(shoot);
		else if(RC_Ctl.rc.s1 == 2 && RC_Ctl.rc.s2 == 2)
			Shoot_Statemachine_2_Stop(shoot);
		
		if(Chassis.flag.remotelose_flag)//ң����ʧ������
			Shoot_Statemachine_2_Stop(shoot);

}


/* �Ѳ���״̬�������ʼ��ģʽ */
void Shoot_Statemachine_2_Init(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Init)
	{
		//����һ��
		Clean_Shoot_CtrlMsg(shoot);
		
		//�й�ȥ
		shoot->State = SHOOT_Init;
	}
}


/* �Ѳ�����״̬������Runģʽ */
void Shoot_Statemachine_2_Run(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_Run)
	{
		//����һ�£��������pid
		Clean_Shoot_CtrlMsg(shoot);
		Motor_4310_Rammer.target_angle = Motor_4310_Rammer.angle;
		//�й�ȥ
		shoot->State = SHOOT_Run;
	}
}

/* �Ѳ�����״̬������Stopģʽ */
void Shoot_Statemachine_2_Stop(Shoot_t* shoot)
{
	if(shoot->State != SHOOT_STOP)
	{
		//����һ�£��������pid
		Clean_Shoot_CtrlMsg(shoot);
		
		//�й�ȥ
		shoot->State = SHOOT_STOP;
	}
}

