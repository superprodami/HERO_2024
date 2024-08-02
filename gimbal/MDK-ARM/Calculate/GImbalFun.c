#include "GimbalFun.h"
#include "Board_can.h"
#include "pid.h"
#include "can.h"
#include "DJI6020.h"
#include "DM4310.h"
#include "my_fun.h"
#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Filter.h"
#include "vision.h"
#include "arm_math.h"

/*500Hz*/
float  DM4310_PITCH_spdpid[3] = {0.35,0.004,0.001};
float  DM4310_PITCH_angpid[3] = {2,0.08,10};

float  DJI6020_PITCH_spdpid[3] = {100,0.4,0};
float  DJI6020_PITCH_angpid[3] = {25,2,2000};//{80, 0.1, 900};

/*1khz*/
//float  DM4310_PITCH_spdpid[3] = {0.6,0.006,0.001};//{0.48,0.006,0.3};//{0.52,0.008,0.5};
//float  DM4310_PITCH_angpid[3] = {1.5,0.01,15};//{1,0,6};//{1.5,0.02,4};//{1.2,0.01,2.8};  


/*��̨��Ϣ��ʼ��*/  
void	 Gimbal_Init(Gimbal_t* gimbal)
{
	Gimbal_flag_Init(gimbal);	

	//��ͨ�˲�
	float IIR_LPF_IMUY[8]={1.0f,1.0f,0.0f,1.0f,-0.7265425f,0.0f,0.1367287f, 1.0f};
	IIR_Init(IIR_LPF_IMUY,&IMUYFilter);
	float IIR_LPF_SPEED[8]={1.0f,1.0f,0.0f,1.0f,-0.5095254f,0.0f,0.2452372f,1.0f};
	IIR_Init(IIR_LPF_SPEED,&SPEEDFilter);
	
	PID_init(&DM4310_Pitch_angleloop,PID_POSITION,DM4310_PITCH_angpid,10,5);
	PID_init(&DM4310_Pitch_speedloop,PID_POSITION,DM4310_PITCH_spdpid,6,3);

	PID_init(&DJI6020_Pitch_angleloop,PID_POSITION,DJI6020_PITCH_angpid,300,30);
	PID_init(&DJI6020_Pitch_speedloop,PID_POSITION,DJI6020_PITCH_spdpid,25000,7000);
	DJI6020_Pitch_speedloop.I_separation = 200;
	gimbal->IMUData.pitch_target = 0;

}

void Gimbal_flag_Init(Gimbal_t* gimbal)
{

	gimbal->flag.remotelose_flag = 1;

	gimbal->State = GIMB_Init;
	gimbal->Action = GIMBAL_NORMAL;
	gimbal->Hanging_Mode = level ; //VB����ģʽ ˮƽ	
}

void Clean_Gimbal_CtrlMsg(Gimbal_t* gimbal)
{
	//�����������
	Motor_4310_PITCH.target_torque = 0;
	Motor_6020.target_current = 0;

	//���PID

	PID_clear(&DM4310_Pitch_angleloop);
	PID_clear(&DM4310_Pitch_speedloop);
	
	PID_clear(&DJI6020_Pitch_angleloop);
	PID_clear(&DJI6020_Pitch_speedloop);

}

/*����ѡ����̨ģʽ*/
void Gimbal_Mode_Choose(Gimbal_t* gimbal)
{
		//�Ҽ���ס����
    if(IF_MOUSE_PRESSED_RIGH)
			gimbal->Action  = GIMBAL_AUTO;
		else
			gimbal->Action = GIMBAL_NORMAL;

}



/* Keyboard������̨���� */
void KeyboardControlGimbal(Gimbal_t* gimbal)
{
    static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������?
    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, ����������ģʽ�����ͳ��Pitchƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����
			if(gimbal->Action  == GIMBAL_NORMAL)
			{
        if(MOUSE_Y_MOVE_SPEED != 0)
        {
           Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * -0.006f;//pitch�Ծ�ʹ�û�еģʽ
        }
        else if(MOUSE_Y_MOVE_SPEED == 0)
        {
            Mouse_Pitch_Stop ++ ;
            if(Mouse_Pitch_Stop > 5) //��곤ʱ��ͣ����ֹͣ�ƶ�
            {
                Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
            }
        }
        else
        {
            Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
        }
        gimbal->IMUData.pitch_target = RampInc_float( &Mouse_Gyro_Pitch, gimbal->IMUData.pitch_target, 10);
			}
			else if(gimbal->Action == GIMBAL_AUTO)

			{
				if(VisionValue.find_bool =='1')
				{
				gimbal->IMUData.pitch_target = 180.f*VisionValue.pitch_value.value/PI;			
				if(fabs(INS_angle[1] - VisionValue.pitch_value.value)<(0.5f*0.075f/VisionValue.distance_t.distance))
				 gimbal->IMUData.pitch_ok = 1;
				else
					gimbal->IMUData.pitch_ok = 0;
				}
				else
				{
					gimbal->IMUData.pitch_target = 180.f*VisionValue.pitch_value.value/PI;
								gimbal->IMUData.pitch_ok = 0;

			}}
			

}

/********************Remote��̨����********************/
void RemoteControlGimbal(Gimbal_t* gimbal)
{
		if(gimbal->Action == GIMBAL_NORMAL)
		{
			gimbal->IMUData.pitch_target -= (float)RC_Ctl.rc.ch1 / 2000;
		}
}


/*********************��̨�������*********************/
void Gimbal_clc(Gimbal_t* gimbal)
{
	
				if(IF_KEY_PRESSED_V&&IF_KEY_PRESSED_CTRL)
					{
						gimbal->IMUData.pitch_target -= 0.001f;
					}
					if(IF_KEY_PRESSED_B&&IF_KEY_PRESSED_CTRL)
						gimbal->IMUData.pitch_target += 0.001f;		
				

			//Pitch�����
			LimitValue_f(&gimbal->IMUData.pitch_target, 15, -40);   //�޷�  			
			Motor_6020.target_angle = gimbal->IMUData.pitch_target;
			PID_Angle_calc(&DJI6020_Pitch_angleloop,IMU_angle[1], Motor_6020.target_angle, -180, 180); //���ػ�PID		
	
			Motor_6020.target_speed_rpm =  DJI6020_Pitch_angleloop.out;

			PID_calc(&DJI6020_Pitch_speedloop,Motor_6020.speed_rpm ,Motor_6020.target_speed_rpm);
			Motor_6020.target_current  = DJI6020_Pitch_speedloop.out;
		
//			Motor_4310_PITCH.target_angle = gimbal->IMUData.pitch_target;
//			PID_Angle_calc(&DM4310_Pitch_angleloop,Motor_4310_PITCH.angle, Motor_4310_PITCH.target_angle, -180, 180); //���ػ�PID		
//	
//			Motor_4310_PITCH.target_speed_rpm = - DM4310_Pitch_angleloop.out;

//			PID_calc(&DM4310_Pitch_speedloop,Motor_4310_PITCH.speed_rpm ,Motor_4310_PITCH.target_speed_rpm);
//			Motor_4310_PITCH.target_torque = DM4310_Pitch_speedloop.out;

		
}







/************************״̬������*****************/

/* �ѵ���״̬������Stopģʽ */
void Gimbal_Statemachine_2_Stop(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Stop)
	{
		//����һ�£��������pid
		Clean_Gimbal_CtrlMsg(gimbal);
		
		//�й�ȥ
		gimbal->State = GIMB_Stop;
	}
}

/* �ѵ���״̬������remoteģʽ */
void Gimbal_Statemachine_2_remote(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Remote)
	{
		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Remote;
	}
}


/* �ѵ���״̬���������ģʽ */
void Gimbal_Statemachine_2_keyboard(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Keyboard)
	{
		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Keyboard;
	}
}

/* �ѵ���״̬�������ʼ��ģʽ */
void Gimbal_Statemachine_2_Init(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Init)
	{
		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Init;
	}
}

float angle_test;
		
/* ״̬�������� */
void GimbalSelate(Gimbal_t* gimbal)
{
		angle_test = Yaw_angle_t.Yaw_angle.value;
	if(RC_Ctl.rc.s1 == 1)
	{
		Gimbal_Statemachine_2_remote(gimbal);
	}
	else if(RC_Ctl.rc.s1 == 3 && RC_Ctl.rc.s2 == 1)
	{	
		Gimbal_Statemachine_2_keyboard(gimbal);
	}
	else if(RC_Ctl.rc.s1 == 2 && RC_Ctl.rc.s2 == 2)
	{
		Gimbal_Statemachine_2_Stop(gimbal);
	}

		if(gimbal->flag.remotelose_flag) 
		Gimbal_Statemachine_2_Stop(gimbal);
	if((gimbal->flag.remotelose_flag) ||(RC_Ctl.Rx_Frequency[0]==0)||(RC_Ctl.Rx_Frequency[1]==0)||(RC_Ctl.Rx_Frequency[2]==0)) //ң����orͨ��ʧ��
		Gimbal_Statemachine_2_Stop(gimbal);
		
}


