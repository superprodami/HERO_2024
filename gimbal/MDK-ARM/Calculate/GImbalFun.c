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


/*云台信息初始化*/  
void	 Gimbal_Init(Gimbal_t* gimbal)
{
	Gimbal_flag_Init(gimbal);	

	//低通滤波
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
	gimbal->Hanging_Mode = level ; //VB吊射模式 水平	
}

void Clean_Gimbal_CtrlMsg(Gimbal_t* gimbal)
{
	//先清除控制量
	Motor_4310_PITCH.target_torque = 0;
	Motor_6020.target_current = 0;

	//清掉PID

	PID_clear(&DM4310_Pitch_angleloop);
	PID_clear(&DM4310_Pitch_speedloop);
	
	PID_clear(&DJI6020_Pitch_angleloop);
	PID_clear(&DJI6020_Pitch_speedloop);

}

/*键鼠选择云台模式*/
void Gimbal_Mode_Choose(Gimbal_t* gimbal)
{
		//右键按住自瞄
    if(IF_MOUSE_PRESSED_RIGH)
			gimbal->Action  = GIMBAL_AUTO;
		else
			gimbal->Action = GIMBAL_NORMAL;

}



/* Keyboard控制云台函数 */
void KeyboardControlGimbal(Gimbal_t* gimbal)
{
    static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响?
    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, 键盘陀螺仪模式下鼠标统计Pitch偏移量,此值会自己缓慢减小,防止甩头过快
			if(gimbal->Action  == GIMBAL_NORMAL)
			{
        if(MOUSE_Y_MOVE_SPEED != 0)
        {
           Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * -0.006f;//pitch仍旧使用机械模式
        }
        else if(MOUSE_Y_MOVE_SPEED == 0)
        {
            Mouse_Pitch_Stop ++ ;
            if(Mouse_Pitch_Stop > 5) //鼠标长时间停留，停止移动
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

/********************Remote云台控制********************/
void RemoteControlGimbal(Gimbal_t* gimbal)
{
		if(gimbal->Action == GIMBAL_NORMAL)
		{
			gimbal->IMUData.pitch_target -= (float)RC_Ctl.rc.ch1 / 2000;
		}
}


/*********************云台输出计算*********************/
void Gimbal_clc(Gimbal_t* gimbal)
{
	
				if(IF_KEY_PRESSED_V&&IF_KEY_PRESSED_CTRL)
					{
						gimbal->IMUData.pitch_target -= 0.001f;
					}
					if(IF_KEY_PRESSED_B&&IF_KEY_PRESSED_CTRL)
						gimbal->IMUData.pitch_target += 0.001f;		
				

			//Pitch轴计算
			LimitValue_f(&gimbal->IMUData.pitch_target, 15, -40);   //限幅  			
			Motor_6020.target_angle = gimbal->IMUData.pitch_target;
			PID_Angle_calc(&DJI6020_Pitch_angleloop,IMU_angle[1], Motor_6020.target_angle, -180, 180); //带回环PID		
	
			Motor_6020.target_speed_rpm =  DJI6020_Pitch_angleloop.out;

			PID_calc(&DJI6020_Pitch_speedloop,Motor_6020.speed_rpm ,Motor_6020.target_speed_rpm);
			Motor_6020.target_current  = DJI6020_Pitch_speedloop.out;
		
//			Motor_4310_PITCH.target_angle = gimbal->IMUData.pitch_target;
//			PID_Angle_calc(&DM4310_Pitch_angleloop,Motor_4310_PITCH.angle, Motor_4310_PITCH.target_angle, -180, 180); //带回环PID		
//	
//			Motor_4310_PITCH.target_speed_rpm = - DM4310_Pitch_angleloop.out;

//			PID_calc(&DM4310_Pitch_speedloop,Motor_4310_PITCH.speed_rpm ,Motor_4310_PITCH.target_speed_rpm);
//			Motor_4310_PITCH.target_torque = DM4310_Pitch_speedloop.out;

		
}







/************************状态机控制*****************/

/* 把底盘状态机切入Stop模式 */
void Gimbal_Statemachine_2_Stop(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Stop)
	{
		//处理一下，比如清空pid
		Clean_Gimbal_CtrlMsg(gimbal);
		
		//切过去
		gimbal->State = GIMB_Stop;
	}
}

/* 把底盘状态机切入remote模式 */
void Gimbal_Statemachine_2_remote(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Remote)
	{
		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Remote;
	}
}


/* 把底盘状态机切入键盘模式 */
void Gimbal_Statemachine_2_keyboard(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Keyboard)
	{
		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Keyboard;
	}
}

/* 把底盘状态机切入初始化模式 */
void Gimbal_Statemachine_2_Init(Gimbal_t* gimbal)
{
	if(gimbal->State != GIMB_Init)
	{
		Clean_Gimbal_CtrlMsg(gimbal);
		
		gimbal->State = GIMB_Init;
	}
}

float angle_test;
		
/* 状态机控制器 */
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
	if((gimbal->flag.remotelose_flag) ||(RC_Ctl.Rx_Frequency[0]==0)||(RC_Ctl.Rx_Frequency[1]==0)||(RC_Ctl.Rx_Frequency[2]==0)) //遥控器or通信失联
		Gimbal_Statemachine_2_Stop(gimbal);
		
}


