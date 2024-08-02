/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "main.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
	  float bata;
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

	  float I_separation; //积分分离	
	
	
    float max_out;  
    float max_iout; 

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    bool  flag_Slop; //限幅，使拨弹盘更硬 flag为1开启，否则忽略。
		float Slop;
} pid_type_def;





/*变量*/
extern pid_type_def  DM4310_Pitch_angleloop,DM4310_Pitch_speedloop;
extern pid_type_def DJI6020_Pitch_angleloop,DJI6020_Pitch_speedloop;

extern pid_type_def M3508_speedloop[2];


/*函数*/

extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_calc(pid_type_def *pid, float ref, float set);
extern float PID_Angle_calc(pid_type_def *pid, float ref, float set, float min, float max);
extern void PID_clear(pid_type_def *pid);
extern void PID_change(pid_type_def	*pid, float PID[3]);












#endif
