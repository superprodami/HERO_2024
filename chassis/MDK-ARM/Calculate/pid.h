/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
	
    float max_out; 
		 
    float max_iout; 

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
    bool  flag_Slop; //�޷���ʹ�����̸�Ӳ flagΪ1������������ԡ�
		float Slop;
} pid_type_def;







/*����*/
extern pid_type_def M3508_speedloop[4],Chassis_Follow_PID;
extern pid_type_def DM4310_Rammer_angleloop,DM4310_Rammer_speedloop;
extern pid_type_def DM4310_Yaw_angleloop,DM4310_Yaw_speedloop,DM4310_Yaw_Enangleloop; 
extern pid_type_def DM4310_Yaw_Enangleloop,DM4310_Yaw_Enspeedloop; 	
extern pid_type_def buffer_pid; 




/*����*/

extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_calc(pid_type_def *pid, float ref, float set);
extern float PID_Angle_calc(pid_type_def *pid, float ref, float set, float min, float max);
extern void PID_clear(pid_type_def *pid);











#endif