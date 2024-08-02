#include "ChassisFun.h"
#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "dbus.h"
#include "my_fun.h"
#include "arm_math.h"
#include "math.h"
#include "DJI6020.h"
#include "DJI3508.h"
#include "Board_can.h"
#include "cap.h" 
#include "judge.h"

/*���ֱ�������*/
Chassis_Speed_t orispeed;				//����ǰԭʼĿ���ٶ�
int16_t chassis_setspeed[4];    //����Ŀ��ת��


/* �õ��ĺ������� */
extern chass_t Chassis;
void Chassis_Statemachine_2_Stop(chass_t* chassis);
void Chassis_Statemachine_2_remote(chass_t* chassis);


/*���̳�ʼ��*/
void Chassis_Init(chass_t* chassis)
{

	Chassis_flag_Init(chassis);
	
	float M3508_spdloop0[3] = {10, 0.39f ,0};
	float M3508_spdloop1[3] = {10, 0.39f ,0};
	float M3508_spdloop2[3] = {10, 0.39f ,0};
	float M3508_spdloop3[3] = {10, 0.39f ,0};
	float Chassis_Follow_pid[3] = {0.01, 0, 0};

	PID_init(&M3508_speedloop[0],PID_DELTA,M3508_spdloop0,20000,5000);
	PID_init(&M3508_speedloop[1],PID_DELTA,M3508_spdloop1,20000,5000);
	PID_init(&M3508_speedloop[2],PID_DELTA,M3508_spdloop2,20000,5000);
	PID_init(&M3508_speedloop[3],PID_DELTA,M3508_spdloop3,20000,5000);
	PID_init(&Chassis_Follow_PID,PID_POSITION,Chassis_Follow_pid,3.0,0.003);

}

//flag�Լ�״̬��ʼ��
void Chassis_flag_Init(chass_t* chassis)
{
	chassis->flag.remotelose_flag =1;
	chassis->flag.gimbal_follow = GIMBAL_HEAD;

	
	chassis->State 	= CHASS_Init;
	chassis->Action =	CHASS_NORMAL;
}



/* ��������� */
void Clean_Chassis_CtrlMsg(chass_t* chassis)
{
	//�����������
	Motor_3508[0].target_current = Motor_3508[1].target_current =Motor_3508[2].target_current =Motor_3508[3].target_current =0;
	//���PID
	PID_clear(&M3508_speedloop[0]);
	PID_clear(&M3508_speedloop[1]);
	PID_clear(&M3508_speedloop[2]);
	PID_clear(&M3508_speedloop[3]);
	PID_clear(&Chassis_Follow_PID);
}



/*************************************************************************�����ٶȴ���*************************************************************************/
static void mecanum_calc(Chassis_Speed_t *speed, int16_t *out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;

    wheel_rpm[0] = ( speed->vx - speed->vy - speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[1] = ( speed->vx + speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[2] = (-speed->vx + speed->vy - speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[3] = (-speed->vx - speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}


/*
 * @param absolute_speed ����������Ҫ���ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
 */
static void Absolute_Cal(Chassis_Speed_t *absolute_speed, float angle )
{

		Chassis_Speed_t temp_speed;
    temp_speed.vw = absolute_speed->vw;
    temp_speed.vx = absolute_speed->vx * cos(angle) - absolute_speed->vy * sin(angle);
    temp_speed.vy = absolute_speed->vx * sin(angle) + absolute_speed->vy * cos(angle);
    mecanum_calc(&temp_speed, chassis_setspeed);
}

/*�ҳ���+y/-y����Сƫ���*/
static float Find_Y_AnglePNY(void)
{
	float angle;
     if(fabsf(Motor_4310_Yaw.angle)<(PI/2))
			 angle = Motor_4310_Yaw.angle;
		 else if(Motor_4310_Yaw.angle< -(PI/2))
			 angle = -(-PI-Motor_4310_Yaw.angle);
		 else 
			 angle = -(PI-Motor_4310_Yaw.angle);

		 if(fabsf(angle)<0.0055f)
			 angle = 0;
		 
		 return angle;
}

/*�жϵ�ǰ����*/
void Gimbal_follow_Judge(chass_t* chassis)
{
		if(fabsf(Motor_4310_Yaw.angle)<(PI/2))
      chassis->flag.gimbal_follow = GIMBAL_HEAD;
		else
			chassis->flag.gimbal_follow = GIMBAL_TAIL;
}
	

/*��������ĸ������Ŀ���ٶ�*/
void RemoteControlChassis(chass_t* chassis)
{
	
	
    /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
    switch(chassis->Action)
    {
			case CHASS_FOLLOW://������̨
				Gimbal_follow_Judge(&Chassis);
			if(chassis->flag.gimbal_follow == GIMBAL_HEAD)
			{
				orispeed.vx = RAMP_float((float)RC_Ctl.rc.ch3 / 200 , orispeed.vx  , 0.1);
				orispeed.vy = RAMP_float((float)RC_Ctl.rc.ch2 / 200 , orispeed.vy  , 0.1);
			}
			else
			{
				orispeed.vx = RAMP_float(-(float)RC_Ctl.rc.ch3 / 200 , orispeed.vx  , 0.1);
				orispeed.vy = RAMP_float(-(float)RC_Ctl.rc.ch2 / 200 , orispeed.vy  , 0.1);			
			}
				if(!Gimbal.flag.Fist_encoder) //��ֹ����ģʽ��Ʈ
					orispeed.vw = RAMP_float(PID_calc(&Chassis_Follow_PID, Find_Y_AnglePNY(), 0),orispeed.vw, 0.0008); //PIDʹ���̸�����̨�ٶ�
				else
					orispeed.vw = 0;

				break;
			case CHASS_NORMAL://��������̨
				orispeed.vx = RAMP_float((float)RC_Ctl.rc.ch3 / 200 , orispeed.vx  , 0.1);
				orispeed.vy = RAMP_float((float)RC_Ctl.rc.ch2 / 200 , orispeed.vy  , 0.1);
				orispeed.vw = 0;
				break;
			case CHASS_GYROSCOPE:    //С����ģʽ
				orispeed.vx = (float)RC_Ctl.rc.ch3 / 200;
				orispeed.vy = (float)RC_Ctl.rc.ch2 / 200;
				if(RC_Ctl.rc.wheel>500)	
					orispeed.vw = -0.0032f;
				else
					orispeed.vw = 0.0032f;
				break;
    }
}



void CHASSIS_clc(chass_t* chassis)
{

		if(chassis->Action == CHASS_FOLLOW)   
      Absolute_Cal(&orispeed, 0);
		else
      Absolute_Cal(&orispeed,  -Motor_4310_Yaw.angle);

			Motor_3508[0].target_speed_rpm = -chassis_setspeed[0]; 
			Motor_3508[1].target_speed_rpm = chassis_setspeed[1]; 
			Motor_3508[2].target_speed_rpm = -chassis_setspeed[2]; 
			Motor_3508[3].target_speed_rpm = chassis_setspeed[3]; 
				
			PID_calc(&M3508_speedloop[0],Motor_3508[0].speed_rpm ,Motor_3508[0].target_speed_rpm);
			PID_calc(&M3508_speedloop[1],Motor_3508[1].speed_rpm ,Motor_3508[1].target_speed_rpm);
			PID_calc(&M3508_speedloop[2],Motor_3508[2].speed_rpm ,Motor_3508[2].target_speed_rpm);
			PID_calc(&M3508_speedloop[3],Motor_3508[3].speed_rpm ,Motor_3508[3].target_speed_rpm);
			
			Motor_3508[0].target_current = M3508_speedloop[0].out;
			Motor_3508[1].target_current = M3508_speedloop[1].out;
			Motor_3508[2].target_current = M3508_speedloop[2].out;
			Motor_3508[3].target_current = M3508_speedloop[3].out;
		
}


/*ѡ�����ģʽ*/
void Chassis_Choose_remote(chass_t* chassis)
{
	if(RC_Ctl.rc.s1 == 1)
	{
		if(RC_Ctl.rc.s2 == 1)
				chassis->Action = CHASS_GYROSCOPE;		
		else if(RC_Ctl.rc.s2 == 3)
				chassis->Action = CHASS_FOLLOW;				
		else if(RC_Ctl.rc.s2 == 2)
				chassis->Action = CHASS_NORMAL;
	}
	else
			chassis->Action = CHASS_NORMAL;
}

void Chassis_Choose_keyboard(chass_t* chassis)
{
	if(!IF_KEY_PRESSED_F)
			RC_Ctl.flag.F = 0;
	if(IF_KEY_PRESSED_F && RC_Ctl.flag.F == 0) 
	{
		RC_Ctl.flag.F = 1;
		if(chassis->Action == CHASS_FOLLOW)
		{
			chassis->Action = CHASS_NORMAL;
		}
		else
		{
			chassis->Action = CHASS_FOLLOW;
		}
	
	}

	
	if(IF_KEY_PRESSED_SHIFT)
	{
			chassis->Action = CHASS_GYROSCOPE;
	}
	else if(!IF_KEY_PRESSED_SHIFT&&chassis->Action == CHASS_GYROSCOPE)
	{
		chassis->Action = CHASS_FOLLOW;
	}
}

/**
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
/************���̸���ģʽ��һЩ��������*************/
float    Chassis_Standard_Move_Max;                 //����ǰ������ƽ������
int16_t  timeXFron, timeXBack, timeYLeft, timeYRigh;//����  s  w  d  a

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

static void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec )
{
    static portTickType  ulCurrentTime = 0;
    static uint32_t  ulDelay = 0;

	  static uint16_t w_cnt = 0;
	  static bool W = 0;
		static uint16_t s_cnt = 0;
		static bool S = 0;
    static uint16_t a_cnt = 0;
    static bool A = 0;
    static uint16_t d_cnt = 0;
    static bool D = 0;
	

    ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��

    if (ulCurrentTime >= ulDelay)//ÿ10ms�仯һ��б����
    {
        ulDelay = ulCurrentTime + 10;

        if (IF_KEY_PRESSED_W)
        {
            w_cnt = 0;
            W = 1;
            timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
        }
        else
        {
            w_cnt++;
        }
        if(w_cnt > 10)
        {
            w_cnt = 0;
            W = 0;
        }

        if (IF_KEY_PRESSED_S)
        {
            s_cnt = 0;
            S = 1;
            timeXFron = 0;//ͬ��
        }
        else
        {
            s_cnt++;
        }
        if(s_cnt > 10)
        {
            s_cnt = 0;
            S = 0;
        }

        if (IF_KEY_PRESSED_D)
        {
            d_cnt = 0;
            D=1;
            timeYRigh = 0;
        }
        else
        {
            d_cnt++;
        }
        if(d_cnt > 10)
        {
            d_cnt = 0;
            D = 0;
        }

        if (IF_KEY_PRESSED_A)
        {
            a_cnt = 0;
            A=1;
            timeYLeft = 0;
        }
        else
        {
            a_cnt++;
        }
        if(a_cnt > 10)
        {
            a_cnt = 0;
            A = 0;
        }



        Slope_Chassis_Move_Fron = (int16_t)( sMoveMax *
                                             Chassis_Key_MoveRamp( W, &timeXFron, sMoveRamp_inc, sMoveRamp_dec ) );
        
				Slope_Chassis_Move_Back = (int16_t)( -sMoveMax *
                                             Chassis_Key_MoveRamp( S, &timeXBack, sMoveRamp_inc, sMoveRamp_dec ) );

        Slope_Chassis_Move_Left = (int16_t)( -sMoveMax *
                                             Chassis_Key_MoveRamp( A, &timeYRigh, sMoveRamp_inc / 2.0f, sMoveRamp_dec ) );

        Slope_Chassis_Move_Righ = (int16_t)( sMoveMax *
                                             Chassis_Key_MoveRamp( D, &timeYLeft, sMoveRamp_inc / 2.0f, sMoveRamp_dec ) );

        if(Chassis.Action == CHASS_FOLLOW)
        {
					Gimbal_follow_Judge(&Chassis);
					if(Chassis.flag.gimbal_follow == GIMBAL_HEAD)
						{
							if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
							{
								orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //ǰ�����
								orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //���Ҽ���
								
							}else{
								orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
								orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
							}

            }
            else
            {
							if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
							{
								orispeed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //ǰ�����
								orispeed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //���Ҽ���
								
							}else{
								orispeed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
								orispeed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
							}
            }
        }
        else
        {
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
					{
						orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //ǰ�����
						orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //���Ҽ���
						
					}else{
						orispeed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
						orispeed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
					}
        }
    }
}


/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
    float  factor = 0;
    factor = 0.15 * sqrt( 0.15 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����

    if (status == 1){  //����������
		
        if (factor < 1)//��ֹtime̫��
			*time += inc;
		
    }else{  //�����ɿ�
        if (factor > 0)
        {
            *time -= dec;
            if (*time < 0)
				*time = 0;
        }
    }
    LimitValue_f(&factor,1,0);//ע��һ����float�����޷�
	
    return factor;  //ע�ⷽ��
}





/**
  * @brief  ���̿��Ƶ���ģʽ
  * @param  void
  * @retval void
  * @attention
  */
void KeyboardControlChassis(chass_t* chassis)
{
			if(IF_KEY_PRESSED_Z)
		{
			Chassis_Keyboard_Move_Calculate(4500, 10, 2000);
			
		}
		else
		{
			if(chassis->Action == CHASS_NORMAL||chassis->Action == CHASS_FOLLOW)
		{
			if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN &&JUDGE_fGetRemainEnergy()>10)
				Chassis_Keyboard_Move_Calculate(15000, 10, 2000);
			else
				Chassis_Keyboard_Move_Calculate(10000, 10, 2000);
			if(!Gimbal.flag.Fist_encoder&&chassis->Action == CHASS_FOLLOW)
			orispeed.vw = RAMP_float(PID_calc(&Chassis_Follow_PID, Find_Y_AnglePNY(), 0), orispeed.vw, 0.0008);
	else
		{
			orispeed.vw = 0;
		}					
	}
	else if(chassis->Action == CHASS_GYROSCOPE)
	{
		if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
			Chassis_Keyboard_Move_Calculate(15000, 8, 30);
		else
			Chassis_Keyboard_Move_Calculate(10000, 8, 30);
		uint16_t power_limit = JUDGE_usGetPowerLimit();

		if(power_limit <= 70)
		 orispeed.vw=RAMP_float(0.0025, orispeed.vw, 0.00005);
		else if(power_limit>70 && power_limit <100)
		 orispeed.vw=RAMP_float(0.0030, orispeed.vw, 0.00005);
		else
		 orispeed.vw=RAMP_float(0.0045, orispeed.vw, 0.00005);

	}
	}
}


void get_chassis_power_and_buffer(float *chassis_power,float *chassis_power_buffer)
{
    *chassis_power=JUDGE_fGetChassisPower();//��ȡʵʱ����
    *chassis_power_buffer=JUDGE_fGetRemainEnergy();//��ȡʣ�ཹ������
}

float test_t,test_p,test_now;

void Chassis_Power_Limit(void)
{
    uint16_t max_power_limit=50;
    float chassis_max_power=0;
    float initial_give_power[4];
    float initial_total_power=0;
    float scaled_give_power[4];
    float chassis_power=0.0f;
    float chassis_power_buffer=0.0f;
    float toque_coefficient=1.99688994e-6f;
    float k1=1.453e-07;
    //ʵ������k2   1.453e-07;  1.255e-07;           1.653e-07
    float k2=1.23e-07;
    //ʵ������k1   1.23e-07;    1.44e-07;           1.43e-07
    float constant=2.081f;
    //             4.081f;       3.343;С���ݲ���   2.081f  ���Ҳ�ǣ����ǲ���С���ݾ���
    get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
		max_power_limit=JUDGE_usGetPowerLimit();

	
	
		if(chassis_power_buffer<30)
		chassis_max_power = max_power_limit -5;
		else if(chassis_power_buffer<10)
			chassis_max_power = max_power_limit -15;
		else
			chassis_max_power = max_power_limit ;

		
    for(uint8_t i=0;i<4;i++)
    {
        initial_give_power[i]=Motor_3508[i].target_current*toque_coefficient*Motor_3508[i].speed_rpm
        +k2*Motor_3508[i].speed_rpm*Motor_3508[i].speed_rpm
        +k1*Motor_3508[i].target_current*Motor_3508[i].target_current
        +constant;
    
    
    if(initial_give_power[i]<0)
        continue;
    initial_total_power+=initial_give_power[i];
    }
    
    if(initial_total_power>chassis_max_power)
    {
    float power_scale =chassis_max_power/initial_total_power;
    for(uint8_t i=0;i<4;i++)
        {
            scaled_give_power[i]=initial_give_power[i]*power_scale;
           if(scaled_give_power[i]<0)
              continue;
              
               float b=toque_coefficient * Motor_3508[i].speed_rpm;
               float c=k2*Motor_3508[i].speed_rpm*Motor_3508[i].speed_rpm-scaled_give_power[i]+constant;
        
        if(Motor_3508[i].target_current>0)
            {
                float temp=(-b+sqrt(b*b-4*k1*c))/(2*k1);
                if(temp>16000)

                {
                  Motor_3508[i].target_current=16000;  
                }
                else Motor_3508[i].target_current=temp;
                
            }
                else 
                {
            float temp=(-b-sqrt(b*b-4*k1*c))/(2*k1);
                if(temp<-16000)
                {
                Motor_3508[i].target_current=-16000;
                }
                else Motor_3508[i].target_current=temp;
                }
            }
    }
}

/*************************************************************************״̬������*************************************************************************/

/* ״̬�������� */
void ChassisSelate(chass_t* chassis)
{
	
		if(RC_Ctl.rc.s1 == 1)
			Chassis_Statemachine_2_remote(chassis);
		else if(RC_Ctl.rc.s1 == 3&RC_Ctl.rc.s2 == 1)
			Chassis_Statemachine_2_keyboard(chassis);
		else if(RC_Ctl.rc.s1 == 2&RC_Ctl.rc.s2 == 2)
			Chassis_Statemachine_2_Stop(chassis);
		
		
		if(RC_Ctl.misstimeout > MISSING_TIMEOUT)//ң����ʧ������
		{
			chassis->flag.remotelose_flag = 1;
			Chassis_Statemachine_2_Stop(chassis);			
		}
		else
		{
			chassis->flag.remotelose_flag = 0;		
		}
}

/* �ѵ���״̬������Stopģʽ */
void Chassis_Statemachine_2_Stop(chass_t* chassis)
{
	if(chassis->State != CHASS_Stop)
	{
		//����һ�£��������pid
		Clean_Chassis_CtrlMsg(chassis);
		
		//�й�ȥ
		chassis->State = CHASS_Stop;
	}
}

/* �ѵ���״̬������remoteģʽ */
void Chassis_Statemachine_2_remote(chass_t* chassis)
{
	if(chassis->State != CHASS_Remote)
	{
		Clean_Chassis_CtrlMsg(chassis);
		
		chassis->State = CHASS_Remote;
	}
}


/* �ѵ���״̬���������ģʽ */
void Chassis_Statemachine_2_keyboard(chass_t* chassis)
{
	if(chassis->State != CHASS_Keyboard)
	{
		Clean_Chassis_CtrlMsg(chassis);
		
		chassis->State = CHASS_Keyboard;
	}
}

/* �ѵ���״̬�������ʼ��ģʽ */
void Chassis_Statemachine_2_Init(chass_t* chassis)
{
	if(chassis->State != CHASS_Init)
	{
		Clean_Chassis_CtrlMsg(chassis);
		chassis->State = CHASS_Init;
	}
}


