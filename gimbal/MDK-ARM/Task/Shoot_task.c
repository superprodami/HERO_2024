#include "cmsis_os.h"
#include "Shoot_task.h"
#include "Gimbal_task.h"
#include "DJI3508.h"
#include "Board_can.h"
#include "pid.h"
#include "can.h"

Shoot_t Shoot;

/*因为比较简单所以懒得写状态机了*/
void Shoot_Fun(void const * argument)
{
	
	ShootInit(&Shoot);

  for(;;)
  {
		
		ShootSelate(&Shoot);
		Shoot_calc(&Shoot);
	  Motor_3508_send(&hcan1,0x200,Motor_3508[0].target_current,Motor_3508[1].target_current,0,0);

    osDelay(1);
  }
}





void ShootInit(Shoot_t* shoot)
{
	
	float M3508_speloop0[3] = {8,0.1,0.2};
	float M3508_speloop1[3] = {8,0.1,0.2};
	
	PID_init(&M3508_speedloop[0],PID_DELTA,M3508_speloop0,20000,5000);
	PID_init(&M3508_speedloop[1],PID_DELTA,M3508_speloop1,20000,5000);
	
	
	shoot->State = SHOOT_STOP;
	shoot->Shoot_target_speed = 5400;
}



void ShootSelate(Shoot_t* shoot)
{
	if((RC_Ctl.rc.s1 == 3 &&RC_Ctl.rc.s2 == 3)||(RC_Ctl.rc.s1 == 3 &&RC_Ctl.rc.s2 == 1)) 
	{
		shoot->State = SHOOT_RUN;
	}
	else if ((RC_Ctl.rc.s1 == 2 &&RC_Ctl.rc.s2 == 2)||(RC_Ctl.rc.s1 == 3 &&RC_Ctl.rc.s2 == 2))
	{
		shoot->State = SHOOT_STOP;
	}
	

}


void Shoot_calc(Shoot_t* shoot)
{
	if(shoot->State == SHOOT_RUN)
	{
		if(!IF_KEY_PRESSED_Q)
			RC_Ctl.flag.Q = 0;
		if(IF_KEY_PRESSED_Q && RC_Ctl.flag.Q == 0)
		{
			RC_Ctl.flag.Q = 1;
			shoot->Shoot_target_speed += 50;
		}
		if(!IF_KEY_PRESSED_E)
			RC_Ctl.flag.E = 0;
		if(IF_KEY_PRESSED_E && RC_Ctl.flag.E == 0)
		{
			RC_Ctl.flag.E = 1;
			shoot->Shoot_target_speed -= 50;
		}
		
		Motor_3508[0].target_speed_rpm = shoot->Shoot_target_speed;
		Motor_3508[1].target_speed_rpm = -shoot->Shoot_target_speed;
		PID_calc(&M3508_speedloop[0],Motor_3508[0].speed_rpm,Motor_3508[0].target_speed_rpm);
		PID_calc(&M3508_speedloop[1],Motor_3508[1].speed_rpm,Motor_3508[1].target_speed_rpm);
		Motor_3508[0].target_current = M3508_speedloop[0].out;
		Motor_3508[1].target_current = M3508_speedloop[1].out;		
	}
	else
	{
		PID_clear(&M3508_speedloop[0]);
		PID_clear(&M3508_speedloop[1]);
		Motor_3508[0].target_current = Motor_3508[1].target_current = 0;
	}

}

