#include "cmsis_os.h"
#include "ShootTask.h"
#include "ShootFun.h"
#include "DM4310.h"
#include "can.h"
#include "cap.h"
Shoot_t Shoot;

 void Shoot_Fun(void const * argument)
{

    Shoot_Init(&Shoot);

  for(;;)
  {
		ShootSelate(&Shoot);
		DM_enable(&hcan1,Motor_4310_Rammer,RAMMER_ContrlID);		
		
		switch(Shoot.State)
		{
			case SHOOT_Init:
			{
			
				Shoot_Statemachine_2_Stop(&Shoot);

				break;			
			}
			case SHOOT_Run:
			{
			
				Shoot_Mode_Choose(&Shoot);
				Shoot_clc();
				break;
			}
		  case SHOOT_STOP:
			{
			
				Clean_Shoot_CtrlMsg(&Shoot);
				break;
			}
			
		}
		//can sand	
			MIT_CtrlMotor(&hcan1,RAMMER_ContrlID,  0,  0, 0, 0,Motor_4310_Rammer.target_torque);
			Send_cap_msg();
		osDelay(2);
  }
}


