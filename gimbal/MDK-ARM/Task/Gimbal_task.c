#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "can.h"
#include "GimbalFun.h"
#include "DJI6020.h"
#include "DM4310.h"
#include "INS_task.h"
#include "Filter.h"


Gimbal_t Gimbal;


void Gimble_Fun(void const * argument)
{

		 Gimbal_Init(&Gimbal);

  for(;;)
  {
 
		GimbalSelate(&Gimbal);
		DM_enable(&hcan1,Motor_4310_PITCH,PITCH_ContrlID);
//		Gimbal_Statemachine_2_Stop(&Gimbal);

		

		switch(Gimbal.State)
		{
			case GIMB_Init:
			{
				
				
				Gimbal_Statemachine_2_Stop(&Gimbal);
				break;
			}
			case GIMB_Stop:
			{
			  Clean_Gimbal_CtrlMsg(&Gimbal);
				
				break;
			}
			case GIMB_Remote:
			{
				
				RemoteControlGimbal(&Gimbal);
				Gimbal_clc(&Gimbal);
				break;
			}
			case GIMB_Keyboard:
			{
				Gimbal_Mode_Choose(&Gimbal);
				KeyboardControlGimbal(&Gimbal);
				Gimbal_clc(&Gimbal);
				
				
				break;
			}
		}

//		MIT_CtrlMotor(&hcan1,PITCH_ContrlID,  0,  0, 0, 0,Motor_4310_PITCH.target_torque);	
			Motor_6020_send(&hcan1,0x1FF,Motor_6020.target_current,0,0,0);
  
		osDelay(2);
  }
}
