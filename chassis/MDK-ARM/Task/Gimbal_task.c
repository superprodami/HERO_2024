#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "can.h"
#include "GimbalFun.h"
#include "DJI6020.h"
#include "DM4310.h"

Gimbal_t Gimbal;



void Gimble_Fun(void const * argument)
{
   portTickType currentTime;
		 Gimbal_Init(&Gimbal);

  for(;;)
  {
		currentTime = xTaskGetTickCount(); // 当前系统时间
		GimbalSelate(&Gimbal);
		DM_enable(&hcan2,Motor_4310_Yaw,YAW_ContrlID);		
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

		if(Gimbal.flag.ControlMode == ENCODER)
				MIT_CtrlMotor(&hcan2,YAW_ContrlID,0,0,0,0,Motor_4310_EnYaw.target_torque);
		else if(Gimbal.flag.ControlMode == IMU)
				MIT_CtrlMotor(&hcan2,YAW_ContrlID,0,0,0,0,Motor_4310_Yaw.target_torque);
		else
				MIT_CtrlMotor(&hcan2,YAW_ContrlID,0,0,0,0,0);

	vTaskDelayUntil(&currentTime, 2); // 绝对延时
//    osDelay(2);
  }
}
