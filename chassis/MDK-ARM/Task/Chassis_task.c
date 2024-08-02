#include "Chassis_task.h"
#include "cmsis_os.h"
#include "ChassisFun.h"
#include "can.h"
#include "cap.h"

chass_t Chassis;



void Chassis_Fun(void const * argument)
{
	portTickType currentTime;
	Chassis_Init(&Chassis);

	
  for(;;)
  {
		//copy imu motor msg...
		//  cope()
		currentTime = xTaskGetTickCount(); // 当前系统时间
		
		// 状态机控制器
		ChassisSelate(&Chassis);

		
		switch(Chassis.State)
		{
			case CHASS_Init:
			{
				
				
				Chassis_Statemachine_2_Stop(&Chassis);
				break;
			}
			case CHASS_Stop:
			{
				
			Clean_Chassis_CtrlMsg(&Chassis);
				break;
			}
			case CHASS_Remote:
			{
			 Chassis_Choose_remote(&Chassis);
			 RemoteControlChassis(&Chassis);
			 CHASSIS_clc(&Chassis);	
				break;
			}
			case CHASS_Keyboard:
			{
				Chassis_Choose_keyboard(&Chassis);
				KeyboardControlChassis(&Chassis);
				CHASSIS_clc(&Chassis);
				break;
			}
		}
		Chassis_Power_Limit();
 		Motor_3508_send(&hcan1,0x200, Motor_3508[0].target_current, Motor_3508[1].target_current,Motor_3508[2].target_current,Motor_3508[3].target_current);	

//		osDelay(2);
		vTaskDelayUntil(&currentTime, 2); // 绝对延时
  }
}

