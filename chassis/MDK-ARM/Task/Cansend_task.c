#include "Cansend_task.h"
#include "DM4310.h"
#include "DJI3508.h"
#include "cap.h"
#include "can.h"
#include "cmsis_os.h"
#include "Gimbal_task.h"
#include "bsp_delay.h"



void Can_sendFun(void const * argument)
{

  for(;;)
  {
//		Send_cap_msg();
//		delay_us(200);
//		
//		Motor_3508_send(&hcan1,0x200, Motor_3508[0].target_current, Motor_3508[1].target_current,Motor_3508[2].target_current,Motor_3508[3].target_current);	
//  	delay_us(200);

//		MIT_CtrlMotor(&hcan1,RAMMER_ContrlID,  0,  0, 0, 0,Motor_4310_Rammer.target_torque);
//		delay_us(200);

//		if(Gimbal.flag.ControlMode == ENCODER)
//				MIT_CtrlMotor(&hcan2,YAW_ContrlID,0,0,0,0,Motor_4310_EnYaw.target_torque);
//		else if(Gimbal.flag.ControlMode == IMU)
//				MIT_CtrlMotor(&hcan2,YAW_ContrlID,0,0,0,0,Motor_4310_Yaw.target_torque);
//		else
//				MIT_CtrlMotor(&hcan2,YAW_ContrlID,0,0,0,0,0);

		
    osDelay(2);
  }
}

