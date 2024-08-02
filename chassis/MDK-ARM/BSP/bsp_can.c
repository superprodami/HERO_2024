#include "bsp_can.h"
#include "DJI3508.h"
#include "DJI6020.h"
#include "DM4310.h"
#include "Board_can.h"
#include "dbus.h"
#include "Gimbal_task.h"
#include "cap.h"

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterTypeDef		sFilterConfig;


   sFilterConfig.FilterIdHigh         = 0x0000;
   sFilterConfig.FilterIdLow          = 0x0000;
   sFilterConfig.FilterMaskIdHigh     = 0x0000;
   sFilterConfig.FilterMaskIdLow      = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterBank=0;
   sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
	{

	}
}

void my_can2_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterTypeDef		sFilterConfig;


   sFilterConfig.FilterIdHigh         = 0x0000;
   sFilterConfig.FilterIdLow          = 0x0000;
   sFilterConfig.FilterMaskIdHigh     = 0x0000;
   sFilterConfig.FilterMaskIdLow      = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterBank=14;
   sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
	{

	}
}

void CAN_Init(void)
{
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Start(&hcan1);
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	my_can2_filter_init_recv_all(&hcan2);
	HAL_CAN_Start(&hcan2);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{
	CAN_RxHeaderTypeDef _RxHeader;
	uint8_t temp[8];

	HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&_RxHeader,temp);


	//ignore can1 or can2.
	switch(_RxHeader.StdId){
		case CANID_Chassis0:
		case CANID_Chassis1:
		case CANID_Chassis2:
		case CANID_Chassis3:
		{
			uint8_t i;
			i = _RxHeader.StdId - CANID_Chassis0;
			Motor_3508_receive(&Motor_3508[i],temp,_RxHeader.StdId);
		}
			break;
		case CANID_YAW:
			Motor_4310_receive(&Motor_4310_Yaw,    temp,_RxHeader.StdId);
			Motor_4310_angle_receive(&Motor_4310_EnYaw,  temp,_RxHeader.StdId);
			break;
		case CANID_RAMMER:
			Motor_4310_receive(&Motor_4310_Rammer,  temp,_RxHeader.StdId);
			break;
		case CANID_CAP:
			 get_cap_info(temp);
			break;
		case CANID_IMU:
			gimbal_msg_receive(&Yaw_angle_t,temp);
			break;
		case CANID_FLAG:
			gimbal_flag_receive(&Yaw_angle_t,temp);
			break;
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void Frequency_calc(void)
{


	
	Motor_4310_Yaw.RX_Frequancy = Motor_4310_Yaw.Rx_add;
	Motor_4310_Yaw.Rx_add = 0;
	Motor_4310_EnYaw.RX_Frequancy = Motor_4310_EnYaw.Rx_add;
	Motor_4310_EnYaw.Rx_add = 0;
	
	Motor_4310_Rammer.RX_Frequancy = Motor_4310_Rammer.Rx_add;
	Motor_4310_Rammer.Rx_add = 0;

	Gimbal.IMUData.Rx_Frequency = Gimbal.IMUData.Rx_add++;
	Gimbal.IMUData.Rx_add = 0;
	
	Gimbal.IMUData.Rx_Frequency_2 = Gimbal.IMUData.Rx_add_2++;
	Gimbal.IMUData.Rx_add_2 = 0;

	
	Motor_3508[0].RX_Frequancy = Motor_3508[0].Rx_add;
	Motor_3508[0].Rx_add = 0;
	Motor_3508[1].RX_Frequancy = Motor_3508[1].Rx_add;
	Motor_3508[1].Rx_add = 0;	
	Motor_3508[2].RX_Frequancy = Motor_3508[2].Rx_add;
	Motor_3508[2].Rx_add = 0;
	Motor_3508[3].RX_Frequancy = Motor_3508[3].Rx_add;
	Motor_3508[3].Rx_add = 0;
	

}


