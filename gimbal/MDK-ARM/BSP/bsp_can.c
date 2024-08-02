#include "bsp_can.h"
#include "DJI3508.h"
#include "DJI6020.h"
#include "DM4310.h"
#include "Board_can.h"


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


void CAN_Init(void)
{
	my_can_filter_init_recv_all(&hcan1);
	HAL_CAN_Start(&hcan1);
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	my_can_filter_init_recv_all(&hcan2);
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
		case CANID_Shoot_l:
		case CANID_Shoot_r:
		{
			uint8_t i;
			i = _RxHeader.StdId - CANID_Shoot_l;
			Motor_3508_receive(&Motor_3508[i],temp,_RxHeader.StdId);
		}
		break;
		case CANID_PITCH_6020:
			 Motor_6020_receive(&Motor_6020,  temp,_RxHeader.StdId);
		break;
		case CANID_CH:
			 dch_receive(&RC_Ctl,  temp);
		break;
		case CANID_SW:
			 dsw_receive(&RC_Ctl,  temp);
		break;
		case CANID_MOUSE:
			 dmouse_receive(&RC_Ctl,  temp);
		break;			
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void Frequency_calc(void)
{
	RC_Ctl.Rx_Frequency[0] = RC_Ctl.Rx_add[0];
	RC_Ctl.Rx_add[0] = 0;		
	RC_Ctl.Rx_Frequency[1] = RC_Ctl.Rx_add[1];
	RC_Ctl.Rx_add[1] = 0;		
	RC_Ctl.Rx_Frequency[2] = RC_Ctl.Rx_add[2];
	RC_Ctl.Rx_add[2] = 0;	

	
	Motor_6020.RX_Frequancy =  Motor_6020.Rx_add;
	Motor_6020.Rx_add = 0;



	Motor_3508[0].Rx_Frequancy = Motor_3508[0].Rx_add;
	Motor_3508[0].Rx_add = 0;
	Motor_3508[1].Rx_Frequancy = Motor_3508[1].Rx_add;
	Motor_3508[1].Rx_add = 0;	
}


