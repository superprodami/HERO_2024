#include "Board_can.h"
#include "can.h"
#include "judge.h"
#include "Chassis_Task.h"
#include "Gimbal_task.h"
#include "dbus.h"
#include "bsp_delay.h"

angle_measure_t Yaw_angle_t;

void set_rc_ch(CAN_HandleTypeDef* hcan, int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x320;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (ch1 >> 8);
	Txtemp[1] = ch1;
	Txtemp[2] = (ch2 >> 8);
	Txtemp[3] = ch2;
	Txtemp[4] = ch3 >> 8;
	Txtemp[5] = ch3;
	Txtemp[6] = ch4 >> 8;
	Txtemp[7] = ch4;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}

}
uint8_t test_v1,test_v2;
uint16_t test_v;
void set_rc_sw(CAN_HandleTypeDef* hcan, int8_t sw1, int8_t sw2, int16_t v, int16_t wheel)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	int8_t color;
	_TxHeader.StdId = 0x321;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	color = is_red_or_blue(); //判断颜色

	Txtemp[0] = sw1;
	Txtemp[1] = sw2;
	Txtemp[2] = (RC_Ctl.key.v >> 8);
	Txtemp[3] =RC_Ctl.key.v ;
	Txtemp[4] = wheel >> 8;
	Txtemp[5] = wheel;
	Txtemp[6] = color;
	Txtemp[7] = Chassis.flag.remotelose_flag;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}

}


void set_rc_mouse(CAN_HandleTypeDef* hcan, int16_t x, int16_t y, int16_t z, int8_t l, int8_t r)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x322;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (x >> 8);
	Txtemp[1] = x;
	Txtemp[2] = (y >> 8);
	Txtemp[3] = y;
	Txtemp[4] = z >> 8;
	Txtemp[5] = z;
	Txtemp[6] = l;
	Txtemp[7] = r;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}

}

void gimbal_msg_receive(angle_measure_t* gim_msg,uint8_t temp[])
{
		gim_msg->Yaw_angle.buff[0]=temp[0];
		gim_msg->Yaw_angle.buff[1]=temp[1];
		gim_msg->Yaw_angle.buff[2]=temp[2];
		gim_msg->Yaw_angle.buff[3]=temp[3];
		gim_msg->vision_value.buff[0]=temp[4];
		gim_msg->vision_value.buff[1]=temp[5];
		gim_msg->vision_value.buff[2]=temp[6];
		gim_msg->vision_value.buff[3]=temp[7];	
		
		Gimbal.IMUData.IMU_Yaw_angle = gim_msg->Yaw_angle.value;
		Gimbal.IMUData.Vision_yaw_target = gim_msg->vision_value.value;

		if(Gimbal.IMUData.Rx_add<10000)
			Gimbal.IMUData.Rx_add++;
		Gimbal.flag.boardlose_time = 0;
}	

void gimbal_flag_receive(angle_measure_t* gim_msg,uint8_t temp[])
{

		Gimbal.IMUData.find_bool = temp[0];
		Gimbal.IMUData.pitch_ok = temp[1];
		Gimbal.IMUData.distance_t.buff[0] = temp[2];
		Gimbal.IMUData.distance_t.buff[1] = temp[3];
		Gimbal.IMUData.distance_t.buff[2] = temp[4];
		Gimbal.IMUData.distance_t.buff[3] = temp[5];
	
		if(Gimbal.IMUData.Rx_add_2<10000)
			Gimbal.IMUData.Rx_add_2++;
}	


//发送信息到云台，详情见main中断的发送函数
void set_chassis_msg(void)
{
		set_rc_ch(&hcan2, RC_Ctl.rc.ch0, RC_Ctl.rc.ch1, RC_Ctl.rc.ch2, RC_Ctl.rc.ch3);
		set_rc_sw(&hcan2, RC_Ctl.rc.s1, RC_Ctl.rc.s2, RC_Ctl.key.v, RC_Ctl.rc.wheel);	
		set_rc_mouse(&hcan2, RC_Ctl.mouse.x, RC_Ctl.mouse.y, RC_Ctl.mouse.x, RC_Ctl.mouse.press_l, RC_Ctl.mouse.press_r);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*can通信2.0*/
void Chassis_can_send(void)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x001;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0x01;
	Txtemp[1] = (RC_Ctl.rc.ch0>>8);
	Txtemp[2] = RC_Ctl.rc.ch0;
	Txtemp[3] = RC_Ctl.rc.ch1>>8;
	Txtemp[4] = RC_Ctl.rc.ch1;
	Txtemp[5] = RC_Ctl.mouse.x >>8;
	Txtemp[6] = RC_Ctl.mouse.x;
	Txtemp[7] =	RC_Ctl.rc.s1;
//	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 );
	if(HAL_CAN_AddTxMessage(&hcan2,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){}
	

	Txtemp[0] = 0x02;
	Txtemp[1] = RC_Ctl.rc.ch2>>8;
	Txtemp[2] = RC_Ctl.rc.ch2;
	Txtemp[3] = RC_Ctl.rc.ch3>>8;
	Txtemp[4] = RC_Ctl.rc.ch3;
	Txtemp[5] =	RC_Ctl.mouse.y>>8;
	Txtemp[6] = RC_Ctl.mouse.y;
	Txtemp[7] =	RC_Ctl.rc.s2;
//	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 );
	if(HAL_CAN_AddTxMessage(&hcan2,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){}
		
	uint8_t color;
	color = is_red_or_blue(); //判断颜色	
		
	Txtemp[0] = 0x03;
	Txtemp[1] =	RC_Ctl.mouse.press_l;
	Txtemp[2] = RC_Ctl.mouse.press_r;
	Txtemp[3] = RC_Ctl.key.v>>8;
	Txtemp[4] = RC_Ctl.key.v;
	Txtemp[5] = color;
	Txtemp[6] = Chassis.flag.remotelose_flag;


//	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 );
	if(HAL_CAN_AddTxMessage(&hcan2,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){}
		


}




