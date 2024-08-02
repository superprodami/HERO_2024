#include "Board_can.h"
#include "Gimbal_Task.h"
#include "vision.h"
#include "arm_math.h"

chassis_msg_t Chassia_msg;


RC_Ctl_t RC_Ctl;
angle_measure_t Yaw_angle_t;

void dch_receive(RC_Ctl_t* _rc, uint8_t temp[])
{
	_rc->rc.ch0 = (int16_t)(temp[0]<<8 | temp[1]);
	_rc->rc.ch1 = (int16_t)(temp[2]<<8 | temp[3]);
	_rc->rc.ch2 = (int16_t)(temp[4]<<8 | temp[5]);
	_rc->rc.ch3 = (int16_t)(temp[6]<<8 | temp[7]);
	if(RC_Ctl.Rx_add[0]<10000)
	RC_Ctl.Rx_add[0]++;
}



void dmouse_receive(RC_Ctl_t* _rc, uint8_t temp[])
{
	_rc->mouse.x = (int16_t)(temp[0]<<8 | temp[1]);
	_rc->mouse.y = (int16_t)(temp[2]<<8 | temp[3]);

	_rc->mouse.z = (int16_t)(temp[4]<<8 | temp[5]);
	_rc->mouse.press_l = temp[6];
	_rc->mouse.press_r = temp[7];
	if(RC_Ctl.Rx_add[1]<10000)
	RC_Ctl.Rx_add[1]++;
}

uint8_t test_v1,test_v2;
uint16_t test_v3;
void dsw_receive(RC_Ctl_t* _rc, uint8_t temp[])
{
	_rc->rc.s1 = temp[0];
	_rc->rc.s2 = temp[1];
	_rc->key.v = (int16_t)(temp[2]<<8|temp[3]);
		test_v1 = temp[2]<<8;
	test_v2 = temp[3];
	test_v3 = (int16_t)(temp[2]<<8 | temp[3]);
	_rc->rc.wheel = (int16_t)(temp[4]<<8 | temp[5]);
	Gimbal.Robot_color = temp[6];
	Gimbal.flag.remotelose_flag = temp[7];
	if(RC_Ctl.Rx_add[2]<10000)	
	RC_Ctl.Rx_add[2]++;
}

	
//发送陀螺仪，详情见main中断的发送函数
void set_gimbal_angle(CAN_HandleTypeDef* hcan, float angle)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x300;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Yaw_angle_t.Yaw_angle.value = angle;
	Yaw_angle_t.vison_yaw.value = 180.f*VisionValue.yaw_value.value/PI;
	Txtemp[0] = Yaw_angle_t.Yaw_angle.buff[0];
	Txtemp[1] = Yaw_angle_t.Yaw_angle.buff[1];
	Txtemp[2] = Yaw_angle_t.Yaw_angle.buff[2];
	Txtemp[3] = Yaw_angle_t.Yaw_angle.buff[3];

	Txtemp[4] = Yaw_angle_t.vison_yaw.buff[0];  
	Txtemp[5] = Yaw_angle_t.vison_yaw.buff[1];
	Txtemp[6] = Yaw_angle_t.vison_yaw.buff[2];
	Txtemp[7] = Yaw_angle_t.vison_yaw.buff[3];

	
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
	}
}

void set_gimbal_flag(CAN_HandleTypeDef* hcan)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x301;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = VisionValue.find_bool;
	Txtemp[1] = Gimbal.IMUData.pitch_ok;
	Txtemp[2] = VisionValue.distance_t.buff[0];
	Txtemp[3] = VisionValue.distance_t.buff[1];
	Txtemp[4] = VisionValue.distance_t.buff[2]; 
	Txtemp[5] = VisionValue.distance_t.buff[3];
	Txtemp[6] = 0;
	Txtemp[7] = 0;
	
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
	}
}




/*can通信2.0*/
void Chassis_can_recevie(uint8_t temp[] , RC_Ctl_t* _rc ,Gimbal_t* gimbal)
{
	switch(temp[0])
	{
		case 0x01:
		{
			_rc->rc.ch0  = (int16_t)(temp[1]<<8 | temp[2]);
			_rc->rc.ch1  = (int16_t)(temp[3]<<8 | temp[4]);		
			_rc->mouse.x = (int16_t)(temp[5]<<8 | temp[6]);
			_rc->rc.s1   =  temp[7];
			if(_rc->Rx_add[0]<10000)
				_rc->Rx_add[0]++;
		}
			break;
		case 0x02:
		{
			_rc->rc.ch2  = (int16_t)(temp[1]<<8 | temp[2]);
			_rc->rc.ch3  = (int16_t)(temp[3]<<8 | temp[4]);		
		  _rc->mouse.y = (int16_t)(temp[5]<<8 | temp[6]);
			_rc->rc.s2   =  temp[7];
				if(_rc->Rx_add[1]<10000)
				_rc->Rx_add[1]++;
		}
			break;
		case 0x03:
		{
			_rc->mouse.press_l = temp[1];
			_rc->mouse.press_r = temp[2];
			_rc->key.v = (int16_t)(temp[3]<<8 | temp[4]);
			Gimbal.Robot_color = temp[5];
			Gimbal.flag.remotelose_flag = temp[6];
				if(_rc->Rx_add[2]<10000)
				_rc->Rx_add[2]++;			
		}
			break;
		default:
			break;
	}
}

