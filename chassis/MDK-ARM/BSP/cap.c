
/*
 * 超级电容接受报文格式
 * 标识符:0x4FF
 *【增加】  [2]0x03飞坡模式  [3]0xcc软件复位，在操作手按下复位按键时【务必先发送0xcc】，主控再复位
 *
 * [0]0xAE                                    ， 固定连接标志
 * [1]0x1a                                    ， 数据信号1
 * [2]0x01关机 0x02开机 0x03飞坡模式暴力放电  ，操作手发射
 * [3]0xaa活着 0xbb死了 oxcc软件复位          ， 裁判系统血量0或者遥控器关闭发死了，非0发活着，不用管是否电源断电
 * [4]缓冲能量0                               ， 缓冲能量联合体内存[0]   
 * [5]缓冲能量1                               ， 缓冲能量联合体内存[1]  
 * [6]缓冲能量2                               ， 缓冲能量联合体内存[2]  
 * [7]缓冲能量3                               ， 缓冲能量联合体内存[3]  
 
 * [0]0xAE                                    ， 固定连接标志
 * [1]0x2b                                    ， 数据信号2
 * [2]0x01关机 0x02开机 0x03飞坡模式暴力放电  ，操作手发射
 * [3]0xaa活着 0xbb死了 oxcc软件复位          ， 裁判系统血量0或者遥控器关闭发死了，非0发活着，不用管是否电源断电
 * [4]裁判功率0                               ， 裁判功率联合体内存[0]  
 * [5]裁判功率1                               ， 裁判功率联合体内存[1]  
 * [6]裁判功率2                               ， 裁判功率联合体内存[2]  
 * [7]裁判功率3                               ， 裁判功率联合体内存[3] 
 */
 
 
/*
 * 超级电容发射报文格式
 * 标识符:0x210 
 *
 *[1]字节非1时 UI显示 【Waiting】
 *一段时间未收到超级电容信号时自动切换到发送discharge模式（即[2]字节发送0x01），并在UI显示【Error】
 *[2]字节反馈的普通开机模式和飞坡模式务必在【UI上明显区分】
 *
 * [0]0x1a                ， 数据信号1
 * [1]0x01                ,  正常运行 其他数据表示致命故障或正在开机,此时其他反馈数值无效，并需要至于充电模式
 * [2]0x01关机 0x02开机 0x03飞坡模式  ,  实时开关机状态
 * [3]0x00                ,  没想好这位发什么
 * [4]剩余能量0           ， 剩余能量联合体内存[0]
 * [5]剩余能量1           ， 剩余能量联合体内存[1]
 * [6]剩余能量2           ， 剩余能量联合体内存[2]
 * [7]剩余能量3           ， 剩余能量联合体内存[3]
                          
 */
#include "cap.h"
#include "judge.h"
#include "bsp_can.h"
#include "main.h"
#include "dbus.h"
#include "Chassis_Task.h"
#include "Board_can.h"


float cap_percent;
JudgeValueType JudgeValue;
CapInfo cap_info;

uint8_t robot_status = 0xbb; 
uint8_t cap_switch = 0x01;

uint16_t power_limit = 0;  //功率限制

/**
* @brief  电容各种模式启动发送函数
  * @param  void
  * @retval void
  * @attention 电容的保护 模式切换相关
  * C为启动键 G为模式切换键 
  */
void Send_cap_msg(void)
{
	static uint8_t cap_step = 0;
	float joule_residue = 0;  //剩余焦耳缓冲能量
	
	cap_info.cap_step++;
	if(cap_info.cap_step > 1000)
		cap_info.cap_step = 1000;
	
	if(cap_info.cap_step > 800)
	{
		cap_info.cap_status = 0x00;
	}
	
	if(Chassis.State == CHASS_Keyboard)
	{
	    /////////////////C键电容开关-短按切换开关///////////////////
		if(!IF_KEY_PRESSED_C)
		{
			RC_Ctl.flag.C = 0;
		}
		if(IF_KEY_PRESSED_C && RC_Ctl.flag.C == 0)
		{
			RC_Ctl.flag.C = 1;
			
			if(cap_switch == CAP_SWITCH_CLOSE && (cap_info.cap_status == CAP_STATUS_FLAG))
			{
				cap_switch = CAP_SWITCH_OPEN;
				cap_info.cap_mode = CAP_NOMAL;
			}
			else{
				cap_switch = CAP_SWITCH_CLOSE;
			}
		}
		

		///////////当电容能量剩余小于1%时自动关闭--防止低能量下使用电容超功率/////////////
		if(cap_info.cap_joule_residue <= 1) 
			cap_switch = CAP_SWITCH_CLOSE;
			
	}
	
	if(!Judge_If_Death()&&Chassis.State == CHASS_Keyboard)
	{
		robot_status = CAP_STATUS_WORK;
	}
	else
		robot_status = CAP_STATUS_OFFWORK;
	
	if(++cap_step == 7)  //电容数据
	{
		/*获取功率限制*/			
		power_limit = JUDGE_usGetPowerLimit();
		/*剩余焦耳能量*/
		joule_residue = JUDGE_fGetRemainEnergy();
		
		set_cap0(&hcan2, robot_status, cap_switch, joule_residue);
		set_cap1(&hcan2, robot_status, cap_switch, power_limit);
		
		cap_step = 0;
	
	}
}

/*
 * 超级电容接受报文格式
 * 标识符:0x4FF
 *
 * [0]0xAE                ， 固定连接标志
 * [1]0x1a                ， 数据信号1
 * [2]0x01关机 0x02开机   ， 操作手发射
 * [3]0xaa活着 0xbb死了 0xcc重启   ， 裁判系统血量0发死了，非0发活着，不用管是否电源断电
 * [4]缓冲能量0           ， 缓冲能量联合体内存[0]   
 * [5]缓冲能量1           ， 缓冲能量联合体内存[1]  
 * [6]缓冲能量2           ， 缓冲能量联合体内存[2]  
 * [7]缓冲能量3           ， 缓冲能量联合体内存[3]  
 
 * [0]0xAE                ， 固定连接标志
 * [1]0x2b                ， 数据信号2
 * [2]0x01关机 0x02开机   ， 操作手发射
 * [3]0xaa活着 0xbb死了   ， 裁判系统血量0发死了，非0发活着，不用管是否电源断电
 * [4]裁判功率0           ， 裁判功率联合体内存[0]  
 * [5]裁判功率1           ， 裁判功率联合体内存[1]  
 * [6]裁判功率2           ， 裁判功率联合体内存[2]  
 * [7]裁判功率3           ， 裁判功率联合体内存[3]   
 */
void set_cap0(CAN_HandleTypeDef* hcan, int8_t robot_status, int8_t cap_switch, float joule_residue)
{
//	float    Joule_Residue = 0;//剩余焦耳缓冲能量
	JudgeValue.joule_residue.value = joule_residue;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x4FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xAE;
	Txtemp[1] = 0x1a;
	Txtemp[2] =	cap_switch;
	Txtemp[3] =	robot_status;
	Txtemp[4] =	JudgeValue.joule_residue.buff[0];
	Txtemp[5] = JudgeValue.joule_residue.buff[1];
	Txtemp[6] = JudgeValue.joule_residue.buff[2];
	Txtemp[7] = JudgeValue.joule_residue.buff[3];
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}

void set_cap1(CAN_HandleTypeDef* hcan, int8_t robot_status, int8_t cap_switch, float power_limit)
{
	JudgeValue.power_limit.value = power_limit;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x4FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xAE;
	Txtemp[1] = 0x2b;
	Txtemp[2] =	cap_switch;
	Txtemp[3] =	robot_status;
	Txtemp[4] =	JudgeValue.power_limit.buff[0];
	Txtemp[5] = JudgeValue.power_limit.buff[1];
	Txtemp[6] = JudgeValue.power_limit.buff[2];
	Txtemp[7] = JudgeValue.power_limit.buff[3];
//	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}


/*
 * 超级电容发射报文格式
 * 标识符:0x210 
 *
 * [0]0x1a                ， 数据信号1
 * [1]0x01                ,  正常运行 其他数据表示致命故障或正在开机
 * [2]0x01关机 0x02开机 0x03飞坡   ,  实时开关机状态
 * [3]0x00                ,  没想好这位发什么
 * [4]剩余能量0           ， 剩余能量联合体内存[0]
 * [5]剩余能量1           ， 剩余能量联合体内存[1]
 * [6]剩余能量2           ， 剩余能量联合体内存[2]
 * [7]剩余能量3           ， 剩余能量联合体内存[3]
 
 * [0]0x2b                ， 数据信号2
 * [1]0x01                ,  正常运行 其他数据表示致命故障或正在开机
 * [2]0x01关机 0x02开机   ,  实时开关机状态
 * [3]0x00                ,  没想好这位发什么
 * [4]输出功率0           ， 输出功率联合体内存[0]
 * [5]输出功率1           ， 输出功率联合体内存[1]
 * [6]输出功率2           ， 输出功率联合体内存[2]
 * [7]输出功率3           ， 输出功率联合体内存[3] 
 
 * [0]0x3c                ， 数据信号3
 * [1]0x01                ,  正常运行 其他数据表示致命故障或正在开机
 * [2]0x01关机 0x02开机   ,  实时开关机状态
 * [3]0x00                ,  没想好这位发什么
 * [4]输入功率0           ， 输入功率联合体内存[0]
 * [5]输入功率1           ， 输入功率联合体内存[1]
 * [6]输入功率2           ， 输入功率联合体内存[2]
 * [7]输入功率3           ， 输入功率联合体内存[3]
 */
void get_cap_info(uint8_t temp[])
{
	if(temp[0] == 0x1a)
	{
		cap_info.cap_status = temp[1];
		cap_info.switch_status = temp[2];
		if(cap_info.cap_status == CAP_STATUS_FLAG)
		{
			JudgeValue.cap_joule_residue.buff[0] = temp[4];
			JudgeValue.cap_joule_residue.buff[1] = temp[5];
			JudgeValue.cap_joule_residue.buff[2] = temp[6];
			JudgeValue.cap_joule_residue.buff[3] = temp[7];
			cap_info.cap_joule_residue = JudgeValue.cap_joule_residue.value;
		}
		
	}
	cap_info.cap_step = 0;
    cap_percent = cap_info.cap_joule_residue;
}

void cap_init(void)
{
	cap_info.cap_step = 0;
}
