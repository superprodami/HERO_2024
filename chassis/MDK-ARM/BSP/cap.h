#ifndef __CAP_H
#define __CAP_H

#include "main.h"


#define CAP_SWITCH_CLOSE     0x01
#define CAP_SWITCH_OPEN      0x02
#define CAP_SWITCH_SLOPE     0x03

#define CAP_STATUS_WORK      0xaa
#define CAP_STATUS_OFFWORK   0xbb
#define CAP_STATUS_UNUSUAL   0xcc

#define CAP_STATUS_FLAG      0x01  //表示超电正常运行

#define CAP_NOMAL  0x01
#define CAP_SLOPE   0x02

#define CANID_CAP 0x210
extern uint8_t robot_status; 
extern uint8_t cap_switch;

typedef struct
{
    union
    {
        uint8_t buff[4];
        float value;
    } joule_residue;
    union
    {
        uint8_t buff[4];
        float value;
    } power_limit;
	
	union
    {
        uint8_t buff[4];
        float value;
    } cap_joule_residue;
	
	union
    {
        uint8_t buff[4];
        float value;
    } speed_42mm;
		
	uint8_t color_judge;

} JudgeValueType;  //用于解包/压缩发送裁判系统数据

typedef struct{
	uint8_t cap_status;
	uint8_t switch_status;
	float cap_joule_residue;
	uint8_t cap_mode;
	uint32_t cap_step;
}CapInfo;

extern JudgeValueType JudgeValue;
extern CapInfo cap_info;
extern float cap_percent;

extern void Send_cap_msg(void);
extern void Cap_callback_handler(uint8_t *buff);

extern void set_cap0(CAN_HandleTypeDef* hcan, int8_t robot_status, int8_t cap_switch, float joule_residue);
extern void set_cap1(CAN_HandleTypeDef* hcan, int8_t robot_status, int8_t cap_switch, float power_limit);
extern void cap_init(void);

extern void get_cap_info(uint8_t temp[]);

#endif

