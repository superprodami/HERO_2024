#ifndef __BOARD_H__
#define __BOARD_H__

#include "main.h"


#define CANID_IMU 	0x300
#define CANID_FLAG  0x301



typedef struct
{
    union
    {
        int8_t buff[4];
        float value;
		
    }Yaw_angle;
	
	    union
    {
        int8_t buff[4];
        float value;
    }vision_value;
		
}angle_measure_t;





extern angle_measure_t Yaw_angle_t;

extern void set_rc_ch(CAN_HandleTypeDef* hcan, int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4);
extern void set_rc_sw(CAN_HandleTypeDef* hcan, int8_t sw1, int8_t sw2, int16_t v, int16_t wheel);
extern void set_rc_mouse(CAN_HandleTypeDef* hcan, int16_t x, int16_t y, int16_t z, int8_t l, int8_t r);
extern void gimbal_msg_receive(angle_measure_t* gim_msg,uint8_t temp[]);
extern void gimbal_flag_receive(angle_measure_t* gim_msg,uint8_t temp[]);

extern void set_chassis_msg(void);

extern void Chassis_can_send(void);

#endif


