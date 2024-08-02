#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h" 

extern void CAN_Init(void);
extern void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
extern void my_can2_filter_init_recv_all(CAN_HandleTypeDef* _hcan);

extern void Frequency_calc(void);

#endif 

