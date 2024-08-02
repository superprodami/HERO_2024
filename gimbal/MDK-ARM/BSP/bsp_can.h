#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h" 



void CAN_Init(void);
void my_can_filter0_init_recv_all(CAN_HandleTypeDef* _hcan);
void my_can_filter1_init_recv_all(CAN_HandleTypeDef* _hcan);
extern void Frequency_calc(void);


#endif 

