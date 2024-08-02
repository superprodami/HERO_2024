#ifndef __UITASK_H__
#define __UITASK_H__

#include "main.h"
#include "judge.h"
#include "fifo.h"

void Info_Transmit_task(void const * argument);

/* ����ϵͳ����˫������ */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* ����ϵͳ�������ݶ��� */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
extern unpack_data_t Referee_Unpack_OBJ;
extern void MY_UI_task(void const * argument);


#endif

