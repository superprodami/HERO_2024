#ifndef _bspvision_h
#define _bspvision_h
#include "main.h"
#define VISION_HUART huart8
#define VISION_MAX_LEN 50
#define VISION_BUFLEN 32
#define VISION_SEND 4

#define	GET_LOW_BYTE0(x)	((x >>  0) & 0x00000000000000ff)	
#define	GET_LOW_BYTE1(x)	((x >>  8) & 0x00000000000000ff)	
#define	GET_LOW_BYTE2(x)	((x >> 16) & 0x00000000000000ff)	
#define	GET_LOW_BYTE3(x)	((x >> 24) & 0x00000000000000ff)
#define	GET_LOW_BYTE4(x)	((x >> 32) & 0x00000000000000ff)
#define	GET_LOW_BYTE5(x)	((x >> 40) & 0x00000000000000ff)
#define	GET_LOW_BYTE6(x)	((x >> 48) & 0x00000000000000ff)
#define	GET_LOW_BYTE7(x)	((x >> 56) & 0x00000000000000ff)	



typedef struct{
    union {
    uint8_t buff[4];
        float value;
    }yaw_value;         // ƫ����
    
    union{
    uint8_t buff[4];
        float value;
        float value_offset;
    }pitch_value;       // ������
    union{
			uint8_t buff[4];
			float distance;
		}distance_t;
		
		char find_bool;     // ׷��,�Ƿ�ʶ��Ŀ��
}VisionValueType;

typedef struct{
    uint8_t enemy_team_color;   // �з���ɫ����ɫΪ 1
    uint8_t mode;               // ģʽ 0������ 1����
    uint8_t rune_flag;          // ��ģʽ 0�����ɼ��� 1��С�� 2:���
    
    union{
        uint8_t buff[4];
        float value;
       
    }yaw;                       // ƫ���� ����̨
    
    union{
        uint8_t buff[4];
        float value; 
        
    }pitch;                     // ������
}VisionTransmitType;



extern VisionValueType VisionValue;
extern VisionTransmitType VisionTransmit;
extern uint8_t vision_buf[VISION_BUFLEN];




extern float INS_quat[4];



extern uint8_t Vision_Send_Buff[16];


extern uint8_t vision_transmit_buff[VISION_BUFLEN];

void vision_transmit(void);


void vision_callback_handler(uint8_t *buff);
void vision_uart_init(void);
void vision_buff_package(uint8_t mode,uint8_t level);
#endif


