#ifndef __DBUS_H__
#define __DBUS_H__

#include "main.h"

#define MISSING_TIMEOUT 100

#define KEY_V	16384
#define KEY_Q	64
#define KEY_W	1
#define KEY_A 4
#define KEY_S 2
#define KEY_D	8

#define KEY_E	128
#define KEY_R	256
#define KEY_B	32768
#define KEY_Z	2048
#define KEY_X	4096
#define KEY_SHIFT 16

/* ----------------------- RC Channel Definition---------------------------- */

#define    RC_CH_VALUE_MIN       ((int16_t)-660 )
#define    RC_CH_VALUE_OFFSET    ((int16_t)0)
#define    RC_CH_VALUE_MAX       ((int16_t)660)


/* ----------------------- RC Switch Definition----------------------------- */

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)


typedef struct
{
	struct
	{ 
		 short ch0;
		 short ch1;
		 short ch2;
		 short ch3;
		 char s1;
		 char s2;
		 short wheel;
	}rc;
	
	struct 
	{
		 short x;
		 short y;
		 short z;
		 char press_l;
		 char press_r;
	}mouse;
	
	struct
	{
		uint16_t v;
	}key;
	uint32_t misstimeout;
		
	struct
	{
		bool CTRL;
		bool F;
		bool X;
		bool C;
		bool Q;
		bool E;
	}flag;//��־λ�������л�ģʽ����֤����˲��ֻ���뺯��һ��
	
}RC_Ctl_t;

/* ��ȡң����ҡ��ƫ����
   ����ң�����ĵ���

��ҡ�ˣ�    ��ҡ�ˣ�
����Ϊch2   ����Ϊch0
����Ϊch3   ����Ϊch1

                        ��   660
��    ��       ��
-660   0      660       ��   0

                        ��  -660  */


/* ��ȡң����ҡ��ƫ��ֵ
   RLR����ҡ�������ƶ�  LUD����ҡ�������ƶ�	*/
#define    RC_CH0_RLR_OFFSET    (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET)
#define    RC_CH1_RUD_OFFSET  	(RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET)
#define    RC_CH2_LLR_OFFSET  	(RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET)
#define    RC_CH3_LUD_OFFSET  	(RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET)


/* ���ң��������״̬ */
#define    IF_RC_SW1_UP      (RC_Ctl.rc.sw1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (RC_Ctl.rc.sw1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (RC_Ctl.rc.sw1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (RC_Ctl.rc.sw2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (RC_Ctl.rc.sw2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (RC_Ctl.rc.sw2 == RC_SW_DOWN)


/* ��ȡ���������ƶ��ٶ� */
#define    MOUSE_X_MOVE_SPEED    (RC_Ctl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (RC_Ctl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (RC_Ctl.mouse.z)


/* �����갴��״̬
   ����Ϊ1��û����Ϊ0*/
#define    IF_MOUSE_PRESSED_LEFT    (RC_Ctl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (RC_Ctl.mouse.press_r == 1)


/* �����̰���״̬
   ����Ӧ���������£����߼����ʽ��ֵΪ1������Ϊ0 */
#define    IF_KEY_PRESSED         (  RC_Ctl.key.v  )
#define    IF_KEY_PRESSED_W       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#define    IF_KEY_PRESSED_LAST         (  RC_Ctl.key.v_last  )
#define    IF_KEY_PRESSED_W_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R_LAST       ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL_LAST    ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT_LAST   ( (RC_Ctl.key.v_last & KEY_PRESSED_OFFSET_SHIFT) != 0 )

extern RC_Ctl_t RC_Ctl;
extern uint8_t dbus_connect;

void dbus_read(void);
void DBUS_Init(void);
void dbus_uart_init(void);
void uart_receive_handler(UART_HandleTypeDef *huart);
uint8_t REMOTE_IfDataError( void );

#endif

