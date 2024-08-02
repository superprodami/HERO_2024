//#ifndef __BOARD_UART_H__
//#define __BOARD_UART_H__

//#include "main.h"


//#define CMNT_RX_MAX 45
//#define CMNT_LONG   22

//#include "main.h"
//#include "Gimbal_Task.h"

//#define MISSING_TIMEOUT 500


//typedef struct
//{
//	struct
//	{ 
//		 short ch0;
//		 short ch1;
//		 short ch2;
//		 short ch3;
//		 char s1;
//		 char s2;
//			char s1_last;
//		char s2_last;
//		 short wheel;
//	}rc;
//	
//	struct 
//	{
//		 short x;
//		 short y;
//		 short z;
//		 char press_l;
//		 char press_r;
//	}mouse;
//	
//	struct
//	{
//		unsigned short v;
//		unsigned short v_last;
//	}key;
//	uint16_t misstimeout;
//	
//		struct
//	{
//		bool CTRL;
//		bool F;
//		bool X;
//		bool C;
//		bool Q;
//		bool E;
//	}flag;//标志位，用来切换模式，保证按键瞬间只进入函数一次
//	uint32_t Rx_add;
//	uint16_t Rx_Frequency;
//}RC_Ctl_t;


//typedef struct
//{
//    union
//    {
//        uint8_t buff[4];
//        float value;
//		
//    }Yaw_angle;
//	
//}angle_measure_t;

//extern angle_measure_t Yaw_angle_t;

///* 获取鼠标三轴的移动速度 */
//#define    MOUSE_X_MOVE_SPEED    (RC_Ctl.mouse.x)
//#define    MOUSE_Y_MOVE_SPEED    (RC_Ctl.mouse.y)
//#define    MOUSE_Z_MOVE_SPEED    (RC_Ctl.mouse.z)


///* ----------------------- PC Key Definition-------------------------------- */
//#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
//#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
//#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
//#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
//#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
//#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
//#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
//#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
//#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
//#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
//#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
//#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
//#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
//#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
//#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
//#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)


///* 检测鼠标按键状态 
//   按下为1，没按下为0*/
//#define    IF_MOUSE_PRESSED_LEFT    (RC_Ctl.mouse.press_l == 1)
//#define    IF_MOUSE_PRESSED_RIGH    (RC_Ctl.mouse.press_r == 1)


///* 检测键盘按键状态 
//   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
//#define    IF_KEY_PRESSED         (  RC_Ctl.key.v  )
//#define    IF_KEY_PRESSED_W       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
//#define    IF_KEY_PRESSED_S       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
//#define    IF_KEY_PRESSED_A       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
//#define    IF_KEY_PRESSED_D       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
//#define    IF_KEY_PRESSED_Q       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
//#define    IF_KEY_PRESSED_E       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
//#define    IF_KEY_PRESSED_G       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
//#define    IF_KEY_PRESSED_X       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
//#define    IF_KEY_PRESSED_Z       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
//#define    IF_KEY_PRESSED_C       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
//#define    IF_KEY_PRESSED_B       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
//#define    IF_KEY_PRESSED_V       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
//#define    IF_KEY_PRESSED_F       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
//#define    IF_KEY_PRESSED_R       ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
//#define    IF_KEY_PRESSED_CTRL    ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
//#define    IF_KEY_PRESSED_SHIFT   ( (RC_Ctl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

//extern RC_Ctl_t RC_Ctl;

//extern uint8_t board_rx_buf[2][CMNT_RX_MAX];

//extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

//extern void CMNT_receive_UART(uint8_t *temp);

//#endif


