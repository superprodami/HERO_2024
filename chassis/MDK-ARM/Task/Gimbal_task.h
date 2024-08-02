#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"

/*��̨״̬��*/
typedef enum
{
	GIMB_Init     = 0x00,
	GIMB_Stop     = 0x01,
	GIMB_Remote   = 0x02,
	GIMB_Keyboard = 0x03
}GIMBALState;


//���������ݽṹ��
typedef struct IMUType
{
	float    IMU_Yaw_angle;
	float    yaw_target;
	float    Vision_yaw_target; //����Ŀ��ֵ
	char    find_bool; //�����Ƿ��ҵ�Ŀ��
	union{
		uint8_t buff[4];
		float  distance;
	}distance_t;//װ�װ���룻
	uint8_t	   pitch_ok;//���鵽λ
	uint8_t		 yaw_ok;
	uint32_t Rx_add;
	uint32_t Rx_add_2;	
	uint16_t Rx_Frequency;
	uint16_t Rx_Frequency_2;	
}IMU_t;




/*����ģʽ*/
typedef enum  
{
   vertical = 0x00,  //��ֱ
	 level    = 0x01,  //ˮƽ
}Hanging_Mode_t;


/*����ģʽ*/
typedef enum  
{
   GIMBAL_NORMAL 	= 0x00,  //����ģʽ
	 GIMBAL_AUTO    = 0x01,  //����
}Gimbal_action_t;


#define ENCODER 0
#define IMU     1

typedef struct Gimbal_flag_t
{
  bool     Move_flag;   		//�Ƿ��ƶ�������������Ʈ�ж�
	bool     Fist_encoder; 		//��һ�ν��������ģʽ
	bool     ControlMode;      //�ж��Ǳ�����ģʽor������ģʽ 0
	uint32_t boardlose_time;//���ͨ��ʧ���ж�
}Gimbal_flag_t;


//��̨��Ϣ
typedef struct GimbalDataType
{
	GIMBALState       State;
	Gimbal_flag_t	    flag;
	IMU_t							IMUData;
	Hanging_Mode_t		Hanging_Mode;
	Gimbal_action_t   Action;
}Gimbal_t;

extern Gimbal_t Gimbal;


///***********************YAW����̨���������ض�ֵ******************/
//#define GIMBAL_YAW_ENCODER_MIDDLE1   2.445     	//���̺���̨������ͬ1��ָ��y
//#define GIMBAL_YAW_ENCODER_MIDDLE2  -0.697   		//���̺���̨������ͬ2��ָ��-y
//#define GIMBAL_YAW_ENCODER_NINETY1   0.874      //130			//���̺���̨����90�㣬ָ��+90��
//#define GIMBAL_YAW_ENCODER_NINETY2  -2.267  	  //4205			//���̺���̨����90�㣬ָ��-90��




#endif

