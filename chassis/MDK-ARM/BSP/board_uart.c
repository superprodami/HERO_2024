//#include "board_uart.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "usart.h"
//#include "cmsis_os.h"
//#include "dbus.h"
//#include "Chassis_task.h"
//#include "judge.h"
//#include "Gimbal_task.h"
//#include "arm_math.h"


//angle_measure_t Yaw_angle_t;

//uint16_t IMU_test;
//double test_imu[20];

//void CMNT_receive_UART(uint8_t *temp)
//{
//	if(temp[0] == 0xAF && temp[CMNT_LONG-1] == 0xBF)
//	{
//		Yaw_angle_t.Yaw_angle.buff[0]=temp[1];
//		Yaw_angle_t.Yaw_angle.buff[1]=temp[2];
//		Yaw_angle_t.Yaw_angle.buff[2]=temp[3];
//		Yaw_angle_t.Yaw_angle.buff[3]=temp[4];
//		Yaw_angle_t.yaw_value.buff[0]=temp[5];
//		Yaw_angle_t.yaw_value.buff[1]=temp[6];
//		Yaw_angle_t.yaw_value.buff[2]=temp[7];
//		Yaw_angle_t.yaw_value.buff[3]=temp[8];
//		Gimbal.IMUData.find_bool     =temp[9];
//		Gimbal.IMUData.pitch_ok			 =temp[10];
//		/*copy value*/
//		Gimbal.IMUData.IMU_Yaw_angle = Yaw_angle_t.Yaw_angle.value;
//		Gimbal.IMUData.Vision_yaw_target = 180.f*Yaw_angle_t.yaw_value.value/PI;


//		if(Gimbal.IMUData.IMU_Yaw_angle<-200||Gimbal.IMUData.IMU_Yaw_angle>200)
//		{
//			if(IMU_test<18)
//				test_imu[IMU_test] = Yaw_angle_t.Yaw_angle.value;
//				IMU_test++;
//		}
//		
//		if(Gimbal.IMUData.Rx_add<10000)
//			Gimbal.IMUData.Rx_add++;
//		Gimbal.flag.boardlose_time = 0;

//		
//	}
//}





//void Board_uart_Fun(void const * argument)
//{
//	
//	portTickType currentTime;

//	for(;;)
//	{
//		currentTime = xTaskGetTickCount(); //当前系统时间
//		
//		uint8_t Txtemp[22];
//    bool color;
//		color = is_red_or_blue();
//		
//		Txtemp[0]  = 0xAE;
//		Txtemp[1] = (RC_Ctl.rc.ch0>>8);
//		Txtemp[2] = RC_Ctl.rc.ch0;
//		Txtemp[3] = RC_Ctl.rc.ch1>>8;
//		Txtemp[4] = RC_Ctl.rc.ch1;
//		Txtemp[5] = RC_Ctl.rc.ch2>>8;
//		Txtemp[6] = RC_Ctl.rc.ch2;
//		Txtemp[7] = RC_Ctl.rc.ch3>>8;
//		Txtemp[8] = RC_Ctl.rc.ch3;
//		Txtemp[9] = RC_Ctl.mouse.x >>8;
//		Txtemp[10] =  RC_Ctl.mouse.x;
//		Txtemp[11] =	RC_Ctl.mouse.y>>8;
//		Txtemp[12] =  RC_Ctl.mouse.y;
//		Txtemp[13] =	RC_Ctl.rc.s1;		
//		Txtemp[14] =	RC_Ctl.rc.s2;
//		Txtemp[15] =	RC_Ctl.mouse.press_l;
//		Txtemp[16] =  RC_Ctl.mouse.press_r;
//		Txtemp[17] =  RC_Ctl.key.v>>8;
//		Txtemp[18] =  RC_Ctl.key.v;
//		Txtemp[19] =  color;
//		Txtemp[20] =  Chassis.flag.remotelose_flag;	
//		Txtemp[21] =  0xBE;	

//		if(HAL_UART_Transmit_DMA(&huart1, Txtemp, 22) == HAL_OK)
//		{
//		}

//		
//		vTaskDelayUntil(&currentTime, 2); //绝对延时
//	}
//}



//extern DMA_HandleTypeDef hdma_usart1_rx;
// 
////接收原始数据，
//uint8_t board_rx_buf[2][CMNT_RX_MAX];
// 
//void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//    //使能DMA串口接收
//    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
//    //使能串口空闲中断
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    }
//    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
//    //内存缓冲区1
//    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
//    //内存缓冲区2
//    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
//    //数据长度
//    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
//    //使能双缓冲区
//    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
//    //使能DMA
//    __HAL_DMA_ENABLE(&hdma_usart1_rx);	
//}


