//#include "board_uart.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "usart.h"
//#include "cmsis_os.h"
//#include "Gimbal_Task.h"
//#include "INS_task.h"
//#include "vision.h"


//RC_Ctl_t RC_Ctl;
//angle_measure_t Yaw_angle_t;

//void CMNT_receive_UART(uint8_t *temp)
//{
//	if(temp[0] == 0xAE && temp[CMNT_LONG-1] == 0xBE)
//	{

//			RC_Ctl.rc.ch0  = (int16_t)(temp[1]<<8 | temp[2]);
//			RC_Ctl.rc.ch1  = (int16_t)(temp[3]<<8 | temp[4]);
//			RC_Ctl.rc.ch2  = (int16_t)(temp[5]<<8 | temp[6]);
//			RC_Ctl.rc.ch3  = (int16_t)(temp[7]<<8 | temp[8]);			
//			RC_Ctl.mouse.x = (int16_t)(temp[9]<<8 | temp[10]);
//		  RC_Ctl.mouse.y = (int16_t)(temp[11]<<8| temp[12]);
//			RC_Ctl.rc.s1   =  temp[13];
//			RC_Ctl.rc.s2   =  temp[14];
//			RC_Ctl.mouse.press_l = temp[15];
//			RC_Ctl.mouse.press_r = temp[16];
//			RC_Ctl.key.v = (int16_t)(temp[17]<<8 | temp[18]);
//			Gimbal.Robot_color = temp[19];
//			Gimbal.flag.remotelose_flag = temp[20];

//			if(RC_Ctl.Rx_add<10000)
//				RC_Ctl.Rx_add++;	

//	}
//}






//void Board_uart_Fun(void const * argument)
//{
//	
//	portTickType currentTime;

//	for(;;)
//	{
//		currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
//		
//		uint8_t Txtemp[12];
//		
//		Yaw_angle_t.Yaw_angle.value = IMU_angle[0];

//		Txtemp[0]  = 0xAF;
//		Txtemp[1] = Yaw_angle_t.Yaw_angle.buff[0];
//		Txtemp[2] = Yaw_angle_t.Yaw_angle.buff[1];
//		Txtemp[3] = Yaw_angle_t.Yaw_angle.buff[2];
//		Txtemp[4] = Yaw_angle_t.Yaw_angle.buff[3];
//		Txtemp[5] = VisionValue.yaw_value.buff[0];  
//		Txtemp[6] = VisionValue.yaw_value.buff[1];
//		Txtemp[7] = VisionValue.yaw_value.buff[2];
//		Txtemp[8] = VisionValue.yaw_value.buff[3];
//		Txtemp[9] = VisionValue.find_bool;													
//		Txtemp[10] = Gimbal.IMUData.pitch_ok;
//		Txtemp[11] =	0xBF;
//	

//		if(HAL_UART_Transmit_DMA(&huart1, Txtemp, 12) == HAL_OK)
//		{
//		}
//	
//		vTaskDelayUntil(&currentTime, 2); //������ʱ
//	}
//}



//extern DMA_HandleTypeDef hdma_usart1_rx;
// 
////����ԭʼ���ݣ�Ϊ8���ֽڣ�����16���ֽڳ��ȣ���ֹDMA����Խ��
//uint8_t board_rx_buf[2][CMNT_RX_MAX];
// 
//void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//    //ʹ��DMA���ڽ���
//    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
//    //ʹ�ܴ��ڿ����ж�
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//    //ʧЧDMA
//    __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    }
//    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
//    //�ڴ滺����1
//    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
//    //�ڴ滺����2
//    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
//    //���ݳ���
//    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
//    //ʹ��˫������
//    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
//    //ʹ��DMA
//    __HAL_DMA_ENABLE(&hdma_usart1_rx);	
//}

