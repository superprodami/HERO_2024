#include "UITask.h"
#include "tuxin.h"
#include "cmsis_os.h"
#include "judge.h"
#include "Chassis_task.h"
#include "Gimbal_task.h"
#include "cap.h"


uint8_t Robot_ID_Current = Robot_ID_Red_Hero;


void UI_Fun(void const * argument)
{
	/* 动态UI控制变量 */
	uint16_t UI_PushUp_Counter = 261;
	
	/* 裁判系统初始化 */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		
		
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
    
		Robot_ID_Current = Game_Robot_State.robot_id;
		
		/* UI更新 */
		UI_PushUp_Counter++;

		
		if(UI_PushUp_Counter % 301 == 0) //添加中央标尺1
		{
			UI_Draw_Line(&UI_Graph5.Graphic[0], "001", UI_Graph_Add,    0, UI_Color_Orange, 2,  951 ,412-3 ,951 ,412+3 ); //10m竖线
			UI_Draw_Line(&UI_Graph5.Graphic[1], "002", UI_Graph_Add,    0, UI_Color_Pink, 2,  951-15 ,412 ,951+15 ,412 ); //10m横线
			UI_Draw_Line(&UI_Graph5.Graphic[2], "003", UI_Graph_Add,    0, UI_Color_Pink, 2,  948 ,470-5 ,948 ,470+5 ); //7m竖线
			UI_Draw_Line(&UI_Graph5.Graphic[3], "004", UI_Graph_Add,    0, UI_Color_Pink, 2,  948-15 ,470 ,948+15 ,470); //7m横线			
			UI_Draw_Line(&UI_Graph5.Graphic[4], "005", UI_Graph_Add,    0, UI_Color_Pink, 2,  930 ,	500-5 ,930 ,500+5 ); //7m竖线2.0
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 301 == 2)  //自瞄框
		{
			UI_Draw_Rectangle(&UI_Graph1.Graphic[0] ,"006",UI_Graph_Add,  1, UI_Color_White, 4, 650,300,1250,800);
			UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);
		}
		//自瞄框
		if(UI_PushUp_Counter % 40 == 0 && UI_PushUp_Counter > 310)
		{
			if(Gimbal.IMUData.find_bool == '1')
			{
				UI_Draw_Rectangle(&UI_Graph1.Graphic[0] ,"006",UI_Graph_Change,  1, UI_Color_Pink, 4, 650,300,1250,800);
				UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);			
			}
			else
			{
				UI_Draw_Rectangle(&UI_Graph1.Graphic[0] ,"006",UI_Graph_Change,  1, UI_Color_White, 4, 650,300,1250,800);
				UI_PushUp_Graphs(1, &UI_Graph1, Robot_ID_Current);				
			}
		}

		

		//是否跟随
		if(UI_PushUp_Counter % 21 == 0 && Chassis.Action == CHASS_FOLLOW) 
		{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Add,    2, UI_Color_Pink,  44, 7, 4,  100, 600, "FOLLOW "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
		}
		if(UI_PushUp_Counter % 21 == 2) 
		{
			if(Chassis.Action == CHASS_FOLLOW)
			{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Change,    2, UI_Color_Pink,  44, 7, 4,  100, 600, "FOLLOW "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			else
			{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Delete,    2, UI_Color_Pink,  44, 7, 4,  100, 600, "FOLLOW "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
		}	
			//是否超电
			if(UI_PushUp_Counter % 31 == 0 && cap_info.switch_status == 0x02)
			{
				UI_Draw_String(&UI_String.String, "500", UI_Graph_Add,    3, UI_Color_Pink,  44, 4, 4,  200, 800, "CAP "); 
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			if(UI_PushUp_Counter % 31 == 2)
			{
				if(cap_info.switch_status == 0x02)
				{
					UI_Draw_String(&UI_String.String, "500", UI_Graph_Change,    3, UI_Color_Pink,  44, 4, 8,  200, 800, "CAP "); 
					UI_PushUp_String(&UI_String, Robot_ID_Current);
				}
				else	
				{ 
					UI_Draw_String(&UI_String.String, "500", UI_Graph_Delete,    3, UI_Color_Pink,  44, 4, 8,  200, 800, "CAP"); 
					UI_PushUp_String(&UI_String, Robot_ID_Current);			
				}	
			}			
		
			if(UI_PushUp_Counter % 41 == 0)
			{
//				UI_Draw_Float(&UI_String.String, "400", UI_Graph_Add,    2, UI_Color_Pink,  30, 3,4, 200,  800,cap_info.cap_joule_residue); 
//				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}	


	}

}






