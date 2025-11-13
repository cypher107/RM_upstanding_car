/**
  ****************************辽宁科技大学COD****************************
  * @file       user_task.c/h-雨落长安
  * @brief      一个普通程序，请大家友善对待，谢谢大家，写的demo，未来得及封装，凑合看吧
  ==============================================================================
  @endverbatim
  ****************************辽宁科技大学COD****************************
  */

#include "main.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_Receive.h"//加载摩擦轮发射闭环真实转速
#include "math.h"
#include "arm_math.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "remote_control.h"
#include <stdlib.h>
#include <stdio.h>

#define UI_KEY  KEY_PRESSED_OFFSET_R
Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11;
String_Data CH_FRIC;
String_Data CH_FLRB;
String_Data CH_WZ;
char fric_arr[5]="fric";
char flrb_arr[4]="FRBL";
char wz_arr[2]="wz";
extern chassis_move_t chassis_move;
uint8_t fric_on_j;
uint8_t wz_on_j;
uint8_t flag;
char v_out_arr[10]={0};
String_Data U_OUT;

void user_task(void *pvParameters)
{
//	chassis_move.chassis_RC = get_remote_control_point();
	memset(&G1,0,sizeof(G1));//中心垂线
	memset(&G2,0,sizeof(G2));//5
	memset(&G3,0,sizeof(G3));//4
	memset(&G4,0,sizeof(G4));//3
	memset(&G5,0,sizeof(G5));//1
	memset(&G6,0,sizeof(G6));//0.5
	memset(&G7,0,sizeof(G7));//2?
	memset(&CH_FRIC,0,sizeof(CH_FRIC));//摩擦轮标识
	memset(&G8,0,sizeof(G8));//前装甲板状态
	memset(&U_OUT,0,sizeof(U_OUT));
	memset(&CH_WZ,0,sizeof(CH_WZ));
//    flag=0;
	while(1)
	{
		Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Green,2,922,330,922,620);
		Line_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Purplish_red,2,914,482,931,482);
		Line_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Purplish_red,2,910,486,937,486);
		Line_Draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Green,2,897,491,952,491);
		Line_Draw(&G5,"095",UI_Graph_ADD,9,UI_Color_Green,2,873,459,967,459);
	    Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Green,2,860,427,984,427);
	    Line_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_Purplish_red,2,907,491,940,491);
		Circle_Draw(&G8,"098",UI_Graph_ADD,9,UI_Color_Yellow,15,230,770,15);
		Char_Draw(&CH_WZ,"077",UI_Graph_ADD,7,UI_Color_Yellow,24,2,4,80,780,&wz_arr[0]);
		Char_ReFresh(CH_WZ);
//	    Char_Draw(&CH_FRIC,"087",UI_Graph_ADD,8 ,UI_Color_Yellow,24,5,4,80,780,&fric_arr[0]);
		HAL_Delay(50);
		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7); 
		HAL_Delay(50);		
		Char_Draw(&U_OUT,"078",UI_Graph_ADD,7,UI_Color_Yellow,50,5,4,960,210,&v_out_arr[0]);
		Char_ReFresh(U_OUT);
		HAL_Delay(50);	
		UI_ReFresh(1,G8);     
		HAL_Delay(50);		//绘制图形	
		Char_Draw(&U_OUT,"078",UI_Graph_Change,7,UI_Color_Yellow,50,5,4,960,210,&v_out_arr[0]);
    	Char_ReFresh(U_OUT);
		HAL_Delay(50);
		fric_on_j=fric_on_judge();
		if(fric_on_j==1){
			Circle_Draw(&G8,"098",UI_Graph_Change,9,UI_Color_Green,15,230,770,15);
			UI_ReFresh(1,G8); 
			HAL_Delay(50);
		}else if(fric_on_j==0){
			Circle_Draw(&G8,"098",UI_Graph_Change,9,UI_Color_Purplish_red,15,230,770,15);
			UI_ReFresh(1,G8);
			HAL_Delay(50);
		}
	}	
}

