#include "stm32f10x.h"                  // Device header
#include "Track.h"
#include "sys.h"

int lostcount = 0; //未检测到黑线次数记录
int unlostcount = 0; // 检测到黑线次数记录
	
void unlost_lost_count(void)
{
	
	if(gray_status[1] == 13)
	{
		lostcount ++; //灰度全为0时为丢线（无线） 丢线次数加1
	}
	else
	{
		unlostcount ++; //灰度不全为0时为未丢线（有线） 未丢线次数加1
	}
		
}


void judge_unlost(void)
{
	//第一题
	if(unlostcount == 2 && lostcount ==2)
	{
		Target_Speed = 0;
		MOTOR1 = 0;
		MOTOR2 = 0;
		GPIO_SetBits(GPIOC,GPIO_Pin_13);		
	}
	
}







