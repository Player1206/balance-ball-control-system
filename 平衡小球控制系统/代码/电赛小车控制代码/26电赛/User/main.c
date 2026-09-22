#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include "sys.h"
#include "usart.h"     // uart5_init(电机) / uart4_init(K230) / JY61P
#include "step.h"      // 张大头电机指令
#include "K230.h"      // 视觉球位置
#include "Balance.h"   // 摆球平衡控制

/*全局变量*/
uint8_t KeyScan; //摁键摁下数据
uint8_t KeyNum;  //记录摁键数值
uint8_t Car_stop = 0;
uint8_t Car_state = 0;
float Target_Location = 150;


uint8_t Buzzer_Flag = 0;

int main(void)
{
	/*模块初始化*/
	SYS_Init();
	uart5_init(115200);   // 张大头步进电机串口(UART5, PC12/PD2)
	uart4_init(115200);   // K230 视觉串口(独立 UART4, 不占 USART1/2/3)
	Balance_Init();       // 摆球平衡子系统初始化(使能电机/清零/设参)
	
	/*OLED显示*/
	//OLED_ShowString(1,1,"HelloWorld",OLED_8X16);
	OLED_Update();
	
	while (1)
	{
		//获取摁键摁下
		KeyScan = Key_GetNum();
		if(KeyScan == 1)
		{
			Car_state = !Car_state;
		}
		if(KeyScan == 2)
		{
		  KeyNum = 2;
		}
		if(KeyScan == 3)
		{
			KeyNum = 3;
		}

		//变量显示区//
		//*********************************************陀螺仪打印区
		//OLED_Printf(0,0,OLED_8X16,"Pitch: %.1f",Pitch);
		//OLED_Printf(0,15,OLED_8X16,"Roll: %.1f",Roll);
		//OLED_Printf(0,30,OLED_8X16,"Yaw: %.1f",Yaw);
		
		//*********************************************编码器打印区
		OLED_Printf(0,30,OLED_8X16,"L:%d",Speed_L);
		OLED_Printf(0,45,OLED_8X16,"R:%d",Speed_R);
		
		//*********************************************循迹数据打印区
		OLED_ShowBinNum(0,15,Track,8,OLED_8X16);
		
		//*********************************************标志位打印区
		OLED_Printf(0,0,OLED_8X16,"State:%d",Car_state);
		if(KeyNum == 2)
		{
			OLED_ShowString(70,0,"loop",OLED_8X16);
		}
		if(KeyNum == 3)
		{
			OLED_ShowString(70,0,"AB",OLED_8X16);
		}
			
			
		//OLED_Printf(15,15,OLED_8X16,"Loops:%d",KeyNum);

		OLED_Update();
		
		//串口发送区//
		//printf("Roll:%.1f Pitch:%.1f Yaw:%.1f\r\n",Roll,Pitch,Yaw);
    JY61P_Output_Accel();   // 输出 JY61P x/y/z 加速度
		
		
		delay_ms(50);
	}
}

//10ms中断一次
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();
		Encoder_Read();
		Read_Track_DATA(&Track);
		Track_Control();
		Balance_Control();   // 摆球平衡: 视觉误差 + 加速度前馈 -> 步进电机(每10ms)
		if(KeyNum == 2)
		{
			if(Track == 0 || Track == 0xC3 || Track == 0x81) Car_state = 0;   // 检测到停车黑线: 触发减速停车(不延时)
			Track_Control_loop();
		}
		if(KeyNum == 3)
		{
			Track_Control_AB(Target_Location);
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//清除标志位
	}
}

