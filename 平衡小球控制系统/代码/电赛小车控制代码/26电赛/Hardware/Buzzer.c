#include "stm32f10x.h"                  // Device header
#include "sys.h"

void Buzzer_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);		//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_ResetBits(GPIOE, GPIO_Pin_2);				//设置PE2引脚为低电平
											//PE2高电平使三极管导通 蜂鸣器才响
}


void Buzzer_ON(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_2);		//设置PE2引脚为低电平 打开蜂鸣器
}


void Buzzer_OFF(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_2);		//设置PE2引脚为高电平 关闭蜂鸣器
}