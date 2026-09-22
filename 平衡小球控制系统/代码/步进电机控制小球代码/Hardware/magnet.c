#include "stm32f10x.h"                  // Device header


void Magnet_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PD9、PD10、PD11引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

void Magnet_ON(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
}

void Magnet_OFF(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

void Magnet_toggle(void)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_0,(BitAction)!GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0));
}