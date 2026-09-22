#include "stm32f10x.h"                  // Device header

/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  */
void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);		//开启GPIOD的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);						//将PD9、PD10、PD11引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_ResetBits(GPIOD, GPIO_Pin_9);
	GPIO_ResetBits(GPIOD, GPIO_Pin_10);
	GPIO_ResetBits(GPIOD, GPIO_Pin_11);
}

//D10输出低电平，D11输出高电平，D9输出高电平，则RGB灯发红光
void RedLED_ON(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_9);
	GPIO_SetBits(GPIOD,GPIO_Pin_11);
}

void RedLED_OFF(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_9);
	GPIO_ResetBits(GPIOD,GPIO_Pin_11);	
}

//D10输出高电平，D11输出低电平，D9输出高电平，则RGB灯发绿光
void GreenLED_ON(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_9);
	GPIO_SetBits(GPIOD,GPIO_Pin_10);
}

void GreenLED_OFF(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_9);
	GPIO_ResetBits(GPIOD,GPIO_Pin_10);
}

//D10输出低电平，D11输出低电平，D9输出高电平时，则RGB灯发黄色
void YellowLED_ON(void)
{
	GPIO_SetBits(GPIOD,GPIO_Pin_9);
}

void YellowLED_OFF(void)
{
	GPIO_ResetBits(GPIOD,GPIO_Pin_9);
}


