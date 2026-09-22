#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint8_t Key_Num;

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);		//开启GPIOC的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~3，返回0代表没有按键按下
  */
uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if(Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

uint8_t Key_GetState(void)
{
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1) == 0)
	{
		return 1;
	}
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2) == 0)
	{
		return 2;
	}
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3) == 0)
	{
		return 3;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	Count++;
	if(Count >= 20)
	{
		Count = 0;
		
		PrevState = CurrState;
		CurrState = Key_GetState();
		
		if(CurrState == 0 && PrevState != 0)
		{
			Key_Num = PrevState;
		}
	
	}

}
	
