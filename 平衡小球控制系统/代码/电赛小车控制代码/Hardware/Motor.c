#include "stm32f10x.h"                  // Device header
#include "PWM.h"


void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 先开启AFIO时钟
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  // 禁用JTAG，保留SWD
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    PWM_Init();
}

void left_positive(void) //左轮正传
{
	GPIO_SetBits(GPIOD,GPIO_Pin_6);
	GPIO_ResetBits(GPIOD,GPIO_Pin_7);
}

void left_negative(void) //左轮反转
{
	GPIO_SetBits(GPIOD,GPIO_Pin_7);
	GPIO_ResetBits(GPIOD,GPIO_Pin_6);
}

void right_positive(void) //右轮正转
{
	GPIO_SetBits(GPIOB,GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

void right_negative(void) //右轮反转
{
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
}

/*SetPWM()中直接将负 PWM 值传给TIM_SetComparex()
但该函数参数为无符号整数（uint16_t），负数会被强制转换为大正数，导致电机失控。*/

 void SetPWM(int PWM1,int PWM2)
{
	if(PWM1>0) //正转
	{
		left_negative();
		TIM_SetCompare1(TIM8, PWM1);  
	}
	else       //反转
	{
		left_positive();
		TIM_SetCompare1(TIM8, -PWM1);     
	}	
	
	if(PWM2>0) //正转
	{
		right_negative();
		TIM_SetCompare2(TIM8, PWM2);
	}
	else       //反转
	{
		right_positive();
		TIM_SetCompare2(TIM8, -PWM2);
	}
}


