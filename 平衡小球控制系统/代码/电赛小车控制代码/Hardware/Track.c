#include "stm32f10x.h"                  // Device header
#include "Track.h"
#include "sys.h"

uint8_t Track;
uint8_t TrackN;
float Track_Target_turn;

uint8_t lost;

#define Track_DAT	GPIO_Pin_7//定义GPIO引脚
#define Track_SCL	GPIO_Pin_8//定义GPIO引脚

//初始化函数
void Track_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);//开启GPIOE的时钟
	
	GPIO_InitTypeDef GPIOStructure;
	GPIOStructure.GPIO_Mode = GPIO_Mode_IPU;//输入上拉模式
	GPIOStructure.GPIO_Pin = Track_DAT;
	GPIOStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIOStructure);
	
	GPIOStructure.GPIO_Mode = GPIO_Mode_Out_PP;     
	GPIOStructure.GPIO_Pin = Track_SCL;
	GPIOStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIOStructure);
}

uint8_t Track_Read_bit(void)
{
	uint8_t bit = 0;
	GPIO_ResetBits(GPIOE, Track_SCL);//SCL拉低
	bit = GPIO_ReadInputDataBit(GPIOE, Track_DAT);//读取DAT电平
	GPIO_SetBits(GPIOE, Track_SCL);//SCL拉高
	delay_us(6);
	return bit;
}

void Read_Track_DATA(uint8_t* arr)
{
    uint8_t n = 0;
    uint8_t strackarr[8] = {0};
    static uint8_t last_track = 0;
    uint8_t current_track = 0;

    for (n = 0; n < 8; n++) 
	{
        strackarr[n] = Track_Read_bit();
    }
    // 拼接为8位整数（strackarr[7]为最高位bit7，strackarr[0]为最低位bit0）
    current_track = strackarr[7] + strackarr[6]*2 + strackarr[5]*4 + strackarr[4]*8 +
                    strackarr[3]*16 + strackarr[2]*32 + strackarr[1]*64 + strackarr[0]*128;

    // 均值滤波
    TrackN = (uint8_t)((current_track + last_track) / 2);
    last_track = current_track;
    *arr = TrackN;
}

void Track_Control(void) 
{
    // 8个传感器的权重（左负右正）
    int weight[8] = {8,6,4,2,-2,-4,-6,-8};
    int sum = 0;       // 分数总和
    int count = 0;     // 检测到黑线的传感器数量

    // 计算总分和数量
    for(int i=0; i<8; i++)
	{
        if(!(Track & (1<<i)))
			{  // 第i个传感器检测到黑线
            sum += weight[i];
            count++;
            }
    }

	Track_Target_turn = sum;
		
    // 限制最大转向力度（防止过度转向）
    if(Track_Target_turn>20) Track_Target_turn= 20;
    if(Track_Target_turn<-20) Track_Target_turn= -20;
}