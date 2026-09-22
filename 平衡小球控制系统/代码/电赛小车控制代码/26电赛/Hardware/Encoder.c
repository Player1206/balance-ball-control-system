#include "stm32f10x.h"                  // Device header

#define FILTER_COEFF 0.2f        // 一阶滤波系数
#define WHEEL_DIAMETER 6.5f      // 轮子直径 cm（65mm）
#define WHEEL_CIRCUMFERENCE (3.14f * WHEEL_DIAMETER)  // 轮子周长 cm
#define ENCODER_PPR 390          // 编码器单圈原始脉冲
#define REDUCTION_RATIO 28       // 减速比
#define ENCODER_FREQ 4           // 四倍频
#define ERROR 0.9f                //误差系数
#define DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE / (ENCODER_PPR * ENCODER_FREQ * ERROR));

int Speed_L, Speed_R, Speed_L_Temp, Speed_R_Temp;
float Actual_Location = 0.0f;  // 总行驶距离 cm

void Encoder_R_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65535;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	/*输入捕获初始化*/
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM3, ENABLE);
}

void Encoder_L_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Rising);

    TIM_Cmd(TIM5, ENABLE);
}

// 读取左轮脉冲
int16_t Encoder_L_Get(void)
{
	int16_t Temp = (int16_t)TIM_GetCounter(TIM5);
	TIM_SetCounter(TIM5, 0);
	return Temp;
}

// 读取右轮脉冲
int16_t Encoder_R_Get(void)
{
	int16_t Temp = (int16_t)TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return Temp;
}


void Encoder_Read(void)
{		
    int16_t pulse_L = Encoder_L_Get();
    int16_t pulse_R = Encoder_R_Get();
    
    Actual_Location += (pulse_L + pulse_R)/2.0f * DISTANCE_PER_PULSE;

    Speed_L = (int16_t)(pulse_L * FILTER_COEFF + Speed_L_Temp * (1 - FILTER_COEFF));
    Speed_R = (int16_t)(pulse_R * FILTER_COEFF + Speed_R_Temp * (1 - FILTER_COEFF));
    Speed_L_Temp = Speed_L;
    Speed_R_Temp = Speed_R;
}