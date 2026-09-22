#include "sys.h" 

/**************************************************************************
Buzzer:
    PE2
	
KEY:
    Key1->PC1
	Key2->PC2
	Key3->PC3
	
电机PWM:
    *TIM8
    PWMA->PC6
    PWMB->PC7
	
编码器:
    *TIM5
    E1A->PA0
    E1B->PA1//左电机编码器
    *TIM3
    E2A->PA6
    E2B->PA7//右电机编码器
	
Motor:
    AIN1->PD7
    AIN2->PD6//左电机
    BIN1->PB4
    BIN2->PB5//右电机
	
OLED屏幕:
    SCL->PB8
    SDA->PB9
	
循迹(引出引脚杜邦线连接):
    DAT->PE7
    CLK->PE8

串口3 -- JY61P:
    RX->PB11  (连接传感器TX)
	TX->PB10  (连接传感器RX)
	
NRL:
	CE->PA4
	CSN->PA5 
	SCK->PC4
	MOSI->PC5
	MISO->PB0
**************************************************************************/

void SYS_Init(void)
{
	SystemInit();
	delay_init();
	LED_Init();
	Buzzer_Init();
	OLED_Init();
	Key_Init();
	Track_Init();   
	uart1_init(9600); //9600波特率
	uart3_init(115200); //115200波特率
	Encoder_R_Init();
	Encoder_L_Init();
	PWM_Init();
	Motor_Init();
	Timer_Init();
}
