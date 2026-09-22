#include "usart.h"	  
#include "K230.h"     // K230 视觉球位置解析

float Ax, Ay, Az;    // 加速度
float Wx, Wy, Wz;    // 角速度
float Roll, Pitch, Yaw;  // 角度


//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

u8 USART_RX_BUF[64];     //接收缓冲,最大64个字节.
//接收状态
//bit7，接收完成标志
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
u8 USART_RX_STA=0;       //接收状态标记

void uart1_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//调试口默认只发(printf); K230 已改走独立 UART4
	USART_Cmd(USART1, ENABLE);                    //使能串口 
}

void uart3_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //串口时钟使能	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //GPIO使用使能

	//UARTx TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//UARTx RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure);
	
	
    USART_ITConfig(USART3, USART_IT_RXNE , ENABLE);//串口接收中断
    USART_Cmd(USART3, ENABLE);
}


// JY61P 加速度输出函数（通过USART1调试口输出）
void JY61P_Output_Accel(void)
{
    printf("AX:%.2f AY:%.2f AZ:%.2f\r\n", Ax, Ay, Az);
}

// JY61P 数据解析函数
void JY61P_Decode(uint8_t Data)
{
    // 校验和验证
	static u8 RX_Data[250];
	static u8 len = 0;
	RX_Data[len++]=Data; //将收到的数据放到缓冲区
	
	if(len < 11) return; //数据没满重传
	
	u8 sum = 0;
    for(u8 i=0; i<10; i++) sum += RX_Data[i];
    if(sum != RX_Data[10])  // 校验失败，丢弃
    {
        len = 0;
        return;
    }
	
	
		switch(RX_Data[1])
		{
        case 0x51:  // 加速度
            Ax = (short)((short)RX_Data[3]<<8 | RX_Data[2])/32768.0*16.0;
            Ay = (short)((short)RX_Data[5]<<8 | RX_Data[4])/32768.0*16.0;
            Az = (short)((short)RX_Data[7]<<8 | RX_Data[6])/32768.0*16.0;
            break;

        case 0x52:  // 角速度
            Wx = (short)((short)RX_Data[3]<<8 | RX_Data[2])/32768.0*2000.0;
            Wy = (short)((short)RX_Data[5]<<8 | RX_Data[4])/32768.0*2000.0;
            Wz = (short)((short)RX_Data[7]<<8 | RX_Data[6])/32768.0*2000.0;
            break;

        case 0x53:  // 角度
            Roll =  (short)((short)RX_Data[3]<<8 | RX_Data[2])/32768.0*180.0;
            Pitch = (short)((short)RX_Data[5]<<8 | RX_Data[4])/32768.0*180.0;
            Yaw =   (short)((short)RX_Data[7]<<8 | RX_Data[6])/32768.0*180.0;
            break;
    }
	len = 0; //清理缓存
	
}

// 串口2 接收中断服务函数
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
		uint8_t Data = (USART3 -> DR & (uint8_t)0x01FF);
        JY61P_Decode(Data);
    }
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}


/* ==========================================================================
 *  UART5 —— 张大头闭环步进电机串口总线 (PC12=TX, PD2=RX)
 *  说明：step.c 的 Serial_SendByte() 底层走这里。
 *        若张大头经 RS485 模块连接，需一个方向控制脚(RE/DE)；
 *        若 TTL 直连则把 STEP_RS485_DIR_USED 置 0。
 * ========================================================================== */
#define STEP_RS485_DIR_USED  0          // 1=RS485(需方向脚), 0=TTL 直连
#if STEP_RS485_DIR_USED
#define STEP_DIR_GPIO        GPIOA
#define STEP_DIR_PIN         GPIO_Pin_1  // 按实际接线修改(选空闲脚)
#endif

void uart5_init(u32 bound)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;                // TX (PC12)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                 // RX (PD2)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

#if STEP_RS485_DIR_USED
    GPIO_InitStructure.GPIO_Pin = STEP_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(STEP_DIR_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(STEP_DIR_GPIO, STEP_DIR_PIN);              // 默认接收
#endif

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);   // 可选：读取电机回传状态
    USART_Cmd(UART5, ENABLE);
}

// step.c 依赖的底层单字节发送：经 UART5 发给张大头电机
void Serial_SendByte(uint8_t Byte)
{
#if STEP_RS485_DIR_USED
    GPIO_SetBits(STEP_DIR_GPIO, STEP_DIR_PIN);       // 切到发送
#endif
    USART_SendData(UART5, Byte);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
#if STEP_RS485_DIR_USED
    GPIO_ResetBits(STEP_DIR_GPIO, STEP_DIR_PIN);     // 切回接收
#endif
}

// UART5 接收(电机回传状态，可选解析)
void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
    {
        UART5->DR;   // 暂未解析电机回传，读掉即可
    }
    USART_ClearITPendingBit(UART5, USART_IT_RXNE);
}


/* ==========================================================================
 *  UART4 —— K230 视觉模块独立串口 (PC10=TX, PC11=RX)
 *  F103VET6 为高端密度, 自带 UART4/UART5; K230 独占 UART4,
 *  完全不占用调试 USART1 / 电机 USART2 / 陀螺仪 USART3。
 * ========================================================================== */
void uart4_init(u32 bound)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);   // 接收 K230 球位置
    USART_Cmd(UART4, ENABLE);
}

// UART4 接收 —— K230 视觉球位置
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
    {
        uint8_t Data = (UART4->DR & 0xFF);
        K230_Decode(Data);
    }
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
}