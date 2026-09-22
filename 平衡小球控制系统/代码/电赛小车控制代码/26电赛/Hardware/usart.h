#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


void uart1_init(u32 bound);					//串口1初始化函数
void uart3_init(u32 bound);                 //串口3初始化函数(JY61P)
void uart5_init(u32 bound);                 //串口5初始化函数(张大头步进电机, PC12/PD2)
void uart4_init(u32 bound);                 //串口4初始化函数(K230视觉, 独立不占原有串口)
void JY61P_Decode(uint8_t Data);
void JY61P_Output_Accel(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);


// 全局变量：存储解析后的陀螺仪数据
extern float Ax, Ay, Az;    // 加速度
extern float Wx, Wy, Wz;    // 角速度
extern float Roll, Pitch, Yaw;  // 角度


#endif


