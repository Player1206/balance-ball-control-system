#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


void uart1_init(u32 bound);					//����1��ʼ������
void uart3_init(u32 bound);                 //����3��ʼ������(JY61P)
void uart5_init(u32 bound);                 //����5��ʼ������(�Ŵ�ͷ�������, PC12/PD2)
void uart4_init(u32 bound);                 //����4��ʼ������(K230�Ӿ�, ������ռԭ�д���)
void JY61P_Decode(uint8_t Data);
void JY61P_Output_Accel(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);


// ȫ�ֱ������洢�����������������
extern float Ax, Ay, Az;    // ���ٶ�
extern float Wx, Wy, Wz;    // ���ٶ�
extern float Roll, Pitch, Yaw;  // �Ƕ�


#endif


