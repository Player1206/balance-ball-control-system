#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void left_positive(void); //左轮正传
void left_negative(void); //左轮反转
void right_positive(void); //右轮正转
void right_negative(void); //右轮反转
void SetPWM(int PWM1,int PWM2);



#endif
