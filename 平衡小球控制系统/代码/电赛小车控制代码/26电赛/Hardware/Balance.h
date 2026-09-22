#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>

/* 摆球平衡目标位置(cm)，由任务模式设置：
 *   0     = 中心 O（第4/5/6项要求球稳中心）
 *   +5/-5 = 第3项“+5→0→-5”序列
 */
extern float Ball_Target_X;
extern float Ball_Target_Y;

void Balance_Init(void);     // 上电初始化：使能电机、清零零点、设快速位置参数
void Balance_Control(void);  // 10ms 控制拍调用：视觉误差 + 加速度前馈 -> 步进增量脉冲

#endif /* __BALANCE_H */
