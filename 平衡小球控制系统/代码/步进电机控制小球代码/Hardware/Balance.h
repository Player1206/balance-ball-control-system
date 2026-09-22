#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>

/* 摆球平衡目标位置(cm)，由任务模式设置：
 *   0      = 中心 O（第4/5项要求球稳中心）
 *   +5/-5  = 第3项“+5→-5”序列
 *   任意值 = 第6项按键启动瞬间记录的小球位置
 */
extern float Ball_Target_X;
extern float Ball_Target_Y;

void Balance_Init(void);     // 上电初始化：使能电机、清零零点、设快速位置参数
void Balance_Control(void);  // 10ms 控制拍调用：视觉误差 + 加速度前馈 -> 步进增量脉冲
void Balance_Task3Start(void); // 第3项测试: 静止小车, 小球 O -> +5cm -> -5cm
void Balance_Task3Stop(void);  // 退出第3项测试, 目标恢复到中心 O
void Balance_Task6Start(void); // 第6项测试: 记录当前球位置, 先原地锁球再持续保持
void Balance_Task6Stop(void);  // 退出第6项测试, 目标恢复到中心 O
void Balance_Task6Finish(void); // 第6项外部主控完成跑圈后可调用, 继续保持记录的指定位置
void Balance_StepperDebugStart(void); // 备用步进自检: 低速 +200/-200 脉冲, Key3 当前不再调用
void Balance_StepperDebugTick(void);  // 10ms 调用: 执行步进自检状态机

extern int32_t Step_Pulse_Current;  // 步进电机当前累计脉冲(绝对位置, 反映杆当前倾角状态), 供OLED显示
extern volatile uint8_t Balance_Run;          // 步进电机控制使能: 1=运行, 0=停止/未启动
extern volatile uint8_t Balance_DebugActive;  // 步进自检状态: 1=正在执行固定点动序列
extern volatile uint8_t Balance_Task3State;   // 第3项状态: 0空闲 1去+5 2去-5 3完成 4超时仍保持-5
extern volatile uint16_t Balance_Task3Time;   // 第3项计时, 单位10ms, 500约等于5s
extern volatile uint8_t Balance_Task6State;   // 第6项状态: 0空闲 1原地锁球 2行驶中持续锁球 3完成后保持
extern volatile uint16_t Balance_Task6Time;   // 第6项计时, 单位10ms
extern float Balance_Task6Target;             // 第6项启动瞬间记录的任意指定位置(cm)
extern float Balance_ThetaCmd;                // 当前控制输出角度(rad), 供OLED/调参观察
extern float Balance_AxZero;                  // JY61P X轴零偏；暂停时自动慢速更新
extern float Balance_AxDyn;                   // 扣除零偏后的动态X轴加速度(g)
extern float Balance_ThetaFF;                 // 当前加速度前馈坡度(rad)
extern int32_t Balance_PulseCmd;              // 当前控制输出增量脉冲, 供OLED/调参观察

#endif /* __BALANCE_H */
