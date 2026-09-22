#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;                  // 比例系数
    float Ki;                  // 积分系数
    float Kd;                  // 微分系数
    float Target;            // 目标值
    float error;               // 当前误差
    float last_error;          // 上一次误差
    float integral;            // 积分项
    float derivative;          // 微分项
    float output;              // 输出值
    float integral_limit;      // 积分限幅
    float output_limit;        // 输出限幅
} PID_Init;

// 各PID环实例(在PID.c中定义), 供其他文件(如main.c)直接使用
extern PID_Init LocationPID;
extern PID_Init SpeedPID;
extern PID_Init AnglePID;
extern PID_Init TrackPID;

// 速度目标(无梯度, 直接阶跃给定), 在PID.c中定义
extern float Plan_Speed;   // 当前速度目标
extern float a_plan;       // 预留给K230前馈(当前无梯度, 恒为0)

float PID_Calculate(PID_Init *pid, float actual);
void Track_Control_AB(float Target_Location);
void Track_Control_loop(void);

#endif
