#include "stm32f10x.h"                  // Device header
#include <math.h>
#include "sys.h"

extern uint8_t Car_state;
extern uint8_t Car_stop;     // 停车线触发标志: 1=请求减速停车

// PWM输出
int PWM_L, PWM_R; 
float Target_Speed = 26;
float Target_turn = 0;

// 缓启动/缓停车: 加速度斜坡控制速度目标爬升/下降
float Car_Accel = 1.6f;       // loop_30 加速度, 控制启动/停车爬升斜率
float ramp_v = 0;             // 当前斜坡速度目标(loop_30)

// AB任务独立参数
float Car_Accel_AB = 4.6f;    // AB 加速度, 可调
float ramp_v_AB = 0;          // AB 当前斜坡速度目标
float Target_Speed_AB = 26;   // AB 速度上限

// 初始化PID结构体  Kp Ki Kd,目标，最后两个是限幅
PID_Init LocationPID = {5, 0, 0, 0, 0, 0, 0, 0, 0, 3000, 3000};   // 距离环
PID_Init SpeedPID = {280, 0, 5, 0, 0, 0, 0, 0, 0, 3600, 3600};	   // 速度环
PID_Init AnglePID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 3000, 3000};    // 角度环
PID_Init TrackPID = {40, 0, 5, 0, 0, 0, 0, 0, 0, 3000,3000};     // 循迹环

// PID计算函数
float PID_Calculate(PID_Init *pid, float actual) 
{
    // 计算当前误差
    pid->error = pid->Target - actual;
    
    // 限制角度
    if (pid == &AnglePID) 
	{
        if (pid->error > 180) pid->error -= 360;
        if (pid->error < -180) pid->error += 360;
  }

    
    // 积分项计算
    pid->integral += pid->error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    // 微分项计算
    pid->derivative = pid->error - pid->last_error;
    
    // PID输出计算
    pid->output = pid->Kp * pid->error + 
                 pid->Ki * pid->integral + 
                 pid->Kd * pid->derivative;
    
    // 输出限幅
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
   
    // 保存当前误差供下次计算
    pid->last_error = pid->error;
    
    return pid->output;
}

void Track_Control_AB(float Target_Location)
{
	// 优先级: Car_stop(缓停) > 急停(Car_state=0) > 正常行驶(缓启动)
	// 注: 停车触发(过线/距离到达)由main.c统一置Car_stop=1并冻结计时, 此处只负责按Car_Accel_AB滑停
	if(Car_stop == 1)
	{
		// 请求缓停: 车仍在跑, 速度按Car_Accel_AB滑到0才真正停
		ramp_v_AB -= Car_Accel_AB * 0.01f;
		if(ramp_v_AB <= 0) { ramp_v_AB = 0; Car_stop = 0; Car_state = 0; }
		SpeedPID.Target = ramp_v_AB;
	}
	else if(Car_state == 0)
	{
		ramp_v_AB = 0;
		SetPWM(0,0);
		return;
	}
	else
	{
		// 正常行驶: 速度环+循迹环, 速度按Car_Accel_AB斜坡爬升到Target_Speed_AB, 不用位置环
		if(ramp_v_AB < Target_Speed_AB) ramp_v_AB += Car_Accel_AB * 0.01f;
		if(ramp_v_AB > Target_Speed_AB) ramp_v_AB = Target_Speed_AB;
		SpeedPID.Target = ramp_v_AB;
	}

  PID_Calculate(&SpeedPID,(Speed_L + Speed_R) / 2);
	
	TrackPID.Target = 0;
	PID_Calculate(&TrackPID,Track_Target_turn);
	
    PWM_L = SpeedPID.output + TrackPID.output;  // 左轮PWM
    PWM_R = SpeedPID.output - TrackPID.output;  // 右轮PWM
	
    if (PWM_L > 7200) PWM_L = 7200;
    else if (PWM_L < -7200) PWM_L = -7200;
    if (PWM_R > 7200) PWM_R = 7200;
    else if (PWM_R < -7200) PWM_R = -7200;

	SetPWM(PWM_L,PWM_R);
}

void Track_Control_loop(void)
{
	float v_max = Target_Speed;   // 任务速度上限(loop_30=24, 12s后改17)

	// 优先级: Car_stop(缓停) > 急停(Car_state=0) > 正常行驶(缓启动)
	if(Car_stop == 1)
	{
		// 请求缓停: 车仍在跑(Car_state=1), 速度按Car_Accel滑到0才真正停
		ramp_v -= Car_Accel * 0.01f;
		if(ramp_v <= 0) { ramp_v = 0; Car_stop = 0; Car_state = 0; }
		SpeedPID.Target = ramp_v;
	}
	else if(Car_state == 0)
	{
		// 普通急停(钥匙关闭): 硬断电
		ramp_v = 0;
		SetPWM(0,0);
		return;
	}
	else
	{
		// 缓启动: 速度按Car_Accel斜坡爬升到v_max
		if(ramp_v < v_max) ramp_v += Car_Accel * 0.01f;
		if(ramp_v > v_max) ramp_v = v_max;
		SpeedPID.Target = ramp_v;
	}

	PID_Calculate(&SpeedPID,(Speed_L + Speed_R) / 2);
	
	TrackPID.Target = 0;
	PID_Calculate(&TrackPID,Track_Target_turn);
	
	PWM_L = SpeedPID.output + TrackPID.output;  // 左轮PWM
  PWM_R = SpeedPID.output - TrackPID.output;  // 右轮PWM
	
    if (PWM_L > 7200) PWM_L = 7200;
    else if (PWM_L < -7200) PWM_L = -7200;
    if (PWM_R > 7200) PWM_R = 7200;
    else if (PWM_R < -7200) PWM_R = -7200;

	SetPWM(PWM_L,PWM_R);
}

void Track_Control_loop_20(void)
{
	if(Car_state == 0)
	{
		SetPWM(0,0);
		return;
	}
	SpeedPID.Target = 28;

	PID_Calculate(&SpeedPID,(Speed_L + Speed_R) / 2);
	
	TrackPID.Target = 0;
	PID_Calculate(&TrackPID,Track_Target_turn);
	
	PWM_L = SpeedPID.output + TrackPID.output;  // 左轮PWM
  PWM_R = SpeedPID.output - TrackPID.output;  // 右轮PWM
	
    if (PWM_L > 7200) PWM_L = 7200;
    else if (PWM_L < -7200) PWM_L = -7200;
    if (PWM_R > 7200) PWM_R = 7200;
    else if (PWM_R < -7200) PWM_R = -7200;

	SetPWM(PWM_L,PWM_R);
}



