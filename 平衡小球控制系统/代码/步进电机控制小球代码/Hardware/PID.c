#include "stm32f10x.h"                  // Device header
#include <math.h>
#include "sys.h"

extern uint8_t Car_state;
extern uint8_t Car_stop;     // 停车线触发标志(main.c置位): 1=请求减速停车

// PWM输出
int PWM_L, PWM_R; 

float Target_Speed = 3;
float Target_turn = 0;

float Plan_Speed = 0;       // 当前速度目标, 直接给SpeedPID
float a_plan = 0;           // 预留给K230前馈(当前无梯度, 恒为0)

// 初始化PID结构体  Kp Ki Kd,目标，最后两个是限幅
PID_Init LocationPID = {20, 0, 0, 0, 0, 0, 0, 0, 0, 3000, 3000};   // 距离环
PID_Init SpeedPID = {200, 0, 0, 0, 0, 0, 0, 0, 0, 3600, 3600};	   // 速度环
PID_Init AnglePID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 3000, 3000};    // 角度环
PID_Init TrackPID = {100, 0, 0, 0, 0, 0, 0, 0, 0, 3000,3000};     // 循迹环

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

    
    // 微分项计算(用已环绕处理后的 error)
    pid->derivative = pid->error - pid->last_error;

    // 预积分(先限幅)
    float new_integral = pid->integral + pid->error;
    if (new_integral > pid->integral_limit) new_integral = pid->integral_limit;
    else if (new_integral < -pid->integral_limit) new_integral = -pid->integral_limit;

    // 试算输出(未限幅)
    float tentative = pid->Kp * pid->error + pid->Ki * new_integral + pid->Kd * pid->derivative;

    // 抗饱和(条件积分): 若试算已撞限幅且误差仍在朝同方向推, 则冻结积分,
    // 防止"球被静摩擦钉住→误差累积→角度越抬越高"的积分饱和(windup)
    if ((tentative >  pid->output_limit && pid->error > 0) ||
        (tentative < -pid->output_limit && pid->error < 0))
    {
        // 不累积积分, 保持原 integral
    }
    else
    {
        pid->integral = new_integral;
    }

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
	LocationPID.Target = Target_Location;
	float Location_Out = PID_Calculate(&LocationPID, Actual_Location);

	Plan_Speed = Location_Out;        // 位置环输出直接作为速度目标(无梯度)
	SpeedPID.Target = Plan_Speed;
  PID_Calculate(&SpeedPID,(Speed_L + Speed_R) / 2);
	
	TrackPID.Target = 0;
	PID_Calculate(&TrackPID,Track_Target_turn);
	
	//PWM_L = Speed_Output;
    //PWM_R = Speed_Output;
	
    PWM_L = SpeedPID.output + TrackPID.output;  // 左轮PWM
    PWM_R = SpeedPID.output - TrackPID.output;  // 右轮PWM
	
    if (PWM_L > 3600) PWM_L = 3600;
    else if (PWM_L < -3600) PWM_L = -3600;
    if (PWM_R > 3600) PWM_R = 3600;
    else if (PWM_R < -3600) PWM_R = -3600;
    
	if(Car_state == 0)
	{
		SetPWM(0,0);
		return;
	}
	else
	{
		SetPWM(PWM_L,PWM_R);
	}
}

void Track_Control_loop(void)
{
	// 手动急停(钥匙1关闭): 直接断电停车
	if(Car_state == 0)
	{
		SetPWM(0,0);
		return;
	}

	SpeedPID.Target = Target_Speed;
	PID_Calculate(&SpeedPID,(Speed_L + Speed_R) / 2);
	
	TrackPID.Target = 0;
	PID_Calculate(&TrackPID,Track_Target_turn);
	
	PWM_L = SpeedPID.output + TrackPID.output;  // 左轮PWM
  PWM_R = SpeedPID.output - TrackPID.output;  // 右轮PWM
	
    if (PWM_L > 3600) PWM_L = 3600;
    else if (PWM_L < -3600) PWM_L = -3600;
    if (PWM_R > 3600) PWM_R = 3600;
    else if (PWM_R < -3600) PWM_R = -3600;

	SetPWM(PWM_L,PWM_R);
}



