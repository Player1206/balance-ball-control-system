#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include "sys.h"
#include "usart.h"     // uart4_init(K230) / JY61P
#include "step.h"      // STEP/DIR 脉冲控制
#include "K230.h"      // 视觉球位置
#include "Balance.h"   // 摆球平衡控制

/* ==================== 全局状态变量 ==================== */
uint8_t KeyScan; //摁键摁下数据
uint8_t KeyNum;  //记录摁键数值
uint8_t Car_stop = 0; // 小车停止标志位（对于底盘MCU是生效的，此处为留存定义）
uint8_t Car_state = 0; // 小车行驶状态标志
float Target_Location = 150; // 目标位置（针对第3/6项等任务的目标参数）


uint8_t Buzzer_Flag = 0;

/* JY61P 调试观测量。
 * usart.h 当前是旧编码文件，先在这里声明，避免为了临时显示去改动头文件编码。
 */
extern volatile uint32_t JY61P_FrameCount;

int main(void)
{
	/*模块初始化*/
	SYS_Init();
	TIM_Cmd(TIM1, DISABLE); // 先关控制中断, 等STEP/DIR安全初始化完成后再开
	// 不再使用张大头串口运动模式; PC12/PD2 改由 StepPulse_Init 配置为 STEP/DIR
	uart4_init(115200);   // K230 视觉串口(独立 UART4, 不占 USART1/2/3)
	Balance_Init();       // 摆球平衡子系统初始化(使能电机/清零/设参)
	TIM_Cmd(TIM1, ENABLE);
	
	/*OLED显示*/
	//OLED_ShowString(1,1,"HelloWorld",OLED_8X16);
	OLED_Update();
	
	while (1)
	{
		// ==================== 任务模式切换逻辑 ====================
		// 获取摁键摁下
		KeyScan = Key_GetNum();
		
		// 任务一：普通中心保持模式 (对应电赛第4/5项要求)
		if(KeyScan == 1)
		{
			KeyNum = 0;
			if (Ball_Updated)
			{
				Balance_Task3Stop(); // 按键1: 普通中心保持模式, 退出第3项自动流程
				Balance_Task6Stop(); // 同时退出第6项任意点跑圈模式
				Balance_Run = 1;   // 按键1: 有视觉有效帧时才开始步进闭环控制
			}
			else
			{
				Balance_Run = 0;   // 防止开机/视觉未就绪时误启动
			}
		}
		// 任务二：任意指定位置保持 (对应电赛第6项)
		if(KeyScan == 2)
		{
			KeyNum = 0;       // 按键2: 第6项只负责摆球；循迹/车轮由另一个MCU主控
			if (Balance_DebugActive)
			{
				StepPulse_Stop();
				StepPulse_Enable(0);
				Balance_DebugActive = 0;
			}
			if (Ball_Updated)
			{
				Balance_Task6Start(); // 第6项: 任意指定位置保持 + 小车循迹一圈
			}
			else
			{
				Balance_Run = 0;      // 没有视觉帧时不启动, 防止盲动
				Balance_Task6Stop();
			}
		}
		// 任务三：动态折返任务 (对应电赛第3项：O -> +5cm -> -5cm)
		if(KeyScan == 3)
		{
			KeyNum = 0;
			Car_state = 0;   // 第3项要求小车静止, 先关循迹小车
			SetPWM(0,0);
			Balance_Task6Stop(); // 退出第6项, 避免两个任务同时改目标
			if (Ball_Updated)
			{
				Balance_Task3Start(); // 按键3: 题目第3项, O -> +5cm -> -5cm
			}
			else
			{
				Balance_Run = 0;      // 没有视觉帧时不启动, 防止盲动
			}
		}

		//*********************************************摆球(K230 + 步进)打印区
		OLED_Printf(0,0,OLED_8X16,"X:%.1f V:%.1f",Ball_X, Ball_V);  // K230球位置/速度(cm, 右正左负)
		OLED_Printf(0,15,OLED_8X16,"Ax:%.2f D:%.2f",Ax, Balance_AxDyn); // Ax原始值/扣零偏后的动态值
		OLED_Printf(0,30,OLED_8X16,"FF:%.2f T:%.2f",Balance_ThetaFF, Balance_ThetaCmd); // 前馈坡度/最终坡度
		OLED_Printf(0,45,OLED_8X16,"J%03lu Q3:%d Q6:%d",
		            (unsigned long)(JY61P_FrameCount % 1000),
		            Balance_Task3State,
		            Balance_Task6State);                            // J=有效帧计数，Q3/Q6=任务状态
		
		OLED_Update();
		
		//串口发送区//
		//printf("Roll:%.1f Pitch:%.1f Yaw:%.1f\r\n",Roll,Pitch,Yaw);
    JY61P_Output_Accel();   // 输出 JY61P x/y/z 加速度
		
		
		delay_ms(50);
	}
}

// ==================== 定时器控制核心中断 ====================
// 10ms中断一次，系统所有的核心反馈控制（包括视觉坐标读取后的PID运算）都在这里执行
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();           // 1. 按键消抖扫描
		Encoder_Read();       // 2. 编码器读数
		Read_Track_DATA(&Track); // 3. 灰度循迹传感器数据读取
		Track_Control();      // 4. 循迹控制逻辑更新
		if (Balance_DebugActive)
		{
			Balance_StepperDebugTick();   // 步进自检优先, 避免和闭环控制抢串口
		}
		else
		{
			Balance_Control();   // 摆球平衡: 视觉误差 + 加速度前馈 -> 步进电机(每10ms)
		}
		if (!Balance_DebugActive)
		{
			if(KeyNum == 3)
			{
				Track_Control_AB(Target_Location);
			}
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//清除标志位
	}
}

