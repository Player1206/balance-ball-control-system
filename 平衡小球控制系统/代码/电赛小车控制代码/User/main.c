#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include "sys.h"

/*全局变量*/
uint8_t KeyScan; //摁键摁下数据
uint8_t KeyNum;  //记录摁键数值
uint8_t Car_stop = 0; // 全局缓停标志位，置 1 时底层电机控制会减速停下
uint8_t Car_state = 0; // 小车运行状态(1: 行驶中, 0: 停止)
uint8_t task = 1;     // 当前任务模式(1: loop_20, 2: loop_30, 3: 直线AB)
float Target_Location = 160; // 任务3直线目标行驶距离(cm)

uint8_t Buzzer_Flag = 0;

// ==================== 停车屏蔽时间 ====================
// 行驶多少秒后才允许进入停车标志位检测(防止起步时受车身姿态或原地标线干扰误触发停车)
#define RUN_TIME_MIN_SEC_loop30   24
#define RUN_TIME_MIN_SEC_AB   4
#define RUN_TIME_MIN_SEC_loop20   15

uint32_t RunTime_ms = 0;      // 行驶计时(ms): 检测到启动->停车瞬间, 中断累加, OLED显示
uint8_t RunTime_Running = 0;  // 计时窗口标志: 启动置1, 检测到停车瞬间置0(冻结), 与Car_state缓停解耦
uint8_t Stop_Track = 0;       // loop_30 触发停车时的循迹标志值(Track), OLED显示用

extern float Car_Accel;    // 缓启动加速度(PWM/秒), 由PID.c定义, 停车时Key2+/Key3-可调
extern float Actual_Location;   // 实际总行驶距离(cm), 由Encoder.c累积

int main(void)
{
	/*模块初始化*/
	SYS_Init();
	
	/*OLED显示*/
	//OLED_ShowString(1,1,"HelloWorld",OLED_8X16);
	OLED_Update();
	
	while (1)
	{
		//获取摁键摁下
		KeyScan = Key_GetNum();
		if(KeyScan == 1)
		{
			Car_stop = 0;
			Stop_Track = 0;               // 清上次停车标志显示
			if(!RunTime_Running) RunTime_ms = 0;
			Car_state = !Car_state;
			RunTime_Running = Car_state;  // 启动计时/停止计时
		}
		if(KeyScan == 2)
		{
        task++;
			  if(task > 3) task = 1;
		}
		if(KeyScan == 3)
		{
       KeyNum = 3;
		}

		//变量显示区//
		//*********************************************陀螺仪打印区
		//OLED_Printf(0,0,OLED_8X16,"Pitch: %.1f",Pitch);
		//OLED_Printf(0,15,OLED_8X16,"Roll: %.1f",Roll);
		//OLED_Printf(0,30,OLED_8X16,"Yaw: %.1f",Yaw);
		
		//*********************************************编码器打印区
		OLED_Printf(0,30,OLED_8X16,"L:%d R:%d",Speed_L,Speed_R);
		OLED_Printf(72,30,OLED_8X16,"S_T:0x%02X",Stop_Track);   // loop_30停车时的循迹标志值(Track)

		//*********************************************行驶计时区(以秒为单位)
		OLED_ShowString(0,45,"T:",OLED_8X16);
		OLED_ShowFloatNum(16,45,(double)RunTime_ms/1000.0,2,2,OLED_8X16);
		
		//*********************************************循迹数据打印区
		OLED_ShowBinNum(0,15,Track,8,OLED_8X16);
		OLED_Printf(72,15,OLED_8X16,"D:%.0f a:%d",Actual_Location, (int)Car_Accel);   // 距离(cm)+缓启动加速度
		
		//*********************************************标志位打印区
		OLED_Printf(0,0,OLED_8X16,"State:%d",Car_state);
		if(task == 1)
		{
			OLED_ShowString(70,0,"loop_20",OLED_8X16);
		}
		if(task == 2)
		{
			OLED_ShowString(70,0,"loop_30",OLED_8X16);
		}
		if(task == 3)
		{
			OLED_ShowString(70,0,"AB",OLED_8X16);
		}
			
			
		//OLED_Printf(15,15,OLED_8X16,"Loops:%d",KeyNum);

		OLED_Update();
		
		//串口发送区//
		//printf("Roll:%.1f Pitch:%.1f Yaw:%.1f\r\n",Roll,Pitch,Yaw);
    JY61P_Output_Accel();   // 输出 JY61P x/y/z 加速度

		
		
		delay_ms(50);
	}
}

//10ms中断一次
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();
		Encoder_Read();
		Read_Track_DATA(&Track);
		Track_Control();
		// ==================== 底层任务控制逻辑 ====================
		// task == 1: loop_20 跑内圈圆
		if(task == 1)
		{
			// 如果当前行驶中，且触发了特征线循迹值，并且行驶时间超过屏蔽时间，强制停车
			if(Car_state == 1 &&
			   (Track == 0 || Track == 0xC3 || Track == 0x81 || Track == 0x83 || Track == 0xC1) &&
			   RunTime_ms/1000 >= RUN_TIME_MIN_SEC_loop20)   // 行驶够久才允许停车, 防误触发
			{
				Car_state = 0;
				RunTime_Running = 0;   // 过线硬停瞬间冻结计时
			}
			Track_Control_loop_20(); // 执行 loop_20 的 PID 循迹逻辑
		}
		
		// task == 2: loop_30 跑外圈常规比赛圈
		if(task == 2)
		{
			// 相比于 task1，此处使用 Car_stop = 1 触发缓停逻辑，以提高定点停车精度
			if(Car_state == 1 &&
			   (Track == 0 || Track == 0xC3 || Track == 0x81 || Track == 0x83 || Track == 0xC1) &&
			   RunTime_ms/1000 >= RUN_TIME_MIN_SEC_loop30)   // 行驶够久才允许停车, 防误触发
			{
				Car_stop = 1;          // 请求缓停
				RunTime_Running = 0;   // 检测到停车瞬间即冻结计时(不等地滑停结束)
				Stop_Track = Track;    // 记录触发停车的循迹标志值
			}
			Track_Control_loop();    // 执行常规外圈的循迹逻辑
		}
		
		// task == 3: AB 直线段里程闭环
		if(task == 3)
		{
			// 当检测到特定路口特征，或者编码器测算里程逼近 Target_Location，触发缓停
			if(Car_state == 1 &&
			   ((Track == 0 || Track == 0xC3 || Track == 0x81 || Track == 0x83 || Track == 0xC1) ||
			    fabs(Actual_Location - Target_Location) < 3.0f) &&
			   RunTime_ms/1000 >= RUN_TIME_MIN_SEC_AB)   // 行驶够久才允许停车, 防误触发
			{
				Car_stop = 1;          // 请求缓停
				RunTime_Running = 0;   // 到达距离瞬间即冻结计时
			}
			Track_Control_AB(Target_Location);
		}
		
		// 行驶计时: 计时窗口内(RunTime_Running)累加, 检测到停车瞬间冻结
		if(RunTime_Running)
		{
			RunTime_ms += 10;   // 10ms中断一次
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//清除标志位
	}
}

