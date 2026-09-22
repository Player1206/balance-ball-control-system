#include "Balance.h"
#include "step.h"    // 张大头电机串口指令
#include "usart.h"   // Ax (JY61P 加速度, 单位 g)
#include "K230.h"    // Ball_X (视觉球位置)

/* ===================== 机械 / 电机参数 ===================== */
#define STEP_BAUD           115200      // 张大头默认波特率(菜单可改, 需与 uart5_init 一致)
#define STEP_PULSES_PER_REV 3200.0f     // 16细分: 3200脉冲/圈(拨码改细分需同步改)
#define ROD_GEAR_RATIO      1.0f        // 电机->摆杆传动比(皮带/齿轮减速, 直连=1)
#define PULSES_PER_RAD      (STEP_PULSES_PER_REV * ROD_GEAR_RATIO / (2.0f * 3.14159265f))

#define BALANCE_DIR_SIGN    (1.0f)      // 方向修正：装反了把杆倾错向 -> 改成 -1
#define THETA_MAX           (0.35f)     // 摆杆最大倾角(rad, ~20°)，机械限位保护
#define KP_POS              (0.02f)     // 位置环增益(rad/cm)：球每偏1cm, 杆倾0.02rad
#define KFF                 (1.0f)      // 加速度前馈增益(rad per g)：theta_ff ≈ Ax

float Ball_Target_X = 0;   // 默认稳中心 O
float Ball_Target_Y = 0;

/* STM32 侧摆杆角度估计(rad)：由已下发脉冲累加得到，闭环电机不丢步，与电机位置同步 */
static float Step_Angle_Current = 0;

void Balance_Init(void)
{
    Step_Enable(STEP_ADDR, MOTOR_ENABLE, SYNC_OFF);
    Step_ZeroPos(STEP_ADDR);          // 以当前机械位置为零点
    Step_Angle_Current = 0;

    /* 快速位置模式参数：速度(rpm)、加速度、相对当前位置(POS_REL_CUR)
     * 之后每拍只发 Step_QPosRun(增量脉冲)，由驱动自己积分到绝对位置 */
    Step_QPosSetParams(STEP_ADDR, 600, 50, POS_REL_CUR, SYNC_OFF);
}

/* 10ms 调用一次 */
void Balance_Control(void)
{
    float e_pos, theta_pos, theta_ff, theta_cmd, dtheta;
    int32_t pulses;

    /* 1) 视觉位置环：球偏了 -> 该把杆倾多少(主控制) */
    e_pos    = Ball_Target_X - Ball_X;        // cm（正: 球在目标右侧, 需左倾推回）
    theta_pos = KP_POS * e_pos;

    /* 2) 加速度前馈(开环预补偿)：车加速把球甩后, 提前预倾抵消惯性 */
    theta_ff = KFF * Ax;                       // Ax 单位 g -> theta_ff ≈ Ax(rad), 小角近似

    /* 3) 合成指令角并限幅(机械保护) */
    theta_cmd = theta_pos + theta_ff;
    if (theta_cmd >  THETA_MAX) theta_cmd =  THETA_MAX;
    if (theta_cmd < -THETA_MAX) theta_cmd = -THETA_MAX;

    /* 4) 角度环 -> 增量脉冲：本次只需补 (theta_cmd - 当前角) 的脉冲数 */
    dtheta = theta_cmd - Step_Angle_Current;
    pulses = (int32_t)(BALANCE_DIR_SIGN * dtheta * PULSES_PER_RAD);

    Step_QPosRun(STEP_ADDR, pulses);
    Step_Angle_Current += dtheta;
}
