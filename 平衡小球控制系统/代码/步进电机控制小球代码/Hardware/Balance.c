#include "Balance.h"
#include "step.h"    // STEP/DIR 脉冲控制；当前闭环不走电机串口运动模式
#include "usart.h"   // Ax (JY61P 加速度, 单位 g)
#include "K230.h"    // Ball_X (视觉球位置)
#include "PID.h"     // 复用你的PID结构体与 PID_Calculate

/* ===================== 机械 / 电机参数 =====================
 * 本文件的核心思路：
 *   摄像头给出小球位置 Ball_X(cm) 和速度 Ball_V(cm/s)
 *       -> 控制器算出摆杆目标角度 theta_cmd(rad)
 *       -> 目标角度换算成“还需要补多少个脉冲”
 *       -> StepPulse_Move() 直接输出 STEP/DIR 脉冲
 *
 * 注意：步进电机在这里不再使用“位置模式/速度模式/快速位置模式”。
 *      它只是一个脉冲执行器，STM32 自己用 StepPulse_Position 估计当前位置。
 */
#define STEP_PULSES_PER_REV 3200.0f     // 16细分: 3200脉冲/圈(拨码改细分需同步改)
#define ROD_GEAR_RATIO      1.0f        // 电机->摆杆传动比(皮带/齿轮减速, 直连=1)
#define PULSES_PER_RAD      (STEP_PULSES_PER_REV * ROD_GEAR_RATIO / (2.0f * 3.14159265f))

/* 方向修正：
 *   如果“球在右边，控制后反而更往右跑”，优先改这个符号。
 *   不要同时改 BALANCE_DIR_SIGN 和 STEP_DIR_POS_LEVEL，否则容易把自己绕晕。
 */
#define BALANCE_DIR_SIGN    (-1)

/* 调试期最大摆杆角度。
 *   太小：球回中心很慢，甚至克服不了静摩擦。
 *   太大：容易冲过中心、撞限位、堵转。
 */
#define THETA_MAX           (0.08f)     // rad, 约4.6度

/* 球速阻尼项。
 *   球速太大时，给一个反向倾角，避免小球冲过中心。
 *   如果来回振荡明显，可适当增大 KV 或 PID 的 Kd。
 */
#define KV                  (0.02f)
#define V_DEAD              (3.0f)      // 速度小于这个值时不启用速度前馈，避免视觉速度噪声影响
#define VFF_MAX             (0.08f)     // 速度前馈最大只能贡献这么多角度

/* 中心死区和卡滞补偿。
 *   小球在某些位置不动，通常是静摩擦、管壁局部坡度或视觉速度为0导致。
 *   下面这组参数用于“轻轻多推一点”，让它继续向0点靠拢。
 */
#define CENTER_DEADBAND_CM  (0.35f)     // 误差小于该值就认为够接近中心，不再额外追
#define STUCK_V_DEAD        (0.8f)      // 球速低于该值且还有误差，认为可能卡住
#define STUCK_START_TICKS   (15)        // 10ms一拍，25拍约250ms，持续这么久才开始补偿
#define STUCK_THETA_STEP    (0.0070f)   // 卡住后每10ms逐渐增加的偏置坡度
#define STUCK_THETA_DECAY   (0.0010f)   // 球开始动起来后，每10ms释放一点偏置坡度
#define STUCK_THETA_MAX     (0.055f)    // 卡滞补偿最大角度；太大会冲过中心
#define STUCK_PROGRESS_CM   (0.04f)     // 本拍误差至少减小这么多，才认为确实在靠近中心
#define STUCK_RELEASE_V     (1.5f)      // 球速超过该值，说明已经动起来，可以开始退偏置

/* 题目第3项自动流程参数。
 * 要求：小车静止，小球从 O 点出发 -> +5cm -> 折返 -> -5cm 并稳定，5s内完成。
 * 这里的 +5/-5 直接使用视觉坐标 Ball_X 的符号，请保证 K230 的零点是摆杆中心 O。
 */
typedef struct
{
    float target_pos_cm;       // 第一个目标点: +5cm
    float target_neg_cm;       // 第二个目标点: -5cm
    float reach_err_cm;        // 进入 +5cm 附近这个范围，就认为到达可折返
    float final_err_cm;        // 进入 -5cm 附近这个范围，用于最终稳定判定
    float final_v_dead;        // 最终稳定时球速要足够小，防止只是高速路过 -5cm
    uint16_t pos_hold_ticks;   // 快速折返: 进 +5cm 附近几拍后切到 -5cm
    uint16_t final_hold_ticks; // -5cm 附近连续多少拍，认为已稳定
    uint16_t timeout_ticks;    // 500*10ms=5s，超时后仍继续保持 -5cm
} BalanceTask3Profile;

typedef struct
{
    float fast_theta_max;       // 第3项快速模式角度上限；太冲就先降这个
    float pos_theta_min;        // 去+5cm时的最低起步坡度，用来克服正向静摩擦
    float neg_theta_min;        // 折返去-5cm时的最低坡度，保证折返速度
    float final_hold_theta_min; // 到-5cm后最低保持坡度，防止小球往0点回滚
    float kick_v_dead;          // 速度低于该值才补最低坡度；数值大=更激进
} BalanceTask3Comp;

static const BalanceTask3Profile Task3Profile = {
    5.0f,    // target_pos_cm
    -5.0f,   // target_neg_cm
    1.0f,    // reach_err_cm
    1.0f,    // final_err_cm
    3.2f,    // final_v_dead
    1,       // pos_hold_ticks
    20,      // final_hold_ticks
    500      // timeout_ticks
};

static const BalanceTask3Comp Task3Comp = {
    0.150f,  // fast_theta_max
    0.160f,  // pos_theta_min
    0.180f,  // neg_theta_min
    0.270f,  // final_hold_theta_min
    5.4f     // kick_v_dead
};

typedef struct
{
    float kff;             // 加速度前馈系数: theta_ff = sign * kff * (Ax - AxZero)
    float theta_max;       // 加速度前馈最多贡献多少坡度
    float dead_g;          // 动态加速度死区，过滤静止噪声
    float filter_alpha;    // 动态加速度低通系数，越大响应越快、噪声也越明显
    float zero_alpha;      // 暂停时零偏慢速更新系数
    int8_t sign;           // 前馈方向，若小车加速时球更容易跑偏，就先改这个符号
} BalanceAccelFFParam;

typedef struct
{
    float theta_max;       // 任意目标点锁定时的临时角度上限
    float deadband_cm;     // 小球离目标点超过该值，才认为需要强制守住
    float min_theta;       // 偏离目标点后，朝目标方向的最低坡度
    float release_v;       // 已经明显朝目标点运动时，不再强行加最低坡度
} BalanceTargetLockParam;

/* 普通/中心保持加速度前馈。
 * 这组保持你现场刚调出的手感，主要服务第4/5项中心锁球。
 */
static const BalanceAccelFFParam CenterAccelFF = {
    0.16f,   // kff: 小车加减速时的预补偿强度
    0.080f,  // theta_max
    0.03f,   // dead_g
    0.20f,   // filter_alpha
    0.02f,   // zero_alpha
    1        // sign: 若前馈方向反了，改成 -1
};

/* 第6项任意点锁定加速度前馈。
 * 目标点离中心越远，轨道局部坡度/静摩擦/惯性影响通常越明显，所以分档加大。
 */
static const BalanceAccelFFParam Task6AccelFFNear = {
    0.16f,   // |target| < 2cm: 与中心保持接近
    0.080f,
    0.03f,
    0.20f,
    0.02f,
    1
};

static const BalanceAccelFFParam Task6AccelFFMid = {
    0.20f,   // 2cm <= |target| < 5cm: 中等增强
    0.105f,
    0.03f,
    0.20f,
    0.02f,
    1
};

static const BalanceAccelFFParam Task6AccelFFFar = {
    0.24f,   // 5cm <= |target| < 9cm: 远离中心时更强前馈
    0.130f,
    0.03f,
    0.20f,
    0.02f,
    1
};

static const BalanceAccelFFParam Task6AccelFFEdge = {
    0.30f,   // |target| >= 9cm: 接近边缘，前馈更积极
    0.160f,
    0.03f,
    0.20f,
    0.02f,
    1
};

static const BalanceTargetLockParam CenterLock = {
    0.120f,  // theta_max
    0.20f,   // deadband_cm
    0.110f,  // min_theta
    2.0f     // release_v
};

/* 第6项任意指定位置锁定参数。
 * 近中心不要太猛，避免像0点锁定一样开始抖；
 * 离中心越远越加大最低坡度和角度上限，专门处理“任意点放置会歪”的现象。
 */
static const BalanceTargetLockParam Task6LockNear = {
    0.140f,  // |target| < 2cm
    0.18f,
    0.120f,
    2.2f
};

static const BalanceTargetLockParam Task6LockMid = {
    0.165f,  // 2cm <= |target| < 5cm
    0.16f,
    0.140f,
    2.5f
};

static const BalanceTargetLockParam Task6LockFar = {
    0.200f,  // 5cm <= |target| < 9cm
    0.15f,
    0.165f,
    3.0f
};

static const BalanceTargetLockParam Task6LockEdge = {
    0.230f,  // |target| >= 9cm；一边总长约12.5cm，边缘档要强但要防撞端点
    0.14f,
    0.190f,
    3.5f
};

#define TASK6_TARGET_MID_CM   2.0f
#define TASK6_TARGET_FAR_CM   5.0f
#define TASK6_TARGET_EDGE_CM  9.0f

/* 第6项：任意指定位置保持 + 小车跑一圈。
 * 按下 Key2 时记录当前 Ball_X 作为目标点；
 * 先原地锁球一小段时间，再进入持续锁定状态。
 * 循迹、车轮、电机和跑圈完成判定由另一个MCU主控负责。
 */
#define TASK6_LOCK_TICKS     50        // 50*10ms=0.5s，先原地接住球

float Ball_Target_X = 0;   // 题目设定目标: 稳中心=0; 移动靶=±5
float Ball_Target_Y = 0;

/* STM32 侧摆杆角度估计(rad)：
 *   纯脉冲控制没有电机位置回包，所以用 StepPulse_Position 做软件估计。
 *   如果电机实际丢步、皮带打滑、机械撞限位，这个估计值会不准。
 *   因此调试期必须保留 STEP_PULSE_ABS_LIMIT 软件限位。
 */
static float Step_Angle_Current = 0;
int32_t Step_Pulse_Current = 0;   // 步进电机当前累计脉冲(供OLED显示, 反映杆当前倾角状态)
volatile uint8_t Balance_Run = 0; // 步进电机控制使能: 1=运行, 0=停止/未启动, 默认停止
volatile uint8_t Balance_DebugActive = 0; // 步进自检模式: 1=执行固定点动序列
volatile uint8_t Balance_Task3State = 0;  // 第3项状态: 0空闲 1去+5 2去-5 3完成 4超时仍保持-5
volatile uint16_t Balance_Task3Time = 0;  // 第3项计时, 单位10ms
volatile uint8_t Balance_Task6State = 0;  // 第6项状态: 0空闲 1原地锁球 2行驶中持续锁球 3完成后保持
volatile uint16_t Balance_Task6Time = 0;  // 第6项计时, 单位10ms
float Balance_Task6Target = 0.0f;         // 第6项启动瞬间记录的任意指定位置(cm)
float Balance_ThetaCmd = 0.0f;    // 当前控制输出角度(rad), 供OLED/调参观察
float Balance_AxZero = 0.0f;       // JY61P X轴零偏；暂停时自动慢速更新
float Balance_AxDyn = 0.0f;        // 扣除零偏后的动态X轴加速度(g)，供OLED和前馈观察
float Balance_ThetaFF = 0.0f;      // 当前加速度前馈坡度(rad)
int32_t Balance_PulseCmd = 0;     // 当前控制输出增量脉冲, 供OLED/调参观察

static uint8_t  StepperDebug_State = 0;
static uint16_t StepperDebug_Tick = 0;
static uint16_t Ball_StuckTicks = 0;
static float    Ball_StuckBias = 0.0f;     // 当前额外偏置坡度，方向始终指向零点
static float    Ball_LastErrorAbs = 0.0f;  // 上一拍位置误差绝对值，用来判断有没有继续靠近中心
static uint8_t  Task3_HoldTicks = 0;        // 第3项到点保持计数，过滤视觉偶发跳点
static uint8_t  Accel_ZeroReady = 0;         // 0=还没记录过零偏，1=零偏可用

/* 球位置PID: Target=题目设定(默认0), actual=Ball_X(cm)
 * 输出 = 期望摆杆倾角 theta(rad)。增益单位：Kp[rad/cm], Ki[rad/(cm·步)], Kd[rad/cm每10ms步]
 *   - Kp 把"球偏差"变成"倾角"
 *   - Ki 暂时为0；不要急着开积分，积分很容易把杆慢慢顶到一边
 *   - Kd 对球速微分, 抑制来回过冲
 * output_limit 限到 THETA_MAX, 等于软件机械保护。
 *
 * 调参顺序建议：
 *   1. 方向正确后，只加 Kp，让球能往中心回。
 *   2. 如果冲过中心/来回振荡，增加 Kd 或 KV。
 *   3. 如果某些位置停住不动，优先调 STUCK_*，最后再考虑 Ki。
 */
static PID_Init BallPID = {0.02f, 0.0f, 0.5f, 0, 0, 0, 0, 0, 0, 20.0f, THETA_MAX};

static float Balance_AbsFloat(float x)
{
    return (x < 0.0f) ? -x : x;
}

static const BalanceTargetLockParam *Balance_GetActiveLockParam(void)
{
    float target_abs;

    /* 第6项按“指定位置离中心多远”分档。
     * 这不是把零点改掉，而是承认远离中心的位置更容易受轨道局部坡度和静摩擦影响。
     */
    if (Balance_Task6State != 0)
    {
        target_abs = Balance_AbsFloat(Balance_Task6Target);
        if (target_abs >= TASK6_TARGET_EDGE_CM) return &Task6LockEdge;
        if (target_abs >= TASK6_TARGET_FAR_CM) return &Task6LockFar;
        if (target_abs >= TASK6_TARGET_MID_CM) return &Task6LockMid;
        return &Task6LockNear;
    }

    return &CenterLock;
}

static const BalanceAccelFFParam *Balance_GetActiveAccelFFParam(void)
{
    float target_abs;

    /* 加速度前馈也随第6项指定位置分档：
     * 离中心越远，车体加减速时更容易把球甩离目标点，所以前馈给得更积极。
     */
    if (Balance_Task6State != 0)
    {
        target_abs = Balance_AbsFloat(Balance_Task6Target);
        if (target_abs >= TASK6_TARGET_EDGE_CM) return &Task6AccelFFEdge;
        if (target_abs >= TASK6_TARGET_FAR_CM) return &Task6AccelFFFar;
        if (target_abs >= TASK6_TARGET_MID_CM) return &Task6AccelFFMid;
        return &Task6AccelFFNear;
    }

    return &CenterAccelFF;
}

static void Balance_UpdateAccelFF(uint8_t allow_zero_update)
{
    const BalanceAccelFFParam *ff = Balance_GetActiveAccelFFParam();
    float ax_raw = Ax;
    float ax_dyn;

    /* Ax 里通常含有安装姿态/重力分量。
     * 你现在观察到静止大概是 1 左右，所以必须先扣零偏，不能直接前馈。
     *
     * 暂停时认为小车基本静止，慢速学习零偏；
     * 启动闭环后冻结零偏，只更新动态加速度低通。
     */
    if (!Accel_ZeroReady)
    {
        Balance_AxZero = ax_raw;
        Balance_AxDyn = 0.0f;
        Balance_ThetaFF = 0.0f;
        Accel_ZeroReady = 1;
    }
    else if (allow_zero_update)
    {
        Balance_AxZero += ff->zero_alpha * (ax_raw - Balance_AxZero);
    }

    ax_dyn = ax_raw - Balance_AxZero;
    Balance_AxDyn += ff->filter_alpha * (ax_dyn - Balance_AxDyn);

    if (Balance_AbsFloat(Balance_AxDyn) < ff->dead_g)
    {
        Balance_ThetaFF = 0.0f;
    }
    else
    {
        Balance_ThetaFF = (float)ff->sign * ff->kff * Balance_AxDyn;
        if (Balance_ThetaFF >  ff->theta_max) Balance_ThetaFF =  ff->theta_max;
        if (Balance_ThetaFF < -ff->theta_max) Balance_ThetaFF = -ff->theta_max;
    }
}

static void Balance_ClearStuckComp(void)
{
    /* 换目标或停止时，必须清掉上一段留下的偏置坡度。
     * 否则从 +5 切到 -5 时，上一方向的“脱困推力”可能会多顶几拍。
     */
    Ball_StuckTicks = 0;
    Ball_StuckBias = 0.0f;
    Ball_LastErrorAbs = Balance_AbsFloat(Ball_Target_X - Ball_X);
}

static void Balance_SetTargetX(float target)
{
    /* 统一设置摆球目标，并重置 PID 的历史误差。
     * PID 的 D 项使用 error-last_error；目标从 +5 跳到 -5 时，
     * 如果不重置 last_error，会产生一个很大的瞬时微分输出。
     */
    Ball_Target_X = target;
    BallPID.Target = target;
    BallPID.error = target - Ball_X;
    BallPID.last_error = BallPID.error;
    BallPID.derivative = 0.0f;
    BallPID.integral = 0.0f;
    Balance_ClearStuckComp();
}

void Balance_Task3Start(void)
{
    /* 第3项测试入口：
     *   按下 Key3 后，小车保持静止，摆杆目标先给 +5cm。
     *   请先把小球放在中心 O 附近，再按 Key3；程序不会把开机位置当零点。
     */
    Balance_DebugActive = 0;
    Balance_Task6State = 0;
    Balance_Task6Time = 0;
    Balance_Run = 1;
    StepPulse_Enable(1);
    Balance_Task3State = 1;
    Balance_Task3Time = 0;
    Task3_HoldTicks = 0;
    Balance_SetTargetX(Task3Profile.target_pos_cm);
}

void Balance_Task3Stop(void)
{
    /* 退出第3项：恢复普通中心保持目标。
     * 按 Key2 暂停时会调用这里，方便下一次重新从 O -> +5 -> -5 开始。
     */
    Balance_Task3State = 0;
    Balance_Task3Time = 0;
    Balance_Task6State = 0;
    Balance_Task6Time = 0;
    Balance_Task6Target = 0.0f;
    Task3_HoldTicks = 0;
    Balance_SetTargetX(0.0f);
}

void Balance_Task6Start(void)
{
    /* 第6项测试入口：
     *   题目要求“钢球置于摆杆任意指定位置”。
     *   所以按下 Key2 的瞬间，不把目标设为0，而是记录当前视觉位置 Ball_X。
     *
     * 状态1先原地锁球0.5s；
     * 状态2表示本MCU已进入持续锁球状态，另一个MCU可以负责发车/循迹；
     * 状态3由外部流程需要时调用，表示跑完后继续保持该指定位置。
     */
    Balance_DebugActive = 0;
    Balance_Task3State = 0;
    Balance_Task3Time = 0;
    Task3_HoldTicks = 0;

    Balance_Task6Target = Ball_X;
    Balance_Task6State = 1;
    Balance_Task6Time = 0;

    Balance_Run = 1;
    StepPulse_Enable(1);
    Balance_SetTargetX(Balance_Task6Target);
}

void Balance_Task6Stop(void)
{
    /* 退出第6项：恢复普通中心保持目标。
     * 比赛现场如果每题后都手动复位，这里主要用于切换到其他按键模式时清状态。
     */
    Balance_Task6State = 0;
    Balance_Task6Time = 0;
    Balance_Task6Target = 0.0f;
    Balance_SetTargetX(0.0f);
}

void Balance_Task6Finish(void)
{
    /* 第6项跑完一圈回到A点后可调用。
     * 当前MCU不负责循迹和停车线判断；如果后续两个MCU之间加了完成信号，
     * 可以在收到信号时调用这个函数。
     * 不关闭 Balance_Run，因为题目仍要求球稳定在指定位置附近。
     */
    if (Balance_Task6State != 0)
    {
        Balance_Task6State = 3;
        Balance_Task6Time = 0;
        Balance_SetTargetX(Balance_Task6Target);
    }
}

static void Balance_Task6Tick(void)
{
    if (Balance_Task6State == 0) return;

    if (Balance_Task6Time < 60000) Balance_Task6Time++;

    switch (Balance_Task6State)
    {
        case 1:
            /* 原地锁球阶段：给摆杆闭环一点时间接住启动位置。
             * 到时间后进入状态2，表示可以让另一个MCU启动小车。
             */
            Ball_Target_X = Balance_Task6Target;
            if (Balance_Task6Time >= TASK6_LOCK_TICKS)
            {
                Balance_Task6State = 2;
                Balance_Task6Time = 0;
            }
            break;

        case 2:
        case 3:
            /* 行驶中/完成后都继续锁在启动时记录的位置。 */
            Ball_Target_X = Balance_Task6Target;
            break;

        default:
            Balance_Task6Stop();
            break;
    }
}

static void Balance_Task3Tick(void)
{
    float err;
    float vabs;

    if (Balance_Task3State == 0) return;

    if (Balance_Task3Time < 60000) Balance_Task3Time++;

    switch (Balance_Task3State)
    {
        case 1:
            /* 阶段1：从中心 O 跑到 +5cm。
             * 这里不要求速度很小，因为题目说“到达后折返”，不是必须在 +5cm 停稳。
             * 一旦连续几拍进入 +5cm 附近，就马上把目标切到 -5cm，节省总时间。
             */
            Ball_Target_X = Task3Profile.target_pos_cm;
            err = Balance_AbsFloat(Ball_X - Task3Profile.target_pos_cm);
            if (err <= Task3Profile.reach_err_cm)
            {
                if (Task3_HoldTicks < 200) Task3_HoldTicks++;
                if (Task3_HoldTicks >= Task3Profile.pos_hold_ticks)
                {
                    Balance_Task3State = 2;
                    Task3_HoldTicks = 0;
                    Balance_SetTargetX(Task3Profile.target_neg_cm);
                }
            }
            else
            {
                Task3_HoldTicks = 0;
            }
            break;

        case 2:
            /* 阶段2：折返到 -5cm，并在该点附近稳定。
             * 最终点要同时看位置和速度，避免小球只是高速经过 -5cm 就被误判完成。
             */
            Ball_Target_X = Task3Profile.target_neg_cm;
            err = Balance_AbsFloat(Ball_X - Task3Profile.target_neg_cm);
            vabs = Balance_AbsFloat(Ball_V);
            if (err <= Task3Profile.final_err_cm && vabs <= Task3Profile.final_v_dead)
            {
                if (Task3_HoldTicks < 200) Task3_HoldTicks++;
                if (Task3_HoldTicks >= Task3Profile.final_hold_ticks)
                {
                    Balance_Task3State = 3;       // 完成，但继续保持 -5cm
                    Task3_HoldTicks = 0;
                }
            }
            else
            {
                Task3_HoldTicks = 0;
            }
            break;

        case 3:
            /* 已完成：不要关 Balance_Run，继续闭环保持 -5cm，方便评委读误差。 */
            Ball_Target_X = Task3Profile.target_neg_cm;
            break;

        case 4:
            /* 超时：仍保持 -5cm，便于继续观察和调参。 */
            Ball_Target_X = Task3Profile.target_neg_cm;
            break;

        default:
            Balance_Task3State = 0;
            Task3_HoldTicks = 0;
            break;
    }

    if (Balance_Task3Time >= Task3Profile.timeout_ticks &&
        Balance_Task3State != 3 &&
        Balance_Task3State != 4)
    {
        Balance_Task3State = 4;
        Task3_HoldTicks = 0;
        Balance_SetTargetX(Task3Profile.target_neg_cm);
    }
}

void Balance_Init(void)
{
    /* 初始化 STEP/DIR 输出脚。
     * 这里只清软件零点，不主动发任何运动脉冲。
     * 上电后必须等待 Key1 普通闭环或 Key3 第3项模式启动，电机才应该动作。
     */
    StepPulse_Init();                 // STEP/DIR 脉冲控制, 不走电机串口模式
    StepPulse_Enable(1);
    StepPulse_Zero();                 // 以当前机械位置为零点
    Step_Angle_Current = 0;
    Ball_StuckTicks = 0;
    Ball_StuckBias = 0.0f;
    Ball_LastErrorAbs = 0.0f;
    Balance_Task3State = 0;
    Balance_Task3Time = 0;
    Task3_HoldTicks = 0;
    Balance_AxZero = Ax;
    Balance_AxDyn = 0.0f;
    Balance_ThetaFF = 0.0f;
    Accel_ZeroReady = 0;
    Balance_SetTargetX(0.0f);
}

void Balance_StepperDebugStart(void)
{
    /* 步进自检入口。
     * 现在 Key3 已改为题目第3项模式，这个自检函数只作为备用调试接口保留。
     * 自检期间强制关闭闭环，避免一边测试脉冲方向，一边视觉闭环也在发脉冲。
     */
    Balance_Run = 0;                  // 自检期间关掉闭环, 避免两套指令抢电机
    Balance_DebugActive = 1;
    StepperDebug_State = 0;
    StepperDebug_Tick = 0;
    Balance_ThetaCmd = 0.0f;
    Balance_PulseCmd = 0;
}

/* 10ms 调用一次。固定序列:
 * enable + zero -> +50 pulse -> -50 pulse -> stop.
 * 若这套序列无反应, 优先查驱动供电、PUL/DIR接线、共地、脉冲电平和细分设置。
 *
 * 正常现象：
 *   摆杆轻轻动一下，停约1秒，再反方向回去。
 *   OLED 上 P 应该大致显示 0 -> 50 -> 0。
 */
void Balance_StepperDebugTick(void)
{
    if (!Balance_DebugActive) return;

    Step_Pulse_Current = StepPulse_Position;
    StepperDebug_Tick++;

    switch (StepperDebug_State)
    {
        case 0:
            StepPulse_Enable(1);
            StepPulse_Zero();
            StepperDebug_State = 1;
            StepperDebug_Tick = 0;
            break;

        case 1:
            if (StepperDebug_Tick >= 20)       // 200ms 后正向点动
            {
                Balance_PulseCmd = 50;
                StepPulse_Move(Balance_PulseCmd);
                Step_Pulse_Current = StepPulse_Position;
                StepperDebug_State = 2;
                StepperDebug_Tick = 0;
            }
            break;

        case 2:
            if (StepperDebug_Tick >= 100)      // 等 1s 再反向回去
            {
                Balance_PulseCmd = -50;
                StepPulse_Move(Balance_PulseCmd);
                Step_Pulse_Current = StepPulse_Position;
                StepperDebug_State = 3;
                StepperDebug_Tick = 0;
            }
            break;

        case 3:
            if (StepperDebug_Tick >= 100)
            {
                StepPulse_Stop();
                Balance_PulseCmd = 0;
                Balance_DebugActive = 0;
                StepperDebug_State = 0;
                StepperDebug_Tick = 0;
            }
            break;

        default:
            Balance_DebugActive = 0;
            StepperDebug_State = 0;
            StepperDebug_Tick = 0;
            break;
    }
}

/* 10ms 调用一次 */
void Balance_Control(void)
{
    float theta_cmd, dtheta, ball_error;
    float theta_pos_limit = THETA_MAX;
    float theta_neg_limit = THETA_MAX;
    int32_t pulses;

    /* 每拍先用软件脉冲位置反算当前杆角。
     * BALANCE_DIR_SIGN 参与反算，保证“软件角度”和“实际控制方向”一致。
     */
    Step_Pulse_Current = StepPulse_Position;
    Step_Angle_Current = (float)StepPulse_Position / (BALANCE_DIR_SIGN * PULSES_PER_RAD);
    Balance_UpdateAccelFF((Balance_Run == 0) ? 1 : 0);

    if (!Balance_Run || !Ball_Updated)
    {
        /* 安全门：
         *   1. 没按 Key1 时不闭环。
         *   2. K230 没有有效视觉帧时不闭环。
         * 这样可以避免开机、串口断线、识别丢球时杆自己跑到限位。
         */
        Balance_PulseCmd = 0;
        Balance_ThetaCmd = 0.0f;
        BallPID.integral = 0.0f;
        BallPID.last_error = 0.0f;
        if (!Balance_Run && Balance_Task3State != 0)
        {
            Balance_Task3Stop();
        }
        if (!Balance_Run && Balance_Task6State != 0)
        {
            Balance_Task6Stop();
        }
        Ball_StuckTicks = 0;
        Ball_StuckBias = 0.0f;
        Ball_LastErrorAbs = 0.0f;
        return;   // 未启动或无视觉有效帧: 绝不发闭环脉冲
    }

    /* 如果 Key3 启动了第3项测试，先更新本拍目标：
     *   状态1: 目标 +5cm
     *   状态2/3/4: 目标 -5cm
     * 后面的 PID 和卡滞补偿不需要知道题目流程，只负责跟踪 Ball_Target_X。
     */
    Balance_Task3Tick();
    Balance_Task6Tick();

    /* 1) 球位置PID：
     *   ball_error = 目标位置 - 当前球位置。
     *   例：目标0cm，球在右边 +5cm，则 error = -5cm。
     *   PID 输出不是“脉冲”，而是“希望杆倾斜到多少角度”。
     */
    BallPID.Target = Ball_Target_X;
    ball_error = Ball_Target_X - Ball_X;
    theta_cmd = PID_Calculate(&BallPID, Ball_X);

    /* 静摩擦/浅坑卡滞补偿：带方向的“偏置坡度”
     *
     * 为什么要单独做这一段：
     *   普通 PID 在误差比较小时，输出角度也会很小。
     *   如果小球刚好被静摩擦、轨道局部浅坑、胶带边缘卡住，
     *   这个小角度可能推不动球，于是它会停在非零点。
     *
     * 这里的做法不是左右对称抖动，而是：
     *   1. 判断小球还没进中心死区；
     *   2. 判断小球速度很小；
     *   3. 再判断误差没有继续变小；
     *   4. 满足一段时间后，朝“零点方向”慢慢加一个偏置坡度。
     *
     * 球一旦开始明显运动，偏置坡度就慢慢释放，让 PD 接管。
     * 这样可以避免“越卡越顶”，也避免刚动起来就冲过中心。
     */
    {
        float eabs = (ball_error < 0) ? -ball_error : ball_error;
        float vabs = (Ball_V < 0) ? -Ball_V : Ball_V;
        float stuck_dir = (ball_error >= 0.0f) ? 1.0f : -1.0f;

        if (eabs <= CENTER_DEADBAND_CM)
        {
            /* 已经足够接近中心：清掉脱困补偿。
             * 中心不能按 X==0 判定，真实视觉会有噪声，所以用死区。
             */
            Ball_StuckTicks = 0;
            Ball_StuckBias = 0.0f;
        }
        else if (vabs < STUCK_V_DEAD &&
                 eabs > (Ball_LastErrorAbs - STUCK_PROGRESS_CM))
        {
            /* 误差没明显减小，同时速度又很小：大概率卡住了。
             * 连续卡住 STUCK_START_TICKS 拍之后，才开始逐步加偏置。
             */
            if (Ball_StuckTicks < 200) Ball_StuckTicks++;
            if (Ball_StuckTicks > STUCK_START_TICKS)
            {
                Ball_StuckBias += stuck_dir * STUCK_THETA_STEP;
                if (Ball_StuckBias >  STUCK_THETA_MAX) Ball_StuckBias =  STUCK_THETA_MAX;
                if (Ball_StuckBias < -STUCK_THETA_MAX) Ball_StuckBias = -STUCK_THETA_MAX;
            }
        }
        else
        {
            /* 球已经在动，或者误差正在变小：
             * 不再继续加偏置，而是缓慢退掉。
             * 这里不用一下清零，是为了让小球刚突破静摩擦时还有一点持续推力。
             */
            Ball_StuckTicks = 0;
            if (vabs > STUCK_RELEASE_V || eabs < Ball_LastErrorAbs)
            {
                if (Ball_StuckBias > STUCK_THETA_DECAY) Ball_StuckBias -= STUCK_THETA_DECAY;
                else if (Ball_StuckBias < -STUCK_THETA_DECAY) Ball_StuckBias += STUCK_THETA_DECAY;
                else Ball_StuckBias = 0.0f;
            }
        }

        /* 如果小球越过零点，ball_error 符号会变。
         * 偏置坡度必须立刻跟着目标方向走，不能保留上一侧的推力。
         */
        if ((Ball_StuckBias > 0.0f && stuck_dir < 0.0f) ||
            (Ball_StuckBias < 0.0f && stuck_dir > 0.0f))
        {
            Ball_StuckBias = 0.0f;
        }

        theta_cmd += Ball_StuckBias;
        Ball_LastErrorAbs = eabs;
    }

    /* 2) 小球速度前馈(阻尼)：速度越大越降低倾斜度，抑制惯性过冲
     *    V 右正左负; 球向中心快冲时(V与误差同向)此项抵消位置项 -> 减小倾斜
     *    球冲过头时(V与误差反向)此项反向加力 -> 拉回。即纯阻尼项 -KV*V
     *    PID 数值微分(未除dt)已含阻尼, 故 Kd 已调小, 此处用 K230 干净速度主导 */
    {
        float vabs = (Ball_V < 0) ? -Ball_V : Ball_V;
        if (vabs > V_DEAD)
        {
            float vff = -KV * Ball_V;
            if (vff >  VFF_MAX) vff =  VFF_MAX;
            if (vff < -VFF_MAX) vff = -VFF_MAX;
            theta_cmd += vff;
        }
    }

    /* 3) 加速度前馈(开环预补偿)：
     *   使用 Ax - AxZero，而不是直接使用 Ax。
     *   Ax 正方向目前按你们现场观察：小车速度方向为正。
     *   如果加速时球反而更容易被甩出去，先把当前模式的 sign 改成 -1。
     */
    theta_cmd += Balance_ThetaFF;

    /* 第3项去 +5cm 的定向起步补偿。
     *
     * 现场现象：去 -5cm 很快，去 +5cm 容易被静摩擦卡住。
     * 这说明两个方向的机械阻力/实际水平零位不对称。
     *
     * 处理方式：
     *   第3项模式临时放大角度上限；
     *   Q3:1（O -> +5cm）保证一个正向最低坡度；
     *   Q3:2（+5cm -> -5cm）保证一个负向最低坡度。
     *
     * 注意：
     *   这不是改视觉零点；
     *   它是在第3项冲时间时，允许摆杆更快到达目标坡度。
     */
    if (Balance_Task3State != 0)
    {
        theta_pos_limit = Task3Comp.fast_theta_max;
        theta_neg_limit = Task3Comp.fast_theta_max;
    }
    else
    {
        /* 任意目标点锁定模式。
         * 用在第4/5/6项：
         *   第4/5项目标通常是中心 O；
         *   第6项目标是 Key2 启动瞬间记录的 Ball_X。
         *
         * 普通 PID 在球刚被晃出目标点时，输出可能还不够克服静摩擦和车体加速度扰动。
         * 这里参考第3项的“最低坡度”思路：只要球离开目标点死区，就给一个朝目标点的最低坡度。
         *
         * 如果球已经明显朝目标点滚动，就不再强行加最低坡度，避免把它推过目标点。
         */
        float eabs = Balance_AbsFloat(ball_error);
        float vabs = Balance_AbsFloat(Ball_V);
        const BalanceTargetLockParam *lock = Balance_GetActiveLockParam();
        uint8_t moving_to_target = (ball_error * Ball_V > 0.0f && vabs > lock->release_v);

        theta_pos_limit = lock->theta_max;
        theta_neg_limit = lock->theta_max;

        if (eabs > lock->deadband_cm && !moving_to_target)
        {
            float lock_dir = (ball_error >= 0.0f) ? 1.0f : -1.0f;
            if (lock_dir > 0.0f && theta_cmd < lock->min_theta)
            {
                theta_cmd = lock->min_theta;
            }
            else if (lock_dir < 0.0f && theta_cmd > -lock->min_theta)
            {
                theta_cmd = -lock->min_theta;
            }
        }
    }

    if (Balance_Task3State == 1)
    {
        if (Ball_X < (Task3Profile.target_pos_cm - Task3Profile.reach_err_cm) &&
            Balance_AbsFloat(Ball_V) < Task3Comp.kick_v_dead &&
            theta_cmd < Task3Comp.pos_theta_min)
        {
            theta_cmd = Task3Comp.pos_theta_min;
        }
    }
    else if (Balance_Task3State == 2)
    {
        if (Ball_X > (Task3Profile.target_neg_cm + Task3Profile.final_err_cm) &&
            Balance_AbsFloat(Ball_V) < Task3Comp.kick_v_dead &&
            theta_cmd > -Task3Comp.neg_theta_min)
        {
            theta_cmd = -Task3Comp.neg_theta_min;
        }
    }
    else if (Balance_Task3State == 3 || Balance_Task3State == 4)
    {
        /* 到 -5cm 后的保持补偿。
         * 现场现象：小球能到 -5cm，但停住后会往 0 点方向回滚。
         * 原因是完成状态下只剩普通 PID，小误差时坡度可能小于静摩擦/机械偏置需求。
         *
         * 只要球在 -5cm 的右侧（也就是更靠近 0 点），就至少给一点负向保持坡度。
         * 如果球已经跑到 -5cm 左侧，则不继续负向压，避免越推越远。
         */
        if (Ball_X > Task3Profile.target_neg_cm &&
            theta_cmd > -Task3Comp.final_hold_theta_min)
        {
            theta_cmd = -Task3Comp.final_hold_theta_min;
        }
    }

    /* 4) 合成指令角并限幅(机械保护)。
     * 普通模式按 THETA_MAX 限幅；第3项按 Task3Comp.fast_theta_max 临时放大限幅。
     */
    if (theta_cmd >  theta_pos_limit) theta_cmd =  theta_pos_limit;
    if (theta_cmd < -theta_neg_limit) theta_cmd = -theta_neg_limit;
    Balance_ThetaCmd = theta_cmd;

    /* 5) 角度 -> 增量脉冲：
     *   dtheta 表示“目标角度”和“当前软件估计角度”的差。
     *   pulses 表示这 10ms 内需要补的脉冲。
     *
     * STEP_PULSE_PER_TICK 是速度保护：
     *   太大：响应快但容易抖、堵转、冲过中心。
     *   太小：动作柔和但回中心慢。
     */
    dtheta = theta_cmd - Step_Angle_Current;
    pulses = (int32_t)(BALANCE_DIR_SIGN * dtheta * PULSES_PER_RAD);
    if (pulses >  STEP_PULSE_PER_TICK) pulses =  STEP_PULSE_PER_TICK;
    if (pulses < -STEP_PULSE_PER_TICK) pulses = -STEP_PULSE_PER_TICK;
    Balance_PulseCmd = pulses;

    if (pulses != 0)
    {
        StepPulse_Move(pulses);
    }
}
