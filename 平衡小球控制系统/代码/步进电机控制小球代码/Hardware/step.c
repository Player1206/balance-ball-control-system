#include "step.h"
#include "usart.h"   // 提供 Serial_SendByte() 的声明(底层走 USART2)

/* ===================== STEP/DIR 脉冲控制 =====================
 * 这一段是“普通步进驱动器”的控制方式：
 *   DIR 电平决定方向；
 *   PUL/STEP 每出现一个上升沿，驱动器走一个细分步。
 *
 * 与串口模式不同，这里没有电机回包、没有目标位置查询。
 * 所以我们用 StepPulse_Position 做软件计数，并用 STEP_PULSE_ABS_LIMIT 防止越界。
 */
int32_t StepPulse_Position = 0;   // 软件估计位置(脉冲), 当前位置清零后为0
volatile uint8_t StepPulse_Ready = 0;

static void StepPulse_Delay(void)
{
    /* 简单的软件延时，用来控制 PUL 高/低电平宽度。
     * 这个延时越短，脉冲频率越高；太快时驱动器可能识别不到或电机容易丢步。
     * 如果示波器看到脉冲太窄，或电机偶尔不响应，可以适当增大循环次数。
     */
    volatile uint16_t i;
    for (i = 0; i < 120; i++)
    {
        __NOP();
    }
}

void StepPulse_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    StepPulse_Ready = 0;

    /* 初始化 PUL 和 DIR 为普通推挽输出。
     * 注意：main.c 里已经不再调用 uart5_init()，否则 PC12/PD2 会被配置成串口功能。
     */
    RCC_APB2PeriphClockCmd(STEP_PULSE_RCC | STEP_DIR_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = STEP_PULSE_PIN;
    GPIO_Init(STEP_PULSE_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = STEP_DIR_PIN;
    GPIO_Init(STEP_DIR_GPIO, &GPIO_InitStructure);

    GPIO_ResetBits(STEP_PULSE_GPIO, STEP_PULSE_PIN); // PUL 默认拉低，避免上电毛刺形成误脉冲
    StepPulse_SetDir(1);                             // 给 DIR 一个确定默认状态，避免方向脚悬空
    StepPulse_Position = 0;
    StepPulse_Ready = 1;
}

void StepPulse_Enable(uint8_t enable)
{
    /* 目前没有单独接 ENA 使能脚，所以这里先留空。
     * 如果后面接了 ENA：
     *   1. 在 step.h 里加 ENA_GPIO/ENA_PIN。
     *   2. 在这里根据 enable 拉高/拉低。
     *   3. 注意不同驱动器 ENA 有的是低有效，有的是高有效。
     */
    (void)enable;  // 当前未配置 ENA 脚; 如接了使能脚, 在这里补 GPIO 控制
}

void StepPulse_SetDir(int8_t dir)
{
    /* dir >= 0 表示软件定义的正方向，dir < 0 表示反方向。
     * STEP_DIR_POS_LEVEL 用来把“软件正方向”映射到实际 DIR 电平。
     */
    uint8_t positive = (dir >= 0) ? 1 : 0;
    if (positive == STEP_DIR_POS_LEVEL)
    {
        GPIO_SetBits(STEP_DIR_GPIO, STEP_DIR_PIN);
    }
    else
    {
        GPIO_ResetBits(STEP_DIR_GPIO, STEP_DIR_PIN);
    }
    StepPulse_Delay();  // 方向建立时间
}

void StepPulse_Move(int32_t pulses)
{
    uint32_t n, i;
    int8_t dir;
    int32_t next_pos;

    /* 保护1：没有运动需求，直接返回。 */
    if (pulses == 0) return;

    /* 保护2：初始化完成前禁止发脉冲，避免上电阶段误动作。 */
    if (!StepPulse_Ready) return;

    /* 保护3：软件位置限位。
     * 如果这次运动会超过允许范围，就把 pulses 裁剪到边界。
     * 这样即使上层控制器算错，也不会一直把杆推到机械限位。
     */
    next_pos = StepPulse_Position + pulses;
    if (next_pos > STEP_PULSE_ABS_LIMIT)
    {
        pulses = STEP_PULSE_ABS_LIMIT - StepPulse_Position;
    }
    else if (next_pos < -STEP_PULSE_ABS_LIMIT)
    {
        pulses = -STEP_PULSE_ABS_LIMIT - StepPulse_Position;
    }
    if (pulses == 0) return;

    dir = (pulses > 0) ? 1 : -1;
    n = (pulses > 0) ? (uint32_t)pulses : (uint32_t)(-pulses);
    StepPulse_SetDir(dir);

    /* 逐个输出 STEP 脉冲。
     * 驱动器通常识别 PUL 上升沿，所以下面是：
     *   高电平保持一小段时间 -> 低电平保持一小段时间。
     */
    for (i = 0; i < n; i++)
    {
        GPIO_SetBits(STEP_PULSE_GPIO, STEP_PULSE_PIN);
        StepPulse_Delay();
        GPIO_ResetBits(STEP_PULSE_GPIO, STEP_PULSE_PIN);
        StepPulse_Delay();
    }

    StepPulse_Position += pulses;
}

void StepPulse_Stop(void)
{
    /* 纯 STEP/DIR 下，“停止”就是停止继续发脉冲。
     * 这里拉低 PUL，保证输出脚停在稳定低电平。
     */
    GPIO_ResetBits(STEP_PULSE_GPIO, STEP_PULSE_PIN);
}

void StepPulse_Zero(void)
{
    /* 只清软件位置，不会让电机自动回到机械零点。
     * 使用场景：
     *   上电时手动把摆杆放到水平/中位，再调用 Zero，把当前位置当作0。
     */
    StepPulse_Position = 0;
}

/* ==========================================================================
 *  底层帧发送
 *  帧格式：地址 + 功能码 + data[len] + 固定校验 0x6B
 *  说明：默认校验字节固定为 0x6B（依据官方说明书，非累加和）。
 *        若电机菜单 Checksum 改为 XOR / CRC-8，需在此计算并替换末尾字节。
 * ========================================================================== */
void Step_SendFrame(uint8_t addr, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    Serial_SendByte(addr);
    Serial_SendByte(cmd);

    for (i = 0; i < len; i++)
    {
        Serial_SendByte(data[i]);
    }

    Serial_SendByte(STEP_CHECKSUM);
}

void Step_Init(void)
{
    /* 当前无需硬件初始化；保留接口以备扩展（如 RS485 方向引脚控制） */
}

/* ==========================================================================
 *  1. 运动控制命令
 * ========================================================================== */

/**
 *  @brief  电机使能控制 (功能码 0xF3)
 *  @帧    addr + F3 + AB + 状态(1) + 同步(1) + 6B
 *  @示例  Step_Enable(1, MOTOR_ENABLE, SYNC_OFF) -> 01 F3 AB 01 00 6B
 *  @note  使能与速度模式为两条独立指令，无需先使能即可进入速度模式。
 */
void Step_Enable(uint8_t addr, Motor_Enable_t en, Motor_Sync_t sync)
{
    uint8_t data[3];

    data[0] = 0xAB;                 /* 固定辅助码 */
    data[1] = (uint8_t)en;          /* 使能状态 */
    data[2] = (uint8_t)sync;        /* 多机同步标志 */

    Step_SendFrame(addr, 0xF3, data, 3);
}

/**
 *  @brief  速度模式 (功能码 0xF6)
 *  @帧    addr + F6 + 方向(1) + 速度(2,RPM) + 加速度(1) + 同步(1) + 6B
 *  @示例  Step_RunSpeed(1, CW, 1000, 0, OFF) -> 01 F6 00 03 E8 00 00 6B
 *  @param  rpm    目标转速，0~5000 RPM
 *  @param  accel  加速度 0~255，0 = 直接启动
 */
void Step_RunSpeed(uint8_t addr, Motor_Dir_t dir, uint16_t rpm, uint8_t accel, Motor_Sync_t sync)
{
    uint8_t data[5];

    data[0] = (uint8_t)dir;                 /* 方向 */
    data[1] = (uint8_t)(rpm >> 8);          /* 速度高字节 */
    data[2] = (uint8_t)(rpm & 0xFF);        /* 速度低字节 */
    data[3] = accel;                        /* 加速度 */
    data[4] = (uint8_t)sync;                /* 同步标志 */

    Step_SendFrame(addr, 0xF6, data, 5);
}

/**
 *  @brief  位置模式 (功能码 0xFD)
 *  @帧    addr + FD + 方向(1) + 速度(2) + 加速度(1) + 脉冲数(4) + 运动标志(1) + 同步(1) + 6B
 *  @示例  转 1 圈(16细分=3200脉冲, CW) ->
 *         01 FD 00 03 E8 00 00 00 0C 80 00 00 6B
 *  @param  pulses 目标脉冲数（0 ~ 2^32-1），默认 16 细分下 3200 脉冲 = 1 圈
 *  @param  mode   见 Motor_PosMode_t
 */
void Step_RunPosition(uint8_t addr, Motor_Dir_t dir, uint16_t rpm, uint8_t accel,
                      uint32_t pulses, Motor_PosMode_t mode, Motor_Sync_t sync)
{
    uint8_t data[10];

    data[0] = (uint8_t)dir;                 /* 方向 */
    data[1] = (uint8_t)(rpm >> 8);          /* 速度高字节 */
    data[2] = (uint8_t)(rpm & 0xFF);        /* 速度低字节 */
    data[3] = accel;                        /* 加速度 */
    data[4] = (uint8_t)(pulses >> 24);      /* 脉冲数 字节3（最高） */
    data[5] = (uint8_t)(pulses >> 16);      /* 脉冲数 字节2 */
    data[6] = (uint8_t)(pulses >> 8);       /* 脉冲数 字节1 */
    data[7] = (uint8_t)(pulses & 0xFF);     /* 脉冲数 字节0（最低） */
    data[8] = (uint8_t)mode;                /* 运动模式标志 */
    data[9] = (uint8_t)sync;                /* 同步标志 */

    Step_SendFrame(addr, 0xFD, data, 10);
}

/**
 *  @brief  设置快速位置模式运动参数 (功能码 0xF1)
 *  @帧    addr + F1 + 速度(2) + 加速度(1) + 运动标志(1) + 同步(1) + 6B
 *  @note  先设置参数，再用 Step_QPosRun() 下发脉冲即可连续触发，省去重复传参。
 */
void Step_QPosSetParams(uint8_t addr, uint16_t rpm, uint8_t accel, Motor_PosMode_t mode, Motor_Sync_t sync)
{
    uint8_t data[5];

    data[0] = (uint8_t)(rpm >> 8);          /* 速度高字节 */
    data[1] = (uint8_t)(rpm & 0xFF);        /* 速度低字节 */
    data[2] = accel;                        /* 加速度 */
    data[3] = (uint8_t)mode;                /* 运动模式标志 */
    data[4] = (uint8_t)sync;                /* 同步标志 */

    Step_SendFrame(addr, 0xF1, data, 5);
}

/**
 *  @brief  快速位置模式 (功能码 0xFC)
 *  @帧    addr + FC + 脉冲数(4, 带符号) + 6B
 *  @param  pulses 带符号脉冲数；+3200 = 正转一圈，-3200 = 反转一圈（16细分）
 */
void Step_QPosRun(uint8_t addr, int32_t pulses)
{
    uint8_t data[4];
    uint32_t p = (uint32_t)pulses;          /* 以无符号方式按大端拆字节 */

    data[0] = (uint8_t)(p >> 24);
    data[1] = (uint8_t)(p >> 16);
    data[2] = (uint8_t)(p >> 8);
    data[3] = (uint8_t)(p & 0xFF);

    Step_SendFrame(addr, 0xFC, data, 4);
}

/**
 *  @brief  立即停止 (功能码 0xFE, 辅助码 0x98)
 *  @帧    addr + FE + 98 + 同步(1) + 6B
 *  @示例  Step_Stop(1, OFF) -> 01 FE 98 00 6B
 */
void Step_Stop(uint8_t addr, Motor_Sync_t sync)
{
    uint8_t data[2];

    data[0] = 0x98;                 /* 固定辅助码 */
    data[1] = (uint8_t)sync;        /* 同步标志 */

    Step_SendFrame(addr, 0xFE, data, 2);
}

/**
 *  @brief  触发多机同步运动 (功能码 0xFF, 辅助码 0x66)
 *  @帧    addr + FF + 66 + 6B
 *  @note  仅当各电机此前指令的同步标志置 SYNC_ON 时，此命令才让它们齐动。
 */
void Step_SyncStart(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x66;                 /* 固定辅助码 */

    Step_SendFrame(addr, 0xFF, data, 1);
}

/* ==========================================================================
 *  2. 原点回零命令
 * ========================================================================== */

/**
 *  @brief  设置单圈回零的零点位置 (功能码 0x93, 辅助码 0x88)
 *  @帧    addr + 93 + 88 + 存储标志(1) + 6B
 */
void Step_HomeSetZero(uint8_t addr, Motor_Store_t store)
{
    uint8_t data[2];

    data[0] = 0x88;                 /* 固定辅助码 */
    data[1] = (uint8_t)store;       /* 是否存储零点 */

    Step_SendFrame(addr, 0x93, data, 2);
}

/**
 *  @brief  触发回零 (功能码 0x9A)
 *  @帧    addr + 9A + 回零模式(1) + 同步(1) + 6B
 */
void Step_HomeStart(uint8_t addr, Motor_HomeMode_t mode, Motor_Sync_t sync)
{
    uint8_t data[2];

    data[0] = (uint8_t)mode;        /* 回零模式 */
    data[1] = (uint8_t)sync;        /* 同步标志 */

    Step_SendFrame(addr, 0x9A, data, 2);
}

/**
 *  @brief  强制中断并退出回零 (功能码 0x9C, 辅助码 0x48)
 *  @帧    addr + 9C + 48 + 6B
 */
void Step_HomeStop(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x48;                 /* 固定辅助码 */

    Step_SendFrame(addr, 0x9C, data, 1);
}

/* ==========================================================================
 *  3. 触发动作命令
 * ========================================================================== */

/**
 *  @brief  触发编码器校准 (功能码 0x06, 辅助码 0x45)
 *  @帧    addr + 06 + 45 + 6B
 */
void Step_EncoderCal(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x45;
    Step_SendFrame(addr, 0x06, data, 1);
}

/**
 *  @brief  重启电机 (功能码 0x08, 辅助码 0x97)
 *  @帧    addr + 08 + 97 + 6B
 */
void Step_Reset(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x97;
    Step_SendFrame(addr, 0x08, data, 1);
}

/**
 *  @brief  当前位置清零 (功能码 0x0A, 辅助码 0x6D)
 *  @帧    addr + 0A + 6D + 6B
 */
void Step_ZeroPos(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x6D;
    Step_SendFrame(addr, 0x0A, data, 1);
}

/**
 *  @brief  解除堵转保护 (功能码 0x0E, 辅助码 0x52)
 *  @帧    addr + 0E + 52 + 6B
 */
void Step_ClrStall(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x52;
    Step_SendFrame(addr, 0x0E, data, 1);
}

/**
 *  @brief  恢复出厂设置 (功能码 0x0F, 辅助码 0x5F)
 *  @帧    addr + 0F + 5F + 6B
 *  @warning 会清除用户配置，谨慎调用！
 */
void Step_RestoreFactory(uint8_t addr)
{
    uint8_t data[1];

    data[0] = 0x5F;
    Step_SendFrame(addr, 0x0F, data, 1);
}

/* ==========================================================================
 *  4. 参数读取命令
 * ========================================================================== */

/**
 *  @brief  读取系统参数
 *  @帧    addr + 参数功能码 + 6B
 *  @note  参数功能码直接取自 Motor_SysParam_t（其枚举值即为协议功能码）。
 *         如读取实时转速：Step_ReadParam(1, SYS_VEL) -> 01 35 6B
 */
void Step_ReadParam(uint8_t addr, Motor_SysParam_t param)
{
    Step_SendFrame(addr, (uint8_t)param, NULL, 0);
}

/* ===================== 电机回传解析 (SYS_CPOS 实时位置) ===================== */
int32_t Step_RealPos = 0;     // 电机真实实时位置(相对零点脉冲), 由 UART5 中断解析填入

/**
 *  @brief  解析张大头电机回传帧: 地址 + 0x36 + 4字节(大端 int32) + 0x6B
 *  @note  仅在查询 SYS_CPOS 时电机才回传该帧; 状态机以 addr+func 双重匹配。
 */
void Step_Parse(uint8_t Data)
{
    static uint8_t state = 0;
    static uint8_t idx   = 0;
    static uint8_t buf[4];

    switch (state)
    {
        case 0:                                     // 等待地址
            if (Data == STEP_ADDR) state = 1;
            break;
        case 1:                                     // 等待功能码 0x36(SYS_CPOS)
            if (Data == 0x36) { state = 2; idx = 0; }
            else if (Data == STEP_ADDR) { state = 1; }
            else state = 0;
            break;
        case 2: case 3: case 4: case 5:            // 收 4 字节数据(大端)
            buf[idx++] = Data;
            if (idx >= 4)
            {
                Step_RealPos = ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) |
                               ((int32_t)buf[2] << 8)  |  (int32_t)buf[3];
                state = 0; idx = 0;
            }
            break;
        default:
            state = 0;
            break;
    }
}
