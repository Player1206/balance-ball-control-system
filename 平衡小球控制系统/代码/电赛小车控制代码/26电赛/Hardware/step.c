#include "step.h"
#include "usart.h"   // 提供 Serial_SendByte() 的声明(底层走 USART2)

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
