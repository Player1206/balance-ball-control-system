#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>

/* ==========================================================================
 *  张大头闭环步进电机 —— 串口总线协议驱动（统一封装 / 模块化）
 *
 *  协议帧格式（无帧头，多字节数据一律大端：高位在前）：
 *      地址(1B) + 功能码(1B) + [辅助码] + 数据域(nB) + 校验(1B, 固定0x6B)
 *
 *  物理串口：底层 Serial_SendByte() 走 UART5(PC12=TX, PD2=RX)，见 usart.c。
 *
 *  本文件对照官方示例工程 Emm_V5.c 实现，覆盖：
 *      1) 运动控制：使能 / 速度模式 / 位置模式 / 快速位置 / 停止 / 同步
 *      2) 原点回零：设零点 / 触发回零 / 中断回零
 *      3) 触发动作：编码器校准 / 重启 / 位置清零 / 解除堵转 / 恢复出厂
 *      4) 参数读取：实时转速 / 位置 / 状态标志等
 *
 *  设计原则：
 *      - 分层：底层发送 Step_SendFrame() → 业务 API（Step_*）
 *      - 可移植：仅依赖 Serial_SendByte()，与具体 MCU 解耦
 *      - 统一命名：Step_<动作>；枚举代替裸 0/1，提升可读性
 * ========================================================================== */

/* ============================ 基础配置 ============================ */
#define STEP_ADDR       2       /* 电机地址（与 DIP 拨码一致） */
#define STEP_CHECKSUM   0x6B    /* 默认固定校验字节（菜单 Checksum 可切 XOR/CRC） */

/* ============================ 通用枚举 ============================ */

/* 旋转方向 */
typedef enum {
    MOTOR_DIR_CW  = 0x00,   /* 正转 / 顺时针 */
    MOTOR_DIR_CCW = 0x01    /* 反转 / 逆时针 */
} Motor_Dir_t;

/* 使能状态 */
typedef enum {
    MOTOR_DISABLE = 0x00,   /* 失能（断电保持） */
    MOTOR_ENABLE  = 0x01    /* 使能（上电解锁、可运动） */
} Motor_Enable_t;

/* 多机同步标志 */
typedef enum {
    SYNC_OFF = 0x00,        /* 不启用同步 */
    SYNC_ON  = 0x01         /* 启用同步（配合 Step_SyncStart 多机齐动） */
} Motor_Sync_t;

/* 参数存储标志（掉电保存） */
typedef enum {
    STORE_NO  = 0x00,       /* 仅本次生效，不写入 Flash */
    STORE_YES = 0x01        /* 写入 Flash，下次上电仍有效 */
} Motor_Store_t;

/* 位置运动模式（相对 / 绝对） */
typedef enum {
    POS_REL_LAST = 0,       /* 相对上一目标位置运动 */
    POS_ABS      = 1,       /* 绝对坐标运动 */
    POS_REL_CUR  = 2        /* 相对当前实时位置运动 */
} Motor_PosMode_t;

/* 回零模式 */
typedef enum {
    HOME_SINGLE_NEAR   = 0, /* 单圈就近回零 */
    HOME_SINGLE_DIR    = 1, /* 单圈方向回零 */
    HOME_MULTI_NOLIMIT = 2, /* 多圈无限位碰撞回零 */
    HOME_MULTI_LIMIT   = 3  /* 多圈有限位开关回零 */
} Motor_HomeMode_t;

/* 系统参数读取类型（枚举值即协议功能码，可直接下发） */
typedef enum {
    SYS_VBUS  = 0x24,       /* 总线电压 */
    SYS_CBUS  = 0x26,       /* 总线电流 */
    SYS_CPHA  = 0x27,       /* 相电流 */
    SYS_CLKC  = 0x30,       /* 实时脉冲数 */
    SYS_TPOS  = 0x33,       /* 电机目标位置 */
    SYS_SPOS  = 0x34,       /* 实时设定目标位置 */
    SYS_VEL   = 0x35,       /* 电机实时转速 */
    SYS_CPOS  = 0x36,       /* 电机实时位置 */
    SYS_PERR  = 0x37,       /* 位置误差 */
    SYS_FLAG  = 0x3A,       /* 状态标志位 */
    SYS_OFLAG = 0x3B        /* 回零状态标志位 */
} Motor_SysParam_t;

/* ============================ API 声明 ============================ */

void Step_Init(void);

/* 底层：发送一帧（地址 + 功能码 + data[len] + 固定校验 0x6B） */
void Step_SendFrame(uint8_t addr, uint8_t cmd, const uint8_t *data, uint8_t len);

/* ---------- 1. 运动控制 ---------- */
void Step_Enable(uint8_t addr, Motor_Enable_t en, Motor_Sync_t sync);
void Step_RunSpeed(uint8_t addr, Motor_Dir_t dir, uint16_t rpm, uint8_t accel, Motor_Sync_t sync);
void Step_RunPosition(uint8_t addr, Motor_Dir_t dir, uint16_t rpm, uint8_t accel,
                       uint32_t pulses, Motor_PosMode_t mode, Motor_Sync_t sync);
void Step_QPosSetParams(uint8_t addr, uint16_t rpm, uint8_t accel, Motor_PosMode_t mode, Motor_Sync_t sync);
void Step_QPosRun(uint8_t addr, int32_t pulses);
void Step_Stop(uint8_t addr, Motor_Sync_t sync);
void Step_SyncStart(uint8_t addr);

/* ---------- 2. 原点回零 ---------- */
void Step_HomeSetZero(uint8_t addr, Motor_Store_t store);
void Step_HomeStart(uint8_t addr, Motor_HomeMode_t mode, Motor_Sync_t sync);
void Step_HomeStop(uint8_t addr);

/* ---------- 3. 触发动作 ---------- */
void Step_EncoderCal(uint8_t addr);
void Step_Reset(uint8_t addr);
void Step_ZeroPos(uint8_t addr);
void Step_ClrStall(uint8_t addr);
void Step_RestoreFactory(uint8_t addr);

/* ---------- 4. 参数读取 ---------- */
void Step_ReadParam(uint8_t addr, Motor_SysParam_t param);

#endif /* __STEP_MOTOR_H */
