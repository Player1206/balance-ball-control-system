#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include <stdint.h>
#include "stm32f10x.h"

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

/* ===================== STEP/DIR 脉冲控制配置 =====================
 * 当前控球使用这一套“普通脉冲控制”，不走张大头串口运动模式。
 *
 * 为什么这么做：
 *   串口位置/速度模式内部还有电机自己的运动规划，调试时不容易判断
 *   “到底是控制器输出错了，还是电机模式没响应”。STEP/DIR 更直接：
 *   STM32 发一个脉冲，驱动器就走一步。
 *
 * 默认复用原 UART5 接线脚:
 *   PC12 -> PUL/STEP
 *   PD2  -> DIR
 * 如现场接线不同，只改这几个宏。
 *
 * 驱动器如果是 PUL+/PUL-/DIR+/DIR- 光耦输入：
 *   要按你们驱动器说明书选择共阳或共阴接法，并且 STM32 必须与驱动器共地。
 */
#define STEP_PULSE_GPIO       GPIOC
#define STEP_PULSE_PIN        GPIO_Pin_12
#define STEP_PULSE_RCC        RCC_APB2Periph_GPIOC

#define STEP_DIR_GPIO         GPIOD
#define STEP_DIR_PIN          GPIO_Pin_2
#define STEP_DIR_RCC          RCC_APB2Periph_GPIOD

/* 方向电平：
 *   如果 Key3 自检时“+50脉冲”的方向和预期相反，可以改这个。
 *   如果闭环方向反了，更推荐先改 Balance.c 里的 BALANCE_DIR_SIGN。
 */
#define STEP_DIR_POS_LEVEL    1

/* 软件位置限位：
 *   纯脉冲控制没有真实位置反馈，必须用软件脉冲数限制摆杆最大角度。
 *   调试时先保守一点，确认机械不会撞限位后再放大。
 */
#define STEP_PULSE_ABS_LIMIT  300     /* 调试期软件位置保护: 约 ±34 度(3200脉冲/圈) */

/* 每个 10ms 控制周期最多发多少个脉冲：
 *   这个值相当于“最大杆角速度”保护。
 *   球回得慢可以适当加大；电机抖/卡/冲过头就减小。
 */
#define STEP_PULSE_PER_TICK   32

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

/* ---------- 0. STEP/DIR 脉冲控制 ----------
 * StepPulse_Position 是软件估计位置：
 *   StepPulse_Move(+N) 后加 N，StepPulse_Move(-N) 后减 N。
 *   它不会知道电机是否真实丢步，所以机械卡住后要重新手动回零/复位。
 */
void StepPulse_Init(void);
void StepPulse_Enable(uint8_t enable);
void StepPulse_SetDir(int8_t dir);
void StepPulse_Move(int32_t pulses);
void StepPulse_Stop(void);
void StepPulse_Zero(void);
extern int32_t StepPulse_Position;
extern volatile uint8_t StepPulse_Ready;

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

/* 电机真实实时位置(SYS_CPOS 回传, 相对零点脉冲), 由 UART5 接收解析填入, 连贯准确, 供OLED显示 */
extern int32_t Step_RealPos;
/* UART5 接收中断调用: 解析电机回传的位置帧( addr + 0x36 + 4字节大端 + 0x6B ) */
void Step_Parse(uint8_t Data);

#endif /* __STEP_MOTOR_H */
