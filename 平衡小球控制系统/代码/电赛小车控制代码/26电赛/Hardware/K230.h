#ifndef __K230_H
#define __K230_H

#include <stdint.h>

/* K230 -> STM32 串口协议（请在 K230 端固件匹配）
 * 硬件：K230 接 STM32 的 UART4(PC10=TX, PC11=RX)，独立于调试/电机/陀螺仪串口。
 * 帧格式（无帧头校验，状态机解析）：
 *     AA  55  [int16 X*100]  [int16 Y*100]  [uint8 SUM]  ED
 *   其中 SUM = (0xAA + 0x55 + XH + XL + YH + YL) & 0xFF
 *   X / Y 单位 cm，放大 100 倍传（分辨率 0.01cm）；X 右正左负，Y 沿杆前后（备用）
 */
extern float  Ball_X;        // 球相对中心 X 位置(cm)，右正左负
extern float  Ball_Y;        // 球相对中心 Y 位置(cm)，沿杆前后，备用
extern uint8_t Ball_Updated; // 收到新一帧置 1，控制循环可读取后清零

void K230_Decode(uint8_t Data);

#endif /* __K230_H */
