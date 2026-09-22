#ifndef __K230_H
#define __K230_H

#include <stdint.h>

/* K230 -> STM32 串口协议（ASCII 文本行，与 K230 端固件匹配）
 * 硬件：K230 接 STM32 的 UART4(PC10=TX, PC11=RX)，115200bps，独立于调试/电机/陀螺仪串口。
 * 每帧格式(以 '\n' 或 '\r' 结尾)：
 *     S,<X cm>,<V cm/s>,<flag>\n
 *   - flag=1 检测到球, X=球相对中心位置(cm, 右正左负), V=球速度(cm/s, 右正左负)
 *   - flag=0 丢失,     K230 发 "S,<上次X>,0.00,0"，速度置 0
 *   例: "S,12.34,3.50,1\n"  "S,-5.12,-2.10,1\n"  "S,0.00,0.00,0\n"
 *   仅传 X 单轴; Y 轴未用, 维持 0。
 */
extern float  Ball_X;        // 球相对中心 X 位置(cm)，右正左负
extern float  Ball_V;        // 球 X 方向速度(cm/s)，右正左负(右移为正)
extern float  Ball_Y;        // 球相对中心 Y 位置(cm)，沿杆前后，备用
extern uint8_t Ball_Updated; // 收到新一帧置 1，控制循环可读取后清零
extern volatile uint32_t K230_RxCount;    // UART4 收到的原始字节数
extern volatile uint32_t K230_FrameCount; // 成功解析的 S 帧数
extern volatile uint8_t  K230_LastByte;   // 最近收到的原始字节，串口校准用
extern volatile uint8_t  K230_ParseError; // 解析错误计数(低8位)，串口校准用

void K230_Decode(uint8_t Data);

#endif /* __K230_H */
