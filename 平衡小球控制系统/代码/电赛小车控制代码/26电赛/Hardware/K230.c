#include "K230.h"

float  Ball_X = 0;
float  Ball_Y = 0;
uint8_t Ball_Updated = 0;

/* 接收状态机 */
typedef enum {
    S_HEAD1,   // 等待 0xAA
    S_HEAD2,   // 等待 0x55
    S_XH,      // X 高字节
    S_XL,      // X 低字节
    S_YH,      // Y 高字节
    S_YL,      // Y 低字节
    S_SUM,     // 校验和
    S_TAIL     // 等待 0xED
} K230_State_t;

static K230_State_t kstate = S_HEAD1;
static int16_t rx_x = 0, rx_y = 0;
static uint8_t rx_sum = 0;

void K230_Decode(uint8_t Data)
{
    switch (kstate)
    {
        case S_HEAD1:
            if (Data == 0xAA) kstate = S_HEAD2;
            break;

        case S_HEAD2:
            if (Data == 0x55) { kstate = S_XH; rx_sum = 0xAA + 0x55; }
            else              kstate = S_HEAD1;
            break;

        case S_XH:
            rx_x = (int16_t)(Data << 8);
            rx_sum += Data;
            kstate = S_XL;
            break;

        case S_XL:
            rx_x |= (int16_t)Data;
            rx_sum += Data;
            kstate = S_YH;
            break;

        case S_YH:
            rx_y = (int16_t)(Data << 8);
            rx_sum += Data;
            kstate = S_YL;
            break;

        case S_YL:
            rx_y |= (int16_t)Data;
            rx_sum += Data;
            kstate = S_SUM;
            break;

        case S_SUM:
            if (Data == rx_sum) kstate = S_TAIL;   // 校验通过，等尾
            else                kstate = S_HEAD1;  // 校验失败，丢弃
            break;

        case S_TAIL:
            if (Data == 0xED)
            {
                Ball_X = rx_x / 100.0f;            // cm
                Ball_Y = rx_y / 100.0f;            // cm
                Ball_Updated = 1;
            }
            kstate = S_HEAD1;
            break;

        default:
            kstate = S_HEAD1;
            break;
    }
}
