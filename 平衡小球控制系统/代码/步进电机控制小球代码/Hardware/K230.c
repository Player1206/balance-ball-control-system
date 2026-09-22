#include "K230.h"

float  Ball_X = 0;
float  Ball_V = 0;
float  Ball_Y = 0;
uint8_t Ball_Updated = 0;
volatile uint32_t K230_RxCount = 0;
volatile uint32_t K230_FrameCount = 0;
volatile uint8_t  K230_LastByte = 0;
volatile uint8_t  K230_ParseError = 0;

/* 行缓冲：S,<X>,<flag>\n 最长约 "S,-12.34,1\n" = 11 字符 + 余量 */
#define K230_LINE_MAX 24
static char    line_buf[K230_LINE_MAX];
static uint8_t line_idx = 0;
static uint8_t in_line  = 0;   // 是否处于一帧接收中(已收到 'S')

/* 轻量 atof: 支持 "-12.34" / "12.34" / "0.00"，不依赖浮点 sscanf 库 */
static float k230_atof(const char *s)
{
    float sign = 1.0f, val = 0.0f;
    int   frac = 0, fdiv = 10;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    while (*s)
    {
        if (*s >= '0' && *s <= '9')
        {
            if (frac) { val += (float)(*s - '0') / (float)fdiv; fdiv *= 10; }
            else      { val = val * 10.0f + (float)(*s - '0'); }
        }
        else if (*s == '.')
        {
            frac = 1;
        }
        s++;
    }
    return sign * val;
}

void K230_Decode(uint8_t Data)
{
    K230_RxCount++;
    K230_LastByte = Data;

    /* 帧起始 'S' */
    if (!in_line)
    {
        if (Data == 'S')
        {
            in_line = 1;
            line_idx = 0;
            line_buf[line_idx++] = 'S';
        }
        return;
    }

    /* 行结束：回车或换行都视为一帧结束 */
    if (Data == '\n' || Data == '\r')
    {
        line_buf[line_idx] = '\0';
        /* 解析: S,<X>,<flag> */
        char *p = line_buf;
        if (*p == 'S' && *(p + 1) == ',')
        {
            char *xstr = p + 2;
            char *c1 = xstr;
            while (*c1 && *c1 != ',') c1++;
            if (*c1 == ',')
            {
                *c1 = '\0';
                char *vstr = c1 + 1;
                char *c2 = vstr;
                while (*c2 && *c2 != ',') c2++;
                if (*c2 == ',')
                {
                    *c2 = '\0';
                    int flag = (c2[1] == '1') ? 1 : 0;
                    float x = k230_atof(xstr);
                    float v = k230_atof(vstr);
                    if (flag)
                    {
                        Ball_X = x;          // 右正左负
                        Ball_V = v;          // 右正左负
                        Ball_Y = 0;
                        Ball_Updated = 1;
                        K230_FrameCount++;
                    }
                    else
                    {
                        Ball_V = 0.0f;       // 丢失: 速度清零, 避免持续驱动
                        Ball_Updated = 0;    // 丢失，保持上次位置不更新
                        K230_FrameCount++;
                    }
                }
            }
        }
        in_line = 0;
        line_idx = 0;
        return;
    }

    /* 缓冲溢出保护：太长直接丢弃重来 */
    if (line_idx >= K230_LINE_MAX - 1)
    {
        in_line = 0;
        line_idx = 0;
        K230_ParseError++;
        return;
    }
    line_buf[line_idx++] = (char)Data;
}
