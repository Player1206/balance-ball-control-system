#ifndef __Track_H
#define __Track_H

#include "sys.h"

void unlost_lost_count(void);
void judge_unlost(void);

extern int lostcount; //未检测到黑线次数记录
extern int unlostcount; // 检测到黑线次数记录


#endif

