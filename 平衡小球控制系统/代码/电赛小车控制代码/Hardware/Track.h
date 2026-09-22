#ifndef __Track_H
#define __Track_H

#include "sys.h"

void Track_Init(void);
uint8_t Track_Read_bit(void);
void Read_Track_DATA(uint8_t* arr);
void Track_Control(void);



extern uint8_t Track;
extern float Track_Target_turn;

#endif


