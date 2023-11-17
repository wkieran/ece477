#include "lcd.h"
#include "sequencer.h"
#include "main.h"

uint8_t selmode();
void mode0(sequencer &seq, TIM_HandleTypeDef *htim);
void mode1(sequencer &seq);
void mode2();
void mode3();
void mode4();
