#ifndef HUIDU_H
#define HUIDU_H


#include "ti_msp_dl_config.h"
#include "motor.h"

// 接线
// 八路灰度模块
// VCC       5V
// GND       GND
// S0        GPIOB.5  (PB5)
// S1        GPIOB.15 (PB15)
// S2        GPIOA.10 (PA10)
// S3        GPIOB.16 (PB16)
// S4        GPIOA.11 (PA11)
// S5        GPIOB.12 (PB12)
// S6        GPIOB.13 (PB13)
// S7        GPIOB.23 (PB23)

void huidu_get_value();
void adjust_motor();

#endif
