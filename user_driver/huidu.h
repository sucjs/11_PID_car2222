#ifndef HUIDU_H
#define HUIDU_H

#include "ti_msp_dl_config.h"

// ============================================================
// 八路灰度模块接线
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
// 注意：传感器输出 0=踩黑线, 1=白底
// ============================================================

// 小车运行模式
typedef enum {
    MODE_STOP       = 0,
    MODE_RUN        = 1,
    MODE_TUNE_KP    = 2,
    MODE_TUNE_KD    = 3,
    MODE_TUNE_KI    = 4,
    MODE_TUNE_SPEED = 5,
    MODE_COUNT      = 6
} car_mode_t;

// PID 循迹参数
extern float line_kp;
extern float line_kd;
extern float line_ki;
extern float base_speed;

// 调试/观测变量
extern float line_position;
extern float line_error;
extern float line_turn;
extern uint8_t huidu_value[];

void huidu_get_value(void);
void adjust_motor(void);
void line_pid_reset(void);

#endif
