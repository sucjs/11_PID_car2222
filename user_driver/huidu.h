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
    MODE_STOP       = 0,  // 停车
    MODE_RUN        = 1,  // 循迹运行
    MODE_TUNE_KP    = 2,  // 调节 KP
    MODE_TUNE_KD    = 3,  // 调节 KD
    MODE_TUNE_KI    = 4,  // 调节 KI
    MODE_TUNE_SPEED = 5,  // 调节基础速度
    MODE_COUNT      = 6
} car_mode_t;

// PID 循迹参数（可通过按键或串口在线调节）
extern float line_kp;       // 位置比例系数
extern float line_kd;       // 位置微分系数
extern float line_ki;       // 位置积分系数
extern float base_speed;    // 基础速度 (mm/s)

// 调试/观测变量
extern float line_position; // 当前线位置 (0~7000, 中心=3500)
extern float line_error;    // 当前偏差
extern float line_turn;     // PID 输出的转向修正量

void huidu_get_value(void);
void adjust_motor(void);
void line_pid_reset(void);           // 重置积分和历史误差
void line_pid_tune_param(int dir);   // 按键调参: dir>0增加, dir<0减少

#endif
