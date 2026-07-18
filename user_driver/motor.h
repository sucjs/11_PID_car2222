#ifndef MOTOR_H
#define MOTOR_H

#define PI 3.14

// 编码器线数
#define MOTOR_BIANMAQI 363
// 轮胎直径 mm
#define MOTOR_WHEEL_D 68

// G3507      TB6612
// PB19 <--> AIN2
// PB17 <--> AIN1
// PA12 <--> PWMA  (TIMG0_C0)
// GND <--> GND
// 3V3 <--> VCC
// STBY <--> +5V (硬连接)

// TB6612    电源模块
// VM          7.4V
// GND         GND

// TB6612    直流电机1
// AO1<--> M+
// AO2<--> M-

// G3507    直流电机1编码器
// PA25 <--> A
// PA14 <--> B
// 3V3 <--> VCC
// GND <--> GND

// 直流电机2接线：
// BO1<--> M+
// BO2<--> M-
// PA26 <--> A
// PA27 <--> B
// 3V3 <--> VCC
// GND <--> GND

// G3507    TB6612
// PA13 <--> PWMB  (TIMG0_C1)
// PB24 <--> BIN2
// PA16 <--> BIN1

// 所有的GND都需要连接在一起

#include "ti_msp_dl_config.h"
#include "huidu.h"

void motor_init(uint8_t motor_id);
void motor_pid_start(void);      // 初始化完所有电机后，调用一次启动PID定时器
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

#endif // MOTOR_H
