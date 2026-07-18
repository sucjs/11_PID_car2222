/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"
#include "huidu.h"

int status = 0;
extern float target_speed_1;
extern float target_speed_2;
extern float speed_1;
extern float speed_2;
extern int32_t PWM_1_duty;
extern int32_t PWM_2_duty;
extern uint8_t huidu_value[];

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);      // 0 正常显示，1 反色显示
    OLED_DisplayTurn(0);     // 0 正常显示，1 屏幕翻转显示
    OLED_Clear();
    NVIC_EnableIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);

    // ====== 步骤1: 初始化两个电机（PWM 置零，方向引脚设为惰行） ======
    motor_init(1);
    motor_init(2);

    // ====== 步骤2: 设置两个电机为正转方向 ======
    motor_set_direction(1, 1);  // 电机1 正转
    motor_set_direction(2, 1);  // 电机2 正转

    // ====== 步骤3: 设置初始目标速度 ======
    target_speed_1 = 100;    // 初始直行速度 mm/s（与 huidu.c 中 BASE_SPEED 一致）
    target_speed_2 = 100;

    // ====== 步骤4: 最后启动 PID 定时器（两个电机就绪后才开始控制） ======
    motor_pid_start();

    char huidu_buf[] = "00000000\n";
    while (1) {
        huidu_get_value();
        // 输出：8路灰度 + 电机1(目标速度,实际速度,PWM) + 电机2(目标速度,实际速度,PWM)
        sprintf(huidu_buf, "%d%d%d%d%d%d%d%d L:%d,%d,%d R:%d,%d,%d\n",
                huidu_value[0], huidu_value[1], huidu_value[2], huidu_value[3],
                huidu_value[4], huidu_value[5], huidu_value[6], huidu_value[7],
                (int)target_speed_1, (int)speed_1, (int)PWM_1_duty,
                (int)target_speed_2, (int)speed_2, (int)PWM_2_duty);
        UART_send_string(DEBUG_INST, huidu_buf);
        delay_ms(500);
    }
}
