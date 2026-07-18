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
extern uint8_t huidu_value[];

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示
    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    OLED_Clear();
    NVIC_EnableIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
    motor_init(1);           // 初始化电机1（含方向设置）
    motor_init(2);           // 初始化电机2（含方向设置）
    target_speed_1 = 200;    // 初始直行目标速度 mm/s（不再从0开始）
    target_speed_2 = 200;

    char huidu_buf[] = "00000000\n";
    while (1) {
        huidu_get_value();
        sprintf(huidu_buf, "%d%d%d%d%d%d%d%d\n", huidu_value[0], huidu_value[1], huidu_value[2], huidu_value[3], huidu_value[4], huidu_value[5], huidu_value[6], huidu_value[7]);
        UART_send_string(DEBUG_INST, huidu_buf);
        delay_ms(500);
    }
}
