/*
 * 8路灰度PID循迹小车 — 主程序
 * MCU: MSPM0G3507  主频: 80MHz
 * 
 * 控制架构（级联PID）：
 *   外环(10ms): 灰度位置PID -> 计算左右轮目标速度
 *   内环(10ms): 编码器速度PI  -> 计算PWM占空比
 * 
 * 按键操作：
 *   KEY0: 模式切换  STOP->RUN->TUNE_KP->TUNE_KD->TUNE_KI->TUNE_SPEED
 *   KEY1: TUNE模式=增大参数 / 非TUNE模式=切回上一个模式
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"
#include "huidu.h"

// -----------------------------------------------------------
// 全局状态
// -----------------------------------------------------------
int mode      = MODE_STOP;
int last_mode = -1;

// 外部引用
extern float target_speed_1;
extern float target_speed_2;
extern float speed_1;
extern float speed_2;
extern uint8_t huidu_value[];
extern float line_kp, line_kd, line_ki, base_speed;
extern float line_position, line_error, line_turn;

// -----------------------------------------------------------
// OLED 辅助显示函数
// -----------------------------------------------------------
void OLED_show_sensors(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        uint8_t x = i * 16;       // 每格16像素
        uint8_t y = 14;           // 第2行
        if (!huidu_value[i]) {    // 踩黑线=实心方块
            uint8_t r, c;
            for (r = 0; r < 10; r++)
                for (c = 0; c < 14; c++)
                    OLED_DrawPoint(x + c, y + r);
        } else {                  // 白底=空心框
            uint8_t j;
            for (j = 0; j < 14; j++) {
                OLED_DrawPoint(x + j, y);
                OLED_DrawPoint(x + j, y + 9);
            }
            for (j = 0; j < 10; j++) {
                OLED_DrawPoint(x,      y + j);
                OLED_DrawPoint(x + 13, y + j);
            }
        }
    }
}

void OLED_show_dashboard(void)
{
    char buf[20];

    OLED_Clear();

    // 第1行：模式
    switch (mode) {
        case MODE_STOP:       OLED_ShowString(0, 0, (uint8_t *)"STOP", 12); break;
        case MODE_RUN:        OLED_ShowString(0, 0, (uint8_t *)"RUN ", 12); break;
        case MODE_TUNE_KP:    OLED_ShowString(0, 0, (uint8_t *)"T-KP", 12); break;
        case MODE_TUNE_KD:    OLED_ShowString(0, 0, (uint8_t *)"T-KD", 12); break;
        case MODE_TUNE_KI:    OLED_ShowString(0, 0, (uint8_t *)"T-KI", 12); break;
        case MODE_TUNE_SPEED: OLED_ShowString(0, 0, (uint8_t *)"T-SP", 12); break;
    }

    // 传感器可视化
    OLED_show_sensors();

    // 数据行
    sprintf(buf, "P:%4d E:%4d", (int)line_position, (int)line_error);
    OLED_ShowString(0, 26, (uint8_t *)buf, 12);

    sprintf(buf, "L:%3d R:%3d", (int)speed_1, (int)speed_2);
    OLED_ShowString(0, 38, (uint8_t *)buf, 12);

    sprintf(buf, "Kp:%.2f Kd:%.2f", (double)line_kp, (double)line_kd);
    OLED_ShowString(0, 50, (uint8_t *)buf, 12);

    OLED_Refresh();
}

// -----------------------------------------------------------
// 主函数
// -----------------------------------------------------------
int main(void)
{
    SYSCFG_DL_init();

    // 初始化 OLED
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    OLED_ShowString(16, 20, (uint8_t *)"PID Car V2", 16);
    OLED_ShowString(24, 40, (uint8_t *)"Init OK", 12);
    OLED_Refresh();
    delay_ms(1000);

    // 使能中断（按键 + 编码器）
    NVIC_EnableIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);

    // 初始化电机（内部启动 PID 定时器 ISR）
    motor_init(1);
    motor_init(2);

    // 初始停车
    target_speed_1 = 0;
    target_speed_2 = 0;

    char debug_buf[120];

    // ---- 主循环 ----
    while (1) {
        // 模式变化时重置 PID
        if (mode != last_mode) {
            last_mode = mode;
            line_pid_reset();
            if (mode == MODE_STOP) {
                target_speed_1 = 0;
                target_speed_2 = 0;
            }
        }

        // 根据模式控制电机方向
        if (mode == MODE_RUN || mode >= MODE_TUNE_KP) {
            motor_set_direction(1, 1);  // 正转
            motor_set_direction(2, 1);
        } else {
            motor_set_direction(1, 0);  // 停止
            motor_set_direction(2, 0);
            target_speed_1 = 0;
            target_speed_2 = 0;
        }

        // OLED 刷新
        OLED_show_dashboard();

        // UART 调试输出（每秒5帧）
        sprintf(debug_buf,
            "M%d|S:%d%d%d%d%d%d%d%d|P:%.0f E:%.0f T:%.0f|SpL:%d SpR:%d|Kp:%.3f Kd:%.3f Ki:%.4f BS:%.0f
",
            mode,
            huidu_value[0], huidu_value[1], huidu_value[2], huidu_value[3],
            huidu_value[4], huidu_value[5], huidu_value[6], huidu_value[7],
            (double)line_position, (double)line_error, (double)line_turn,
            (int)speed_1, (int)speed_2,
            (double)line_kp, (double)line_kd, (double)line_ki, (double)base_speed);
        UART_send_string(DEBUG_INST, debug_buf);

        delay_ms(200);
    }
}
