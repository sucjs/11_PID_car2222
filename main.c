/*
 * 8路灰度PID循迹小车 — 主程序 (5按键菜单版)
 * MCU: MSPM0G3507  主频: 80MHz
 * 
 * 控制架构（级联PID）：
 *   外环(10ms): 灰度位置PID -> 计算左右轮目标速度
 *   内环(10ms): 编码器速度PI  -> 计算PWM占空比
 * 
 * 5按键操作：
 *   PB14(RUN):  快捷启动/停止（任意状态下可用）
 *   PB10(OK):   进入菜单 / 确认选择
 *   PB11(BACK): 返回 / 取消
 *   PB0(UP):    上 / 增大
 *   PB1(DOWN):  下 / 减小
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
extern uint32_t raw_enc_1, raw_enc_2;
extern int32_t PWM_1_duty, PWM_2_duty;
extern float line_position, line_error, line_turn;

// -----------------------------------------------------------
// OLED 显示：8路传感器方块
// -----------------------------------------------------------
void OLED_show_sensors(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        uint8_t x = i * 16;
        uint8_t y = 14;
        if (!huidu_value[i]) {
            uint8_t r, c;
            for (r = 0; r < 10; r++)
                for (c = 0; c < 14; c++)
                    OLED_DrawPoint(x + c, y + r);
        } else {
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

// -----------------------------------------------------------
// OLED 仪表盘 (MENU_OFF)
// -----------------------------------------------------------
void OLED_show_dashboard(void)
{
    char buf[20];

    OLED_Clear();

    if (mode == MODE_STOP)
        OLED_ShowString(0, 0, (uint8_t *)"STOP", 12);
    else if (mode == MODE_RUN)
        OLED_ShowString(0, 0, (uint8_t *)"RUN ", 12);
    else
        OLED_ShowString(0, 0, (uint8_t *)"TUNE", 12);

    sprintf(buf, "Spd:%d", (int)base_speed);
    OLED_ShowString(60, 0, (uint8_t *)buf, 12);

    OLED_show_sensors();

    sprintf(buf, "P:%4d E:%4d", (int)line_position, (int)line_error);
    OLED_ShowString(0, 26, (uint8_t *)buf, 12);

    sprintf(buf, "L:%3d R:%3d T:%d", (int)speed_1, (int)speed_2, (int)line_turn);
    OLED_ShowString(0, 38, (uint8_t *)buf, 12);

    sprintf(buf, "E1:%d E2:%d D1:%d",
            (int)raw_enc_1, (int)raw_enc_2, (int)PWM_1_duty);
    OLED_ShowString(0, 50, (uint8_t *)buf, 12);

    OLED_Refresh();
}

// -----------------------------------------------------------
// OLED 主菜单 (MENU_MAIN)
// -----------------------------------------------------------
void OLED_show_menu(void)
{
    char buf[24];
    uint8_t y, i;
    const char *items[] = {"Run/Stop", "Base Speed", "KP", "KD", "KI", "Exit Menu"};

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"-- MENU --", 12);

    for (i = 0; i < MENU_ITEM_COUNT; i++) {
        y = 12 + i * 9;
        if (i == menu_cursor)
            OLED_ShowString(0, y, (uint8_t *)">", 12);
        else
            OLED_ShowString(0, y, (uint8_t *)" ", 12);

        switch (i) {
            case MENU_ITEM_RUN:
                sprintf(buf, "%s [%s]", items[i],
                        (mode == MODE_STOP) ? "STOP" : "RUN");
                break;
            case MENU_ITEM_SPEED:
                sprintf(buf, "%s: %d", items[i], (int)base_speed);
                break;
            case MENU_ITEM_KP:
                sprintf(buf, "%s: %.2f", items[i], (double)line_kp);
                break;
            case MENU_ITEM_KD:
                sprintf(buf, "%s: %.2f", items[i], (double)line_kd);
                break;
            case MENU_ITEM_KI:
                sprintf(buf, "%s: %.3f", items[i], (double)line_ki);
                break;
            case MENU_ITEM_EXIT:
                sprintf(buf, "%s", items[i]);
                break;
        }
        OLED_ShowString(10, y, (uint8_t *)buf, 12);
    }

    OLED_Refresh();
}

// -----------------------------------------------------------
// OLED 参数编辑 (MENU_EDIT)
// -----------------------------------------------------------
void OLED_show_edit(void)
{
    char buf[24];
    const char *names[] = {"Base Speed", "KP", "KD", "KI"};
    float val;

    switch (edit_param) {
        case 0: val = base_speed; break;
        case 1: val = line_kp;    break;
        case 2: val = line_kd;    break;
        case 3: val = line_ki;    break;
        default: val = 0; break;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"-- EDIT --", 12);

    sprintf(buf, "%s:", names[edit_param]);
    OLED_ShowString(0, 16, (uint8_t *)buf, 16);

    if (edit_param == 3)
        sprintf(buf, "%.4f", (double)val);
    else if (edit_param == 0)
        sprintf(buf, "%d", (int)val);
    else
        sprintf(buf, "%.2f", (double)val);
    OLED_ShowString(0, 34, (uint8_t *)buf, 24);

    OLED_ShowString(0, 56, (uint8_t *)"UP/DN +- OK sav BK can", 12);

    OLED_Refresh();
}

// -----------------------------------------------------------
// 主函数
// -----------------------------------------------------------
int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    OLED_ShowString(20, 20, (uint8_t *)"PID Car V3", 16);
    OLED_ShowString(12, 40, (uint8_t *)"5BTN MENU OK", 12);
    OLED_Refresh();
    delay_ms(1200);

    NVIC_EnableIRQ(KEY_INT_IRQN);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);

    motor_init(1);
    motor_init(2);

    target_speed_1 = 0;
    target_speed_2 = 0;

    char debug_buf[120];

    while (1) {
        // ---- 处理按键事件（菜单状态机） ----
        menu_process();

        // ---- 模式变化时重置 PID ----
        if (mode != last_mode) {
            last_mode = mode;
            line_pid_reset();
            motor_pid_reset();
            if (mode == MODE_STOP) {
                target_speed_1 = 0;
                target_speed_2 = 0;
            }
        }

        // ---- 根据模式控制电机 ----
        if (mode == MODE_RUN || mode >= MODE_TUNE_KP) {
            motor_set_direction(1, 1);
            motor_set_direction(2, 1);
        } else {
            motor_set_direction(1, 0);
            motor_set_direction(2, 0);
            target_speed_1 = 0;
            target_speed_2 = 0;
        }

        // ---- OLED 显示 ----
        if (menu_state == MENU_EDIT)
            OLED_show_edit();
        else if (menu_state == MENU_MAIN)
            OLED_show_menu();
        else
            OLED_show_dashboard();

        // ---- UART 调试 ----
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
