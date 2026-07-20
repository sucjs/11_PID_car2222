/*
 * 5按键中断处理 + 菜单状态机
 * 
 * 按键映射:
 *   PB0  → KEY_UP   (上/增大)
 *   PB1  → KEY_DOWN (下/减小)
 *   PB10 → KEY_OK   (确认/进入菜单)
 *   PB11 → KEY_BACK (返回/取消)
 *   PB14 → KEY_RUN  (快捷启动/停止)
 * 
 * 菜单结构:
 *   MENU_OFF:  正常驾驶 + OLED仪表盘
 *   MENU_MAIN: 主菜单(6项选择)
 *   MENU_EDIT: 参数编辑
 */
#include "key.h"
#include "huidu.h"

// -----------------------------------------------------------
// 全局状态
// -----------------------------------------------------------
volatile uint8_t key_event  = 0;
menu_state_t     menu_state = MENU_OFF;
uint8_t          menu_cursor = 0;
uint8_t          edit_param  = 0;  // 0=base_speed, 1=kp, 2=kd, 3=ki

extern int mode;  // 小车运行模式 (main.c)

// 编码器计数器
uint32_t counter_1_A = 0;
uint32_t counter_2_A = 0;

// -----------------------------------------------------------
// GPIO 状态读取
// -----------------------------------------------------------
uint8_t get_key_state(uint32_t key)
{
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key);
    return ((high_bits & key) != 0) ? 1 : 0;
}

// -----------------------------------------------------------
// 消费按键事件（主循环调用，每次处理后清零）
// -----------------------------------------------------------
void key_event_clear(void)
{
    key_event = 0;
}

// -----------------------------------------------------------
// GROUP1 中断处理：按键(GPIOB) + 编码器(GPIOA)
// -----------------------------------------------------------
void GROUP1_IRQHandler()
{
    uint32_t gpio_b_pending = DL_GPIO_getPendingInterrupt(GPIOB);

    // ---- 处理按键中断 (GPIOB) ----
    switch (gpio_b_pending) {
        case KEY_KEY_UP_IIDX:
            key_event |= KEY_EVENT_UP;
            break;
        case KEY_KEY_DOWN_IIDX:
            key_event |= KEY_EVENT_DOWN;
            break;
        case KEY_KEY_OK_IIDX:
            key_event |= KEY_EVENT_OK;
            break;
        case KEY_KEY_BACK_IIDX:
            key_event |= KEY_EVENT_BACK;
            break;
        case KEY_KEY_RUN_IIDX:
            key_event |= KEY_EVENT_RUN;
            break;
        default:
            break;
    }

    // ---- 处理编码器中断 (GPIOA) ----
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
        case DC_MOTOR_AA_IIDX:
            counter_1_A++;
            break;
        case DC_MOTOR_BA_IIDX:
            counter_2_A++;
            break;
        default:
            break;
    }
}

// -----------------------------------------------------------
// 菜单逻辑处理函数（由 main.c 主循环调用）
// 根据 key_event 和当前 menu_state 执行相应操作
// 返回值: 1=有操作发生, 0=无操作
// -----------------------------------------------------------
int menu_process(void)
{
    if (key_event == 0) return 0;

    uint8_t ev = key_event;
    key_event_clear();

    // ==========================================
    // KEY_RUN: 在任何状态下，快捷切换 运行/停止
    // ==========================================
    if (ev & KEY_EVENT_RUN) {
        if (mode == MODE_STOP) {
            mode = MODE_RUN;
            line_pid_reset();
        } else {
            mode = MODE_STOP;
        }
        menu_state = MENU_OFF;  // 退出菜单回主界面
        return 1;
    }

    // ==========================================
    // MENU_OFF 状态：正常显示，OK进入菜单
    // ==========================================
    if (menu_state == MENU_OFF) {
        if (ev & KEY_EVENT_OK) {
            menu_state  = MENU_MAIN;
            menu_cursor = 0;
            return 1;
        }
        // UP/DOWN 在 MENU_OFF 无菜单操作
        return 0;
    }

    // ==========================================
    // MENU_MAIN 状态：主菜单选择
    // ==========================================
    if (menu_state == MENU_MAIN) {
        if (ev & KEY_EVENT_UP) {
            menu_cursor = (menu_cursor + MENU_ITEM_COUNT - 1) % MENU_ITEM_COUNT;
            return 1;
        }
        if (ev & KEY_EVENT_DOWN) {
            menu_cursor = (menu_cursor + 1) % MENU_ITEM_COUNT;
            return 1;
        }
        if (ev & KEY_EVENT_BACK) {
            menu_state = MENU_OFF;  // 退出菜单
            return 1;
        }
        if (ev & KEY_EVENT_OK) {
            switch (menu_cursor) {
                case MENU_ITEM_RUN:
                    // 切换运行/停止
                    if (mode == MODE_STOP) {
                        mode = MODE_RUN;
                        line_pid_reset();
                    } else {
                        mode = MODE_STOP;
                    }
                    break;

                case MENU_ITEM_SPEED:
                    edit_param  = 0;  // base_speed
                    menu_state  = MENU_EDIT;
                    break;

                case MENU_ITEM_KP:
                    edit_param  = 1;  // kp
                    menu_state  = MENU_EDIT;
                    break;

                case MENU_ITEM_KD:
                    edit_param  = 2;  // kd
                    menu_state  = MENU_EDIT;
                    break;

                case MENU_ITEM_KI:
                    edit_param  = 3;  // ki
                    menu_state  = MENU_EDIT;
                    break;

                case MENU_ITEM_EXIT:
                    menu_state = MENU_OFF;
                    break;
            }
            return 1;
        }
    }

    // ==========================================
    // MENU_EDIT 状态：参数编辑
    // ==========================================
    if (menu_state == MENU_EDIT) {
        float step = 0;

        if (ev & KEY_EVENT_BACK) {
            menu_state = MENU_MAIN;  // 取消，返回菜单
            return 1;
        }

        if (ev & KEY_EVENT_OK) {
            menu_state = MENU_MAIN;  // 确认，返回菜单
            line_pid_reset();
            return 1;
        }

        // 根据编辑参数确定步长
        switch (edit_param) {
            case 0: step = 10.0f;  break;  // base_speed
            case 1: step = 0.01f; break;  // kp
            case 2: step = 0.05f; break;  // kd
            case 3: step = 0.001f; break; // ki
        }

        if (ev & KEY_EVENT_UP) {
            switch (edit_param) {
                case 0: base_speed += step; if (base_speed > 250) base_speed = 250; break;
                case 1: line_kp    += step; if (line_kp > 1.0f)  line_kp = 1.0f;  break;
                case 2: line_kd    += step; if (line_kd > 2.0f)  line_kd = 2.0f;  break;
                case 3: line_ki    += step; if (line_ki > 0.1f)  line_ki = 0.1f;  break;
            }
            return 1;
        }

        if (ev & KEY_EVENT_DOWN) {
            switch (edit_param) {
                case 0: base_speed -= step; if (base_speed < 30)   base_speed = 30;    break;
                case 1: line_kp    -= step; if (line_kp < 0.0f)    line_kp = 0.0f;     break;
                case 2: line_kd    -= step; if (line_kd < 0.0f)    line_kd = 0.0f;     break;
                case 3: line_ki    -= step; if (line_ki < 0.0f)    line_ki = 0.0f;     break;
            }
            return 1;
        }
    }

    return 0;
}
