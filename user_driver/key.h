#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

// -----------------------------------------------------------
// 按键事件标志（位掩码，ISR 设置，主循环消费）
// -----------------------------------------------------------
#define KEY_EVENT_UP    0x01   // PB0: 上 / 增大
#define KEY_EVENT_DOWN  0x02   // PB1: 下 / 减小
#define KEY_EVENT_OK    0x04   // PB10: 确认 / 进入
#define KEY_EVENT_BACK  0x08   // PB11: 返回 / 取消
#define KEY_EVENT_RUN   0x10   // PB14: 快捷启动/停止

// 菜单状态
typedef enum {
    MENU_OFF  = 0,  // 正常驾驶显示
    MENU_MAIN = 1,  // 主菜单选择
    MENU_EDIT = 2   // 参数编辑中
} menu_state_t;

// 菜单项索引
#define MENU_ITEM_RUN    0
#define MENU_ITEM_SPEED  1
#define MENU_ITEM_KP     2
#define MENU_ITEM_KD     3
#define MENU_ITEM_KI     4
#define MENU_ITEM_EXIT   5
#define MENU_ITEM_COUNT  6

// 全局状态
extern volatile uint8_t key_event;   // 待处理的按键事件
extern menu_state_t     menu_state;  // 当前菜单状态
extern uint8_t          menu_cursor; // 菜单光标位置
extern uint8_t          edit_param;  // 正在编辑的参数索引 (0=speed,1=kp,2=kd,3=ki)

// 函数
uint8_t get_key_state(uint32_t key);
void    key_event_clear(void);       // 消费按键事件
int     menu_process(void);           // 菜单状态机处理

#endif
