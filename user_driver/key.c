#include "key.h"
#include "huidu.h"

extern int mode;  // 定义在 main.c

uint8_t get_key_state(uint32_t key) {
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key);
    if((high_bits & key) != 0) return 1;
    else return 0;
}

uint32_t counter_1_A = 0;
uint32_t counter_2_A = 0;

void GROUP1_IRQHandler()
{
    // ---- 处理 GPIOB 中断（按键） ----
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_KEY0_IIDX:
        // KEY0: 循环切换模式
        mode = (mode + 1) % MODE_COUNT;
        break;

    case KEY_KEY1_IIDX:
        // KEY1: 调参模式下增大参数，非调参模式切回上一个模式
        if (mode == MODE_TUNE_KP || mode == MODE_TUNE_KD ||
            mode == MODE_TUNE_KI || mode == MODE_TUNE_SPEED) {
            line_pid_tune_param(1);  // 增大参数
        } else {
            mode = (mode + MODE_COUNT - 1) % MODE_COUNT;  // 切到上一个模式
        }
        break;

    default:
        break;
    }

    // ---- 处理 GPIOA 中断（编码器） ----
    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_AA_IIDX:
        counter_1_A ++;
        break;
    case DC_MOTOR_BA_IIDX:
        counter_2_A ++;
        break;

    default:
        break;
    }
}
