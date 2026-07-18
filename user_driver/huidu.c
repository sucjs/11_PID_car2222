#include "huidu.h"

uint8_t huidu_value[] = {0, 0, 0, 0, 0, 0, 0, 0};

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio) {
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((high_bits & gpio) != 0) return 1;
    else return 0;
}

void huidu_get_value()
{
    huidu_value[0] = get_gpio_state(HUIDU_S0_PORT, HUIDU_S0_PIN);
    huidu_value[1] = get_gpio_state(HUIDU_S1_PORT, HUIDU_S1_PIN);
    huidu_value[2] = get_gpio_state(HUIDU_S2_PORT, HUIDU_S2_PIN);
    huidu_value[3] = get_gpio_state(HUIDU_S3_PORT, HUIDU_S3_PIN);
    huidu_value[4] = get_gpio_state(HUIDU_S4_PORT, HUIDU_S4_PIN);
    huidu_value[5] = get_gpio_state(HUIDU_S5_PORT, HUIDU_S5_PIN);
    huidu_value[6] = get_gpio_state(HUIDU_S6_PORT, HUIDU_S6_PIN);
    huidu_value[7] = get_gpio_state(HUIDU_S7_PORT, HUIDU_S7_PIN);
}

extern float target_speed_1;
extern float target_speed_2;

// 五档速度 (mm/s)，比原来降低一半，避免疯转
// 索引:       [0]   [1]   [2]   [3]   [4]
float spd[] = { 60,   80,  100,  180,  220};

void adjust_motor()
{
    huidu_get_value();

    // ---- 8路全黑：完全没有线 → 直行慢速找线 ----
    if (huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 && huidu_value[3] == 0 &&
        huidu_value[4] == 0 && huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        target_speed_1 = spd[1];  // 80
        target_speed_2 = spd[1];
    }
    // ---- 8路全白：全部检测到线 → 停车 ----
    else if (huidu_value[0] == 1 && huidu_value[1] == 1 && huidu_value[2] == 1 && huidu_value[3] == 1 &&
             huidu_value[4] == 1 && huidu_value[5] == 1 && huidu_value[6] == 1 && huidu_value[7] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }
    // ---- 直行：只有中间 S3,S4 踩线 ----
    else if (huidu_value[2] == 0 && huidu_value[3] == 1 && huidu_value[4] == 1 && huidu_value[5] == 0)
    {
        target_speed_1 = spd[2];  // 100
        target_speed_2 = spd[2];
    }
    // ---- 线偏左：S1,S2 踩线 → 左轮慢、右轮快 ----
    else if (huidu_value[1] == 1 && huidu_value[2] == 1 &&
             huidu_value[3] == 0 && huidu_value[4] == 0)
    {
        target_speed_1 = spd[2];  // 左轮 100（慢）
        target_speed_2 = spd[3];  // 右轮 180（快）
    }
    // ---- 线更偏左：S0,S1 踩线 ----
    else if (huidu_value[0] == 1 && huidu_value[1] == 1 &&
             huidu_value[2] == 0 && huidu_value[3] == 0)
    {
        target_speed_1 = spd[1];  // 左轮 80（更慢）
        target_speed_2 = spd[3];  // 右轮 180（快）
    }
    // ---- 线在最左边：只有 S0 踩线 ----
    else if (huidu_value[0] == 1 && huidu_value[1] == 0 &&
             huidu_value[2] == 0 && huidu_value[3] == 0 && huidu_value[4] == 0)
    {
        target_speed_1 = spd[0];  // 左轮 60（最慢）
        target_speed_2 = spd[3];  // 右轮 180（快）
    }
    // ---- 线偏右：S5,S6 踩线 → 右轮慢、左轮快 ----
    else if (huidu_value[4] == 0 && huidu_value[5] == 1 && huidu_value[6] == 1 && huidu_value[7] == 0)
    {
        target_speed_1 = spd[3];  // 左轮 180（快）
        target_speed_2 = spd[2];  // 右轮 100（慢）
    }
    // ---- 线更偏右：S6,S7 踩线 ----
    else if (huidu_value[5] == 0 && huidu_value[6] == 1 && huidu_value[7] == 1)
    {
        target_speed_1 = spd[3];  // 左轮 180（快）
        target_speed_2 = spd[1];  // 右轮 80（更慢）
    }
    // ---- 线在最右边：只有 S7 踩线 ----
    else if (huidu_value[4] == 0 && huidu_value[5] == 0 &&
             huidu_value[6] == 0 && huidu_value[7] == 1)
    {
        target_speed_1 = spd[3];  // 左轮 180（快）
        target_speed_2 = spd[0];  // 右轮 60（最慢）
    }
    // ---- 兜底：未匹配的传感器状态 → 保持直行，不改变目标速度 ----
    else
    {
        // 什么都不做，让 PID 继续用上一周期的速度
    }
}
