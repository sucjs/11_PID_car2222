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

// 五档速度 (mm/s)：最慢弯 → 小弯 → 直行 → 快弯外轮 → 最快弯外轮
// 索引:   [0]     [1]    [2]    [3]      [4]
float target_speed_5[] = {125, 175, 200, 400, 500};

void adjust_motor()
{
    huidu_get_value();

    // ---- 全部在白线上（8路全白）→ 到达终点/十字路口，停车 ----
    if (huidu_value[0] == 1 && huidu_value[1] == 1 && huidu_value[2] == 1 && huidu_value[3] == 1 &&
        huidu_value[4] == 1 && huidu_value[5] == 1 && huidu_value[6] == 1 && huidu_value[7] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
        return;
    }

    // ---- 完全离开线（8路全黑）→ 保持直行，不改变方向 ----
    if (huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 && huidu_value[3] == 0 &&
        huidu_value[4] == 0 && huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        target_speed_1 = target_speed_5[2];    // 200 mm/s 基准直行
        target_speed_2 = target_speed_5[2];
        return;
    }

    // ---- 只有中间两个传感器 (S3,S4) 检测到线 → 直行 ----
    if (huidu_value[3] == 1 && huidu_value[4] == 1 &&
        huidu_value[2] == 0 && huidu_value[5] == 0)
    {
        target_speed_1 = target_speed_5[2];    // 200 mm/s 基准直行
        target_speed_2 = target_speed_5[2];
        return;
    }

    // ---- 线偏左：S2,S3 检测到线 → 左轮减速，右轮加速 → 左转 ----
    if (huidu_value[2] == 1 && huidu_value[3] == 1 &&
        huidu_value[1] == 0 && huidu_value[4] == 0)
    {
        target_speed_1 = target_speed_5[2];    // 左轮 200（慢）
        target_speed_2 = target_speed_5[3];    // 右轮 400（快）→ 左转
        return;
    }

    // ---- 线更偏左：S1,S2 检测到线 → 更大左转 ----
    if (huidu_value[1] == 1 && huidu_value[2] == 1 &&
        huidu_value[0] == 0 && huidu_value[3] == 0)
    {
        target_speed_1 = target_speed_5[1];    // 左轮 175（更慢）
        target_speed_2 = target_speed_5[3];    // 右轮 400（快）→ 急左转
        return;
    }

    // ---- 线在最左边：只有 S0 检测到线 → 最大左转 ----
    if (huidu_value[0] == 1 &&
        huidu_value[1] == 0 && huidu_value[2] == 0 &&
        huidu_value[3] == 0)
    {
        target_speed_1 = target_speed_5[0];    // 左轮 125（最慢）
        target_speed_2 = target_speed_5[3];    // 右轮 400（快）→ 最大左转
        return;
    }

    // ---- 线偏右：S4,S5 检测到线 → 右轮减速，左轮加速 → 右转 ----
    if (huidu_value[4] == 1 && huidu_value[5] == 1 &&
        huidu_value[3] == 0 && huidu_value[6] == 0)
    {
        target_speed_1 = target_speed_5[3];    // 左轮 400（快）
        target_speed_2 = target_speed_5[2];    // 右轮 200（慢）→ 右转
        return;
    }

    // ---- 线更偏右：S5,S6 检测到线 → 更大右转 ----
    if (huidu_value[5] == 1 && huidu_value[6] == 1 &&
        huidu_value[4] == 0 && huidu_value[7] == 0)
    {
        target_speed_1 = target_speed_5[3];    // 左轮 400（快）
        target_speed_2 = target_speed_5[1];    // 右轮 175（更慢）→ 急右转
        return;
    }

    // ---- 线在最右边：只有 S7 检测到线 → 最大右转 ----
    if (huidu_value[7] == 1 &&
        huidu_value[4] == 0 && huidu_value[5] == 0 && huidu_value[6] == 0)
    {
        target_speed_1 = target_speed_5[3];    // 左轮 400（快）
        target_speed_2 = target_speed_5[0];    // 右轮 125（最慢）→ 最大右转
        return;
    }

    // ---- 兜底：其他未匹配的传感器组合 → 保持当前目标速度不变 ----
    // （不修改 target_speed_1/2，沿用上一周期的值，避免误判导致停车）
}
