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

// 七档速度：更细粒度，平滑过渡
// 索引:       [0]   [1]   [2]   [3]   [4]   [5]   [6]
float spd[] = { 50,   65,   80,  100,  140,  180,  220};

void adjust_motor()
{
    uint8_t i;
    uint8_t s[8];  // 反转后的传感器值：s[i]=1 表示踩线

    huidu_get_value();

    // 反转：黑线赛道传感器输出 0=踩线，但下面条件都按 1=踩线 写的
    for (i = 0; i < 8; i++) {
        s[i] = !huidu_value[i];
    }

    // =====================================================
    // 边界情况
    // =====================================================

    // 8路全黑：完全没有线 → 直行慢速找线
    if (s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0 &&
        s[4] == 0 && s[5] == 0 && s[6] == 0 && s[7] == 0)
    {
        target_speed_1 = spd[1];  // 65
        target_speed_2 = spd[1];
    }
    // 8路全白：全部检测到线 → 停车
    else if (s[0] == 1 && s[1] == 1 && s[2] == 1 && s[3] == 1 &&
             s[4] == 1 && s[5] == 1 && s[6] == 1 && s[7] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }

    // =====================================================
    // 直行状态
    // =====================================================

    // 直行：S3,S4 踩线（2传感器，线宽适中）
    else if (s[2] == 0 && s[3] == 1 && s[4] == 1 && s[5] == 0)
    {
        target_speed_1 = spd[3];  // 100
        target_speed_2 = spd[3];
    }
    // 直行：仅 S3 踩线（线很窄，偏左一点）
    else if (s[2] == 0 && s[3] == 1 && s[4] == 0)
    {
        target_speed_1 = spd[3];  // 100
        target_speed_2 = spd[3];
    }
    // 直行：仅 S4 踩线（线很窄，偏右一点）
    else if (s[3] == 0 && s[4] == 1 && s[5] == 0)
    {
        target_speed_1 = spd[3];  // 100
        target_speed_2 = spd[3];
    }
    // 宽线偏左：S2,S3,S4 同时踩线 → 微左转
    else if (s[1] == 0 && s[2] == 1 && s[3] == 1 && s[4] == 1 && s[5] == 0)
    {
        target_speed_1 = spd[3];  // 左轮 100
        target_speed_2 = spd[4];  // 右轮 140
    }
    // 宽线偏右：S3,S4,S5 同时踩线 → 微右转
    else if (s[2] == 0 && s[3] == 1 && s[4] == 1 && s[5] == 1 && s[6] == 0)
    {
        target_speed_1 = spd[4];  // 左轮 140
        target_speed_2 = spd[3];  // 右轮 100
    }

    // =====================================================
    // 左转状态（线在左边 → 左轮慢、右轮快）
    // =====================================================

    // 微微偏左：S2,S3 踩线
    else if (s[1] == 0 && s[2] == 1 && s[3] == 1 && s[4] == 0)
    {
        target_speed_1 = spd[3];  // 左轮 100
        target_speed_2 = spd[4];  // 右轮 140
    }
    // 偏左：S1,S2 踩线
    else if (s[0] == 0 && s[1] == 1 && s[2] == 1 && s[3] == 0)
    {
        target_speed_1 = spd[2];  // 左轮 80
        target_speed_2 = spd[5];  // 右轮 180
    }
    // 更偏左：S0,S1 踩线
    else if (s[0] == 1 && s[1] == 1 &&
             s[2] == 0 && s[3] == 0)
    {
        target_speed_1 = spd[1];  // 左轮 65
        target_speed_2 = spd[6];  // 右轮 220
    }
    // 最左边：只有 S0 踩线
    else if (s[0] == 1 && s[1] == 0 &&
             s[2] == 0 && s[3] == 0 && s[4] == 0)
    {
        target_speed_1 = spd[0];  // 左轮 50
        target_speed_2 = spd[6];  // 右轮 220
    }

    // =====================================================
    // 右转状态（线在右边 → 右轮慢、左轮快）
    // =====================================================

    // 微微偏右：S4,S5 踩线
    else if (s[3] == 0 && s[4] == 1 && s[5] == 1 && s[6] == 0)
    {
        target_speed_1 = spd[4];  // 左轮 140
        target_speed_2 = spd[3];  // 右轮 100
    }
    // 偏右：S5,S6 踩线
    else if (s[4] == 0 && s[5] == 1 && s[6] == 1 && s[7] == 0)
    {
        target_speed_1 = spd[5];  // 左轮 180
        target_speed_2 = spd[2];  // 右轮 80
    }
    // 更偏右：S6,S7 踩线
    else if (s[4] == 0 && s[5] == 0 &&
             s[6] == 1 && s[7] == 1)
    {
        target_speed_1 = spd[6];  // 左轮 220
        target_speed_2 = spd[1];  // 右轮 65
    }
    // 最右边：只有 S7 踩线
    else if (s[4] == 0 && s[5] == 0 &&
             s[6] == 0 && s[7] == 1)
    {
        target_speed_1 = spd[6];  // 左轮 220
        target_speed_2 = spd[0];  // 右轮 50
    }

    // =====================================================
    // 兜底：未匹配 → 保持上一周期速度不变
    // =====================================================
    else
    {
        // 不改变 target_speed_1/2
    }
}
