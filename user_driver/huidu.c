#include "huidu.h"

// ==================== 循迹参数（可根据实际赛道调整） ====================

#define BASE_SPEED   100.0f   // 直行基准速度 mm/s（慢速，稳）
#define TURN_GAIN     22.0f   // 转向增益（越大转弯越猛）
#define MIN_SPEED      35.0f   // 最低轮速 mm/s（不能为0，否则单边停转）
#define MAX_SPEED     180.0f   // 最高轮速 mm/s

// ==================== 传感器读取 ====================

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

// ==================== 质心法循迹控制 ====================
//
// 传感器布局（俯视）：
//   S0   S1   S2   S3   S4   S5   S6   S7
//   左 ←――――――――――― 中心 ―――――――――――――→ 右
//   权重 0    1    2    3    4    5    6    7
//
// 线位置（质心）= Σ(i × Si) / Σ(Si)，范围 0.0~7.0，中心 = 3.5
// 偏差 > 0 → 线偏左 → 左转（左轮慢、右轮快）
// 偏差 < 0 → 线偏右 → 右转（右轮慢、左轮快）

extern float target_speed_1;
extern float target_speed_2;

void adjust_motor()
{
    uint8_t i;

    huidu_get_value();

    // ---- 统计有多少传感器检测到线（值=1表示踩线） ----
    uint8_t sum = 0;
    for (i = 0; i < 8; i++) {
        sum += huidu_value[i];
    }

    // ---- 8路全白 = 到达宽线/路口/终点 → 停车 ----
    if (sum == 8) {
        target_speed_1 = 0;
        target_speed_2 = 0;
        return;
    }

    // ---- 8路全黑 = 完全丢线 → 保持直行，慢速前进直到重新找到线 ----
    if (sum == 0) {
        target_speed_1 = BASE_SPEED * 0.6f;  // 60 mm/s 慢速找线
        target_speed_2 = BASE_SPEED * 0.6f;
        return;
    }

    // ---- 正常情况：计算质心位置 ----
    // 加权求和：每个传感器位置 × 是否踩线（0或1）
    uint16_t weighted_sum = 0;
    for (i = 0; i < 8; i++) {
        weighted_sum += i * huidu_value[i];
    }
    float position = (float)weighted_sum / sum;  // 0.0(最左) ~ 7.0(最右)

    // 偏差 = 中心位置(3.5) - 实际位置
    // 正值 → 线在左边 → 需要左转
    // 负值 → 线在右边 → 需要右转
    float error = 3.5f - position;

    // ---- 差速转向：左轮和右轮按偏差反向调节 ----
    // 线偏左(error>0)：左轮减速、右轮加速 → 小车左转追线
    // 线偏右(error<0)：左轮加速、右轮减速 → 小车右转追线
    target_speed_1 = BASE_SPEED - error * TURN_GAIN;  // 左轮
    target_speed_2 = BASE_SPEED + error * TURN_GAIN;  // 右轮

    // ---- 限幅：防止速度为0或过大 ----
    if (target_speed_1 > MAX_SPEED) target_speed_1 = MAX_SPEED;
    if (target_speed_1 < MIN_SPEED) target_speed_1 = MIN_SPEED;
    if (target_speed_2 > MAX_SPEED) target_speed_2 = MAX_SPEED;
    if (target_speed_2 < MIN_SPEED) target_speed_2 = MIN_SPEED;
}
