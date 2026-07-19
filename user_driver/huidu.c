/*
 * 8路灰度循迹 PID 位置控制器
 * 
 * 算法说明：
 *   1. 读取8路灰度传感器（0=黑线, 1=白底）
 *   2. 取反后计算黑线的加权质心位置 (0~7000, 中心=3500)
 *   3. 位置偏差送入增量式 PID 控制器，输出转向修正量
 *   4. 转向修正叠加到左右轮基础速度上（差速转向）
 * 
 * 级联结构：外环位置PID(本文件) -> 内环速度PI(motor.c)
 */
#include "huidu.h"

// -----------------------------------------------------------
// 全局变量
// -----------------------------------------------------------
uint8_t huidu_value[] = {0, 0, 0, 0, 0, 0, 0, 0};

// PID 参数（可在线调节）
float line_kp    = 0.04f;   // P: 偏差放大系数
float line_kd    = 0.18f;   // D: 阻尼/超前矫正
float line_ki    = 0.002f;  // I: 稳态消除（需防积分饱和）

float base_speed = 120.0f;  // 基础速度 mm/s

// 观测变量
float line_position = 3500.0f;
float line_error    = 0.0f;
float line_turn     = 0.0f;

// PID 内部状态
static float last_error = 0.0f;
static float integral   = 0.0f;

// -----------------------------------------------------------
// 单路传感器读取
// -----------------------------------------------------------
uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    return ((high_bits & gpio) != 0) ? 1 : 0;
}

// -----------------------------------------------------------
// 8路传感器一次性读取
// -----------------------------------------------------------
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

// -----------------------------------------------------------
// 重置 PID 积分和历史（切换模式/起跑时调用）
// -----------------------------------------------------------
void line_pid_reset(void)
{
    last_error = 0.0f;
    integral   = 0.0f;
    line_turn  = 0.0f;
}

// -----------------------------------------------------------
// 外环位置 PID：根据8路传感器值计算差速
// 由定时器 ISR 每 10ms 调用一次
// -----------------------------------------------------------
void adjust_motor()
{
    uint8_t i;
    uint8_t s[8];              // 取反后：1=踩黑线
    int32_t weighted_sum = 0;  // sum(位置权重 * 1000)
    int32_t count         = 0;  // 踩线传感器数 * 1000

    // ---- 第1步：读取传感器并取反 ----
    huidu_get_value();
    for (i = 0; i < 8; i++) {
        // 传感器 0=踩黑线 -> s[i]=1 表示踩线
        s[i] = !huidu_value[i];
    }

    // ---- 第2步：计算黑线加权质心 ----
    // 每个传感器位置权重 = i * 1000 (范围 0~7000, 中心 3500)
    for (i = 0; i < 8; i++) {
        if (s[i]) {
            weighted_sum += (int32_t)i * 1000;
            count += 1000;
        }
    }

    // ---- 第3步：处理边界情况 ----
    extern float target_speed_1;
    extern float target_speed_2;

    if (count == 0) {
        // 所有传感器都在白底上 -> 完全丢线
        // 保持低速直行，等待找回黑线
        target_speed_1 = base_speed * 0.35f;
        target_speed_2 = base_speed * 0.35f;
        line_position = -1.0f;  // 标记丢线
        line_error    = 0.0f;
        return;
    }

    if (count == 8000) {
        // 所有传感器都在黑线上 -> 十字路口或停车线
        // 停车等待（可根据实际需求改为减速通过）
        target_speed_1 = 0.0f;
        target_speed_2 = 0.0f;
        line_position = 3500.0f;
        line_error    = 0.0f;
        line_pid_reset();
        return;
    }

    // ---- 第4步：计算位置偏差 ----
    line_position = (float)weighted_sum / (float)count;  // 0.0 ~ 7000.0
    line_error    = line_position - 3500.0f;             // 负=偏左, 正=偏右

    // ---- 第5步：增量式 PID 控制器 ----
    // 积分项
    integral += line_error;
    // 积分限幅（防饱和）
    const float I_LIMIT = 2000.0f;
    if (integral >  I_LIMIT) integral =  I_LIMIT;
    if (integral < -I_LIMIT) integral = -I_LIMIT;

    // 微分项
    float derivative = line_error - last_error;
    last_error = line_error;

    // PID 输出 = 转向修正量 (正=向右转, 负=向左转)
    line_turn = line_kp * line_error
              + line_kd * derivative
              + line_ki * integral;

    // ---- 第6步：转向修正限幅 ----
    float turn_max = base_speed * 0.8f;
    if (line_turn >  turn_max) line_turn =  turn_max;
    if (line_turn < -turn_max) line_turn = -turn_max;

    // ---- 第7步：差速分配 ----
    // 左轮：基础 - 修正 (线偏左时 error<0 -> turn<0 -> 左轮加速)
    // 右轮：基础 + 修正 (线偏左时 error<0 -> turn<0 -> 右轮减速)
    target_speed_1 = base_speed - line_turn;
    target_speed_2 = base_speed + line_turn;

    // 低速保护：防止一侧完全停转导致不稳定
    float min_speed = 20.0f;
    if (target_speed_1 < min_speed) target_speed_1 = min_speed;
    if (target_speed_2 < min_speed) target_speed_2 = min_speed;

    // 高速上限保护
    float max_speed = 250.0f;
    if (target_speed_1 > max_speed) target_speed_1 = max_speed;
    if (target_speed_2 > max_speed) target_speed_2 = max_speed;
}

// -----------------------------------------------------------
// 按键调参：dir>0 增大参数, dir<0 减小参数
// 在 TUNE 模式下由 key.c 的中断处理函数调用
// -----------------------------------------------------------
void line_pid_tune_param(int dir)
{
    extern int mode;  // 当前模式，定义在 main.c

    switch (mode) {
        case MODE_TUNE_KP:
            line_kp += (dir > 0) ? 0.01f : -0.01f;
            if (line_kp < 0.0f)  line_kp = 0.0f;
            if (line_kp > 1.0f)  line_kp = 1.0f;
            break;

        case MODE_TUNE_KD:
            line_kd += (dir > 0) ? 0.05f : -0.05f;
            if (line_kd < 0.0f)  line_kd = 0.0f;
            if (line_kd > 2.0f)  line_kd = 2.0f;
            break;

        case MODE_TUNE_KI:
            line_ki += (dir > 0) ? 0.001f : -0.001f;
            if (line_ki < 0.0f)  line_ki = 0.0f;
            if (line_ki > 0.1f)  line_ki = 0.1f;
            break;

        case MODE_TUNE_SPEED:
            base_speed += (dir > 0) ? 10.0f : -10.0f;
            if (base_speed < 30.0f)  base_speed = 30.0f;
            if (base_speed > 250.0f) base_speed = 250.0f;
            break;

        default:
            break;
    }
    line_pid_reset();  // 参数改变后重置PID状态
}
