#include "motor.h"

// ==================== 电机初始化 ====================

void motor_init(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 初始化为惰行（两个引脚都 LOW），避免刹车抱死
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
        // 立即设置方向为正转
        motor_set_direction(1, 1);
    }
    else if (motor_id == 2) {
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
        // 立即设置方向为正转
        motor_set_direction(2, 1);
    }
    // 启动 PWM 定时器（每次调用都确保在运行）
    DL_Timer_startCounter(PWMAB_INST);
}

// PID 定时器启动 —— 必须在两个电机都初始化完成后调用一次
void motor_pid_start(void)
{
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

// ==================== 占空比限幅 ====================

int32_t limit_duty(int32_t duty)
{
    if (duty > 4000) {
        duty = 4000;
    }
    if (duty < 0) {
        duty = 0;
    }
    return duty;
}

// ==================== PWM 占空比设置 ====================

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    if (duty > 4000) {
        duty = 4000;
    }
    if (motor_id == 1) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    }
    else if (motor_id == 2) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

// ==================== 方向控制 ====================
// direction: 0 = 惰行停止，1 = 正转，2 = 反转
// 注意：正转/反转的逻辑已根据实际轮子接线交换

void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1) {
        if (direction == 0) {
            // 惰行停止
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 1) {
            // 正转：AIN1=LOW, AIN2=HIGH
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 2) {
            // 反转：AIN1=HIGH, AIN2=LOW
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if (motor_id == 2) {
        if (direction == 0) {
            // 惰行停止
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 1) {
            // 正转：BIN1=LOW, BIN2=HIGH
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 2) {
            // 反转：BIN1=HIGH, BIN2=LOW
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

// ==================== 速度计算 ====================

extern uint32_t counter_1_A;
float speed_1 = 0;

extern uint32_t counter_2_A;
float speed_2 = 0;

void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 轮速 mm/s = (脉冲数 / 编码器线数) * π * 轮径 * 100
        // 100 = 1/Ts = 1/0.01s (PID 周期 10ms)
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_1_A = 0;
    }
    if (motor_id == 2) {
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_2_A = 0;
    }
}

// ==================== 位置式 PI 控制器 ====================
// 公式: duty = kp * error + ki * ∫error
// 抗积分饱和：积分项被限幅在 ±4000 内

float kp = 1.5;    // 比例系数（低速需较大值以克服静摩擦力）
float ki = 0.4;    // 积分系数（消除稳态误差，低速时适当增大）

int32_t PWM_1_duty = 0;
float target_speed_1 = 0;    // 目标速度 mm/s（main.c 中赋初值）
float integral_1 = 0;        // 电机1 积分累加

int32_t PWM_2_duty = 0;
float target_speed_2 = 0;    // 目标速度 mm/s（main.c 中赋初值）
float integral_2 = 0;        // 电机2 积分累加

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    int32_t output;

    if (motor_id == 1) {
        error = target_speed_1 - speed_1;

        // 积分累加
        integral_1 += ki * error;
        // 抗积分饱和：积分项限幅
        if (integral_1 > 4000.0f) integral_1 = 4000.0f;
        if (integral_1 < -2000.0f) integral_1 = -2000.0f;

        // 位置式 PI 输出
        output = (int32_t)(kp * error + integral_1);
        PWM_1_duty = limit_duty(output);

        // 输出饱和时回退积分（抗饱和）
        if (output != PWM_1_duty) {
            integral_1 -= ki * error;
        }

        motor_set_duty(motor_id, (uint32_t)PWM_1_duty);
    }

    if (motor_id == 2) {
        error = target_speed_2 - speed_2;

        // 积分累加
        integral_2 += ki * error;
        // 抗积分饱和：积分项限幅
        if (integral_2 > 4000.0f) integral_2 = 4000.0f;
        if (integral_2 < -2000.0f) integral_2 = -2000.0f;

        // 位置式 PI 输出
        output = (int32_t)(kp * error + integral_2);
        PWM_2_duty = limit_duty(output);

        // 输出饱和时回退积分（抗饱和）
        if (output != PWM_2_duty) {
            integral_2 -= ki * error;
        }

        motor_set_duty(motor_id, (uint32_t)PWM_2_duty);
    }
}

// ==================== PID 定时器中断（每 10ms 触发一次） ====================

void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        adjust_motor();        // 1. 读取灰度传感器，更新目标速度
        calculate_speed(1);    // 2. 计算电机1当前速度（清零计数器）
        calculate_speed(2);    // 3. 计算电机2当前速度（清零计数器）
        DC_MOTOR_PID(1);       // 4. PID 控制电机1
        DC_MOTOR_PID(2);       // 5. PID 控制电机2
        break;

    default:
        break;
    }
}
