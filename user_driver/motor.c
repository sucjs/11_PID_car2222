#include "motor.h"

// ==================== 电机初始化 ====================

void motor_init(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 方向引脚初始化为 LOW（惰行）
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
    }
    else if (motor_id == 2) {
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(PWMAB_INST);
}

// PID 定时器：两个电机都初始化完成后调用一次
void motor_pid_start(void)
{
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

// ==================== 占空比限幅 ====================

int32_t limit_duty(int32_t duty)
{
    if (duty > 4000) duty = 4000;
    if (duty < 0)    duty = 0;
    return duty;
}

// ==================== PWM 占空比设置 ====================

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    if (duty > 4000) duty = 4000;

    if (motor_id == 1) {
        DL_TimerG_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    }
    else if (motor_id == 2) {
        DL_TimerG_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

// ==================== 方向控制 ====================
// direction: 0 = 惰行，1 = 正转，2 = 反转
//
// TB6612 真值表（单通道）：
//   IN1=LOW  IN2=LOW  → 惰行/停止
//   IN1=HIGH IN2=LOW  → 正转
//   IN1=LOW  IN2=HIGH → 反转
//   IN1=HIGH IN2=HIGH → 刹车
//
// 如果轮子转动方向相反，交换 direction==1 和 direction==2 的代码块，
// 或者直接交换电机 M+ M- 的物理接线。

void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1) {
        if (direction == 0) {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 1) {
            // 正转：IN1=HIGH, IN2=LOW
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if (direction == 2) {
            // 反转：IN1=LOW, IN2=HIGH
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if (motor_id == 2) {
        if (direction == 0) {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 1) {
            // 正转：IN1=HIGH, IN2=LOW
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if (direction == 2) {
            // 反转：IN1=LOW, IN2=HIGH
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
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
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_1_A = 0;
    }
    if (motor_id == 2) {
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_2_A = 0;
    }
}

// ==================== 位置式 PI 控制器 ====================

float kp = 1.5f;
float ki = 0.4f;

int32_t PWM_1_duty = 0;
float target_speed_1 = 0;
float integral_1 = 0;

int32_t PWM_2_duty = 0;
float target_speed_2 = 0;
float integral_2 = 0;

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    int32_t output;

    if (motor_id == 1) {
        error = target_speed_1 - speed_1;

        integral_1 += ki * error;
        if (integral_1 > 4000.0f)  integral_1 = 4000.0f;
        if (integral_1 < -2000.0f) integral_1 = -2000.0f;

        output = (int32_t)(kp * error + integral_1);
        PWM_1_duty = limit_duty(output);
        if (output != PWM_1_duty) {
            integral_1 -= ki * error;
        }

        motor_set_duty(1, (uint32_t)PWM_1_duty);
    }

    if (motor_id == 2) {
        error = target_speed_2 - speed_2;

        integral_2 += ki * error;
        if (integral_2 > 4000.0f)  integral_2 = 4000.0f;
        if (integral_2 < -2000.0f) integral_2 = -2000.0f;

        output = (int32_t)(kp * error + integral_2);
        PWM_2_duty = limit_duty(output);
        if (output != PWM_2_duty) {
            integral_2 -= ki * error;
        }

        motor_set_duty(2, (uint32_t)PWM_2_duty);
    }
}

// ==================== PID 定时器中断（每 10ms） ====================

void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        adjust_motor();
        calculate_speed(1);
        calculate_speed(2);
        DC_MOTOR_PID(1);
        DC_MOTOR_PID(2);
        break;

    default:
        break;
    }
}
