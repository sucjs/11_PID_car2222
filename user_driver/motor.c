#include "motor.h"

void motor_init(uint8_t motor_id)
{
    if(motor_id == 1){
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(PWMAB_INST);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

// 限幅函数
int32_t limit_duty(int32_t duty)
{
    if(duty > 4000) duty = 4000;
    if(duty < 0)    duty = 0;
    return duty;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty(duty);
    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

// direction: 0 停止，1 正转，2 反转
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

// -----------------------------------------------------------
// 编码器 + 速度计算
// -----------------------------------------------------------
extern uint32_t counter_1_A;
extern uint32_t counter_2_A;
float speed_1 = 0;
float speed_2 = 0;

// 编码器原始计数值（调试用）
uint32_t raw_enc_1 = 0;
uint32_t raw_enc_2 = 0;

void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        raw_enc_1 = counter_1_A;
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_1_A = 0;
    }
    if (motor_id == 2) {
        raw_enc_2 = counter_2_A;
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100;
        counter_2_A = 0;
    }
}

// -----------------------------------------------------------
// 速度 PI 控制器（位置式，带积分分离和抗饱和）
// -----------------------------------------------------------
#define SPD_KP   1.5f    // 速度比例系数
#define SPD_KI   0.6f    // 速度积分系数
#define SPD_KC   0.8f    // 积分分离阈值系数

float target_speed_1 = 0;
float target_speed_2 = 0;

static float integral_1 = 0;
static float integral_2 = 0;
static float last_err_1 = 0;
static float last_err_2 = 0;

int32_t PWM_1_duty = 0;
int32_t PWM_2_duty = 0;

void motor_pid_reset(void)
{
    integral_1  = 0;
    integral_2  = 0;
    last_err_1  = 0;
    last_err_2  = 0;
    PWM_1_duty  = 0;
    PWM_2_duty  = 0;
}

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error, duty_f;
    int32_t duty;

    if (motor_id == 1) {
        error = target_speed_1 - speed_1;

        // 积分分离：误差大时不积分，防止饱和
        if (error < target_speed_1 * SPD_KC && error > -target_speed_1 * SPD_KC) {
            integral_1 += error;
            // 积分限幅
            if (integral_1 >  2000.0f) integral_1 =  2000.0f;
            if (integral_1 < -500.0f)  integral_1 = -500.0f;
        } else {
            integral_1 *= 0.9f;  // 缓慢泄放
        }

        // 位置式 PI：Duty = Kp*e + Ki*∫e
        duty_f = SPD_KP * error + SPD_KI * integral_1;

        // 目标速度为0时直接停
        if (target_speed_1 <= 0.5f) {
            duty_f = 0;
            integral_1 = 0;
        }

        duty = (int32_t)duty_f;
        if (duty < 0) duty = 0;
        PWM_1_duty = limit_duty(duty);
        last_err_1 = error;
        motor_set_duty(1, PWM_1_duty);
    }

    if (motor_id == 2) {
        error = target_speed_2 - speed_2;

        if (error < target_speed_2 * SPD_KC && error > -target_speed_2 * SPD_KC) {
            integral_2 += error;
            if (integral_2 >  2000.0f) integral_2 =  2000.0f;
            if (integral_2 < -500.0f)  integral_2 = -500.0f;
        } else {
            integral_2 *= 0.9f;
        }

        duty_f = SPD_KP * error + SPD_KI * integral_2;

        if (target_speed_2 <= 0.5f) {
            duty_f = 0;
            integral_2 = 0;
        }

        duty = (int32_t)duty_f;
        if (duty < 0) duty = 0;
        PWM_2_duty = limit_duty(duty);
        last_err_2 = error;
        motor_set_duty(2, PWM_2_duty);
    }
}

// -----------------------------------------------------------
// 定时器 ISR：10ms 周期
// -----------------------------------------------------------
void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        adjust_motor();       // 外环：灰度位置PID → target_speed
        calculate_speed(1);   // 读编码器1
        DC_MOTOR_PID(1);      // 内环：速度PI → PWM_duty
        calculate_speed(2);   // 读编码器2
        DC_MOTOR_PID(2);      // 内环：速度PI → PWM_duty
        break;

    default:
        break;
    }
}
