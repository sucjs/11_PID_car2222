#include "motor.h"

void motor_init(uint8_t motor_id)
{
    if(motor_id == 1){
        // 初始化为惰行/停止（两个引脚都LOW），避免刹车抱死
        DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(PWMAB_INST);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
    // 初始化完成后立即设置为正转方向（避免主循环延迟500ms才设置方向）
    motor_set_direction(motor_id, 1);
}

// 限幅函数：限制占空比在 0~4000 范围内
int32_t limit_duty(int32_t duty)
{
    if(duty > 4000){
        duty = 4000;
    }
    if(duty < 0){
        duty = 0;
    }
    return duty;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    if(duty > 4000){
        duty = 4000;
    }
    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

// direction: 0 停止（惰行），1 正转，2 反转
// 注意：如果轮子实际转向相反，请交换 direction==1 和 direction==2 的代码块
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            // 停止：两个引脚都 LOW = 惰行
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            // 正转：AIN1=LOW, AIN2=HIGH（修复反转：原来 HIGH/LOW → LOW/HIGH）
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            // 反转：AIN1=HIGH, AIN2=LOW（修复反转：原来 LOW/HIGH → HIGH/LOW）
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            // 停止：两个引脚都 LOW = 惰行
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            // 正转：BIN1=LOW, BIN2=HIGH（修复反转：原来 HIGH/LOW → LOW/HIGH）
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            // 反转：BIN1=HIGH, BIN2=LOW（修复反转：原来 LOW/HIGH → HIGH/LOW）
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}


extern uint32_t counter_1_A;
float speed_1 = 0;

extern uint32_t counter_2_A;
float speed_2 = 0;

void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100; // 轮速 mm/s
        counter_1_A = 0; // 计算完速度后清零计数器
    }
    if (motor_id == 2) {
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100; // 轮速 mm/s
        counter_2_A = 0; // 计算完速度后清零计数器
    }
}

float kp = 0.5; // 比例系数
float ki = 0.4; // 积分系数

int32_t PWM_1_duty = 0;     // 修复：改为 int32_t，避免负向调节时 uint16_t 回绕溢出
float target_speed_1 = 0;   // 目标速度 mm/s（在 main.c 中赋初值）
float last_error_1 = 0;
float current_error_1 = 0;

int32_t PWM_2_duty = 0;     // 修复：改为 int32_t，避免负向调节时 uint16_t 回绕溢出
float target_speed_2 = 0;   // 目标速度 mm/s（在 main.c 中赋初值）
float last_error_2 = 0;
float current_error_2 = 0;

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    if (motor_id == 1) {
        error = target_speed_1 - speed_1;
        current_error_1 = error;
        // 增量式 PI：Δu = kp*(e_k - e_{k-1}) + ki*e_k
        PWM_1_duty += (int32_t)(kp * (current_error_1 - last_error_1) + ki * current_error_1);
        // 限幅防止溢出和负值
        PWM_1_duty = limit_duty(PWM_1_duty);
        last_error_1 = current_error_1;
        motor_set_duty(motor_id, (uint32_t)PWM_1_duty);
    }
    if (motor_id == 2) {
        error = target_speed_2 - speed_2;
        current_error_2 = error;
        // 增量式 PI：Δu = kp*(e_k - e_{k-1}) + ki*e_k
        PWM_2_duty += (int32_t)(kp * (current_error_2 - last_error_2) + ki * current_error_2);
        // 限幅防止溢出和负值
        PWM_2_duty = limit_duty(PWM_2_duty);
        last_error_2 = current_error_2;
        motor_set_duty(motor_id, (uint32_t)PWM_2_duty);
    }
}

void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        adjust_motor();        // 先读取灰度传感器，更新目标速度
        calculate_speed(1);    // 计算电机1当前速度
        DC_MOTOR_PID(1);       // PID控制电机1
        calculate_speed(2);    // 计算电机2当前速度
        DC_MOTOR_PID(2);       // PID控制电机2
        break;

    default:
        break;
    }
}
