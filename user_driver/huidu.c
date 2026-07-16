#include "huidu.h"

uint8_t huidu_value[] = {0, 0, 0, 0, 0, 0, 0, 0};

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio) {
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if((high_bits & gpio) != 0) return 1;
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

float target_speed_5[] = {125, 175, 200, 400, 500};

void adjust_motor()
{
    huidu_get_value();
    // 如果完全没有线 (8路全黑)
    if(huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 && huidu_value[3] == 0 &&
       huidu_value[4] == 0 && huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
        target_speed_1 = min_speed;
        target_speed_2 = min_speed;
    }
    // 如果全部检测到线 (8路全白) - 停车
    else if(huidu_value[0] == 1 && huidu_value[1] == 1 && huidu_value[2] == 1 && huidu_value[3] == 1 &&
            huidu_value[4] == 1 && huidu_value[5] == 1 && huidu_value[6] == 1 && huidu_value[7] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }
    // 只有中间检测到线 - 直行
    else if(huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 &&
            huidu_value[3] == 1 && huidu_value[4] == 1 &&
            huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
        target_speed_1 = min_speed;
        target_speed_2 = min_speed;
    }
    // 线偏左 - 左转 (S1,S2检测到线)
    else if(huidu_value[0] == 0 && huidu_value[1] == 1 && huidu_value[2] == 1 &&
            huidu_value[3] == 0 && huidu_value[4] == 0 &&
            huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        target_speed_1 = target_speed_5[2];
        target_speed_2 = target_speed_5[3];
    }
    // 线更偏左 (S0,S1检测到线)
    else if(huidu_value[0] == 1 && huidu_value[1] == 1 &&
            huidu_value[2] == 0 && huidu_value[3] == 0 && huidu_value[4] == 0 &&
            huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        target_speed_1 = target_speed_5[1];
        target_speed_2 = target_speed_5[3];
    }
    // 线在最左边 (只有S0)
    else if(huidu_value[0] == 1 &&
            huidu_value[1] == 0 && huidu_value[2] == 0 &&
            huidu_value[3] == 0 && huidu_value[4] == 0 &&
            huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 0)
    {
        target_speed_1 = target_speed_5[0];
        target_speed_2 = target_speed_5[3];
    }
    // 线偏右 (S5,S6检测到线)
    else if(huidu_value[0] == 0 && huidu_value[1] == 0 &&
            huidu_value[2] == 0 && huidu_value[3] == 0 && huidu_value[4] == 0 &&
            huidu_value[5] == 1 && huidu_value[6] == 1 && huidu_value[7] == 0)
    {
        target_speed_1 = target_speed_5[3];
        target_speed_2 = target_speed_5[2];
    }
    // 线更偏右 (S6,S7检测到线)
    else if(huidu_value[0] == 0 && huidu_value[1] == 0 &&
            huidu_value[2] == 0 && huidu_value[3] == 0 && huidu_value[4] == 0 &&
            huidu_value[5] == 0 && huidu_value[6] == 1 && huidu_value[7] == 1)
    {
        target_speed_1 = target_speed_5[3];
        target_speed_2 = target_speed_5[1];
    }
    // 线在最右边 (只有S7)
    else if(huidu_value[0] == 0 && huidu_value[1] == 0 &&
            huidu_value[2] == 0 && huidu_value[3] == 0 && huidu_value[4] == 0 &&
            huidu_value[5] == 0 && huidu_value[6] == 0 && huidu_value[7] == 1)
    {
        target_speed_1 = target_speed_5[3];
        target_speed_2 = target_speed_5[0];
    }
}
