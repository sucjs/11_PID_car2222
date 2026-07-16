#include "huidu.h"

uint8_t huidu_value[] = {0, 0, 0, 0, 0};

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio) {
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio); 
    if((high_bits & gpio) != 0) return 1;
    else return 0;
}

void huidu_get_value()
{
    huidu_value[0] = get_gpio_state(HUIDU_L2_PORT, HUIDU_L2_PIN);
    huidu_value[1] = get_gpio_state(HUIDU_L1_PORT, HUIDU_L1_PIN);
    huidu_value[2] = get_gpio_state(HUIDU_M_PORT, HUIDU_M_PIN);
    huidu_value[3] = get_gpio_state(HUIDU_R1_PORT, HUIDU_R1_PIN);
    huidu_value[4] = get_gpio_state(HUIDU_R2_PORT, HUIDU_R2_PIN);
}
extern float target_speed_1;
extern float target_speed_2;

float target_speed_5[] = {125, 175, 200, 400, 500};

void adjust_motor()
{
    huidu_get_value();
    // 如果完全没有线
    if(huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 0 && huidu_value[3] == 0 && huidu_value[4] == 0)
    {
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
        target_speed_1 = min_speed;
        target_speed_2 = min_speed;
    }
    else if(huidu_value[0] == 1 && huidu_value[1] == 1 && huidu_value[2] == 1 && huidu_value[3] == 1 && huidu_value[4] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }
    else if(huidu_value[0] == 0 && huidu_value[1] == 0 && huidu_value[2] == 1 && huidu_value[3] == 0 && huidu_value[4] == 0)
    {
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
        target_speed_1 = min_speed;
        target_speed_2 = min_speed;
    }
    else if(huidu_value[0] == 0 && huidu_value[1] == 1)
    {
        target_speed_1 = target_speed_5[2];
        target_speed_2 = target_speed_5[3];
    }
    else if(huidu_value[0] == 1 && huidu_value[1] == 1)
    {
        target_speed_1 = target_speed_5[1];
        target_speed_2 = target_speed_5[3];
    }
    else if(huidu_value[0] == 1)
    {
        target_speed_1 = target_speed_5[0];
        target_speed_2 = target_speed_5[3];
    }
    else if(huidu_value[3] == 1 && huidu_value[4] == 0)
    {
        target_speed_1 = target_speed_5[3];
        target_speed_2 = target_speed_5[2];
    }
    else if(huidu_value[3] == 1 && huidu_value[4] == 1)
    {
        target_speed_1 = target_speed_5[3];
        target_speed_2 = target_speed_5[1];
    }
    else if(huidu_value[4] == 1)
    {
        target_speed_1 = target_speed_5[3];
        target_speed_2 = target_speed_5[0];
    }
}
