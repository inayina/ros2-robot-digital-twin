#include "led_alarm.h"
#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim2;

static uint8_t current_state = 0;
static uint8_t blink_tick = 0;

void LED_Alarm_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

void LED_Alarm_SetState(uint8_t state)
{
    current_state = state;
}

void LED_Alarm_Update(void)
{
    blink_tick++;
    
    uint32_t green = 0, yellow = 0, red = 0;
    
    switch (current_state) {
        case 0:
            green = 300;
            yellow = 0;
            red = 0;
            break;
        case 1:
            green = 0;
            yellow = (blink_tick % 20 < 10) ? 300 : 0;
            red = 0;
            break;
        case 2:
            green = 0;
            yellow = 0;
            red = (blink_tick % 5 < 3) ? 300 : 0;
            break;
        case 3:
            green = 0;
            yellow = 0;
            red = 400;
            break;
        default:
            green = 0;
            yellow = 0;
            red = 0;
            break;
    }
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, green);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, red);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, yellow);
}

uint8_t LED_Alarm_GetState(void)
{
    return current_state;
}