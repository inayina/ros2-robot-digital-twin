#include "tb6612.h"

#include "tim.h"

extern TIM_HandleTypeDef htim3;

static uint8_t tb6612_initialized = 0;

static uint16_t clampDutyToRange(int32_t duty)
{
    uint32_t max_duty = __HAL_TIM_GET_AUTORELOAD(&htim3);

    if (duty < 0) {
        duty = -duty;
    }

    if ((uint32_t)duty > max_duty) {
        duty = (int32_t)max_duty;
    }

    return (uint16_t)duty;
}

static void motorASetDirection(GPIO_PinState ain1, GPIO_PinState ain2)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, ain1);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, ain2);
}

uint8_t TB6612_Init(void)
{
    if (!tb6612_initialized) {
        if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
            return 0;
        }
        tb6612_initialized = 1;
    }

    TB6612_EmergencyStop();
    return 1;
}

void TB6612_SetStandby(uint8_t enabled)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void TB6612_MotorA_SetDuty(int16_t duty)
{
    uint16_t magnitude = clampDutyToRange(duty);

    if (magnitude == 0U) {
        TB6612_MotorA_Stop();
        return;
    }

    if (duty > 0) {
        motorASetDirection(GPIO_PIN_SET, GPIO_PIN_RESET);
    } else {
        motorASetDirection(GPIO_PIN_RESET, GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, magnitude);
}

void TB6612_MotorA_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    motorASetDirection(GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void TB6612_EmergencyStop(void)
{
    TB6612_MotorA_Stop();
    TB6612_SetStandby(0);
}

uint16_t TB6612_MotorA_GetMaxDuty(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim3);
}
