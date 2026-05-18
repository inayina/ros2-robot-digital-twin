#ifndef MOTOR_RESPONSE_MODEL_H
#define MOTOR_RESPONSE_MODEL_H

#include <stdint.h>

#include "motor/motor_control_shared.h"

struct MotorResponseModelConfig {
    uint32_t command_timeout_ms;
    float response_alpha;
    float zero_epsilon_rpm;
    float max_abs_rpm;
};

struct MotorResponseModelRuntime {
    float actual_rpm;
};

void motorResponseModelInit(MotorResponseModelRuntime& runtime);
MotorControlStateSnapshot motorResponseModelUpdate(
    MotorResponseModelRuntime& runtime,
    const MotorResponseModelConfig& config,
    const MotorControlCommandSnapshot& command,
    uint32_t now_ms,
    uint32_t loop_count);

#endif
