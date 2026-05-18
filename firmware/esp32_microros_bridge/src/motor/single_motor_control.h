#ifndef SINGLE_MOTOR_CONTROL_H
#define SINGLE_MOTOR_CONTROL_H

#include <stdint.h>

#include "motor/motor_control_shared.h"
#include "motor/speed_pid.h"

struct SingleMotorControlConfig {
    uint32_t command_timeout_ms;
    float max_abs_target_rpm;
    SpeedPidConfig pid;
};

struct SingleMotorControlRuntime {
    SpeedPidRuntime pid;
};

void singleMotorControlInit(SingleMotorControlRuntime& runtime);
MotorControlStateSnapshot singleMotorControlUpdate(
    SingleMotorControlRuntime& runtime,
    const SingleMotorControlConfig& config,
    const MotorControlCommandSnapshot& command,
    float actual_rpm,
    uint32_t now_ms,
    uint32_t dt_ms,
    uint32_t loop_count);

#endif
