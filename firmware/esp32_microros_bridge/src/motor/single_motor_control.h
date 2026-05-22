#ifndef SINGLE_MOTOR_CONTROL_H
#define SINGLE_MOTOR_CONTROL_H

#include <stdint.h>

#include "motor/motor_control_shared.h"
#include "motor/speed_pid.h"

struct SingleMotorControlConfig {
    uint32_t command_timeout_ms;
    uint32_t min_command_timeout_ms;
    uint32_t max_command_timeout_ms;
    uint32_t direction_change_coast_ms;
    float max_abs_target_rpm;
    float min_effective_pwm;
    float deadband_rpm;
    float default_motor_cmd_max_pwm;
    float max_command_max_pwm;
    SpeedPidConfig pid;
};

struct SingleMotorControlRuntime {
    SpeedPidRuntime pid;
    int8_t last_direction;
    uint32_t direction_hold_until_ms;
};

void singleMotorControlInit(SingleMotorControlRuntime& runtime);
MotorControlStateSnapshot singleMotorControlUpdate(
    SingleMotorControlRuntime& runtime,
    const SingleMotorControlConfig& config,
    const MotorControlCommandSnapshot& command,
    float actual_rpm,
    bool actual_rpm_valid,
    uint32_t now_ms,
    uint32_t dt_ms,
    uint32_t loop_count);

#endif
