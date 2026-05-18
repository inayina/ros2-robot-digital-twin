#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

struct SpeedPidConfig {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
};

struct SpeedPidRuntime {
    float integral;
    float previous_error;
    bool has_previous_error;
};

struct SpeedPidOutput {
    float output;
    float error;
    float proportional;
    float integral;
    float derivative;
    bool saturated;
};

void speedPidInit(SpeedPidRuntime& runtime);
void speedPidReset(SpeedPidRuntime& runtime);
SpeedPidOutput speedPidUpdate(
    SpeedPidRuntime& runtime,
    const SpeedPidConfig& config,
    float target_rpm,
    float actual_rpm,
    uint32_t dt_ms);

#endif
