#include "motor/speed_pid.h"

namespace {

float clampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

}  // namespace

void speedPidInit(SpeedPidRuntime& runtime) {
    speedPidReset(runtime);
}

void speedPidReset(SpeedPidRuntime& runtime) {
    runtime.integral = 0.0f;
    runtime.previous_error = 0.0f;
    runtime.has_previous_error = false;
}

SpeedPidOutput speedPidUpdate(
    SpeedPidRuntime& runtime,
    const SpeedPidConfig& config,
    float target_rpm,
    float actual_rpm,
    uint32_t dt_ms) {
    SpeedPidOutput result = {};
    const float dt_s = (dt_ms == 0U) ? 0.0f : (float)dt_ms / 1000.0f;

    result.error = target_rpm - actual_rpm;
    result.proportional = config.kp * result.error;

    const float previous_integral_state = runtime.integral;
    if (dt_s > 0.0f) {
        runtime.integral += result.error * dt_s;
        runtime.integral = clampFloat(runtime.integral, config.integral_min, config.integral_max);
    }

    if (runtime.has_previous_error && dt_s > 0.0f) {
        result.derivative = config.kd * ((result.error - runtime.previous_error) / dt_s);
    } else {
        result.derivative = 0.0f;
    }

    result.integral = config.ki * runtime.integral;
    float unclamped_output = result.proportional + result.integral + result.derivative;

    const bool saturated_high = unclamped_output > config.output_max && result.error > 0.0f;
    const bool saturated_low = unclamped_output < config.output_min && result.error < 0.0f;
    if ((saturated_high || saturated_low) && dt_s > 0.0f) {
        runtime.integral = previous_integral_state;
        result.integral = config.ki * runtime.integral;
        unclamped_output = result.proportional + result.integral + result.derivative;
    }

    runtime.previous_error = result.error;
    runtime.has_previous_error = true;

    result.output = clampFloat(unclamped_output, config.output_min, config.output_max);
    result.saturated = result.output != unclamped_output;
    return result;
}
