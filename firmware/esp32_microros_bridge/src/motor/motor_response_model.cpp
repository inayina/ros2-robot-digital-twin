#include "motor/motor_response_model.h"

#include <math.h>

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

int8_t directionForRpm(float rpm) {
    if (rpm > 0.0f) {
        return 1;
    }
    if (rpm < 0.0f) {
        return -1;
    }
    return 0;
}

bool commandTimedOut(const MotorControlCommandSnapshot& command,
                     const MotorResponseModelConfig& config,
                     uint32_t now_ms) {
    const bool has_command = command.source != MotorCommandSource::kNone && command.updated_ms != 0U;
    return !has_command || (uint32_t)(now_ms - command.updated_ms) > config.command_timeout_ms;
}

}  // namespace

void motorResponseModelInit(MotorResponseModelRuntime& runtime) {
    runtime.actual_rpm = 0.0f;
}

MotorControlStateSnapshot motorResponseModelUpdate(
    MotorResponseModelRuntime& runtime,
    const MotorResponseModelConfig& config,
    const MotorControlCommandSnapshot& command,
    uint32_t now_ms,
    uint32_t loop_count) {
    const bool timeout_active = commandTimedOut(command, config, now_ms);
    const bool target_rpm_active =
        !timeout_active && command.source == MotorCommandSource::kTargetRpm;
    const float requested_target_rpm = target_rpm_active ? command.target_rpm : 0.0f;
    const float target_rpm = clampFloat(requested_target_rpm, -config.max_abs_rpm, config.max_abs_rpm);
    const bool saturated = target_rpm_active && target_rpm != requested_target_rpm;

    runtime.actual_rpm += (target_rpm - runtime.actual_rpm) * config.response_alpha;

    if (!target_rpm_active && fabsf(runtime.actual_rpm) < config.zero_epsilon_rpm) {
        runtime.actual_rpm = 0.0f;
    }

    MotorControlStateSnapshot state = {};
    state.target_rpm = target_rpm;
    state.actual_rpm = runtime.actual_rpm;
    state.error_rpm = state.target_rpm - state.actual_rpm;
    state.pwm_duty = target_rpm_active ? fabsf(target_rpm) / config.max_abs_rpm : 0.0f;
    state.updated_ms = now_ms;
    state.loop_count = loop_count;
    state.last_command_sequence = command.sequence;
    state.active_source = timeout_active ? MotorCommandSource::kNone : command.source;
    state.direction = directionForRpm(state.target_rpm);
    state.control_enabled = target_rpm_active;
    state.timeout_active = timeout_active;
    state.legacy_bridge_active =
        !timeout_active && command.source == MotorCommandSource::kLegacyCmdVel;
    state.saturated = saturated;
    state.estop_active = false;
    state.fault_active = false;
    return state;
}
