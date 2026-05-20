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
    uint32_t timeout_ms = config.command_timeout_ms;
    if (command.source == MotorCommandSource::kMotorCmd && command.timeout_ms != 0U) {
        timeout_ms = command.timeout_ms;
    }
    const uint32_t min_timeout_ms =
        config.min_command_timeout_ms != 0U ? config.min_command_timeout_ms : config.command_timeout_ms;
    const uint32_t max_timeout_ms =
        config.max_command_timeout_ms >= min_timeout_ms ? config.max_command_timeout_ms : min_timeout_ms;
    timeout_ms = (uint32_t)clampFloat(
        (float)timeout_ms,
        (float)min_timeout_ms,
        (float)max_timeout_ms);

    const bool has_command = command.source != MotorCommandSource::kNone && command.updated_ms != 0U;
    return !has_command || (uint32_t)(now_ms - command.updated_ms) > timeout_ms;
}

uint32_t effectiveTimeoutMs(const MotorControlCommandSnapshot& command,
                            const MotorResponseModelConfig& config) {
    uint32_t timeout_ms = config.command_timeout_ms;
    if (command.source == MotorCommandSource::kMotorCmd && command.timeout_ms != 0U) {
        timeout_ms = command.timeout_ms;
    }
    const uint32_t min_timeout_ms =
        config.min_command_timeout_ms != 0U ? config.min_command_timeout_ms : config.command_timeout_ms;
    const uint32_t max_timeout_ms =
        config.max_command_timeout_ms >= min_timeout_ms ? config.max_command_timeout_ms : min_timeout_ms;
    return (uint32_t)clampFloat(
        (float)timeout_ms,
        (float)min_timeout_ms,
        (float)max_timeout_ms);
}

bool commandEnabled(const MotorControlCommandSnapshot& command) {
    switch (command.source) {
        case MotorCommandSource::kTargetRpm:
            return true;
        case MotorCommandSource::kMotorCmd:
            return command.enabled;
        case MotorCommandSource::kLegacyCmdVel:
        case MotorCommandSource::kNone:
        default:
            return false;
    }
}

bool commandClosedLoop(const MotorControlCommandSnapshot& command) {
    switch (command.source) {
        case MotorCommandSource::kTargetRpm:
            return true;
        case MotorCommandSource::kMotorCmd:
            return command.closed_loop;
        case MotorCommandSource::kLegacyCmdVel:
        case MotorCommandSource::kNone:
        default:
            return false;
    }
}

float effectiveMaxPwmLimit(const MotorControlCommandSnapshot& command,
                           const MotorResponseModelConfig& config) {
    if (command.source == MotorCommandSource::kTargetRpm) {
        return 1.0f;
    }

    const float max_command_max_pwm =
        config.max_command_max_pwm > 0.0f ? config.max_command_max_pwm : 1.0f;
    const float default_motor_cmd_max_pwm =
        config.default_motor_cmd_max_pwm > 0.0f ? config.default_motor_cmd_max_pwm : max_command_max_pwm;

    float max_pwm = 0.0f;
    if (command.source == MotorCommandSource::kMotorCmd) {
        max_pwm = command.max_pwm > 0.0f ? command.max_pwm : default_motor_cmd_max_pwm;
    }

    return clampFloat(max_pwm, 0.0f, max_command_max_pwm);
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
    const uint32_t command_timeout_ms = effectiveTimeoutMs(command, config);
    const bool enabled = !timeout_active && commandEnabled(command);
    const bool closed_loop = !timeout_active && commandClosedLoop(command);
    const bool stop_active =
        !timeout_active &&
        command.source == MotorCommandSource::kMotorCmd &&
        command.stop_requested;
    const bool control_enabled =
        !timeout_active &&
        !stop_active &&
        enabled &&
        closed_loop &&
        (command.source == MotorCommandSource::kTargetRpm ||
         command.source == MotorCommandSource::kMotorCmd);
    const float requested_target_rpm = control_enabled ? command.target_rpm : 0.0f;
    const float target_rpm = clampFloat(requested_target_rpm, -config.max_abs_rpm, config.max_abs_rpm);
    const float max_pwm_limit =
        !timeout_active ? effectiveMaxPwmLimit(command, config) : 0.0f;
    const float effective_target_rpm = control_enabled ? target_rpm * max_pwm_limit : 0.0f;
    const float requested_pwm_duty =
        control_enabled ? fabsf(target_rpm) / config.max_abs_rpm : 0.0f;
    const float pwm_duty =
        control_enabled ? clampFloat(requested_pwm_duty, 0.0f, max_pwm_limit) : 0.0f;
    const bool saturated =
        control_enabled &&
        (target_rpm != requested_target_rpm || pwm_duty != requested_pwm_duty);

    runtime.actual_rpm += (effective_target_rpm - runtime.actual_rpm) * config.response_alpha;

    if (!control_enabled && fabsf(runtime.actual_rpm) < config.zero_epsilon_rpm) {
        runtime.actual_rpm = 0.0f;
    }

    MotorControlStateSnapshot state = {};
    state.target_rpm = target_rpm;
    state.actual_rpm = runtime.actual_rpm;
    state.error_rpm = state.target_rpm - state.actual_rpm;
    state.pwm_duty = pwm_duty;
    state.max_pwm_limit = max_pwm_limit;
    state.updated_ms = now_ms;
    state.loop_count = loop_count;
    state.last_command_sequence = command.sequence;
    state.command_timeout_ms = command_timeout_ms;
    state.active_source = timeout_active ? MotorCommandSource::kNone : command.source;
    state.direction = control_enabled ? directionForRpm(state.target_rpm) : 0;
    state.enabled = enabled;
    state.control_enabled = control_enabled;
    state.closed_loop = closed_loop;
    state.timeout_active = timeout_active;
    state.legacy_bridge_active =
        !timeout_active && command.source == MotorCommandSource::kLegacyCmdVel;
    state.saturated = saturated;
    state.estop_active = stop_active;
    state.fault_active = false;
    return state;
}
