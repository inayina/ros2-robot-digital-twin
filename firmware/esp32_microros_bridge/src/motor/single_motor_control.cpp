#include "motor/single_motor_control.h"

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

uint32_t effectiveTimeoutMs(const MotorControlCommandSnapshot& command,
                            const SingleMotorControlConfig& config) {
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

bool commandTimedOut(const MotorControlCommandSnapshot& command,
                     const SingleMotorControlConfig& config,
                     uint32_t now_ms) {
    const bool has_command = command.source != MotorCommandSource::kNone && command.updated_ms != 0U;
    return !has_command || (uint32_t)(now_ms - command.updated_ms) > effectiveTimeoutMs(command, config);
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
                           const SingleMotorControlConfig& config) {
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

void singleMotorControlInit(SingleMotorControlRuntime& runtime) {
    speedPidInit(runtime.pid);
    runtime.last_direction = 0;
    runtime.direction_hold_until_ms = 0U;
}

MotorControlStateSnapshot singleMotorControlUpdate(
    SingleMotorControlRuntime& runtime,
    const SingleMotorControlConfig& config,
    const MotorControlCommandSnapshot& command,
    float actual_rpm,
    bool actual_rpm_valid,
    uint32_t now_ms,
    uint32_t dt_ms,
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
    const float target_rpm =
        clampFloat(requested_target_rpm, -config.max_abs_target_rpm, config.max_abs_target_rpm);
    const float max_pwm_limit =
        !timeout_active ? effectiveMaxPwmLimit(command, config) : 0.0f;

    MotorControlStateSnapshot state = {};
    state.target_rpm = target_rpm;
    state.actual_rpm = actual_rpm;
    state.error_rpm = state.target_rpm - state.actual_rpm;
    state.max_pwm_limit = max_pwm_limit;
    state.updated_ms = now_ms;
    state.loop_count = loop_count;
    state.last_command_sequence = command.sequence;
    state.command_timeout_ms = command_timeout_ms;
    state.active_source = timeout_active ? MotorCommandSource::kNone : command.source;
    state.direction = directionForRpm(state.target_rpm);
    state.enabled = enabled;
    state.control_enabled = control_enabled;
    state.closed_loop = closed_loop;
    state.timeout_active = timeout_active;
    state.legacy_bridge_active =
        !timeout_active && command.source == MotorCommandSource::kLegacyCmdVel;
    state.estop_active = stop_active;
    state.fault_active = false;

    if (!control_enabled || !actual_rpm_valid) {
        speedPidReset(runtime.pid);
        if (!control_enabled) {
            runtime.last_direction = 0;
            runtime.direction_hold_until_ms = 0U;
        }
        state.pwm_duty = 0.0f;
        state.saturated = false;
        if (!actual_rpm_valid) {
            state.direction = 0;
        }
        return state;
    }

    const int8_t requested_direction = directionForRpm(state.target_rpm);
    if (requested_direction != 0 &&
        runtime.last_direction != 0 &&
        requested_direction != runtime.last_direction &&
        now_ms >= runtime.direction_hold_until_ms) {
        runtime.direction_hold_until_ms = now_ms + config.direction_change_coast_ms;
        speedPidReset(runtime.pid);
    }

    const bool direction_hold_active = now_ms < runtime.direction_hold_until_ms;
    if (state.target_rpm == 0.0f || direction_hold_active) {
        speedPidReset(runtime.pid);
        state.pwm_duty = 0.0f;
        state.direction = 0;
        state.saturated = false;
        runtime.last_direction = requested_direction;
        return state;
    }

    const SpeedPidOutput pid_output =
        speedPidUpdate(runtime.pid, config.pid, state.target_rpm, state.actual_rpm, dt_ms);

    float signed_output = clampFloat(pid_output.output, -max_pwm_limit, max_pwm_limit);
    const bool pwm_clamped = signed_output != pid_output.output;
    if ((state.target_rpm > 0.0f && signed_output < 0.0f) ||
        (state.target_rpm < 0.0f && signed_output > 0.0f) ||
        fabsf(state.error_rpm) <= config.deadband_rpm) {
        signed_output = 0.0f;
    }

    if (signed_output != 0.0f && fabsf(signed_output) < config.min_effective_pwm) {
        signed_output = (signed_output > 0.0f) ? config.min_effective_pwm : -config.min_effective_pwm;
    }

    state.pwm_duty = fabsf(signed_output);
    state.direction = directionForRpm(signed_output);
    state.saturated =
        pid_output.saturated || pwm_clamped || state.target_rpm != requested_target_rpm;
    runtime.last_direction = requested_direction;
    return state;
}
