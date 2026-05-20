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

bool commandTimedOut(const MotorControlCommandSnapshot& command,
                     const SingleMotorControlConfig& config,
                     uint32_t now_ms) {
    const bool has_command = command.source != MotorCommandSource::kNone && command.updated_ms != 0U;
    return !has_command || (uint32_t)(now_ms - command.updated_ms) > config.command_timeout_ms;
}

}  // namespace

void singleMotorControlInit(SingleMotorControlRuntime& runtime) {
    speedPidInit(runtime.pid);
}

MotorControlStateSnapshot singleMotorControlUpdate(
    SingleMotorControlRuntime& runtime,
    const SingleMotorControlConfig& config,
    const MotorControlCommandSnapshot& command,
    float actual_rpm,
    uint32_t now_ms,
    uint32_t dt_ms,
    uint32_t loop_count) {
    const bool timeout_active = commandTimedOut(command, config, now_ms);
    const bool target_rpm_active =
        !timeout_active && command.source == MotorCommandSource::kTargetRpm;
    const float requested_target_rpm = target_rpm_active ? command.target_rpm : 0.0f;
    const float target_rpm =
        clampFloat(requested_target_rpm, -config.max_abs_target_rpm, config.max_abs_target_rpm);

    MotorControlStateSnapshot state = {};
    state.target_rpm = target_rpm;
    state.actual_rpm = actual_rpm;
    state.error_rpm = state.target_rpm - state.actual_rpm;
    state.max_pwm_limit = 1.0f;
    state.updated_ms = now_ms;
    state.loop_count = loop_count;
    state.last_command_sequence = command.sequence;
    state.command_timeout_ms = config.command_timeout_ms;
    state.active_source = timeout_active ? MotorCommandSource::kNone : command.source;
    state.direction = directionForRpm(state.target_rpm);
    state.enabled = target_rpm_active;
    state.control_enabled = target_rpm_active;
    state.closed_loop = target_rpm_active;
    state.timeout_active = timeout_active;
    state.legacy_bridge_active =
        !timeout_active && command.source == MotorCommandSource::kLegacyCmdVel;
    state.estop_active = false;
    state.fault_active = false;

    if (!target_rpm_active) {
        speedPidReset(runtime.pid);
        state.pwm_duty = 0.0f;
        state.saturated = false;
        return state;
    }

    const SpeedPidOutput pid_output =
        speedPidUpdate(runtime.pid, config.pid, state.target_rpm, state.actual_rpm, dt_ms);

    state.pwm_duty = fabsf(pid_output.output);
    state.saturated = pid_output.saturated || state.target_rpm != requested_target_rpm;
    return state;
}
