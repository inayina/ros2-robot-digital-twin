#include "motor/motor_control_shared.h"

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace {

portMUX_TYPE g_motor_shared_mux = portMUX_INITIALIZER_UNLOCKED;

MotorControlCommandSnapshot g_command = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    MotorCommandSource::kNone,
    false,
    false,
    false,
};

MotorControlStateSnapshot g_state = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    0U,
    MotorCommandSource::kNone,
    0,
    false,
    false,
    false,
    true,
    false,
    false,
    false,
    false,
};

}  // namespace

void motorSharedInit() {
    portENTER_CRITICAL(&g_motor_shared_mux);
    g_command = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0U,
        0U,
        0U,
        MotorCommandSource::kNone,
        false,
        false,
        false,
    };
    g_state = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0U,
        0U,
        0U,
        0U,
        MotorCommandSource::kNone,
        0,
        false,
        false,
        false,
        true,
        false,
        false,
        false,
        false,
    };
    portEXIT_CRITICAL(&g_motor_shared_mux);
}

void motorSharedSetLegacyCmdVel(float linear_x, float angular_z, uint32_t now_ms) {
    portENTER_CRITICAL(&g_motor_shared_mux);
    g_command.linear_x = linear_x;
    g_command.angular_z = angular_z;
    g_command.target_rpm = 0.0f;
    g_command.max_pwm = 0.0f;
    g_command.updated_ms = now_ms;
    g_command.sequence++;
    g_command.timeout_ms = 0U;
    g_command.source = MotorCommandSource::kLegacyCmdVel;
    g_command.enabled = false;
    g_command.closed_loop = false;
    g_command.stop_requested = false;
    portEXIT_CRITICAL(&g_motor_shared_mux);
}

void motorSharedSetTargetRpm(float target_rpm, uint32_t now_ms) {
    portENTER_CRITICAL(&g_motor_shared_mux);
    g_command.linear_x = 0.0f;
    g_command.angular_z = 0.0f;
    g_command.target_rpm = target_rpm;
    g_command.max_pwm = 0.0f;
    g_command.updated_ms = now_ms;
    g_command.sequence++;
    g_command.timeout_ms = 0U;
    g_command.source = MotorCommandSource::kTargetRpm;
    g_command.enabled = true;
    g_command.closed_loop = true;
    g_command.stop_requested = false;
    portEXIT_CRITICAL(&g_motor_shared_mux);
}

void motorSharedSetMotorCmd(float target_rpm,
                            bool enabled,
                            bool closed_loop,
                            float max_pwm,
                            uint32_t timeout_ms,
                            bool stop_requested,
                            uint32_t now_ms) {
    portENTER_CRITICAL(&g_motor_shared_mux);
    g_command.linear_x = 0.0f;
    g_command.angular_z = 0.0f;
    g_command.target_rpm = target_rpm;
    g_command.max_pwm = max_pwm;
    g_command.updated_ms = now_ms;
    g_command.sequence++;
    g_command.timeout_ms = timeout_ms;
    g_command.source = MotorCommandSource::kMotorCmd;
    g_command.enabled = enabled;
    g_command.closed_loop = closed_loop;
    g_command.stop_requested = stop_requested;
    portEXIT_CRITICAL(&g_motor_shared_mux);
}

MotorControlCommandSnapshot motorSharedGetCommand() {
    portENTER_CRITICAL(&g_motor_shared_mux);
    MotorControlCommandSnapshot snapshot = g_command;
    portEXIT_CRITICAL(&g_motor_shared_mux);
    return snapshot;
}

void motorSharedSetState(const MotorControlStateSnapshot& state) {
    portENTER_CRITICAL(&g_motor_shared_mux);
    g_state = state;
    portEXIT_CRITICAL(&g_motor_shared_mux);
}

MotorControlStateSnapshot motorSharedGetState() {
    portENTER_CRITICAL(&g_motor_shared_mux);
    MotorControlStateSnapshot snapshot = g_state;
    portEXIT_CRITICAL(&g_motor_shared_mux);
    return snapshot;
}

const char* motorCommandSourceName(MotorCommandSource source) {
    switch (source) {
        case MotorCommandSource::kLegacyCmdVel:
            return "legacy_cmd_vel";
        case MotorCommandSource::kTargetRpm:
            return "target_rpm";
        case MotorCommandSource::kMotorCmd:
            return "motor_cmd";
        case MotorCommandSource::kNone:
        default:
            return "none";
    }
}
