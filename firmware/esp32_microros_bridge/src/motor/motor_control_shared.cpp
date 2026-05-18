#include "motor/motor_control_shared.h"

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace {

portMUX_TYPE g_motor_shared_mux = portMUX_INITIALIZER_UNLOCKED;

MotorControlCommandSnapshot g_command = {
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    MotorCommandSource::kNone,
};

MotorControlStateSnapshot g_state = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    MotorCommandSource::kNone,
    0,
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
        0U,
        0U,
        MotorCommandSource::kNone,
    };
    g_state = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0U,
        0U,
        0U,
        MotorCommandSource::kNone,
        0,
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
    g_command.updated_ms = now_ms;
    g_command.sequence++;
    g_command.source = MotorCommandSource::kLegacyCmdVel;
    portEXIT_CRITICAL(&g_motor_shared_mux);
}

void motorSharedSetTargetRpm(float target_rpm, uint32_t now_ms) {
    portENTER_CRITICAL(&g_motor_shared_mux);
    g_command.target_rpm = target_rpm;
    g_command.updated_ms = now_ms;
    g_command.sequence++;
    g_command.source = MotorCommandSource::kTargetRpm;
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
        case MotorCommandSource::kNone:
        default:
            return "none";
    }
}
