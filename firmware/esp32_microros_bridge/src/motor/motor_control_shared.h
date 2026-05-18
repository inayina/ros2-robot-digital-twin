#ifndef MOTOR_CONTROL_SHARED_H
#define MOTOR_CONTROL_SHARED_H

#include <stdint.h>

enum class MotorCommandSource : uint8_t {
    kNone = 0,
    kLegacyCmdVel = 1,
    kTargetRpm = 2,
};

struct MotorControlCommandSnapshot {
    float linear_x;
    float angular_z;
    float target_rpm;
    uint32_t updated_ms;
    uint32_t sequence;
    MotorCommandSource source;
};

struct MotorControlStateSnapshot {
    float target_rpm;
    float actual_rpm;
    float error_rpm;
    float pwm_duty;
    uint32_t updated_ms;
    uint32_t loop_count;
    uint32_t last_command_sequence;
    MotorCommandSource active_source;
    int8_t direction;
    bool control_enabled;
    bool timeout_active;
    bool legacy_bridge_active;
    bool saturated;
    bool estop_active;
    bool fault_active;
};

void motorSharedInit();
void motorSharedSetLegacyCmdVel(float linear_x, float angular_z, uint32_t now_ms);
void motorSharedSetTargetRpm(float target_rpm, uint32_t now_ms);
MotorControlCommandSnapshot motorSharedGetCommand();
void motorSharedSetState(const MotorControlStateSnapshot& state);
MotorControlStateSnapshot motorSharedGetState();
const char* motorCommandSourceName(MotorCommandSource source);

#endif
