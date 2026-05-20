#ifndef MOTOR_COMMAND_PARSER_H
#define MOTOR_COMMAND_PARSER_H

#include <stdint.h>

struct MotorCommandMessage {
    float target_rpm;
    float max_pwm;
    uint32_t timeout_ms;
    bool enabled;
    bool closed_loop;
    bool stop;
    bool has_target_rpm;
    bool has_max_pwm;
    bool has_timeout_ms;
    bool has_enabled;
    bool has_closed_loop;
    bool has_stop;
};

bool motorCommandParseJson(const char* payload, MotorCommandMessage& message);

#endif
