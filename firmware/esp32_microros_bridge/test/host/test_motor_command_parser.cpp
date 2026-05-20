#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "bridge/motor_command_parser.h"

namespace {

void assertNear(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

void test_parses_dashboard_motor_command_payload() {
    MotorCommandMessage command = {};
    const bool ok = motorCommandParseJson(
        "{\"target_rpm\":120.5,\"enabled\":true,\"closed_loop\":true,"
        "\"max_pwm\":0.25,\"timeout_ms\":800,\"stop\":false}",
        command);

    assert(ok);
    assert(command.has_target_rpm);
    assert(command.has_enabled);
    assert(command.has_closed_loop);
    assert(command.has_max_pwm);
    assert(command.has_timeout_ms);
    assert(command.has_stop);
    assertNear(command.target_rpm, 120.5f, 0.001f);
    assert(command.enabled);
    assert(command.closed_loop);
    assertNear(command.max_pwm, 0.25f, 0.001f);
    assert(command.timeout_ms == 800U);
    assert(!command.stop);
}

void test_stop_only_command_is_valid() {
    MotorCommandMessage command = {};
    const bool ok = motorCommandParseJson("{\"stop\":true}", command);

    assert(ok);
    assert(command.has_stop);
    assert(command.stop);
    assert(!command.has_target_rpm);
}

void test_invalid_payload_is_rejected() {
    MotorCommandMessage command = {};
    assert(!motorCommandParseJson("raw-command", command));
}

}  // namespace

int main() {
    test_parses_dashboard_motor_command_payload();
    test_stop_only_command_is_valid();
    test_invalid_payload_is_rejected();

    puts("host motor_command_parser tests passed");
    return 0;
}
