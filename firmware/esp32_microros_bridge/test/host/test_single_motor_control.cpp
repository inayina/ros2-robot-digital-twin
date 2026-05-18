#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor/single_motor_control.h"

namespace {

void assertNear(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

SingleMotorControlConfig makeConfig() {
    SingleMotorControlConfig config = {};
    config.command_timeout_ms = 500U;
    config.max_abs_target_rpm = 300.0f;
    config.pid.kp = 0.01f;
    config.pid.ki = 0.0f;
    config.pid.kd = 0.0f;
    config.pid.output_min = -1.0f;
    config.pid.output_max = 1.0f;
    config.pid.integral_min = -10.0f;
    config.pid.integral_max = 10.0f;
    return config;
}

MotorControlCommandSnapshot makeTargetRpmCommand(float target_rpm, uint32_t updated_ms) {
    MotorControlCommandSnapshot command = {};
    command.target_rpm = target_rpm;
    command.updated_ms = updated_ms;
    command.sequence = 7U;
    command.source = MotorCommandSource::kTargetRpm;
    return command;
}

void test_target_rpm_generates_pid_duty() {
    SingleMotorControlRuntime runtime = {};
    singleMotorControlInit(runtime);

    const MotorControlStateSnapshot state = singleMotorControlUpdate(
        runtime,
        makeConfig(),
        makeTargetRpmCommand(80.0f, 1000U),
        50.0f,
        1100U,
        10U,
        1U);

    assert(state.control_enabled);
    assert(!state.timeout_active);
    assert(state.active_source == MotorCommandSource::kTargetRpm);
    assert(state.direction == 1);
    assertNear(state.target_rpm, 80.0f, 0.001f);
    assertNear(state.actual_rpm, 50.0f, 0.001f);
    assertNear(state.error_rpm, 30.0f, 0.001f);
    assertNear(state.pwm_duty, 0.3f, 0.001f);
}

void test_timeout_resets_pid_and_disables_duty() {
    SingleMotorControlRuntime runtime = {};
    singleMotorControlInit(runtime);

    singleMotorControlUpdate(
        runtime,
        makeConfig(),
        makeTargetRpmCommand(80.0f, 1000U),
        50.0f,
        1100U,
        10U,
        1U);

    const MotorControlStateSnapshot state = singleMotorControlUpdate(
        runtime,
        makeConfig(),
        makeTargetRpmCommand(80.0f, 1000U),
        50.0f,
        1601U,
        10U,
        2U);

    assert(!state.control_enabled);
    assert(state.timeout_active);
    assert(state.active_source == MotorCommandSource::kNone);
    assertNear(state.pwm_duty, 0.0f, 0.001f);
    assertNear(runtime.pid.integral, 0.0f, 0.001f);
}

void test_target_clamp_sets_saturated() {
    SingleMotorControlRuntime runtime = {};
    singleMotorControlInit(runtime);

    const MotorControlStateSnapshot state = singleMotorControlUpdate(
        runtime,
        makeConfig(),
        makeTargetRpmCommand(400.0f, 1000U),
        0.0f,
        1100U,
        10U,
        3U);

    assert(state.control_enabled);
    assert(state.saturated);
    assertNear(state.target_rpm, 300.0f, 0.001f);
    assertNear(state.pwm_duty, 1.0f, 0.001f);
}

}  // namespace

int main() {
    test_target_rpm_generates_pid_duty();
    test_timeout_resets_pid_and_disables_duty();
    test_target_clamp_sets_saturated();

    puts("host single_motor_control tests passed");
    return 0;
}
