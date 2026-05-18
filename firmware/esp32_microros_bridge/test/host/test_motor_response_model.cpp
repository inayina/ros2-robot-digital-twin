#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor/motor_response_model.h"

namespace {

MotorResponseModelConfig makeTestConfig() {
    MotorResponseModelConfig config = {};
    config.command_timeout_ms = 500U;
    config.response_alpha = 0.1f;
    config.zero_epsilon_rpm = 0.05f;
    config.max_abs_rpm = 300.0f;
    return config;
}

MotorControlCommandSnapshot makeTargetRpmCommand(float target_rpm, uint32_t updated_ms) {
    MotorControlCommandSnapshot command = {};
    command.target_rpm = target_rpm;
    command.updated_ms = updated_ms;
    command.sequence = 1U;
    command.source = MotorCommandSource::kTargetRpm;
    return command;
}

void assertNear(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

void test_tracks_target_rpm_with_first_order_response() {
    MotorResponseModelRuntime runtime = {};
    motorResponseModelInit(runtime);

    const MotorControlStateSnapshot state = motorResponseModelUpdate(
        runtime,
        makeTestConfig(),
        makeTargetRpmCommand(100.0f, 1000U),
        1100U,
        1U);

    assert(state.control_enabled);
    assert(!state.timeout_active);
    assert(state.active_source == MotorCommandSource::kTargetRpm);
    assert(state.direction == 1);
    assertNear(state.target_rpm, 100.0f, 0.001f);
    assertNear(state.actual_rpm, 10.0f, 0.001f);
    assertNear(state.error_rpm, 90.0f, 0.001f);
    assertNear(state.pwm_duty, 100.0f / 300.0f, 0.001f);
}

void test_clamps_target_and_sets_saturated() {
    MotorResponseModelRuntime runtime = {};
    motorResponseModelInit(runtime);

    const MotorControlStateSnapshot state = motorResponseModelUpdate(
        runtime,
        makeTestConfig(),
        makeTargetRpmCommand(500.0f, 1000U),
        1100U,
        2U);

    assert(state.control_enabled);
    assert(state.saturated);
    assertNear(state.target_rpm, 300.0f, 0.001f);
    assertNear(state.actual_rpm, 30.0f, 0.001f);
    assertNear(state.pwm_duty, 1.0f, 0.001f);
}

void test_timeout_disables_control_and_decays_to_zero() {
    MotorResponseModelRuntime runtime = {};
    motorResponseModelInit(runtime);
    runtime.actual_rpm = 20.0f;

    const MotorControlStateSnapshot state = motorResponseModelUpdate(
        runtime,
        makeTestConfig(),
        makeTargetRpmCommand(120.0f, 1000U),
        1601U,
        3U);

    assert(!state.control_enabled);
    assert(state.timeout_active);
    assert(state.active_source == MotorCommandSource::kNone);
    assert(state.direction == 0);
    assertNear(state.target_rpm, 0.0f, 0.001f);
    assertNear(state.actual_rpm, 18.0f, 0.001f);
    assertNear(state.pwm_duty, 0.0f, 0.001f);
}

void test_legacy_cmd_vel_is_not_motor_rpm_control() {
    MotorResponseModelRuntime runtime = {};
    motorResponseModelInit(runtime);

    MotorControlCommandSnapshot command = {};
    command.linear_x = 0.2f;
    command.angular_z = 0.0f;
    command.updated_ms = 2000U;
    command.sequence = 4U;
    command.source = MotorCommandSource::kLegacyCmdVel;

    const MotorControlStateSnapshot state =
        motorResponseModelUpdate(runtime, makeTestConfig(), command, 2100U, 4U);

    assert(!state.control_enabled);
    assert(!state.timeout_active);
    assert(state.legacy_bridge_active);
    assert(state.active_source == MotorCommandSource::kLegacyCmdVel);
    assertNear(state.target_rpm, 0.0f, 0.001f);
    assertNear(state.actual_rpm, 0.0f, 0.001f);
}

}  // namespace

int main() {
    test_tracks_target_rpm_with_first_order_response();
    test_clamps_target_and_sets_saturated();
    test_timeout_disables_control_and_decays_to_zero();
    test_legacy_cmd_vel_is_not_motor_rpm_control();

    puts("host motor_response_model tests passed");
    return 0;
}
