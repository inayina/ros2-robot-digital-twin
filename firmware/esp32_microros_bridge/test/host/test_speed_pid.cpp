#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "motor/speed_pid.h"

namespace {

void assertNear(float actual, float expected, float tolerance) {
    assert(fabsf(actual - expected) <= tolerance);
}

SpeedPidConfig makeConfig() {
    SpeedPidConfig config = {};
    config.kp = 0.01f;
    config.ki = 0.02f;
    config.kd = 0.0f;
    config.output_min = -1.0f;
    config.output_max = 1.0f;
    config.integral_min = -10.0f;
    config.integral_max = 10.0f;
    return config;
}

void test_pid_output_uses_p_and_i_terms() {
    SpeedPidRuntime runtime = {};
    speedPidInit(runtime);

    const SpeedPidOutput output =
        speedPidUpdate(runtime, makeConfig(), 100.0f, 80.0f, 100U);

    assertNear(output.error, 20.0f, 0.001f);
    assertNear(output.proportional, 0.2f, 0.001f);
    assertNear(output.integral, 0.04f, 0.001f);
    assertNear(output.output, 0.24f, 0.001f);
    assert(!output.saturated);
}

void test_output_saturates() {
    SpeedPidRuntime runtime = {};
    speedPidInit(runtime);

    SpeedPidConfig config = makeConfig();
    config.kp = 1.0f;
    config.ki = 0.0f;

    const SpeedPidOutput output =
        speedPidUpdate(runtime, config, 100.0f, 0.0f, 100U);

    assertNear(output.output, 1.0f, 0.001f);
    assert(output.saturated);
}

void test_reset_clears_integral() {
    SpeedPidRuntime runtime = {};
    speedPidInit(runtime);

    speedPidUpdate(runtime, makeConfig(), 100.0f, 0.0f, 100U);
    assert(runtime.integral > 0.0f);

    speedPidReset(runtime);
    assertNear(runtime.integral, 0.0f, 0.001f);
    assert(!runtime.has_previous_error);
}

}  // namespace

int main() {
    test_pid_output_uses_p_and_i_terms();
    test_output_saturates();
    test_reset_clears_integral();

    puts("host speed_pid tests passed");
    return 0;
}
