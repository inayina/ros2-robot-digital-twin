#include "motor/motor_controller.h"

#include "config/app_config.h"

namespace {

MotorResponseModelConfig makeMotorResponseModelConfig() {
    MotorResponseModelConfig config = {};
    config.command_timeout_ms = app_config::kMotorControlCommandTimeoutMs;
    config.response_alpha = app_config::kMockMotorResponseAlpha;
    config.zero_epsilon_rpm = app_config::kMockMotorZeroEpsilonRpm;
    config.max_abs_rpm = app_config::kMockMotorMaxAbsRpm;
    return config;
}

}  // namespace

void motorControllerInit(MotorControllerRuntime& runtime) {
    motorResponseModelInit(runtime.response_model);
}

MotorControlStateSnapshot motorControllerUpdateMock(
    MotorControllerRuntime& runtime,
    const MotorControlCommandSnapshot& command,
    uint32_t now_ms,
    uint32_t loop_count) {
    return motorResponseModelUpdate(
        runtime.response_model,
        makeMotorResponseModelConfig(),
        command,
        now_ms,
        loop_count);
}

void motorControllerApplyHardwareOutputs(const MotorControlStateSnapshot& /*state*/) {
    // Tb6612Driver is available, but hardware PWM/DIR output is intentionally
    // not enabled until GPIO, timer, driver polarity, and safety behavior are
    // validated on the bench.
}

float motorControllerReadEncoderRpm() {
    // Real encoder A/B and PCNT-based RPM calculation will replace the mock
    // response after the N20 wiring and CPR/PPR are measured.
    return 0.0f;
}
