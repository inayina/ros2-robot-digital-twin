#include "motor/motor_controller.h"

#include "config/app_config.h"
#include "motor/tb6612_driver.h"

namespace {

Tb6612Driver g_motor_driver;
bool g_motor_driver_attempted = false;
bool g_motor_driver_ready = false;
volatile bool g_runtime_motor_hardware_outputs_enabled =
    app_config::kEnableMotorHardwareOutputs;

bool motorHardwareOutputsAllowed() {
    return app_config::kEnableMotorHardwareOutputs ||
        g_runtime_motor_hardware_outputs_enabled;
}

Tb6612DriverConfig makeMotorHardwareDriverConfig() {
    Tb6612DriverConfig config = tb6612MakeUnsetDriverConfig();
    config.channel_b.pwm_pin = app_config::kTb6612BenchChannelBPwmPin;
    config.channel_b.dir_in1_pin = app_config::kTb6612BenchChannelBDirIn1Pin;
    config.channel_b.dir_in2_pin = app_config::kTb6612BenchChannelBDirIn2Pin;
    config.channel_b.pwm_channel = app_config::kTb6612BenchChannelBPwmChannel;
    config.channel_b.pwm_frequency_hz = app_config::kTb6612BenchPwmFrequencyHz;
    config.channel_b.pwm_resolution_bits = app_config::kTb6612BenchPwmResolutionBits;
    config.channel_b.invert_direction = false;
    config.standby_pin = app_config::kTb6612BenchStandbyPin;
    config.standby_active_high = true;
    config.enable_on_begin = false;
    return config;
}

bool ensureMotorHardwareReady() {
    if (!motorHardwareOutputsAllowed()) {
        return false;
    }

    if (g_motor_driver_ready) {
        return true;
    }

    if (g_motor_driver_attempted) {
        return false;
    }

    g_motor_driver_attempted = true;
    g_motor_driver_ready = g_motor_driver.begin(makeMotorHardwareDriverConfig());
    return g_motor_driver_ready;
}

MotorResponseModelConfig makeMotorResponseModelConfig() {
    MotorResponseModelConfig config = {};
    config.command_timeout_ms = app_config::kMotorControlCommandTimeoutMs;
    config.min_command_timeout_ms = app_config::kMotorControlCommandTimeoutMinMs;
    config.max_command_timeout_ms = app_config::kMotorControlCommandTimeoutMaxMs;
    config.response_alpha = app_config::kMockMotorResponseAlpha;
    config.zero_epsilon_rpm = app_config::kMockMotorZeroEpsilonRpm;
    config.max_abs_rpm = app_config::kMockMotorMaxAbsRpm;
    config.default_motor_cmd_max_pwm = app_config::kMotorControlDefaultMaxPwm;
    config.max_command_max_pwm = app_config::kMotorControlCommandMaxPwm;
    return config;
}

}  // namespace

void motorControllerSetHardwareOutputsEnabled(bool enabled) {
    g_runtime_motor_hardware_outputs_enabled = enabled;

    if (!enabled) {
        if (g_motor_driver_ready) {
            g_motor_driver.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
            g_motor_driver.setStandby(false);
        }
        return;
    }

    if (!g_motor_driver_ready) {
        g_motor_driver_attempted = false;
    }
}

bool motorControllerHardwareOutputsEnabled() {
    return motorHardwareOutputsAllowed();
}

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

void motorControllerApplyHardwareOutputs(const MotorControlStateSnapshot& state) {
    if (!ensureMotorHardwareReady()) {
        return;
    }

    const bool allow_drive =
        state.enabled &&
        state.control_enabled &&
        !state.timeout_active &&
        !state.estop_active &&
        !state.fault_active;

    if (!allow_drive) {
        g_motor_driver.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
        g_motor_driver.setStandby(false);
        return;
    }

    g_motor_driver.setStandby(true);
    g_motor_driver.writeDuty(
        Tb6612Channel::kB,
        (float)state.direction * state.pwm_duty);
}

float motorControllerReadEncoderRpm() {
    // Real encoder A/B and PCNT-based RPM calculation will replace the mock
    // response after the N20 wiring and CPR/PPR are measured.
    return 0.0f;
}
