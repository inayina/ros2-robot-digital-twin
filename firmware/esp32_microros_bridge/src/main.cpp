#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <stdio.h>

#include "config/app_config.h"
#include "bridge/motor_command_parser.h"
#include "motor/motor_controller.h"
#include "motor/motor_control_shared.h"
#include "motor/speed_pid.h"
#include "motor/tb6612_driver.h"
#include "bridge/stm32_serial_parser.h"
#include "ros/uros_core.h"
#include "ros/uros_pub.h"
#include "ros/uros_sub.h"

namespace {

HardwareSerial STM32Serial(1);

TaskHandle_t ros_comm_task_handle = nullptr;
TaskHandle_t motor_control_task_handle = nullptr;

bool wifi_connected = false;
bool transport_configured = false;
bool micro_ros_initialized = false;
unsigned long last_wifi_attempt_ms = 0;
unsigned long last_micro_ros_init_attempt_ms = 0;
unsigned long last_runtime_debug_ms = 0;
unsigned long last_imu_publish_ms = 0;
unsigned long last_filtered_imu_publish_ms = 0;
unsigned long last_robot_state_publish_ms = 0;
unsigned long last_motor_actual_rpm_publish_ms = 0;
unsigned long last_motor_state_json_publish_ms = 0;

struct RuntimeTaskStats {
    uint32_t ros_comm_loop_count;
    uint32_t ros_comm_max_loop_interval_ms;
    uint32_t motor_control_loop_count;
    uint32_t motor_control_max_jitter_ms;
};

portMUX_TYPE runtime_stats_mux = portMUX_INITIALIZER_UNLOCKED;
RuntimeTaskStats runtime_task_stats = {};

portMUX_TYPE encoder_bench_mux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t n20_encoder_bench_count = 0;
volatile uint32_t n20_encoder_bench_invalid_transitions = 0;
volatile uint8_t n20_encoder_bench_last_state = 0;

struct N20ClosedLoopBenchConfig {
    uint32_t control_period_ms;
    uint32_t print_interval_ms;
    bool invert_measured_ticks;
    float max_pwm;
    uint32_t profile_step1_start_ms;
    uint32_t profile_step2_start_ms;
    uint32_t profile_step3_start_ms;
    uint32_t profile_step4_start_ms;
    uint32_t profile_step5_start_ms;
    float profile_step0_target_ticks_per_sec;
    float profile_step1_target_ticks_per_sec;
    float profile_step2_target_ticks_per_sec;
    float profile_step3_target_ticks_per_sec;
    float profile_step4_target_ticks_per_sec;
    float profile_step5_target_ticks_per_sec;
    SpeedPidConfig pid;
};

struct N20ClosedLoopBenchRuntime {
    Tb6612Driver tb6612;
    SpeedPidRuntime pid;
    int32_t last_encoder_count;
    uint32_t last_control_ms;
    uint32_t last_print_ms;
    uint32_t start_ms;
    float last_measured_ticks_per_sec;
    float last_target_ticks_per_sec;
    float last_pwm;
    float last_error;
    bool csv_header_printed;
    bool completed;
    bool initialized;
    bool driver_ready;
};

void handleParsedImuSample(const STM32ImuSample& sample);
void handleParsedState(int32_t state);
void handleCmdVel(float linear_x, float angular_z);
void handleTargetRpm(float target_rpm);
void handleMotorCmd(const char* payload);
void handleDebugLine(const char* line);
Tb6612DriverConfig makeTb6612ChannelBBenchConfig();

STM32SerialParser stm32_parser(
    handleParsedImuSample,
    handleParsedState,
    handleDebugLine,
    app_config::kAcceptTrainCsvAsImu);

bool isIntervalDue(unsigned long now_ms, unsigned long last_ms, unsigned long interval_ms) {
    return last_ms == 0 || (now_ms - last_ms) >= interval_ms;
}

float clampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void noteRosCommLoop() {
    static uint32_t last_loop_ms = 0;
    const uint32_t now_ms = millis();
    const uint32_t interval_ms = (last_loop_ms == 0) ? 0U : now_ms - last_loop_ms;
    last_loop_ms = now_ms;

    portENTER_CRITICAL(&runtime_stats_mux);
    runtime_task_stats.ros_comm_loop_count++;
    if (interval_ms > runtime_task_stats.ros_comm_max_loop_interval_ms) {
        runtime_task_stats.ros_comm_max_loop_interval_ms = interval_ms;
    }
    portEXIT_CRITICAL(&runtime_stats_mux);
}

void noteMotorControlLoop() {
    static uint32_t last_loop_ms = 0;
    const uint32_t now_ms = millis();
    const uint32_t interval_ms = (last_loop_ms == 0) ? app_config::kMotorControlTaskPeriodMs : now_ms - last_loop_ms;
    const uint32_t jitter_ms =
        (interval_ms > app_config::kMotorControlTaskPeriodMs)
            ? interval_ms - app_config::kMotorControlTaskPeriodMs
            : app_config::kMotorControlTaskPeriodMs - interval_ms;
    last_loop_ms = now_ms;

    portENTER_CRITICAL(&runtime_stats_mux);
    runtime_task_stats.motor_control_loop_count++;
    if (jitter_ms > runtime_task_stats.motor_control_max_jitter_ms) {
        runtime_task_stats.motor_control_max_jitter_ms = jitter_ms;
    }
    portEXIT_CRITICAL(&runtime_stats_mux);
}

RuntimeTaskStats snapshotRuntimeTaskStatsAndResetPeaks() {
    portENTER_CRITICAL(&runtime_stats_mux);
    const RuntimeTaskStats snapshot = runtime_task_stats;
    runtime_task_stats.ros_comm_max_loop_interval_ms = 0U;
    runtime_task_stats.motor_control_max_jitter_ms = 0U;
    portEXIT_CRITICAL(&runtime_stats_mux);
    return snapshot;
}

uint32_t stackHighWaterBytes(TaskHandle_t handle) {
    if (handle == nullptr) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(handle);
}

uint8_t readN20EncoderBenchState() {
    const uint8_t a =
        (uint8_t)gpio_get_level((gpio_num_t)app_config::kN20BenchEncoderAPin);
    const uint8_t b =
        (uint8_t)gpio_get_level((gpio_num_t)app_config::kN20BenchEncoderBPin);
    return (uint8_t)((a << 1U) | b);
}

void IRAM_ATTR handleN20EncoderBenchChange() {
    const uint8_t next_state = readN20EncoderBenchState();

    portENTER_CRITICAL_ISR(&encoder_bench_mux);
    const uint8_t transition =
        (uint8_t)((n20_encoder_bench_last_state << 2U) | next_state);
    switch (transition) {
        case 0x1:
        case 0x7:
        case 0xE:
        case 0x8:
            n20_encoder_bench_count++;
            break;
        case 0x2:
        case 0xB:
        case 0xD:
        case 0x4:
            n20_encoder_bench_count--;
            break;
        case 0x0:
        case 0x5:
        case 0xA:
        case 0xF:
            break;
        default:
            n20_encoder_bench_invalid_transitions++;
            break;
    }
    n20_encoder_bench_last_state = next_state;
    portEXIT_CRITICAL_ISR(&encoder_bench_mux);
}

int32_t n20EncoderBenchCountSnapshot() {
    portENTER_CRITICAL(&encoder_bench_mux);
    const int32_t count = n20_encoder_bench_count;
    portEXIT_CRITICAL(&encoder_bench_mux);
    return count;
}

uint32_t n20EncoderBenchInvalidTransitionsSnapshot() {
    portENTER_CRITICAL(&encoder_bench_mux);
    const uint32_t invalid = n20_encoder_bench_invalid_transitions;
    portEXIT_CRITICAL(&encoder_bench_mux);
    return invalid;
}

void startN20EncoderBench() {
    pinMode(app_config::kN20BenchEncoderAPin, INPUT_PULLUP);
    pinMode(app_config::kN20BenchEncoderBPin, INPUT_PULLUP);

    const uint8_t initial_state = readN20EncoderBenchState();
    portENTER_CRITICAL(&encoder_bench_mux);
    n20_encoder_bench_count = 0;
    n20_encoder_bench_invalid_transitions = 0;
    n20_encoder_bench_last_state = initial_state;
    portEXIT_CRITICAL(&encoder_bench_mux);

    attachInterrupt(digitalPinToInterrupt(app_config::kN20BenchEncoderAPin),
                    handleN20EncoderBenchChange,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(app_config::kN20BenchEncoderBPin),
                    handleN20EncoderBenchChange,
                    CHANGE);

    Serial.print("[ENC] N20 bench encoder start state=");
    Serial.println(initial_state);
}

void stopN20EncoderBench() {
    detachInterrupt(digitalPinToInterrupt(app_config::kN20BenchEncoderAPin));
    detachInterrupt(digitalPinToInterrupt(app_config::kN20BenchEncoderBPin));
}

N20ClosedLoopBenchConfig makeN20ClosedLoopBenchConfig() {
    N20ClosedLoopBenchConfig config = {};
    config.control_period_ms = app_config::kN20ClosedLoopBenchControlPeriodMs;
    config.print_interval_ms = app_config::kN20ClosedLoopBenchPrintIntervalMs;
    config.invert_measured_ticks = app_config::kN20ClosedLoopBenchInvertMeasuredTicks;
    config.max_pwm = app_config::kN20ClosedLoopBenchMaxPwm;
    config.profile_step1_start_ms = app_config::kN20ClosedLoopBenchProfileStep1StartMs;
    config.profile_step2_start_ms = app_config::kN20ClosedLoopBenchProfileStep2StartMs;
    config.profile_step3_start_ms = app_config::kN20ClosedLoopBenchProfileStep3StartMs;
    config.profile_step4_start_ms = app_config::kN20ClosedLoopBenchProfileStep4StartMs;
    config.profile_step5_start_ms = app_config::kN20ClosedLoopBenchProfileStep5StartMs;
    config.profile_step0_target_ticks_per_sec =
        app_config::kN20ClosedLoopBenchProfileStep0TargetTicksPerSec;
    config.profile_step1_target_ticks_per_sec =
        app_config::kN20ClosedLoopBenchProfileStep1TargetTicksPerSec;
    config.profile_step2_target_ticks_per_sec =
        app_config::kN20ClosedLoopBenchProfileStep2TargetTicksPerSec;
    config.profile_step3_target_ticks_per_sec =
        app_config::kN20ClosedLoopBenchProfileStep3TargetTicksPerSec;
    config.profile_step4_target_ticks_per_sec =
        app_config::kN20ClosedLoopBenchProfileStep4TargetTicksPerSec;
    config.profile_step5_target_ticks_per_sec =
        app_config::kN20ClosedLoopBenchProfileStep5TargetTicksPerSec;
    config.pid.kp = app_config::kN20ClosedLoopBenchKp;
    config.pid.ki = app_config::kN20ClosedLoopBenchKi;
    config.pid.kd = app_config::kN20ClosedLoopBenchKd;
    config.pid.output_min = -app_config::kN20ClosedLoopBenchMaxPwm;
    config.pid.output_max = app_config::kN20ClosedLoopBenchMaxPwm;
    config.pid.integral_min = app_config::kN20ClosedLoopBenchIntegralMin;
    config.pid.integral_max = app_config::kN20ClosedLoopBenchIntegralMax;
    return config;
}

float n20ClosedLoopBenchTargetForElapsedMs(const N20ClosedLoopBenchConfig& config,
                                           uint32_t elapsed_ms) {
    if (elapsed_ms < config.profile_step1_start_ms) {
        return config.profile_step0_target_ticks_per_sec;
    }
    if (elapsed_ms < config.profile_step2_start_ms) {
        return config.profile_step1_target_ticks_per_sec;
    }
    if (elapsed_ms < config.profile_step3_start_ms) {
        return config.profile_step2_target_ticks_per_sec;
    }
    if (elapsed_ms < config.profile_step4_start_ms) {
        return config.profile_step3_target_ticks_per_sec;
    }
    if (elapsed_ms < config.profile_step5_start_ms) {
        return config.profile_step4_target_ticks_per_sec;
    }
    return config.profile_step5_target_ticks_per_sec;
}

void n20ClosedLoopBenchStop(N20ClosedLoopBenchRuntime& runtime, const char* reason) {
    if (reason != nullptr && reason[0] != '\0') {
        Serial.print("[N20_CLOSED_LOOP] stop: ");
        Serial.println(reason);
    }

    if (runtime.driver_ready) {
        runtime.tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
        runtime.tb6612.setStandby(false);
    }

    stopN20EncoderBench();
    speedPidReset(runtime.pid);
    runtime.last_encoder_count = n20EncoderBenchCountSnapshot();
    runtime.last_control_ms = 0U;
    runtime.last_print_ms = 0U;
    runtime.start_ms = 0U;
    runtime.last_measured_ticks_per_sec = 0.0f;
    runtime.last_target_ticks_per_sec = 0.0f;
    runtime.last_pwm = 0.0f;
    runtime.last_error = 0.0f;
    runtime.csv_header_printed = false;
    runtime.driver_ready = false;
    runtime.initialized = false;
}

bool n20ClosedLoopBenchInit(N20ClosedLoopBenchRuntime& runtime,
                            const N20ClosedLoopBenchConfig& config,
                            uint32_t now_ms) {
    if (runtime.initialized) {
        return runtime.driver_ready;
    }

    runtime = {};
    speedPidInit(runtime.pid);

    if (!runtime.tb6612.begin(makeTb6612ChannelBBenchConfig())) {
        Serial.println("[N20_CLOSED_LOOP] TB6612 B init failed");
        return false;
    }

    runtime.tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
    runtime.tb6612.setStandby(true);

    startN20EncoderBench();
    runtime.last_encoder_count = n20EncoderBenchCountSnapshot();
    runtime.start_ms = now_ms;
    runtime.last_control_ms = now_ms;
    runtime.last_print_ms = 0U;
    runtime.last_measured_ticks_per_sec = 0.0f;
    runtime.last_target_ticks_per_sec = 0.0f;
    runtime.last_pwm = 0.0f;
    runtime.last_error = 0.0f;
    runtime.csv_header_printed = false;
    runtime.completed = false;
    runtime.driver_ready = true;
    runtime.initialized = true;

    Serial.print("[N20_CLOSED_LOOP] enabled step profile period_ms=");
    Serial.println(config.control_period_ms);
    return true;
}

void n20ClosedLoopBenchPrint(const N20ClosedLoopBenchRuntime& runtime,
                             uint32_t elapsed_ms,
                             int32_t encoder_count) {
    if (!runtime.csv_header_printed) {
        Serial.println(
            "timestamp_ms,target_ticks_per_sec,measured_ticks_per_sec,pwm,error,encoder_count,invalid_transitions");
    }

    Serial.print(elapsed_ms);
    Serial.print(",");
    Serial.print(runtime.last_target_ticks_per_sec, 3);
    Serial.print(",");
    Serial.print(runtime.last_measured_ticks_per_sec, 3);
    Serial.print(",");
    Serial.print(runtime.last_pwm, 3);
    Serial.print(",");
    Serial.print(runtime.last_error, 3);
    Serial.print(",");
    Serial.print(encoder_count);
    Serial.print(",");
    Serial.println(n20EncoderBenchInvalidTransitionsSnapshot());
}

void serviceN20ClosedLoopBench(N20ClosedLoopBenchRuntime& runtime, uint32_t now_ms) {
    const N20ClosedLoopBenchConfig config = makeN20ClosedLoopBenchConfig();

    if (config.control_period_ms == 0U) {
        n20ClosedLoopBenchStop(runtime, "invalid control period");
        return;
    }

    if (runtime.completed) {
        return;
    }

    if (!n20ClosedLoopBenchInit(runtime, config, now_ms)) {
        return;
    }

    if (!isIntervalDue(now_ms, runtime.last_control_ms, config.control_period_ms)) {
        return;
    }

    const uint32_t dt_ms = now_ms - runtime.last_control_ms;
    if (dt_ms == 0U) {
        return;
    }

    const int32_t encoder_count = n20EncoderBenchCountSnapshot();
    const int32_t delta_count = encoder_count - runtime.last_encoder_count;
    runtime.last_encoder_count = encoder_count;
    runtime.last_control_ms = now_ms;
    const uint32_t elapsed_ms = now_ms - runtime.start_ms;
    runtime.last_target_ticks_per_sec =
        n20ClosedLoopBenchTargetForElapsedMs(config, elapsed_ms);

    runtime.last_measured_ticks_per_sec =
        ((float)delta_count * 1000.0f) / (float)dt_ms;
    if (config.invert_measured_ticks) {
        runtime.last_measured_ticks_per_sec = -runtime.last_measured_ticks_per_sec;
    }
    runtime.last_error =
        runtime.last_target_ticks_per_sec - runtime.last_measured_ticks_per_sec;

    float pwm = 0.0f;
    if (runtime.last_target_ticks_per_sec == 0.0f) {
        speedPidReset(runtime.pid);
        runtime.tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
    } else {
        const SpeedPidOutput pid_output = speedPidUpdate(
            runtime.pid,
            config.pid,
            runtime.last_target_ticks_per_sec,
            runtime.last_measured_ticks_per_sec,
            dt_ms);
        pwm = clampFloat(pid_output.output,
                         -config.max_pwm,
                         config.max_pwm);
        if ((runtime.last_target_ticks_per_sec > 0.0f && pwm < 0.0f) ||
            (runtime.last_target_ticks_per_sec < 0.0f && pwm > 0.0f)) {
            pwm = 0.0f;
        }
        if (!runtime.tb6612.writeDuty(Tb6612Channel::kB, pwm)) {
            n20ClosedLoopBenchStop(runtime, "TB6612 writeDuty failed");
            return;
        }
    }

    runtime.last_pwm = pwm;

    if (runtime.last_print_ms == 0U ||
        isIntervalDue(now_ms, runtime.last_print_ms, config.print_interval_ms)) {
        runtime.last_print_ms = now_ms;
        n20ClosedLoopBenchPrint(runtime, elapsed_ms, encoder_count);
        runtime.csv_header_printed = true;
    }

    if (elapsed_ms >= config.profile_step5_start_ms &&
        runtime.last_target_ticks_per_sec == 0.0f) {
        runtime.tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
        runtime.tb6612.setStandby(false);
        stopN20EncoderBench();
        speedPidReset(runtime.pid);
        runtime.driver_ready = false;
        runtime.completed = true;
        Serial.println("[N20_CLOSED_LOOP] profile complete");
    }
}

void printRuntimeStatsIfDue() {
    if (!app_config::kEnableBridgeRuntimeDebug) {
        return;
    }
    if (app_config::kEnableN20ClosedLoopBench) {
        return;
    }

    const unsigned long now_ms = millis();
    if (last_runtime_debug_ms == 0) {
        last_runtime_debug_ms = now_ms;
        return;
    }
    if (!isIntervalDue(now_ms, last_runtime_debug_ms, app_config::kRuntimeStatsPrintIntervalMs)) {
        return;
    }

    static uint32_t last_ros_loop_count = 0U;
    static uint32_t last_motor_loop_count = 0U;
    static uint32_t last_imu_frame_count = 0U;
    static uint32_t last_state_frame_count = 0U;
    static uint32_t last_imu_publish_count = 0U;
    static uint32_t last_filtered_publish_count = 0U;
    static uint32_t last_robot_state_publish_count = 0U;
    static uint32_t last_motor_actual_publish_count = 0U;
    static uint32_t last_motor_state_publish_count = 0U;
    static uint32_t last_motor_status_publish_count = 0U;
    static uint32_t last_imu_publish_error_count = 0U;
    static uint32_t last_filtered_publish_error_count = 0U;
    static uint32_t last_state_publish_error_count = 0U;
    static uint32_t last_motor_actual_publish_error_count = 0U;
    static uint32_t last_motor_state_publish_error_count = 0U;
    static uint32_t last_motor_status_publish_error_count = 0U;

    const RuntimeTaskStats task_stats = snapshotRuntimeTaskStatsAndResetPeaks();
    const UrosPubStats pub_stats = urosPubGetStats();
    const uint32_t imu_frame_count = stm32_parser.imuFrameCount();
    const uint32_t state_frame_count = stm32_parser.stateFrameCount();
    const uint32_t ros_loop_delta = task_stats.ros_comm_loop_count - last_ros_loop_count;
    const uint32_t motor_loop_delta = task_stats.motor_control_loop_count - last_motor_loop_count;
    const uint32_t imu_frame_delta = imu_frame_count - last_imu_frame_count;
    const uint32_t state_frame_delta = state_frame_count - last_state_frame_count;

    Serial.print("[RUNTIME] window_ms=");
    Serial.print((last_runtime_debug_ms == 0) ? now_ms : now_ms - last_runtime_debug_ms);
    Serial.print(" ros_loop=");
    Serial.print(ros_loop_delta);
    Serial.print(" ros_max_interval_ms=");
    Serial.print(task_stats.ros_comm_max_loop_interval_ms);
    Serial.print(" motor_loop=");
    Serial.print(motor_loop_delta);
    Serial.print(" motor_max_jitter_ms=");
    Serial.println(task_stats.motor_control_max_jitter_ms);

    Serial.print("[RUNTIME] stm32_imu_frames=");
    Serial.print(imu_frame_delta);
    Serial.print(" robot_state_frames=");
    Serial.print(state_frame_delta);
    Serial.print(" stm32_dropped_total=");
    Serial.println(stm32_parser.droppedFrameCount());

    Serial.print("[RUNTIME] pub_ok imu=");
    Serial.print(pub_stats.imu_data_publish_count - last_imu_publish_count);
    Serial.print(" filtered=");
    Serial.print(pub_stats.filtered_imu_publish_count - last_filtered_publish_count);
    Serial.print(" robot_state=");
    Serial.print(pub_stats.robot_state_publish_count - last_robot_state_publish_count);
    Serial.print(" motor_actual=");
    Serial.print(pub_stats.motor_actual_rpm_publish_count - last_motor_actual_publish_count);
    Serial.print(" motor_state=");
    Serial.print(pub_stats.motor_state_publish_count - last_motor_state_publish_count);
    Serial.print(" motor_status=");
    Serial.println(pub_stats.motor_status_publish_count - last_motor_status_publish_count);

    Serial.print("[RUNTIME] pub_err imu=");
    Serial.print(pub_stats.imu_data_publish_error_count - last_imu_publish_error_count);
    Serial.print(" filtered=");
    Serial.print(pub_stats.filtered_imu_publish_error_count - last_filtered_publish_error_count);
    Serial.print(" robot_state=");
    Serial.print(pub_stats.robot_state_publish_error_count - last_state_publish_error_count);
    Serial.print(" motor_actual=");
    Serial.print(pub_stats.motor_actual_rpm_publish_error_count - last_motor_actual_publish_error_count);
    Serial.print(" motor_state=");
    Serial.print(pub_stats.motor_state_publish_error_count - last_motor_state_publish_error_count);
    Serial.print(" motor_status=");
    Serial.println(pub_stats.motor_status_publish_error_count - last_motor_status_publish_error_count);

    Serial.print("[RUNTIME] free_heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" ros_stack_hwm_bytes=");
    Serial.print(stackHighWaterBytes(ros_comm_task_handle));
    Serial.print(" motor_stack_hwm_bytes=");
    Serial.println(stackHighWaterBytes(motor_control_task_handle));

    last_ros_loop_count = task_stats.ros_comm_loop_count;
    last_motor_loop_count = task_stats.motor_control_loop_count;
    last_imu_frame_count = imu_frame_count;
    last_state_frame_count = state_frame_count;
    last_imu_publish_count = pub_stats.imu_data_publish_count;
    last_filtered_publish_count = pub_stats.filtered_imu_publish_count;
    last_robot_state_publish_count = pub_stats.robot_state_publish_count;
    last_motor_actual_publish_count = pub_stats.motor_actual_rpm_publish_count;
    last_motor_state_publish_count = pub_stats.motor_state_publish_count;
    last_motor_status_publish_count = pub_stats.motor_status_publish_count;
    last_imu_publish_error_count = pub_stats.imu_data_publish_error_count;
    last_filtered_publish_error_count = pub_stats.filtered_imu_publish_error_count;
    last_state_publish_error_count = pub_stats.robot_state_publish_error_count;
    last_motor_actual_publish_error_count = pub_stats.motor_actual_rpm_publish_error_count;
    last_motor_state_publish_error_count = pub_stats.motor_state_publish_error_count;
    last_motor_status_publish_error_count = pub_stats.motor_status_publish_error_count;
    last_runtime_debug_ms = now_ms;
}

Tb6612DriverConfig makeTb6612ChannelABenchConfig() {
    Tb6612DriverConfig config = tb6612MakeUnsetDriverConfig();

    config.channel_a.pwm_pin = app_config::kTb6612BenchChannelAPwmPin;
    config.channel_a.dir_in1_pin = app_config::kTb6612BenchChannelADirIn1Pin;
    config.channel_a.dir_in2_pin = app_config::kTb6612BenchChannelADirIn2Pin;
    config.channel_a.pwm_channel = app_config::kTb6612BenchChannelAPwmChannel;
    config.channel_a.pwm_frequency_hz = app_config::kTb6612BenchPwmFrequencyHz;
    config.channel_a.pwm_resolution_bits = app_config::kTb6612BenchPwmResolutionBits;
    config.channel_a.invert_direction = false;
    config.standby_pin = app_config::kTb6612BenchStandbyPin;
    config.standby_active_high = true;
    config.enable_on_begin = false;
    return config;
}

Tb6612DriverConfig makeTb6612ChannelBBenchConfig() {
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

void runTb6612ChannelABenchTest() {
    if (!app_config::kEnableTb6612ChannelABenchTest) {
        return;
    }

    Tb6612Driver tb6612;
    Serial.println("[TB6612] A-channel 130 motor bench test starting");
    Serial.println("[TB6612] sequence: coast -> forward pulse -> coast -> reverse pulse -> coast");

    if (!tb6612.begin(makeTb6612ChannelABenchConfig())) {
        Serial.println("[TB6612] init failed; check bench pin config");
        return;
    }

    tb6612.setStandby(true);
    tb6612.writeOutput(Tb6612Channel::kA, Tb6612OutputMode::kCoast, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchCoastMs));

    Serial.println("[TB6612] A forward low-duty pulse");
    tb6612.writeDuty(Tb6612Channel::kA, app_config::kTb6612BenchTestDuty);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchPulseMs));

    Serial.println("[TB6612] A coast");
    tb6612.writeOutput(Tb6612Channel::kA, Tb6612OutputMode::kCoast, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchCoastMs));

    Serial.println("[TB6612] A reverse low-duty pulse");
    tb6612.writeDuty(Tb6612Channel::kA, -app_config::kTb6612BenchTestDuty);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchPulseMs));

    tb6612.writeOutput(Tb6612Channel::kA, Tb6612OutputMode::kCoast, 0.0f);
    tb6612.setStandby(false);
    Serial.println("[TB6612] A-channel bench test complete; driver standby disabled");
}

void runTb6612ChannelBBenchTest() {
    if (!app_config::kEnableTb6612ChannelBBenchTest ||
        app_config::kEnableN20ClosedLoopBench) {
        return;
    }

    Tb6612Driver tb6612;
    Serial.println("[TB6612] B-channel N20 bench test starting");
    Serial.println("[TB6612] sequence: coast -> forward pulse -> coast -> reverse pulse -> coast");

    if (!tb6612.begin(makeTb6612ChannelBBenchConfig())) {
        Serial.println("[TB6612] init failed; check B-channel bench pin config");
        return;
    }

    startN20EncoderBench();
    const int32_t start_count = n20EncoderBenchCountSnapshot();

    tb6612.setStandby(true);
    tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchCoastMs));

    Serial.println("[TB6612] B forward low-duty pulse");
    tb6612.writeDuty(Tb6612Channel::kB, app_config::kTb6612BenchTestDuty);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchPulseMs));
    const int32_t forward_count = n20EncoderBenchCountSnapshot();
    Serial.print("[ENC] after forward count=");
    Serial.print(forward_count);
    Serial.print(" delta=");
    Serial.println(forward_count - start_count);

    Serial.println("[TB6612] B coast");
    tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchCoastMs));

    Serial.println("[TB6612] B reverse low-duty pulse");
    tb6612.writeDuty(Tb6612Channel::kB, -app_config::kTb6612BenchTestDuty);
    vTaskDelay(pdMS_TO_TICKS(app_config::kTb6612BenchPulseMs));
    const int32_t reverse_count = n20EncoderBenchCountSnapshot();
    Serial.print("[ENC] after reverse count=");
    Serial.print(reverse_count);
    Serial.print(" delta=");
    Serial.println(reverse_count - forward_count);

    tb6612.writeOutput(Tb6612Channel::kB, Tb6612OutputMode::kCoast, 0.0f);
    tb6612.setStandby(false);
    stopN20EncoderBench();
    Serial.print("[ENC] invalid_transitions=");
    Serial.println(n20EncoderBenchInvalidTransitionsSnapshot());
    Serial.println("[TB6612] B-channel bench test complete; driver standby disabled");
}

void startWiFiConnectionAttempt() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(app_config::kWifiSsid);
    WiFi.begin(app_config::kWifiSsid, app_config::kWifiPass);
    last_wifi_attempt_ms = millis();
}

void configureMicroRosTransport() {
    if (transport_configured) {
        return;
    }

    static micro_ros_agent_locator locator;
    locator.address.fromString(app_config::kAgentIp);
    locator.port = app_config::kAgentPort;

    rmw_uros_set_custom_transport(
        false,
        (void*)&locator,
        arduino_wifi_transport_open,
        arduino_wifi_transport_close,
        arduino_wifi_transport_write,
        arduino_wifi_transport_read);

    transport_configured = true;
    Serial.println("micro-ROS WiFi transport configured");
}

void printImuDebug(const char* source, float ax, float ay, float az,
                   float qx, float qy, float qz, float qw) {
    static unsigned long last_imu_debug_ms = 0;

    if (!app_config::kEnableImuDebug) {
        return;
    }
    if (app_config::kEnableN20ClosedLoopBench) {
        return;
    }

    if ((millis() - last_imu_debug_ms) < app_config::kImuDebugIntervalMs) {
        return;
    }
    last_imu_debug_ms = millis();

    Serial.print("[IMU ");
    Serial.print(source);
    Serial.print("] ax:");
    Serial.print(ax);
    Serial.print(" ay:");
    Serial.print(ay);
    Serial.print(" az:");
    Serial.print(az);
    Serial.print(" q:");
    Serial.print(qx, 3);
    Serial.print(",");
    Serial.print(qy, 3);
    Serial.print(",");
    Serial.print(qz, 3);
    Serial.print(",");
    Serial.println(qw, 3);
}

void printMotorDebug(const MotorControlStateSnapshot& state) {
    static unsigned long last_motor_debug_ms = 0;

    if (!app_config::kEnableBridgeRuntimeDebug) {
        return;
    }

    if ((millis() - last_motor_debug_ms) < app_config::kMotorControlDebugIntervalMs) {
        return;
    }
    last_motor_debug_ms = millis();

    Serial.print("[MOTOR] core=");
    Serial.print(xPortGetCoreID());
    Serial.print(" loop=");
    Serial.print(state.loop_count);
    Serial.print(" src=");
    Serial.print(motorCommandSourceName(state.active_source));
    Serial.print(" target_rpm=");
    Serial.print(state.target_rpm, 3);
    Serial.print(" actual_rpm=");
    Serial.print(state.actual_rpm, 3);
    Serial.print(" error_rpm=");
    Serial.print(state.error_rpm, 3);
    Serial.print(" enabled=");
    Serial.print(state.enabled ? 1 : 0);
    Serial.print(" control=");
    Serial.print(state.control_enabled ? 1 : 0);
    Serial.print(" closed_loop=");
    Serial.print(state.closed_loop ? 1 : 0);
    Serial.print(" max_pwm=");
    Serial.print(state.max_pwm_limit, 3);
    Serial.print(" timeout=");
    Serial.print(state.timeout_active ? 1 : 0);
    Serial.print(" stop=");
    Serial.print(state.estop_active ? 1 : 0);
    Serial.print(" legacy=");
    Serial.println(state.legacy_bridge_active ? 1 : 0);
}

void handleParsedImuSample(const STM32ImuSample& sample) {
    const unsigned long now_ms = millis();

    if (isIntervalDue(now_ms, last_imu_publish_ms, app_config::kImuPublishIntervalMs)) {
        urosPubPublishImuRaw(sample.ax, sample.ay, sample.az,
                             sample.gx, sample.gy, sample.gz,
                             sample.qx, sample.qy, sample.qz, sample.qw);
        last_imu_publish_ms = now_ms;
    }

    if (isIntervalDue(now_ms, last_filtered_imu_publish_ms, app_config::kFilteredImuPublishIntervalMs)) {
        urosPubPublishFilteredImu(sample.ax, sample.ay, sample.az,
                                  sample.gx, sample.gy, sample.gz,
                                  sample.qx, sample.qy, sample.qz, sample.qw);
        last_filtered_imu_publish_ms = now_ms;
    }

    printImuDebug(sample.source, sample.ax, sample.ay, sample.az,
                  sample.qx, sample.qy, sample.qz, sample.qw);
}

void handleParsedState(int32_t state) {
    const unsigned long now_ms = millis();
    if (!isIntervalDue(now_ms, last_robot_state_publish_ms, app_config::kRobotStatePublishIntervalMs)) {
        return;
    }

    urosPubPublishState(state);
    last_robot_state_publish_ms = now_ms;
}

void handleDebugLine(const char* line) {
    if (!app_config::kEnableStm32DebugLinePrint) {
        return;
    }

    Serial.print("[STM32] ");
    Serial.println(line);
}

void handleCmdVel(float linear_x, float angular_z) {
    char buffer[64];
    const int len = snprintf(buffer, sizeof(buffer), "CMDVEL,%.3f,%.3f\n", linear_x, angular_z);
    static unsigned long last_cmd_debug_ms = 0;

    if (len <= 0) {
        return;
    }

    STM32Serial.write((const uint8_t*)buffer, (size_t)len);
    motorSharedSetLegacyCmdVel(linear_x, angular_z, millis());

    if (!app_config::kEnableCmdVelDebug) {
        return;
    }

    if ((millis() - last_cmd_debug_ms) < app_config::kCmdVelDebugIntervalMs) {
        return;
    }

    Serial.print("[CMDVEL] core=");
    Serial.print(xPortGetCoreID());
    Serial.print(" vx=");
    Serial.print(linear_x, 3);
    Serial.print(" wz=");
    Serial.println(angular_z, 3);
    last_cmd_debug_ms = millis();
}

void handleTargetRpm(float target_rpm) {
    static unsigned long last_target_rpm_debug_ms = 0;

    motorSharedSetTargetRpm(target_rpm, millis());

    if (!app_config::kEnableTargetRpmDebug) {
        return;
    }

    if ((millis() - last_target_rpm_debug_ms) < app_config::kTargetRpmDebugIntervalMs) {
        return;
    }

    Serial.print("[TARGET_RPM] core=");
    Serial.print(xPortGetCoreID());
    Serial.print(" rpm=");
    Serial.println(target_rpm, 3);
    last_target_rpm_debug_ms = millis();
}

void handleMotorCmd(const char* payload) {
    static unsigned long last_motor_cmd_debug_ms = 0;
    static unsigned long last_motor_cmd_error_ms = 0;

    MotorCommandMessage command = {};
    if (!motorCommandParseJson(payload, command)) {
        if ((millis() - last_motor_cmd_error_ms) >= 1000UL) {
            Serial.print("[MOTOR_CMD] invalid payload: ");
            Serial.println(payload == nullptr ? "<null>" : payload);
            last_motor_cmd_error_ms = millis();
        }
        return;
    }

    const bool enabled = command.has_enabled ? command.enabled : true;
    const bool closed_loop = command.has_closed_loop ? command.closed_loop : true;
    const bool stop_requested = command.has_stop ? command.stop : false;
    const float target_rpm = command.has_target_rpm ? command.target_rpm : 0.0f;
    const float max_pwm = command.has_max_pwm ? command.max_pwm : 0.0f;
    const uint32_t timeout_ms = command.has_timeout_ms ? command.timeout_ms : 0U;

    motorSharedSetMotorCmd(
        target_rpm,
        enabled,
        closed_loop,
        max_pwm,
        timeout_ms,
        stop_requested,
        millis());

    if ((millis() - last_motor_cmd_debug_ms) < app_config::kTargetRpmDebugIntervalMs) {
        return;
    }

    Serial.print("[MOTOR_CMD] rpm=");
    Serial.print(target_rpm, 3);
    Serial.print(" enabled=");
    Serial.print(enabled ? 1 : 0);
    Serial.print(" closed_loop=");
    Serial.print(closed_loop ? 1 : 0);
    Serial.print(" max_pwm=");
    Serial.print(max_pwm, 3);
    Serial.print(" timeout_ms=");
    Serial.print(timeout_ms);
    Serial.print(" stop=");
    Serial.println(stop_requested ? 1 : 0);
    last_motor_cmd_debug_ms = millis();
}

bool createMicroRosEntities() {
    Serial.print("Connecting to micro-ROS Agent at ");
    Serial.print(app_config::kAgentIp);
    Serial.print(":");
    Serial.println(app_config::kAgentPort);

    if (!transport_configured) {
        configureMicroRosTransport();
    }

    delay(app_config::kTransportSettleDelayMs);

    if (!urosCoreInit(Serial, app_config::kRosNodeName)) {
        return false;
    }
    if (!urosPubInit(Serial, app_config::kImuFilterAlpha)) {
        urosCoreFini(Serial);
        return false;
    }
    if (!urosSubInit(Serial)) {
        urosPubFini(Serial);
        urosCoreFini(Serial);
        return false;
    }

    urosSubSetCmdVelCallback(handleCmdVel);
    urosSubSetTargetRpmCallback(handleTargetRpm);
    urosSubSetMotorCmdCallback(handleMotorCmd);
    Serial.println("micro-ROS connected!");
    return true;
}

void destroyMicroRosEntities(const char* reason) {
    if (reason != nullptr && reason[0] != '\0') {
        Serial.println(reason);
    }

    urosSubFini(Serial);
    urosPubFini(Serial);
    urosCoreFini(Serial);
}

void serviceWiFiConnection() {
    const wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED) {
        if (!wifi_connected) {
            wifi_connected = true;
            Serial.print("WiFi Connected! IP: ");
            Serial.println(WiFi.localIP());
            if (!transport_configured) {
                configureMicroRosTransport();
            }
            last_micro_ros_init_attempt_ms = 0;
        }
        return;
    }

    if (wifi_connected) {
        wifi_connected = false;
        Serial.println("WiFi disconnected, waiting to reconnect...");
        if (micro_ros_initialized) {
            destroyMicroRosEntities("WiFi lost, tearing down micro-ROS entities...");
            micro_ros_initialized = false;
        }
    }

    if ((millis() - last_wifi_attempt_ms) >= app_config::kWifiRetryIntervalMs) {
        startWiFiConnectionAttempt();
    }
}

void serviceMicroRosConnection() {
    if (!wifi_connected || micro_ros_initialized) {
        return;
    }

    if (last_micro_ros_init_attempt_ms != 0 &&
        (millis() - last_micro_ros_init_attempt_ms) < app_config::kMicroRosInitRetryIntervalMs) {
        return;
    }

    last_micro_ros_init_attempt_ms = millis();

    if (createMicroRosEntities()) {
        micro_ros_initialized = true;
        return;
    }

    destroyMicroRosEntities("Failed to create micro-ROS entities, retrying...");
}

void serviceConnectedRuntime() {
    if (!micro_ros_initialized) {
        return;
    }

    urosSubSpinSome(app_config::kExecutorSpinTimeoutMs);

    const unsigned long now_ms = millis();
    if (isIntervalDue(now_ms,
                      last_motor_actual_rpm_publish_ms,
                      app_config::kMotorActualRpmPublishIntervalMs)) {
        const MotorControlStateSnapshot motor_state = motorSharedGetState();
        urosPubPublishMotorActualRpm(motor_state.actual_rpm);
        last_motor_actual_rpm_publish_ms = now_ms;
    }

    if (isIntervalDue(now_ms,
                      last_motor_state_json_publish_ms,
                      app_config::kMotorStateJsonPublishIntervalMs)) {
        const MotorControlStateSnapshot motor_state = motorSharedGetState();
        urosPubPublishMotorState(motor_state);
        last_motor_state_json_publish_ms = now_ms;
    }
}

void rosCommTask(void* /*arg*/) {
    esp_task_wdt_add(NULL);

    Serial.print("ros_comm_task started on core ");
    Serial.println(xPortGetCoreID());
    startWiFiConnectionAttempt();

    for (;;) {
        esp_task_wdt_reset();
        noteRosCommLoop();

        stm32_parser.handle(STM32Serial);

        while (Serial.available()) {
            STM32Serial.write((uint8_t)Serial.read());
        }

        serviceWiFiConnection();
        serviceMicroRosConnection();
        serviceConnectedRuntime();
        printRuntimeStatsIfDue();

        vTaskDelay(pdMS_TO_TICKS(app_config::kRosCommTaskDelayMs));
    }
}

void motorControlTask(void* /*arg*/) {
    esp_task_wdt_add(NULL);

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t loop_count = 0;
    MotorControllerRuntime motor_runtime;
    N20ClosedLoopBenchRuntime closed_loop_bench_runtime = {};
    motorControllerInit(motor_runtime);

    Serial.print("motor_control_task started on core ");
    Serial.println(xPortGetCoreID());
    runTb6612ChannelABenchTest();
    runTb6612ChannelBBenchTest();

    for (;;) {
        esp_task_wdt_reset();
        noteMotorControlLoop();
        loop_count++;

        const uint32_t now_ms = millis();
        if (app_config::kEnableN20ClosedLoopBench) {
            serviceN20ClosedLoopBench(closed_loop_bench_runtime, now_ms);
        } else {
            const MotorControlCommandSnapshot command = motorSharedGetCommand();
            const MotorControlStateSnapshot state =
                motorControllerUpdateMock(motor_runtime, command, now_ms, loop_count);

            motorControllerApplyHardwareOutputs(state);
            motorSharedSetState(state);
            printMotorDebug(state);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(app_config::kMotorControlTaskPeriodMs));
    }
}

void createPinnedTask(TaskFunction_t task_fn,
                      const char* task_name,
                      uint32_t stack_bytes,
                      UBaseType_t priority,
                      TaskHandle_t* out_handle,
                      BaseType_t core_id) {
    BaseType_t ret = xTaskCreatePinnedToCore(
        task_fn,
        task_name,
        stack_bytes,
        nullptr,
        priority,
        out_handle,
        core_id);

    if (ret == pdPASS) {
        return;
    }

    Serial.print("Failed to create task: ");
    Serial.println(task_name);
    for (;;) {
        delay(1000);
    }
}

}  // namespace

void setup() {
    esp_task_wdt_init(60, true);

    Serial.begin(app_config::kUsbSerialBaud);
    delay(1000);
    Serial.println("ESP32-S3 micro-ROS Bridge v1.2 dual-core - Starting...");
    Serial.print("Build: ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);

    motorSharedInit();

    STM32Serial.begin(app_config::kUartBaud, SERIAL_8N1, app_config::kUartRx, app_config::kUartTx);
    Serial.println("STM32 Serial initialized");
    delay(50);
    STM32Serial.write('G');
    STM32Serial.write('\n');
    Serial.println("Requested STM32 GAZEBO mode");

    createPinnedTask(
        rosCommTask,
        "ros_comm_task",
        app_config::kRosCommTaskStackBytes,
        app_config::kRosCommTaskPriority,
        &ros_comm_task_handle,
        app_config::kRosCommTaskCore);

    createPinnedTask(
        motorControlTask,
        "motor_control_task",
        app_config::kMotorControlTaskStackBytes,
        app_config::kMotorControlTaskPriority,
        &motor_control_task_handle,
        app_config::kMotorControlTaskCore);

    Serial.println("Dual-core task layout ready");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
