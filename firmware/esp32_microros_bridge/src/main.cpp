#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <stdio.h>

#include "config/app_config.h"
#include "motor/motor_controller.h"
#include "motor/motor_control_shared.h"
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
unsigned long last_motor_state_publish_ms = 0;

void handleParsedImuSample(const STM32ImuSample& sample);
void handleParsedState(int32_t state);
void handleCmdVel(float linear_x, float angular_z);
void handleTargetRpm(float target_rpm);
void handleDebugLine(const char* line);

STM32SerialParser stm32_parser(
    handleParsedImuSample,
    handleParsedState,
    handleDebugLine,
    app_config::kAcceptTrainCsvAsImu);

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
    Serial.print(state.control_enabled ? 1 : 0);
    Serial.print(" timeout=");
    Serial.print(state.timeout_active ? 1 : 0);
    Serial.print(" legacy=");
    Serial.println(state.legacy_bridge_active ? 1 : 0);
}

void handleParsedImuSample(const STM32ImuSample& sample) {
    urosPubPublishImu(sample.ax, sample.ay, sample.az,
                      sample.gx, sample.gy, sample.gz,
                      sample.qx, sample.qy, sample.qz, sample.qw);
    printImuDebug(sample.source, sample.ax, sample.ay, sample.az,
                  sample.qx, sample.qy, sample.qz, sample.qw);
}

void handleParsedState(int32_t state) {
    urosPubPublishState(state);
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

    if (app_config::kEnableBridgeRuntimeDebug &&
        (millis() - last_runtime_debug_ms) >= app_config::kBridgeRuntimeDebugIntervalMs) {
        const MotorControlStateSnapshot motor_state = motorSharedGetState();

        Serial.print("System running normally imu=");
        Serial.print(stm32_parser.imuFrameCount());
        Serial.print(" state=");
        Serial.print(stm32_parser.stateFrameCount());
        Serial.print(" drop=");
        Serial.print(stm32_parser.droppedFrameCount());
        Serial.print(" motor_src=");
        Serial.print(motorCommandSourceName(motor_state.active_source));
        Serial.print(" motor_timeout=");
        Serial.println(motor_state.timeout_active ? 1 : 0);
        last_runtime_debug_ms = millis();
    }

    if ((millis() - last_motor_state_publish_ms) >= app_config::kMotorStatePublishIntervalMs) {
        const MotorControlStateSnapshot motor_state = motorSharedGetState();
        urosPubPublishMotorActualRpm(motor_state.actual_rpm);
        urosPubPublishMotorState(motor_state);
        last_motor_state_publish_ms = millis();
    }
}

void rosCommTask(void* /*arg*/) {
    esp_task_wdt_add(NULL);

    Serial.print("ros_comm_task started on core ");
    Serial.println(xPortGetCoreID());
    startWiFiConnectionAttempt();

    for (;;) {
        esp_task_wdt_reset();

        stm32_parser.handle(STM32Serial);

        while (Serial.available()) {
            STM32Serial.write((uint8_t)Serial.read());
        }

        serviceWiFiConnection();
        serviceMicroRosConnection();
        serviceConnectedRuntime();

        vTaskDelay(pdMS_TO_TICKS(app_config::kRosCommTaskDelayMs));
    }
}

void motorControlTask(void* /*arg*/) {
    esp_task_wdt_add(NULL);

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t loop_count = 0;
    MotorControllerRuntime motor_runtime;
    motorControllerInit(motor_runtime);

    Serial.print("motor_control_task started on core ");
    Serial.println(xPortGetCoreID());
    runTb6612ChannelABenchTest();

    for (;;) {
        esp_task_wdt_reset();
        loop_count++;

        const uint32_t now_ms = millis();
        const MotorControlCommandSnapshot command = motorSharedGetCommand();
        const MotorControlStateSnapshot state =
            motorControllerUpdateMock(motor_runtime, command, now_ms, loop_count);

        motorControllerApplyHardwareOutputs(state);
        motorSharedSetState(state);
        printMotorDebug(state);

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
