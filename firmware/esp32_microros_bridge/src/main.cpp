#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <stdio.h>

#include "app_config.h"
#include "stm32_serial_parser.h"
#include "uros_core.h"
#include "uros_pub.h"
#include "uros_sub.h"

namespace {

HardwareSerial STM32Serial(1);

bool wifi_connected = false;
bool transport_configured = false;
bool micro_ros_initialized = false;
unsigned long last_wifi_attempt_ms = 0;
unsigned long last_micro_ros_init_attempt_ms = 0;
unsigned long last_runtime_debug_ms = 0;

void handleParsedImuSample(const STM32ImuSample& sample);
void handleParsedState(int32_t state);
void handleCmdVel(float linear_x, float angular_z);
void handleDebugLine(const char* line);

STM32SerialParser stm32_parser(
    handleParsedImuSample,
    handleParsedState,
    handleDebugLine,
    app_config::kAcceptTrainCsvAsImu);

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
    int len = snprintf(buffer, sizeof(buffer), "CMDVEL,%.3f,%.3f\n", linear_x, angular_z);
    static unsigned long last_cmd_debug_ms = 0;

    if (len <= 0) {
        return;
    }

    STM32Serial.write((const uint8_t*)buffer, (size_t)len);

    if (!app_config::kEnableCmdVelDebug) {
        return;
    }

    if ((millis() - last_cmd_debug_ms) < app_config::kCmdVelDebugIntervalMs) {
        return;
    }

    Serial.print("[CMDVEL] vx=");
    Serial.print(linear_x, 3);
    Serial.print(" wz=");
    Serial.println(angular_z, 3);
    last_cmd_debug_ms = millis();
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
    wl_status_t status = WiFi.status();

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
        Serial.print("System running normally imu=");
        Serial.print(stm32_parser.imuFrameCount());
        Serial.print(" state=");
        Serial.print(stm32_parser.stateFrameCount());
        Serial.print(" drop=");
        Serial.println(stm32_parser.droppedFrameCount());
        last_runtime_debug_ms = millis();
    }
}

}  // namespace

void setup() {
    esp_task_wdt_init(60, true);
    esp_task_wdt_add(NULL);

    Serial.begin(app_config::kUsbSerialBaud);
    delay(1000);
    Serial.println("ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...");
    Serial.print("Build: ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);

    STM32Serial.begin(app_config::kUartBaud, SERIAL_8N1, app_config::kUartRx, app_config::kUartTx);
    Serial.println("STM32 Serial initialized");
    delay(50);
    STM32Serial.write('G');
    STM32Serial.write('\n');
    Serial.println("Requested STM32 GAZEBO mode");

    startWiFiConnectionAttempt();
    Serial.println("Setup complete, waiting for WiFi and micro-ROS...");
}

void loop() {
    esp_task_wdt_reset();

    stm32_parser.handle(STM32Serial);

    while (Serial.available()) {
        STM32Serial.write((uint8_t)Serial.read());
    }

    serviceWiFiConnection();
    serviceMicroRosConnection();
    serviceConnectedRuntime();

    delay(1);
}
