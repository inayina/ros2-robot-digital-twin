#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/ping.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "stm32_serial_parser.h"
#include "uros_pub.h"
#include "wifi_config.h"

#define UART_RX 16
#define UART_TX 17
#define UART_BAUD 921600

const float IMU_FILTER_ALPHA = 0.2f;
const bool ACCEPT_TRAIN_CSV_AS_IMU = false;

HardwareSerial STM32Serial(1);

void handleParsedImuSample(const STM32ImuSample& sample);
void handleParsedState(int32_t state);

STM32SerialParser stm32_parser(handleParsedImuSample, handleParsedState, ACCEPT_TRAIN_CSV_AS_IMU);

bool wifi_connected = false;

void connectWiFi();

void printImuDebug(const char* source, float ax, float ay, float az,
                   float qx, float qy, float qz, float qw) {
    static unsigned long last_imu_debug = 0;
    if (millis() - last_imu_debug < 1000) {
        return;
    }
    last_imu_debug = millis();

    Serial.print("[IMU ");
    Serial.print(source);
    Serial.print("] ax:"); Serial.print(ax);
    Serial.print(" ay:"); Serial.print(ay);
    Serial.print(" az:"); Serial.print(az);
    Serial.print(" q:");
    Serial.print(qx, 3); Serial.print(",");
    Serial.print(qy, 3); Serial.print(",");
    Serial.print(qz, 3); Serial.print(",");
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

void micRosTask(void *pvParameters) {
    delay(1000);
    
    Serial.print("Connecting to micro-ROS Agent at ");
    Serial.print(AGENT_IP);
    Serial.print(":");
    Serial.println(AGENT_PORT);
    
    Serial.println("Setting micro-ROS WiFi transports...");
    set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
    
    Serial.println("Waiting for transport setup...");
    delay(1000);

    Serial.println("Pinging micro-ROS Agent over WiFi UDP...");
    rmw_ret_t ping_ret = rmw_uros_ping_agent(1000, 5);
    if (ping_ret != RMW_RET_OK) {
        Serial.println("micro-ROS Agent ping failed; check Agent IP/port and run: micro_ros_agent udp4 --port 8888");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("micro-ROS Agent reachable");
    
    if (!urosPubInit(Serial, "stm32_bridge", IMU_FILTER_ALPHA)) {
        vTaskDelete(NULL);
        return;
    }
    Serial.println("micro-ROS connected!");
    
    vTaskDelete(NULL);
}

void setup() {
    // 初始化看门狗，60秒超时
    esp_task_wdt_init(60, true);
    esp_task_wdt_add(NULL);
    
    Serial.begin(921600);
    delay(1000);  // 给串口时间初始化
    Serial.println("ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...");
    Serial.print("Build: ");
    Serial.print(__DATE__);
    Serial.print(" ");
    Serial.println(__TIME__);
    
    STM32Serial.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial.println("STM32 Serial initialized");
    delay(50);
    STM32Serial.write('G');
    STM32Serial.write('\n');
    Serial.println("Requested STM32 GAZEBO mode");
    
    connectWiFi();
    
    if (wifi_connected) {
        Serial.println("WiFi connected, initializing micro-ROS in separate task...");
        BaseType_t task_created = xTaskCreatePinnedToCore(
            micRosTask,           // Task function
            "microROS Task",      // Task name
            20000,                // Stack size (20KB)
            NULL,                 // Parameters
            1,                    // Priority
            NULL,                 // Task handle
            1                     // Core ID (core 1)
        );
        if (task_created != pdPASS) {
            Serial.println("Failed to create micro-ROS task");
        }
    } else {
        Serial.println("WiFi failed, skipping micro-ROS");
    }
    
    Serial.println("Setup complete, waiting for micro-ROS...");
}

void connectWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts % 10 == 0) {
            Serial.print(" (attempt ");
            Serial.print(attempts);
            Serial.println(")");
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("");
        Serial.print("WiFi Connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("");
        Serial.println("WiFi connection failed!");
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());
    }
}

void loop() {
    // 喂看门狗
    esp_task_wdt_reset();
    
    static unsigned long last_debug = 0;
    
    stm32_parser.handle(STM32Serial);

    while (Serial.available()) {
        STM32Serial.write((uint8_t)Serial.read());
    }
    
    if (!wifi_connected) {
        if (WiFi.status() == WL_CONNECTED) {
            wifi_connected = true;
            Serial.println("WiFi reconnected!");
        }
    }
    
    if (urosPubIsConnected()) {
        if (millis() - last_debug > 10000) {
            Serial.print("System running normally imu=");
            Serial.print(stm32_parser.imuFrameCount());
            Serial.print(" state=");
            Serial.print(stm32_parser.stateFrameCount());
            Serial.print(" drop=");
            Serial.println(stm32_parser.droppedFrameCount());
            last_debug = millis();
        }
    }
    
    delay(1);
}
