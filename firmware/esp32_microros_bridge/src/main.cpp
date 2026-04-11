#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/ping.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include "wifi_config.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define UART_RX 16
#define UART_TX 17
#define UART_BAUD 921600

rcl_node_t node;
rcl_publisher_t imu_publisher;
rcl_publisher_t filtered_imu_publisher;
rcl_publisher_t state_publisher;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Imu filtered_imu_msg;
std_msgs__msg__Int32 state_msg;
rclc_support_t support;
rcl_allocator_t allocator;

HardwareSerial STM32Serial(1);

bool wifi_connected = false;
bool microros_connected = false;

void printRclError(const char* prefix) {
    Serial.print(prefix);
    Serial.print(": ");
    Serial.println(rcl_get_error_string().str);
    rcl_reset_error();
}

void error_loop() {
    while(1) {
        delay(100);
    }
}

void connectWiFi();

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
    
    Serial.println("Initializing rclc support...");
    allocator = rcl_get_default_allocator();
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printRclError("Failed to initialize rclc support");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("rclc support initialized");

    Serial.println("Creating ROS node...");
    ret = rclc_node_init_default(&node, "stm32_bridge", "", &support);
    
    if (ret != RCL_RET_OK) {
        printRclError("Failed to create ROS node");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("ROS node created");
    
    Serial.println("Creating IMU publisher...");
    ret = rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );
    
    if (ret != RCL_RET_OK) {
        printRclError("Failed to create IMU publisher");
    }
    Serial.println("IMU publisher created");
    
    Serial.println("Creating filtered IMU publisher...");
    ret = rclc_publisher_init_default(
        &filtered_imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/filtered"
    );
    
    if (ret != RCL_RET_OK) {
        printRclError("Failed to create filtered IMU publisher");
    }
    Serial.println("Filtered IMU publisher created");
    
    Serial.println("Creating state publisher...");
    ret = rclc_publisher_init_default(
        &state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/state"
    );
    
    if (ret != RCL_RET_OK) {
        printRclError("Failed to create state publisher");
    }
    Serial.println("State publisher created");
    
    microros_connected = true;
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

void parseSTM32Frame(const String& line) {
    if (line.startsWith("IMU,")) {
        float ax, ay, az, gx, gy, gz, temp;
        if (sscanf(line.c_str(), "IMU,%f,%f,%f,%f,%f,%f,%f", 
                   &ax, &ay, &az, &gx, &gy, &gz, &temp) == 7) {
            
            if (microros_connected) {
                imu_msg.header.stamp.sec = millis() / 1000;
                imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
                
                imu_msg.linear_acceleration.x = ax;
                imu_msg.linear_acceleration.y = ay;
                imu_msg.linear_acceleration.z = az;
                
                imu_msg.angular_velocity.x = gx * M_PI / 180.0;
                imu_msg.angular_velocity.y = gy * M_PI / 180.0;
                imu_msg.angular_velocity.z = gz * M_PI / 180.0;
                
                RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
                
                // 发布滤波后的IMU (目前与原始相同，可添加滤波逻辑)
                filtered_imu_msg = imu_msg;
                // 简单设置orientation为单位四元数 (可根据需要计算实际姿态)
                filtered_imu_msg.orientation.w = 1.0;
                filtered_imu_msg.orientation.x = 0.0;
                filtered_imu_msg.orientation.y = 0.0;
                filtered_imu_msg.orientation.z = 0.0;
                
                RCSOFTCHECK(rcl_publish(&filtered_imu_publisher, &filtered_imu_msg, NULL));
            }
            
            Serial.print("[IMU] ax:"); Serial.print(ax);
            Serial.print(" ay:"); Serial.print(ay);
            Serial.print(" az:"); Serial.println(az);
        }
    }
    else if (line.startsWith("State:")) {
        int state = 0;
        if (sscanf(line.c_str(), "State:%d", &state) == 1) {
            if (microros_connected) {
                state_msg.data = state;
                RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
            }
            Serial.print("[STATE] ");
            switch(state) {
                case 0: Serial.println("Normal"); break;
                case 1: Serial.println("Warning"); break;
                case 2: Serial.println("Alert"); break;
                case 3: Serial.println("Critical"); break;
                default: Serial.println("Unknown"); break;
            }
        }
    }
}

void loop() {
    // 喂看门狗
    esp_task_wdt_reset();
    
    static unsigned long last_debug = 0;
    
    if (STM32Serial.available()) {
        String line = STM32Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            Serial.print("RX: ");
            Serial.println(line);
            parseSTM32Frame(line);
        }
    }
    
    if (!wifi_connected) {
        if (WiFi.status() == WL_CONNECTED) {
            wifi_connected = true;
            Serial.println("WiFi reconnected!");
        }
    }
    
    if (microros_connected) {
        if (millis() - last_debug > 10000) {
            Serial.println("System running normally");
            last_debug = millis();
        }
    }
    
    delay(1);
}
