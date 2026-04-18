#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <string.h>
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
#define STM32_LINE_MAX 256

const float IMU_FILTER_ALPHA = 0.2f;
const bool ACCEPT_TRAIN_CSV_AS_IMU = false;

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
char stm32_line_buffer[STM32_LINE_MAX];
size_t stm32_line_len = 0;
uint32_t imu_frame_count = 0;
uint32_t state_frame_count = 0;
uint32_t dropped_frame_count = 0;

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

void applyLowPassFilter(sensor_msgs__msg__Imu& filtered, const sensor_msgs__msg__Imu& raw) {
    static bool initialized = false;
    static float filtered_ax = 0.0f;
    static float filtered_ay = 0.0f;
    static float filtered_az = 0.0f;
    static float filtered_gx = 0.0f;
    static float filtered_gy = 0.0f;
    static float filtered_gz = 0.0f;

    if (!initialized) {
        filtered_ax = raw.linear_acceleration.x;
        filtered_ay = raw.linear_acceleration.y;
        filtered_az = raw.linear_acceleration.z;
        filtered_gx = raw.angular_velocity.x;
        filtered_gy = raw.angular_velocity.y;
        filtered_gz = raw.angular_velocity.z;
        initialized = true;
    } else {
        filtered_ax = IMU_FILTER_ALPHA * raw.linear_acceleration.x + (1.0f - IMU_FILTER_ALPHA) * filtered_ax;
        filtered_ay = IMU_FILTER_ALPHA * raw.linear_acceleration.y + (1.0f - IMU_FILTER_ALPHA) * filtered_ay;
        filtered_az = IMU_FILTER_ALPHA * raw.linear_acceleration.z + (1.0f - IMU_FILTER_ALPHA) * filtered_az;
        filtered_gx = IMU_FILTER_ALPHA * raw.angular_velocity.x + (1.0f - IMU_FILTER_ALPHA) * filtered_gx;
        filtered_gy = IMU_FILTER_ALPHA * raw.angular_velocity.y + (1.0f - IMU_FILTER_ALPHA) * filtered_gy;
        filtered_gz = IMU_FILTER_ALPHA * raw.angular_velocity.z + (1.0f - IMU_FILTER_ALPHA) * filtered_gz;
    }

    filtered = raw;
    filtered.linear_acceleration.x = filtered_ax;
    filtered.linear_acceleration.y = filtered_ay;
    filtered.linear_acceleration.z = filtered_az;
    filtered.angular_velocity.x = filtered_gx;
    filtered.angular_velocity.y = filtered_gy;
    filtered.angular_velocity.z = filtered_gz;
}

void publishImuSample(float ax, float ay, float az, float gx, float gy, float gz,
                      float qx, float qy, float qz, float qw) {
    if (microros_connected) {
        imu_msg.header.stamp.sec = millis() / 1000;
        imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
        imu_msg.orientation.x = qx;
        imu_msg.orientation.y = qy;
        imu_msg.orientation.z = qz;
        imu_msg.orientation.w = qw;

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;

        imu_msg.angular_velocity.x = gx * M_PI / 180.0;
        imu_msg.angular_velocity.y = gy * M_PI / 180.0;
        imu_msg.angular_velocity.z = gz * M_PI / 180.0;

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

        applyLowPassFilter(filtered_imu_msg, imu_msg);

        RCSOFTCHECK(rcl_publish(&filtered_imu_publisher, &filtered_imu_msg, NULL));
    }
}

void publishImuSample(float ax, float ay, float az, float gx, float gy, float gz) {
    publishImuSample(ax, ay, az, gx, gy, gz, 0.0f, 0.0f, 0.0f, 1.0f);
}

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

int countCommas(const char* line) {
    int count = 0;
    while (*line != '\0') {
        if (*line == ',') {
            count++;
        }
        line++;
    }
    return count;
}

bool isQuaternionValid(float qx, float qy, float qz, float qw) {
    float norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    return norm_sq > 0.5f && norm_sq < 1.5f;
}

bool parseSTM32Frame(const char* line) {
    if (strncmp(line, "IMUQ,", 5) == 0) {
        float ax, ay, az, gx, gy, gz, qx, qy, qz, qw, temp;
        if (sscanf(line, "IMUQ,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &ax, &ay, &az, &gx, &gy, &gz, &qx, &qy, &qz, &qw, &temp) == 11) {
            if (!isQuaternionValid(qx, qy, qz, qw)) {
                return false;
            }
            publishImuSample(ax, ay, az, gx, gy, gz, qx, qy, qz, qw);
            imu_frame_count++;
            printImuDebug("IMUQ", ax, ay, az, qx, qy, qz, qw);
            return true;
        }
        return false;
    }
    else if (strncmp(line, "IMU,", 4) == 0) {
        float ax, ay, az, gx, gy, gz, temp;
        if (sscanf(line, "IMU,%f,%f,%f,%f,%f,%f,%f",
                   &ax, &ay, &az, &gx, &gy, &gz, &temp) == 7) {
            publishImuSample(ax, ay, az, gx, gy, gz);
            imu_frame_count++;
            printImuDebug("IMU", ax, ay, az, 0.0f, 0.0f, 0.0f, 1.0f);
            return true;
        }
        return false;
    }
    else if (ACCEPT_TRAIN_CSV_AS_IMU && isDigit(line[0]) && countCommas(line) == 7) {
        unsigned long sample_ms;
        float ax, ay, az, gx, gy, gz;
        int state;
        if (sscanf(line, "%lu,%f,%f,%f,%f,%f,%f,%d",
                   &sample_ms, &ax, &ay, &az, &gx, &gy, &gz, &state) == 8) {
            publishImuSample(ax, ay, az, gx, gy, gz);
            imu_frame_count++;
            printImuDebug("CSV", ax, ay, az, 0.0f, 0.0f, 0.0f, 1.0f);
            return true;
        }
        return false;
    }

    if (strncmp(line, "State:", 6) == 0) {
        int state = 0;
        if (sscanf(line, "State:%d", &state) == 1) {
            if (microros_connected) {
                state_msg.data = state;
                RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
            }
            state_frame_count++;
            return true;
        }
        return false;
    }

    return false;
}

void handleSTM32Serial() {
    while (STM32Serial.available()) {
        char c = (char)STM32Serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (stm32_line_len > 0) {
                stm32_line_buffer[stm32_line_len] = '\0';
                if (!parseSTM32Frame(stm32_line_buffer)) {
                    dropped_frame_count++;
                }
                stm32_line_len = 0;
            }
            continue;
        }

        if (stm32_line_len < STM32_LINE_MAX - 1) {
            stm32_line_buffer[stm32_line_len++] = c;
        } else {
            stm32_line_len = 0;
            dropped_frame_count++;
        }
    }
}

void loop() {
    // 喂看门狗
    esp_task_wdt_reset();
    
    static unsigned long last_debug = 0;
    
    handleSTM32Serial();

    while (Serial.available()) {
        STM32Serial.write((uint8_t)Serial.read());
    }
    
    if (!wifi_connected) {
        if (WiFi.status() == WL_CONNECTED) {
            wifi_connected = true;
            Serial.println("WiFi reconnected!");
        }
    }
    
    if (microros_connected) {
        if (millis() - last_debug > 10000) {
            Serial.print("System running normally imu=");
            Serial.print(imu_frame_count);
            Serial.print(" state=");
            Serial.print(state_frame_count);
            Serial.print(" drop=");
            Serial.println(dropped_frame_count);
            last_debug = millis();
        }
    }
    
    delay(1);
}
