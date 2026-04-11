# Debug Log - ESP32/STM32 micro-ROS Bridge Project

## 项目概述

- ESP32-S3 DevKitC-1 作为 micro-ROS 客户端，通过 WiFi UDP 连接到 micro-ROS Agent
- STM32 作为传感器节点，通过 UART 发送 IMU 和状态数据到 ESP32
- ESP32 接收数据并发布到 ROS2 话题

## 当前状态

### micro-ROS 桥已跑通 (2026-04-11)

**验证结果**:

- ESP32 通过 WiFi 连接到 `192.168.1.8:8888` 的 micro-ROS Agent
- ROS2 中已经出现 `/imu/data`、`/imu/filtered`、`/robot/state`
- `/imu/data` 已输出 IMU 数据，`linear_acceleration.z` 约为 `9.93`
- `/robot/state` 已确认有数据

**ROS2 话题列表**:

```text
/imu/data
/imu/filtered
/parameter_events
/robot/state
/rosout
```

**关键确认点**:

- 新固件启动日志必须包含 `ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...`
- 新固件 transport 日志必须是 `Setting micro-ROS WiFi transports...`
- 如果看到 `Setting micro-ROS transports...`，说明板子仍在运行旧固件或错误固件

## 问题日志

### 1. ACM0 串口无数据 (2026-04-10)

**问题描述**: PlatformIO device monitor 显示无数据输出。

**根本原因**: ESP32-S3 USB CDC 未启用，默认使用 USB-Serial/JTAG 模式。

**解决方案**:

- 在 `platformio.ini` 中添加 `board_build.cdc_on_boot = yes`
- 确保 build_flags 中有 `-D ARDUINO_USB_CDC_ON_BOOT=1`
- 重新编译上传 ESP32 固件
- 重启 ESP32，断开重连 USB

**验证**: ESP32 启动日志正常显示。

### 2. micro-ROS 初始化失败 (2026-04-10)

**问题描述**: ESP32 WiFi 连接成功，但卡在 `Initializing rclc support...`，ROS 话题列表为空。

**根本原因**:

- micro-ROS Agent 未运行
- 版本不匹配，ESP32 与 Agent 需要同为 Jazzy

**解决方案**:

- 启动 micro-ROS Agent: `./build/micro_ros_agent/micro_ros_agent udp4 --port 8888`
- 更新 ESP32 库到 Jazzy: `lib_deps = https://github.com/micro-ROS/micro_ros_arduino.git#jazzy`
- 重新编译上传 ESP32

**验证**: Agent 显示连接日志，ROS 话题 `/imu/data` 和 `/robot/state` 出现。

### 3. 串口出现 `~...XRCE...` 乱码，ROS2 没有业务话题 (2026-04-11)

**问题描述**:

- ESP32 串口打印到 `Initializing rclc support...` 后出现 `~...XRCE...` 二进制乱码
- `ros2 topic list` 只有 `/parameter_events` 和 `/rosout`
- 日志中出现 `Setting micro-ROS transports...`

**根本原因**:

- 当前烧录到 ESP32 的固件不是本工程最新 WiFi-only 固件
- 旧固件或默认 transport 会把 micro-ROS XRCE-DDS 数据包写到 USB Serial，导致串口出现 `XRCE` 乱码
- 这时 ROS2 Agent 收不到来自 WiFi UDP 的 micro-ROS 节点创建请求，所以业务话题不会出现

**解决方案**:

- 重新编译并上传当前工程固件:
  ```bash
  cd /home/ina/Documents/PlatformIO/Projects/microros_node
  ./.venv/bin/python3 -m platformio run --target upload --upload-port /dev/ttyACM0
  ```
- 如果 `/dev/ttyACM0` 消失，关闭串口监视器，重新插拔 ESP32
- 必要时按住 `BOOT`，点按 `RESET/EN`，松开 `BOOT` 后再上传
- 上传后重新打开监视器，确认启动日志包含:
  ```text
  ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
  Setting micro-ROS WiFi transports...
  Pinging micro-ROS Agent over WiFi UDP...
  ```

**最终验证**:

- `ros2 topic list` 出现 `/imu/data`、`/imu/filtered`、`/robot/state`
- `ros2 topic echo /imu/data` 有 IMU 数据
- `ros2 topic echo /robot/state` 有状态数据

### 4. STM32 不发送 State 数据 (2026-04-10)

**问题描述**: ESP32 只接收 IMU 数据，没有 State 数据。

**根本原因**: STM32 代码只在 printf 中输出 State，未通过 UART 发送。

**解决方案**:

- 修改 `algo_task.c`，添加 UART 发送 State 数据:
  ```c
  char state_buffer[32];
  int state_len = snprintf(state_buffer, sizeof(state_buffer), "State:%d\n", features.anomaly_state);
  HAL_UART_Transmit(&huart1, (uint8_t*)state_buffer, state_len, HAL_MAX_DELAY);
  ```
- 重新编译上传 STM32 固件

**验证**: ESP32 接收到 `State:X` 格式数据，ROS2 `/robot/state` 有数据。

### 5. USB 连接不稳定 (2026-04-10)

**问题描述**: ESP32 经常断开连接，monitor 显示 `device disconnected`。

**可能原因**:

- USB 线缆接触不良
- 供电不足
- USB 端口问题

**解决方案**:

- 使用高质量 USB 数据线
- 尝试不同 USB 端口
- 检查 ESP32 供电，必要时考虑外部电源

**状态**: 链路已跑通，但 USB 线缆/供电仍建议保持稳定。

### 6. ACM0 进入下载模式或上传失败 (2026-04-11)

**问题描述**:

- 串口输出 `boot:0x23 (DOWNLOAD(USB/UART0))`
- 显示 `waiting for download`
- 或上传时报 `/dev/ttyACM0` 不存在

**原因分析**:

- ESP32-S3 复位时进入下载模式，不是 `main.cpp` 崩溃
- 串口监视器的 DTR/RTS 控制线可能触发自动复位/下载
- USB 断开后设备节点会消失，需要重新枚举

**解决方案**:

- 在 `platformio.ini` 中固定:
  ```ini
  monitor_dtr = 0
  monitor_rts = 0
  ```
- 上传前关闭所有串口监视器
- 必要时手动进入下载模式: 按住 `BOOT`，点按 `RESET/EN`，松开 `BOOT`
- 上传成功后按 `RESET/EN` 运行应用固件

## 配置总结

### ESP32 (platformio.ini)

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 921600
upload_speed = 921600

monitor_port = /dev/ttyACM0
monitor_dtr = 0
monitor_rts = 0

board_build.cdc_on_boot = yes

lib_deps =
    https://github.com/micro-ROS/micro_ros_arduino.git#jazzy

build_flags =
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D ARDUINO_ARCH_ESP32
    -D ESP32
    -D RMW_UXRCE_TRANSPORT_UDP
    -D RMW_UXRCE_TRANSPORT_IPV4
    -D CONFIG_MICRO_ROS_TRANSPORT_UDP_NAME=\"192.168.1.8\"
    -D CONFIG_MICRO_ROS_TRANSPORT_UDP_PORT=8888
    -L.pio/libdeps/esp32-s3-devkitc-1/micro_ros_arduino/src/esp32
    -lmicroros
    -D CONFIG_ESP_TASK_WDT_TIMEOUT_S=60
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_HAS_PSRAM
```

### WiFi 配置

真实配置保存在本地 `include/wifi_config.h`，不要提交到 Git。GitHub 中只保留 `include/wifi_config.example.h`:

```cpp
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define AGENT_IP "192.168.1.8"
#define AGENT_PORT 8888
```

### UART 配置

- ESP32 RX: GPIO16, TX: GPIO17, Baud: 921600
- STM32 UART1: 连接到 ESP32 UART

## 运行步骤

1. 启动 micro-ROS Agent:
   ```bash
   cd /home/ina/microros_ws
   source /opt/ros/jazzy/setup.bash
   ./build/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v 6
   ```

2. 上传 ESP32 固件:
   ```bash
   cd /home/ina/Documents/PlatformIO/Projects/microros_node
   ./.venv/bin/python3 -m platformio run --target upload --upload-port /dev/ttyACM0
   ```

3. 监控 ESP32:
   ```bash
   ./.venv/bin/python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
   ```

   正常启动日志应包含:
   ```text
   ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
   Setting micro-ROS WiFi transports...
   Pinging micro-ROS Agent over WiFi UDP...
   micro-ROS Agent reachable
   micro-ROS connected!
   ```

4. 查看 ROS2 话题:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   ros2 topic echo /imu/data
   ros2 topic echo /imu/filtered
   ros2 topic echo /robot/state
   ```

## 待解决

- 根据实际 IMU 噪声填写 covariance
- `/imu/filtered` 当前使用单位四元数占位，后续可接入真实姿态解算
- USB 线缆/供电仍建议保持稳定，避免 `/dev/ttyACM0` 枚举中断

## 版本信息

- ESP32: micro-ROS Arduino jazzy
- Agent: micro-ROS Agent jazzy
- ROS2: Jazzy Jalisco
- PlatformIO: espressif32
- 固件标识: `ESP32-S3 micro-ROS Bridge v1.1 WiFi-only`
