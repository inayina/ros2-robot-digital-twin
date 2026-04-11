# Robot State Monitor V1

基于 STM32、ESP32 micro-ROS、ROS 2 Jazzy 和 Gazebo Harmonic 的无线状态监测与数字孪生演示系统。

V1 已完成从真实 MPU6050 传感器到 ROS 2 话题，再到 Gazebo 可视化模型的闭环验证。STM32 负责传感器采样和状态输出，ESP32-S3 负责 UART 到 micro-ROS 的无线桥接，PC 端通过 ROS 2 和 Gazebo 显示实时姿态。

## V1 验证状态

已验证：

- STM32 通过 UART 输出 `IMU,ax,ay,az,gx,gy,gz,temp` 和 `State:x`
- ESP32-S3 通过 WiFi UDP 连接 micro-ROS Agent
- ROS 2 Jazzy 中可见 `/imu/data`、`/imu/filtered`、`/robot/state`
- `/imu/data` 和 `/robot/state` 已有实机数据输出
- Gazebo Harmonic 中的 `mpu6050` 标记可跟随真实 MPU6050 板子的 roll/pitch 姿态变化

## System Architecture

```text
MPU6050
  |
  v
STM32F411 sensor node
  |  UART 921600
  v
ESP32-S3 micro-ROS bridge
  |  WiFi UDP :8888
  v
micro-ROS Agent
  |
  v
ROS 2 Jazzy topics
  |  /imu/filtered
  v
Gazebo Harmonic digital twin
```

## Repository Layout

```text
firmware/
  stm32_sensor_node/          # STM32F411 + FreeRTOS + MPU6050 + LED alarm
  esp32_microros_bridge/      # ESP32-S3 PlatformIO micro-ROS bridge
ros2/
  robot_state_monitor/        # ROS 2 package and Gazebo Harmonic bridge
docs/
  resume-bullets.md           # 简历项目描述草稿
```

## Module Responsibilities

### STM32 Sensor Node

Path: `firmware/stm32_sensor_node`

- MPU6050 采样
- FreeRTOS 任务链路
- RMS 阈值状态判别基线
- 三色 LED 状态报警
- UART 输出 IMU 和状态帧

### ESP32 micro-ROS Bridge

Path: `firmware/esp32_microros_bridge`

- 通过 UART 接收 STM32 数据
- 通过 WiFi UDP 连接 micro-ROS Agent
- 发布 ROS 2 话题：
  - `/imu/data`
  - `/imu/filtered`
  - `/robot/state`
- 初始化前 ping Agent，避免 Agent 不可达时卡在 `rclc_support_init`

### ROS 2 Gazebo Monitor

Path: `ros2/robot_state_monitor`

- 订阅 `/imu/filtered`
- 自动生成 Gazebo `mpu6050` 可视化模型
- 通过 Gazebo Harmonic 服务更新模型姿态
- 在 ROS 2 侧使用互补滤波估计 roll/pitch/yaw，用于展示真实板子的姿态变化

## Quick Start

### 1. Configure ESP32 WiFi

真实 WiFi 密码不要提交到 Git。先创建本地配置文件：

```bash
cd firmware/esp32_microros_bridge
cp include/wifi_config.example.h include/wifi_config.h
```

然后编辑 `include/wifi_config.h`：

```cpp
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define AGENT_IP "192.168.1.8"
#define AGENT_PORT 8888
```

如果 Agent IP 变化，也同步修改 `platformio.ini` 里的 `CONFIG_MICRO_ROS_TRANSPORT_UDP_NAME`。

### 2. Start micro-ROS Agent

```bash
cd /home/ina/microros_ws
source /opt/ros/jazzy/setup.bash
./build/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v 6
```

### 3. Build and Upload ESP32

```bash
cd firmware/esp32_microros_bridge
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

Expected ESP32 log:

```text
ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
Setting micro-ROS WiFi transports...
Pinging micro-ROS Agent over WiFi UDP...
micro-ROS Agent reachable
micro-ROS connected!
```

### 4. Build ROS 2 Package

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor
source install/setup.bash
```

### 5. Verify ROS 2 Topics

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
```

Expected topic list includes:

```text
/imu/data
/imu/filtered
/robot/state
```

### 6. Run Gazebo Digital Twin

```bash
