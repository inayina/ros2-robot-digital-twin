# ROS 2 Robot Digital Twin V2

基于 STM32、ESP32 micro-ROS、ROS 2 Jazzy 和 Gazebo Harmonic 的无线状态监测与数字孪生系统。

V2 在 V1 闭环验证基础上，把 6 轴姿态解算下沉到 STM32，新增 `IMUQ` 四元数串口帧，ESP32 将姿态写入 ROS 2 `sensor_msgs/Imu.orientation`，并加入 PC 端 Python 数据记录和实时曲线分析节点。

## V2 状态

已完成：

- STM32F411 采样 MPU6050，输出 `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp` 和 `State:x`
- STM32 保留 RMS 阈值状态判别、本地三色 LED 报警、训练/ROS 双输出模式
- ESP32-S3 通过 WiFi UDP 连接 micro-ROS Agent，并发布 `/imu/data`、`/imu/filtered`、`/robot/state`
- `/imu/filtered` 对线加速度和角速度做一阶低通滤波，保留 STM32 解算出的四元数姿态
- Gazebo Harmonic 数字孪生包保留 V1 可视化闭环
- 新增 `imu_data_logger`，支持 CSV/JSONL 记录和 raw/filtered IMU 实时曲线对比

仍在推进：

- AHT20/BMP280 环境传感器接入
- XGBoost 嵌入式推理替换当前 RMS 阈值基线
- 更完整的数据集采集、训练、评估流程

## System Architecture

```text
MPU6050
  |
  v
STM32F411 sensor node
  |  UART 921600, IMUQ + State frames
  v
ESP32-S3 micro-ROS bridge
  |  WiFi UDP :8888
  v
micro-ROS Agent
  |
  v
ROS 2 Jazzy topics
  |  /imu/data, /imu/filtered, /robot/state
  v
Gazebo Harmonic digital twin + Python analysis tools
```

## Repository Layout

```text
firmware/
  stm32_sensor_node/          # STM32F411 + FreeRTOS + MPU6050 + attitude estimator + LED alarm
  esp32_microros_bridge/      # ESP32-S3 PlatformIO micro-ROS bridge
ros2/
  robot_state_monitor/        # ROS 2 package and Gazebo Harmonic bridge
  imu_data_logger/            # ROS 2 Python logger and live plot analysis nodes
docs/
  resume-bullets.md           # 简历项目描述草稿
```

## Module Responsibilities

### STM32 Sensor Node

Path: `firmware/stm32_sensor_node`

- MPU6050 采样
- 6 轴姿态解算，输出四元数
- RMS 阈值状态判别基线
- 三色 LED 状态报警
- UART 输出 `IMUQ`、旧 `IMU` 兼容格式和 `State:x`

### ESP32 micro-ROS Bridge

Path: `firmware/esp32_microros_bridge`

- 通过 UART 接收 STM32 数据
- 启动后请求 STM32 进入 `G` Gazebo/ROS 输出模式
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

### IMU Data Logger

Path: `ros2/imu_data_logger`

- `imu_logger`：记录 `/imu/data`、`/imu/filtered` 到 CSV，动态记录 `/robot/state` 到 JSONL
- `imu_live_plot`：实时对比 raw/filtered 线加速度、角速度和四元数换算出的 roll/pitch/yaw

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

### 3. Build STM32 Firmware

```bash
cd firmware/stm32_sensor_node
cmake --preset Debug
cmake --build --preset Debug
```

### 4. Build and Upload ESP32

```bash
cd firmware/esp32_microros_bridge
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

Expected ESP32 log:

```text
ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
Requested STM32 GAZEBO mode
Pinging micro-ROS Agent over WiFi UDP...
micro-ROS Agent reachable
micro-ROS connected!
```

### 5. Build ROS 2 Packages

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor imu_data_logger
source install/setup.bash
```

### 6. Verify ROS 2 Topics

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

### 7. Run Data Logger

```bash
source /opt/ros/jazzy/setup.bash
ros2 run imu_data_logger imu_logger --ros-args -p output_dir:=./data/imu_run_001
```

### 8. Run Live Plot

```bash
source /opt/ros/jazzy/setup.bash
ros2 run imu_data_logger imu_live_plot
```

### 9. Run Gazebo Digital Twin

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
```
