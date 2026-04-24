# Robot State Monitor

基于 STM32、ESP32 micro-ROS、ROS 2 Jazzy 和 Gazebo Harmonic 的无线 IMU 状态监测与数字孪生项目。

这个发布版已经整理并同步了当前工作区里的关键改动，包含：

- STM32 输出四元数姿态帧和状态帧
- ESP32 micro-ROS 桥接固件的串口解析/发布模块化重构
- Gazebo 可视化桥接改为直接使用消息四元数，并默认锁定初始 yaw
- `imu_data_logger` 针对 `/robot/state` 的 `BEST_EFFORT` 订阅兼容性修复

## 当前集成状态

- STM32 侧负责 MPU6050 采样、姿态解算、状态判别和串口输出
- ESP32-S3 通过 WiFi UDP 连接 micro-ROS Agent，发布：
  - `/imu/data`
  - `/imu/filtered`
  - `/robot/state`
- `/imu/filtered` 对线加速度和角速度做一阶低通滤波，并保留输入帧中的四元数姿态
- `robot_state_monitor` 在 Gazebo Harmonic 中生成可见的 `mpu6050` 标记模型，并根据 `/imu/filtered` 更新姿态
- `imu_data_logger` 支持 CSV、JSONL 记录和原始/滤波 IMU 的实时曲线对比

## 仓库结构

```text
firmware/
  stm32_sensor_node/          # STM32F411 + FreeRTOS + MPU6050 + 姿态/状态输出
  esp32_microros_bridge/      # ESP32-S3 PlatformIO micro-ROS 桥接固件
ros2/
  robot_state_monitor/        # Gazebo Harmonic 可视化桥接包
  imu_data_logger/            # ROS 2 Python 记录与实时绘图包
docs/
  resume-bullets.md           # 简历项目描述草稿
```

## 这次整理同步了什么

### `firmware/esp32_microros_bridge`

- 将串口协议解析拆分到 `src/stm32_serial_parser.cpp`
- 将 micro-ROS 发布逻辑拆分到 `src/uros_pub.cpp`
- `main.cpp` 只保留 WiFi、任务调度、调试输出和模块组装
- 发布端改为 `best_effort` QoS，更贴近传感器流场景

### `ros2/robot_state_monitor`

- Gazebo bridge 明确使用 `BEST_EFFORT` QoS 订阅硬件 IMU
- 不再在桥接节点内做姿态估算，直接使用消息中的四元数
- 增加 `lock_yaw` 参数，默认锁定初始 yaw，减轻 MPU6050 漂移带来的慢速自转
- `README.md` 和 `Debug.md` 已同步到当前工作区描述

### `ros2/imu_data_logger`

- `imu_logger` 对动态发现的 `/robot/state` 订阅显式使用 `QoSProfile(depth=10, reliability=BEST_EFFORT)`
- `README.md` 已补充记录方式、输出文件格式和绘图参数说明

## 快速开始

### 1. 配置 ESP32 本地 WiFi

真实 WiFi 凭据不要提交到 Git。先复制示例文件：

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

如果 Agent IP 变化，也同步修改 `platformio.ini` 里的
`CONFIG_MICRO_ROS_TRANSPORT_UDP_NAME` 和 `CONFIG_MICRO_ROS_TRANSPORT_UDP_PORT`。

### 2. 启动 micro-ROS Agent

```bash
cd /home/ina/microros_ws
source /opt/ros/jazzy/setup.bash
./build/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v 6
```

### 3. 构建并上传 ESP32 固件

```bash
cd firmware/esp32_microros_bridge
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

正常日志应包含：

```text
ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
Requested STM32 GAZEBO mode
Setting micro-ROS WiFi transports...
Pinging micro-ROS Agent over WiFi UDP...
micro-ROS Agent reachable
micro-ROS connected!
```

### 4. 构建 ROS 2 包

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor imu_data_logger
source install/setup.bash
```

### 5. 验证 ROS 2 数据流

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep imu
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
```

### 6. 记录数据

```bash
ros2 run imu_data_logger imu_logger --ros-args -p output_dir:=./data/imu_run_001
```

### 7. 启动 Gazebo 数字孪生

先跑无头模式：

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

确认模型已经生成：

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

GUI 需要时再启动：

```bash
ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
```

## 重要说明

- `robot_state_monitor` 默认以 `best_effort` 订阅 `/imu/filtered`，用于匹配硬件端传感器流 QoS
- Gazebo bridge 默认 `lock_yaw:=true`；如果你想让 yaw 跟随消息里的值，可以在 launch 时传 `lock_yaw:=false`
- ESP32 固件默认不把训练 CSV 当作 IMU 帧发布；如需临时兼容，可在 `firmware/esp32_microros_bridge/src/main.cpp` 中把 `ACCEPT_TRAIN_CSV_AS_IMU` 改成 `true`
- 如果 Gazebo GUI 因 EGL/NVIDIA 问题闪退，优先使用无头模式验证链路

## 相关文档

- `firmware/stm32_sensor_node/README.md`
- `firmware/esp32_microros_bridge/README.md`
- `ros2/robot_state_monitor/README.md`
- `ros2/imu_data_logger/README.md`
