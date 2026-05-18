# Robot State Monitor

基于 STM32、ESP32 micro-ROS、ROS 2 Jazzy 和 Gazebo Harmonic 的无线 IMU 状态监测与数字孪生项目，并逐步把 N20 编码器电机闭环控制主线迁移到 ESP32。

这个整合仓库已经同步到当前联调版本，当前系统分成三层：

- STM32F411：100 Hz 采样 MPU6050，完成 6 轴姿态解算、窗口 RMS 状态判别、LED 报警；既有 `CMDVEL -> TB6612 A 路` 只保留为 legacy open-loop 验证链路
- ESP32-S3：通过 UART 解析 STM32 文本帧，通过 WiFi UDP 建立 micro-ROS 链路，发布 `/imu/data`、`/imu/filtered`、`/robot/state`，订阅 `/cmd_vel` 和 `/motor/target_rpm`，并已落地双核 motor-control skeleton
- ROS 2 Jazzy：提供 Gazebo Harmonic 数字孪生、IMU CSV/状态 JSONL 记录，以及 raw/filtered 实时曲线可视化

## 仓库结构

```text
firmware/
  stm32_sensor_node/          # STM32F411 + FreeRTOS + MPU6050 + 状态识别 + legacy 电机执行
  esp32_microros_bridge/      # ESP32-S3 PlatformIO micro-ROS 网桥 + motor-control skeleton
ros2/
  robot_state_monitor/        # Gazebo Harmonic 可视化桥接包
  imu_data_logger/            # ROS 2 Python 记录与实时绘图包
docs/
  resume-bullets.md
  data-flow.md              # 当前数据流与话题/串口关系
```

## 当前数据流

完整说明见 `docs/data-flow.md`。这里先放一版当前主链路总览：

```text
启动协商
ESP32 上电
  -> UART "G\n"
  -> STM32 切到 GAZEBO 模式

上行观测链路
MPU6050
  -> STM32 SensorTask (100 Hz 采样 + 6轴姿态解算)
  -> IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp  (~50 Hz, UART 921600)
  -> ESP32 stm32_serial_parser
  -> /imu/data      (best_effort)
  -> 一阶低通 alpha=0.2
  -> /imu/filtered  (best_effort)
  -> robot_state_monitor / imu_data_logger

状态判别链路
STM32 SensorQueue
  -> AlgTask (10 样本窗口 RMS 基线)
  -> State:<n>  (~10 Hz)
  -> ESP32 stm32_serial_parser
  -> /robot/state  (best_effort)
  -> imu_data_logger

下行控制链路
ROS 2 /cmd_vel (geometry_msgs/Twist, ESP32 侧 reliable subscriber)
  -> ESP32 subscriber / executor
  -> CMDVEL,<linear_x>,<angular_z>\n
  -> STM32 UART 行解析
  -> MotorTask (10 ms 调度, 200 ms 超时急停)
  -> TB6612 A 路单电机

ESP32 本地电机控制骨架
ROS 2 /motor/target_rpm (std_msgs/Float32)
  -> ESP32 ros_comm_task (Core 0)
  -> shared motor command
  -> motor_control_task (Core 1)
  -> mock motor response
  -> /motor/actual_rpm + /motor/state
```

## 当前接口

- `STM32 -> ESP32`：`IMUQ,...`、`IMU,...`（兼容旧格式）、`State:<n>`、`DBG:...`
- `ESP32 -> STM32`：`CMDVEL,<linear_x>,<angular_z>\n`
- ROS 2 话题：
  - `/imu/data`：`sensor_msgs/msg/Imu`，`best_effort`
  - `/imu/filtered`：`sensor_msgs/msg/Imu`，`best_effort`
  - `/robot/state`：`std_msgs/msg/Int32`，`best_effort`
  - `/cmd_vel`：`geometry_msgs/msg/Twist`，ESP32 侧默认 `reliable` 订阅
  - `/motor/target_rpm`：`std_msgs/msg/Float32`，ESP32 侧默认 `reliable` 订阅
  - `/motor/actual_rpm`：`std_msgs/msg/Float32`，当前为 mock motor response
  - `/motor/state`：`std_msgs/msg/String`，当前为 mock motor state JSON 字符串

## 快速开始

### 1. 配置 ESP32 本地 WiFi

```bash
cd firmware/esp32_microros_bridge
cp include/wifi_config.example.h include/wifi_config.h
```

编辑 `include/wifi_config.h`，填入你当前网络的 SSID、密码和 Agent 地址。真实凭据不要提交到 Git。

### 2. 启动 micro-ROS Agent

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 3. 构建并上传 ESP32

```bash
cd firmware/esp32_microros_bridge
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

正常日志应包含：

```text
ESP32-S3 micro-ROS Bridge v1.2 dual-core - Starting...
STM32 Serial initialized
Requested STM32 GAZEBO mode
WiFi Connected! IP: ...
micro-ROS WiFi transport configured
Connecting to micro-ROS Agent at ...:8888
micro-ROS connected!
```

如果需要重新编译或烧录 STM32 固件，见 `firmware/stm32_sensor_node/README.md`。

### 4. 构建 ROS 2 包

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor imu_data_logger
source install/setup.bash
```

### 5. 验证数据流

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
ros2 topic info -v /cmd_vel
ros2 topic echo --once /motor/actual_rpm
ros2 topic echo --once /motor/state
```

### 6. 记录数据或启动 Gazebo

记录：

```bash
ros2 run imu_data_logger imu_logger --ros-args -p output_dir:=./data/imu_run_001
```

Gazebo：

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

## 重要说明

- `robot_state_monitor` 默认以 `best_effort` 订阅 `/imu/filtered`
- Gazebo bridge 默认 `lock_yaw:=true`
- `/imu/filtered` 只对线加速度和角速度做一阶低通，保留 STM32 原始姿态四元数
- ESP32 默认不把训练 CSV 当作正式 IMU 输入
- 当前默认不在运行态周期性 `ping` Agent
- ESP32 本地电机闭环当前仍是 skeleton：已实现 `target_rpm` 接收、mock `actual_rpm`、`/motor/state`、host tests 和 TB6612 驱动边界，但真实 PWM / 编码器 / PID 还未接入
- 真实 WiFi 凭据只放在 `firmware/esp32_microros_bridge/include/wifi_config.h`，上传仓库只保留 `wifi_config.example.h`

## 相关文档

- `docs/data-flow.md`
- `firmware/stm32_sensor_node/README.md`
- `firmware/stm32_sensor_node/design.md`
- `firmware/esp32_microros_bridge/README.md`
- `firmware/esp32_microros_bridge/design.md`
- `ros2/robot_state_monitor/README.md`
- `ros2/imu_data_logger/README.md`
