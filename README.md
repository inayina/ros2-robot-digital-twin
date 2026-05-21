# Robot State Monitor

基于 STM32、ESP32 micro-ROS、ROS 2 Jazzy 和 Gazebo Harmonic 的无线 IMU 状态监测与数字孪生项目，并逐步把 N20 编码器电机闭环控制主线迁移到 ESP32。

这个整合仓库已经同步到当前联调版本，当前系统分成三层，并额外提供面向 dashboard backend 的 MQTT 状态桥：

- STM32F411：100 Hz 采样 MPU6050，完成 6 轴姿态解算、窗口 RMS 状态判别、LED 报警；既有 `CMDVEL -> TB6612 A 路` 只保留为 legacy open-loop 验证链路
- ESP32-S3：通过 UART 解析 STM32 文本帧，通过 WiFi UDP 建立 micro-ROS 链路，发布 `/imu/data`、`/imu/filtered`、`/robot/state`、`/motor/status`，订阅 `/cmd_vel`、`/motor/target_rpm` 和 `/motor/cmd`，并已落地双核 motor-control skeleton
- ROS 2 Jazzy：提供 Gazebo Harmonic 数字孪生、IMU CSV/状态 JSONL 记录、raw/filtered 实时曲线可视化，以及 `robot_mqtt_bridge` 电机状态 MQTT 输出

这个仓库已经和 `robot-ops-dashboard` 做过一轮联调。当前对外口径不再写“监控优先”，而是按访问形态收口为 `2` 条交互链路和 `1` 条只读链路：

- 交互链路 1：WMS 链路，来自 `/home/ina/ros2_ws/src/amr_warehouse_sim` 的 Mock WMS 任务流；这条链路之前已经做过调试和验证
- 交互链路 2：`POST /api/robot/motor/cmd -> MQTT robot/motor/cmd -> ROS 2 /motor/cmd -> ESP32 motor_control_task`
- 只读链路：`robot/imu` 与 `robot/motor/status` 由 backend / frontend 消费，不从 dashboard 侧反写到底层设备
- 补充：`/cmd_vel -> ESP32 -> STM32 -> TB6612 A 路` 仍保留为 legacy 本地控制 / 验证链路，但不再作为这里的“交互链路 1”

## 仓库结构

```text
firmware/
  stm32_sensor_node/          # STM32F411 + FreeRTOS + MPU6050 + 状态识别 + legacy 电机执行
  esp32_microros_bridge/      # ESP32-S3 PlatformIO micro-ROS 网桥 + motor-control skeleton
ros2/
  robot_state_monitor/        # Gazebo Harmonic 可视化桥接包
  imu_data_logger/            # ROS 2 Python 记录与实时绘图包
  robot_mqtt_bridge/          # ROS 2 -> MQTT dashboard telemetry bridge
archive/
  robot_status_api_bridge_legacy/  # 已归档的旧 JSON/HTTP 状态聚合桥
docs/
  resume-bullets.md
  data-flow.md              # 当前数据流与话题/串口关系
  dashboard-integration.md  # dashboard / backend 集成边界
  motor_dashboard_interface.md # 电机状态/控制接口契约
  motor_dashboard_progress_2026_05_20.md # 当前联调进度与阻塞
  robot_ops_dashboard_handoff.md # 给 robot-ops-dashboard 的对接说明
  pre_n20_regression_check.md # N20 接入前通信负载回归检查
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
  -> /motor/status + /motor/actual_rpm + /motor/state

Dashboard 电机控制链路
robot-ops-dashboard frontend
  -> backend POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> ros2/robot_mqtt_bridge
  -> ROS 2 /motor/cmd
  -> ESP32 shared motor command
  -> motor_control_task (enable / max_pwm / timeout / stop)

Dashboard IMU 链路
ROS 2 /imu/data 或 /imu/filtered
  -> robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py
  -> MQTT robot/imu
  -> dashboard backend
  -> robot-ops-dashboard frontend

Dashboard 电机链路
ROS 2 /motor/status
  -> ros2/robot_mqtt_bridge
  -> MQTT robot/motor/status
  -> dashboard backend
  -> robot-ops-dashboard frontend
```

按 dashboard 对接口径看，上面这些链路可以归并为：

- 交互链路 1：`amr_warehouse_sim` Mock WMS
- 交互链路 2：`/motor/cmd`
- 只读链路：`robot/imu` + `robot/motor/status`
- 额外保留：`/cmd_vel` legacy 本地控制链路

## 当前接口

- `STM32 -> ESP32`：`IMUQ,...`、`IMU,...`（兼容旧格式）、`State:<n>`、`DBG:...`
- `ESP32 -> STM32`：`CMDVEL,<linear_x>,<angular_z>\n`
- ROS 2 话题：
  - `/imu/data`：`sensor_msgs/msg/Imu`，`best_effort`
  - `/imu/filtered`：`sensor_msgs/msg/Imu`，`best_effort`
  - `/robot/state`：`std_msgs/msg/Int32`，`best_effort`
  - `/cmd_vel`：`geometry_msgs/msg/Twist`，ESP32 侧默认 `reliable` 订阅
  - `/motor/target_rpm`：`std_msgs/msg/Float32`，ESP32 侧默认 `reliable` 订阅
  - `/motor/cmd`：`std_msgs/msg/String`，ESP32 侧默认 `reliable` 订阅，携带 enable / max_pwm / timeout / stop 约束后的电机命令 JSON
  - `/motor/status`：`std_msgs/msg/String`，当前主电机状态 JSON 字符串
  - `/motor/actual_rpm`：`std_msgs/msg/Float32`，当前为 mock motor response
  - `/motor/state`：`std_msgs/msg/String`，兼容旧调试链路的电机状态 JSON 字符串
- dashboard backend 输入：
  - `robot/imu`：由 `robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py` 从 ROS 2 IMU topic 低频镜像
  - `robot/motor/status`：由 `ros2/robot_mqtt_bridge` 从 `/motor/status` 转发
- dashboard backend 输出：
  - `robot/motor/cmd`：由 `robot-ops-dashboard` backend 发布，经过 `ros2/robot_mqtt_bridge` 转成 `/motor/cmd`
- dashboard 对外分工：
  - 交互：`/home/ina/ros2_ws/src/amr_warehouse_sim` Mock WMS 任务态链路
  - 交互：`POST /api/robot/motor/cmd` / `robot/motor/cmd` 主电机命令链路
  - 只读：`robot/imu`、`robot/motor/status`
  - 额外保留：`/cmd_vel` legacy 控制链路

## 快速开始

### 1. 配置 ESP32 本地 WiFi

```bash
cd firmware/esp32_microros_bridge
cp include/wifi_config.example.h include/wifi_config.h
```

编辑 `include/wifi_config.h`，填入你当前网络的 SSID、密码和 Agent 地址。真实凭据不要提交到 Git。

### 2. Start micro-ROS Agent

micro-ROS Agent 是外部依赖，不把源码或 `micro_ros_setup` vendor 到本仓库。默认推荐外部 workspace 使用 `~/uros_ws` 或 `~/micro_ros_ws`；如果路径不同，先设置 `MICRO_ROS_AGENT_SETUP` 指向对应的 `install/local_setup.bash`。

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/start_microros_agent_udp.sh
```

Agent 默认使用 UDP 端口 `8888`。启动 Agent 后再复位 ESP32，等待串口日志出现 `micro-ROS connected!`。

接 N20 前必须先确认 micro-ROS Agent 和 topic 正常：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/check_microros_topics.sh
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
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select \
  robot_state_monitor imu_data_logger robot_mqtt_bridge \
  --symlink-install
source install/setup.bash
```

### 5. 验证数据流

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
ros2 topic info -v /cmd_vel
ros2 topic info -v /motor/target_rpm
ros2 topic info -v /motor/cmd
ros2 topic echo --once /motor/status
ros2 topic echo --once /motor/actual_rpm
ros2 topic echo --once /motor/state
```

如果已经完成烧录并复位板子，推荐直接运行仓库脚本做整链检查：

```bash
./scripts/check_real_hw_chain.sh
./scripts/check_real_hw_chain.sh --dashboard
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

dashboard 电机 MQTT bridge：

```bash
ros2 run robot_mqtt_bridge motor_status_bridge --ros-args \
  -p motor_status_topic:=/motor/status \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p mqtt_topic:=robot/motor/status

ros2 run robot_mqtt_bridge motor_cmd_bridge --ros-args \
  -p ros_cmd_topic:=/motor/cmd \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p mqtt_topic:=robot/motor/cmd
```

dashboard IMU MQTT bridge 目前位于外部仓库 `robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py`。

整链一键启动与联调检查：

```bash
./scripts/start_motor_dashboard_stack.sh
./scripts/check_motor_dashboard_loop.sh
```

## 重要说明

- `robot_state_monitor` 默认以 `best_effort` 订阅 `/imu/filtered`
- Gazebo bridge 默认 `lock_yaw:=true`
- `/imu/filtered` 只对线加速度和角速度做一阶低通，保留 STM32 原始姿态四元数
- ESP32 默认不把训练 CSV 当作正式 IMU 输入
- 当前默认不在运行态周期性 `ping` Agent
- ESP32 本地电机控制当前已补齐 `/motor/cmd`、`/motor/status`、`robot/motor/cmd -> /motor/cmd` 主链路，以及 `enable`、`max_pwm`、命令超时和 `stop` 优先级约束；`/motor/status` 等状态仍以 mock telemetry 为主，但 `kEnableMotorHardwareOutputs` 当前已打开，TB6612 B 路会在 `enabled / control_enabled / timeout / estop / fault` 这些安全门满足时输出，占空比仍受 `max_pwm` 约束
- 接真实 N20 前必须先通过 `docs/pre_n20_regression_check.md`：当前只验证双核任务隔离、发布限频、mock motor telemetry 和 `motor_control_task` jitter
- STM32 可以继续 `100 Hz` 采样，但 ESP32 不需要把所有数据都 `100 Hz` 发布到 ROS 2；当前默认 `/imu/data` 为 `50 Hz`，`/imu/filtered` 为 `25 Hz`
- ESP32 本地电机控制可以保持 `10 ms / 100 Hz`，但 ROS / MQTT / dashboard 状态回传必须降频；当前默认 `/motor/actual_rpm` 为 `20 Hz`，`/motor/status` 与 `/motor/state` JSON 为 `5 Hz`
- 真实 WiFi 凭据只放在 `firmware/esp32_microros_bridge/include/wifi_config.h`，上传仓库只保留 `wifi_config.example.h`

## 相关文档

- `docs/data-flow.md`
- `docs/dashboard-integration.md`
- `docs/robot_ops_dashboard_handoff.md`
- `docs/microros_agent_startup.md`
- `docs/pre_n20_regression_check.md`
- `firmware/stm32_sensor_node/README.md`
- `firmware/stm32_sensor_node/design.md`
- `firmware/esp32_microros_bridge/README.md`
- `firmware/esp32_microros_bridge/design.md`
- `ros2/robot_state_monitor/README.md`
- `ros2/imu_data_logger/README.md`
- `ros2/robot_mqtt_bridge/README.md`
- `archive/robot_status_api_bridge_legacy/README.md`
