# Robot Real-Hardware State And Motor Control

本仓库是 Robot Ops Dashboard 的实机状态与电机控制子系统仓库。

它负责把 STM32 / ESP32 / micro-ROS / IMU / N20 电机控制链路整理成 ROS 2 和 MQTT/backend 可消费的数据源，让 dashboard 可以展示真实下位机状态、IMU 姿态、机器人状态判别、电机目标 / 反馈 / PWM / 安全状态，并通过 backend 下发低频人工电机命令。

## Related Repositories

- [robot-ops-dashboard](https://github.com/inayina/robot-ops-dashboard)：dashboard frontend/backend，消费本仓库输出的 `robot/imu`、`robot/motor/status`，并通过 `robot/motor/cmd` 下发电机命令。
- [amr_warehouse_navigation](https://github.com/inayina/amr_warehouse_navigation)：仓储 AMR 导航 / WMS 任务态相关仓库，作为 dashboard 的任务与导航侧关联系统。
- [ros2-robot-digital-twin](https://github.com/inayina/ros2-robot-digital-twin)：本仓库原始仓库名；当前内容已经收口为实机状态与电机控制子系统。

## Role In The System

本仓库位于机器人硬件和 Robot Ops Dashboard 之间：

```text
STM32 / ESP32 real hardware
  -> micro-ROS / ROS 2 topics
  -> ROS 2 MQTT bridge or dashboard backend bridge
  -> Robot Ops Dashboard backend
  -> Robot Ops Dashboard frontend
```

当前边界：

- STM32F411：采样 MPU6050、输出 `IMUQ`、执行 RMS 状态判别、保留 legacy `/cmd_vel -> TB6612 A` 执行链路。
- ESP32-S3：解析 STM32 UART、建立 micro-ROS UDP 链路、发布 IMU / robot state / motor state，并承接 N20 编码器电机控制主线。
- ROS 2 Jazzy：运行 micro-ROS Agent、接收 topic、运行记录 / 可视化 / MQTT bridge。
- MQTT/backend：把低频状态镜像给 Robot Ops Dashboard backend；dashboard frontend 不直接连接 ROS 2、ESP32 或 micro-ROS runtime。

本仓库不负责 dashboard UI，不把 dashboard frontend 直接接到 ROS 2 或 ESP32，也不引入 `ros2_control` 作为当前主链路。

## Main Data Paths

完整数据流见 [docs/dataflow.md](docs/dataflow.md)。

```text
IMU / robot state:
MPU6050
  -> STM32 SensorTask / AlgTask
  -> IMUQ,... and State:<n>
  -> ESP32 UART parser
  -> /imu/data, /imu/filtered, /robot/state
  -> MQTT/backend
  -> Robot Ops Dashboard

Motor command and status:
Robot Ops Dashboard frontend
  -> dashboard backend POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> ros2/robot_mqtt_bridge
  -> ROS 2 /motor/cmd
  -> ESP32 motor_control_task
  -> TB6612 / N20 bench chain
  -> /motor/status
  -> MQTT robot/motor/status
  -> dashboard backend/frontend

Legacy local control:
ROS 2 /cmd_vel
  -> ESP32
  -> CMDVEL,<linear_x>,<angular_z>
  -> STM32
  -> TB6612 A channel
```

## Repository Layout

```text
firmware/
  stm32_sensor_node/          # STM32F411 + FreeRTOS + MPU6050 + state detection + legacy motor execution
  esp32_microros_bridge/      # ESP32-S3 PlatformIO micro-ROS bridge + N20 motor-control skeleton
ros2/
  robot_state_monitor/        # Gazebo Harmonic visual bridge
  imu_data_logger/            # IMU CSV/JSONL logging and live plot tools
  robot_mqtt_bridge/          # ROS 2 <-> MQTT bridge for dashboard motor topics
archive/
  robot_status_api_bridge_legacy/
docs/
  dataflow.md                 # canonical end-to-end hardware -> dashboard data chain
  data-flow.md                # older data-flow document kept for existing links
  hardware_wiring.md          # STM32 / ESP32 / MPU6050 / TB6612 / N20 wiring notes
  motor_control_design.md     # top-level motor control design and safety boundary
  troubleshooting_log.md      # serial, QoS, Agent, I2C, power troubleshooting notes
  dashboard-integration.md
  motor_dashboard_interface.md
  motor_closed_loop_tuning_process.md
  pre_n20_regression_check.md
```

## Current Interfaces

Protected UART protocol:

- STM32 -> ESP32: `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
- STM32 -> ESP32: `IMU,ax,ay,az,gx,gy,gz,temp` legacy compatibility
- STM32 -> ESP32: `State:<n>`
- ESP32 -> STM32: `CMDVEL,<linear_x>,<angular_z>\n`
- ESP32 -> STM32: `G\n` startup request for Gazebo / ROS output mode

Protected ROS 2 topics:

| Topic | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | ESP32 -> ROS 2 | Raw IMU plus STM32 quaternion |
| `/imu/filtered` | `sensor_msgs/msg/Imu` | ESP32 -> ROS 2 | Low-pass acceleration / gyro for visualization |
| `/robot/state` | `std_msgs/msg/Int32` | ESP32 -> ROS 2 | STM32 state classifier, `0 normal`, `1 vibration`, `2 collision`, `3 tip_over` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 -> ESP32 -> STM32 | Legacy local control path |
| `/motor/target_rpm` | `std_msgs/msg/Float32` | ROS 2 -> ESP32 | Direct target RPM debug path |
| `/motor/cmd` | `std_msgs/msg/String` | ROS 2 -> ESP32 | Main dashboard motor command entry |
| `/motor/status` | `std_msgs/msg/String` | ESP32 -> ROS 2 | Main motor status JSON for dashboard |
| `/motor/actual_rpm` | `std_msgs/msg/Float32` | ESP32 -> ROS 2 | RPM feedback/debug topic |
| `/motor/state` | `std_msgs/msg/String` | ESP32 -> ROS 2 | Legacy compatible motor state JSON |

Dashboard-facing MQTT topics:

- `robot/imu`：dashboard backend 消费的 IMU 状态镜像。
- `robot/motor/status`：dashboard backend 消费的电机状态镜像。
- `robot/motor/cmd`：dashboard backend 发布的电机命令，由 `robot_mqtt_bridge` 转成 `/motor/cmd`。

## Quick Start

### 1. Configure ESP32 WiFi

```bash
cd firmware/esp32_microros_bridge
cp include/wifi_config.example.h include/wifi_config.h
```

编辑 `include/wifi_config.h`，填入当前 WiFi、micro-ROS Agent IP 和端口。真实凭据不要提交。

### 2. Start micro-ROS Agent

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/start_microros_agent_udp.sh
```

Agent 默认 UDP `8888`。启动 Agent 后复位 ESP32，等待串口出现 `micro-ROS connected!`。

### 3. Build ESP32 Firmware

```bash
cd firmware/esp32_microros_bridge
python3 -m platformio run
```

上传和串口监视：

```bash
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

### 4. Build ROS 2 Packages

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select \
  robot_state_monitor imu_data_logger robot_mqtt_bridge \
  --symlink-install
source install/setup.bash
```

### 5. Check Real-Hardware Topics

```bash
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
ros2 topic echo --once /motor/status
ros2 topic echo --once /motor/actual_rpm
ros2 topic echo --once /motor/state
```

也可以运行仓库脚本：

```bash
./scripts/check_real_hw_chain.sh
./scripts/check_real_hw_chain.sh --dashboard
```

### 6. Run Dashboard MQTT Bridges

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

Dashboard IMU MQTT bridge 当前位于 `robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py`。

## Important Safety Notes

- 普通启动默认不应自动输出真实 TB6612 PWM。
- Dashboard 只做低频观察和人工命令入口，不参与实时 PID 闭环。
- 电机控制相关修改前先阅读 [docs/skills/motor_closed_loop_tuning_skill.md](docs/skills/motor_closed_loop_tuning_skill.md)。
- 不要在未确认电机真实能力前提高 `target_rpm`。
- 不要一上来把 `max_pwm` 放到 `1.0`。
- PWM 调大但速度没有变好时，先做 open-loop PWM sweep，而不是直接加 `Ki`。

## Documentation

- [docs/dataflow.md](docs/dataflow.md)
- [docs/hardware_wiring.md](docs/hardware_wiring.md)
- [docs/motor_control_design.md](docs/motor_control_design.md)
- [docs/troubleshooting_log.md](docs/troubleshooting_log.md)
- [docs/dashboard-integration.md](docs/dashboard-integration.md)
- [docs/motor_dashboard_interface.md](docs/motor_dashboard_interface.md)
- [docs/motor_closed_loop_tuning_process.md](docs/motor_closed_loop_tuning_process.md)
- [docs/microros_agent_startup.md](docs/microros_agent_startup.md)
- [firmware/stm32_sensor_node/README.md](firmware/stm32_sensor_node/README.md)
- [firmware/esp32_microros_bridge/README.md](firmware/esp32_microros_bridge/README.md)
- [ros2/robot_mqtt_bridge/README.md](ros2/robot_mqtt_bridge/README.md)
