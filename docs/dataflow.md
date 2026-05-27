# Dataflow

本文件是本仓库面向 Robot Ops Dashboard 的顶层数据链说明。它描述 STM32、ESP32、micro-ROS、ROS 2、MQTT/backend 和 Dashboard 之间的边界。

旧文件 [data-flow.md](data-flow.md) 保留给已有链接；新文档优先使用本文件。

## System View

```text
Real hardware
  MPU6050 / TB6612 / N20
    |
    v
STM32F411 sensor node
  - MPU6050 sampling
  - 6-axis attitude estimate
  - RMS robot state classifier
  - legacy CMDVEL motor execution
    |
    | USART1 921600 bps
    | IMUQ,... / State:<n> / CMDVEL,...
    v
ESP32-S3 micro-ROS bridge
  - STM32 UART parser
  - WiFi UDP micro-ROS client
  - IMU and robot state publishers
  - motor command subscribers
  - N20 motor-control task
    |
    | micro-ROS UDP
    v
micro-ROS Agent on ROS 2 host
    |
    v
ROS 2 Jazzy graph
  /imu/data
  /imu/filtered
  /robot/state
  /motor/cmd
  /motor/status
  /motor/actual_rpm
  /motor/state
    |
    v
ROS 2 MQTT/backend bridge
  robot/imu
  robot/motor/status
  robot/motor/cmd
    |
    v
Robot Ops Dashboard backend
    |
    v
Robot Ops Dashboard frontend
```

## IMU And Robot State Uplink

```text
MPU6050
  -> STM32 SensorTask
     - 100 Hz sample loop
     - MPU6050 accel / gyro read
     - 6-axis quaternion estimate
  -> STM32 USART1
     - IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp
     - about 50 Hz
  -> ESP32 stm32_serial_parser
  -> ROS 2 /imu/data
     - sensor_msgs/msg/Imu
     - best_effort
  -> ESP32 low-pass filter alpha=0.2
  -> ROS 2 /imu/filtered
     - sensor_msgs/msg/Imu
     - best_effort
  -> logger / Gazebo bridge / MQTT IMU bridge
  -> MQTT robot/imu
  -> dashboard backend
  -> dashboard frontend
```

状态判别链路：

```text
STM32 SensorQueue
  -> AlgTask
     - 10-sample RMS window
     - state classifier
  -> STM32 USART1
     - State:<n>
     - about 10 Hz
  -> ESP32 stm32_serial_parser
  -> ROS 2 /robot/state
     - std_msgs/msg/Int32
     - best_effort
```

`/robot/state` 当前语义：

| Value | Meaning |
| --- | --- |
| `0` | `normal` |
| `1` | `vibration` |
| `2` | `collision` |
| `3` | `tip_over` |

## Motor Command And Status

Dashboard 主电机命令链路：

```text
Robot Ops Dashboard frontend
  -> dashboard backend POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> ros2/robot_mqtt_bridge motor_cmd_bridge
  -> ROS 2 /motor/cmd
     - std_msgs/msg/String
     - JSON payload
  -> ESP32 ros_comm_task
  -> shared motor command
  -> ESP32 motor_control_task
     - enable gate
     - stop priority
     - command timeout
     - max_pwm clamp
     - PI/PID speed control path when hardware output is enabled
  -> TB6612 / N20 bench chain
```

主状态回传链路：

```text
ESP32 motor_control_task
  -> shared motor state
  -> ROS 2 /motor/status
     - std_msgs/msg/String
     - dashboard-facing JSON
     - default low-rate status output
  -> ros2/robot_mqtt_bridge motor_status_bridge
  -> MQTT robot/motor/status
  -> dashboard backend
  -> dashboard frontend
```

兼容调试链路：

```text
ESP32 motor_control_task
  -> /motor/actual_rpm  (std_msgs/msg/Float32)
  -> /motor/state       (std_msgs/msg/String)
```

## Legacy CMDVEL Path

`/cmd_vel` 当前保留为本地 legacy 验证链路，不作为 dashboard 主电机控制链路。

```text
ROS 2 /cmd_vel
  -> ESP32 reliable subscriber
  -> CMDVEL,<linear_x>,<angular_z>\n
  -> STM32 UART line parser
  -> STM32 MotorTask
     - 10 ms schedule
     - 200 ms timeout stop
  -> TB6612 A channel single motor
```

## Topic And Transport Contracts

| Contract | Producer | Consumer | Notes |
| --- | --- | --- | --- |
| `IMUQ,...` | STM32 | ESP32 | Main IMU UART format |
| `IMU,...` | STM32 | ESP32 | Legacy IMU UART format |
| `State:<n>` | STM32 | ESP32 | Robot state classifier |
| `CMDVEL,<linear_x>,<angular_z>\n` | ESP32 | STM32 | Legacy downlink |
| `/imu/data` | ESP32 | ROS 2 tools | Raw IMU + quaternion |
| `/imu/filtered` | ESP32 | Gazebo / logger / bridge | Low-pass accel / gyro |
| `/robot/state` | ESP32 | ROS 2 tools | State classifier |
| `/motor/cmd` | ROS 2 bridge | ESP32 | Dashboard motor command |
| `/motor/status` | ESP32 | ROS 2 bridge | Dashboard motor status |
| `robot/imu` | IMU MQTT bridge | Dashboard backend | Low-frequency read-only mirror |
| `robot/motor/status` | `robot_mqtt_bridge` | Dashboard backend | Low-frequency read-only mirror |
| `robot/motor/cmd` | Dashboard backend | `robot_mqtt_bridge` | Low-frequency operator command |

## Ownership Rules

- Firmware keeps real-time sensing and motor control responsibilities.
- ROS 2 owns integration, recording, visualization, and bridge nodes.
- MQTT/backend is a low-frequency dashboard integration boundary.
- Dashboard frontend does not connect directly to ROS 2, ESP32, STM32, micro-ROS Agent, or MQTT broker internals.
- MQTT is not part of the high-rate PID loop.
