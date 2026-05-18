# Data Flow

## End-to-End Overview

```text
启动协商
ESP32 上电初始化
  -> UART "G\n"
  -> STM32 切到 GAZEBO 模式

上行观测链路
MPU6050
  -> STM32 SensorTask
     - 100 Hz 采样
     - 6 轴姿态解算
  -> IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp
     - USART1
     - 921600 bps
     - 约 50 Hz
  -> ESP32 stm32_serial_parser
  -> /imu/data      (sensor_msgs/msg/Imu, best_effort)
  -> ESP32 一阶低通 alpha=0.2
  -> /imu/filtered  (sensor_msgs/msg/Imu, best_effort)
  -> robot_state_monitor / imu_data_logger

状态判别链路
STM32 SensorQueue
  -> AlgTask
     - 10 样本窗口
     - RMS 基线判别
  -> State:<n>
     - USART1
     - 约 10 Hz
  -> ESP32 stm32_serial_parser
  -> /robot/state   (std_msgs/msg/Int32, best_effort)
  -> imu_data_logger

下行控制链路
ROS 2 /cmd_vel      (geometry_msgs/msg/Twist, ESP32 侧 reliable subscriber)
  -> ESP32 uros_sub / executor
  -> CMDVEL,<linear_x>,<angular_z>\n
  -> STM32 UART 行解析
  -> g_motor_linear_x / g_motor_angular_z / g_motor_cmd_recv_tick_ms / g_motor_estop
  -> MotorTask
     - 10 ms 调度
     - 200 ms 超时急停
  -> TB6612 A 路单电机

ESP32 本地电机控制骨架
ROS 2 /motor/target_rpm (std_msgs/msg/Float32, ESP32 侧 reliable subscriber)
  -> ESP32 ros_comm_task / executor
  -> Core 0 / Core 1 shared motor command
  -> ESP32 motor_control_task
     - 10 ms 调度
     - command timeout 检查
     - 当前使用 mock motor response
  -> shared motor state
  -> /motor/actual_rpm (std_msgs/msg/Float32, best_effort)
  -> /motor/state      (std_msgs/msg/String, best_effort)
```

## Roles

- STM32：负责采样、姿态解算、状态判别、本地 LED 报警；既有 TB6612 A 路执行控制保留为 legacy open-loop 验证链路。
- ESP32：负责 UART 文本协议解析、micro-ROS 建链、ROS 2 话题发布、`/cmd_vel` 下行转发，以及后续 N20 编码器电机本地闭环主线。
- ROS 2 主机：负责运行 Agent、Gazebo 可视化、IMU/状态记录和实时绘图。

## ROS 2 Topics

| Topic | Type | Direction | QoS | Current meaning |
| --- | --- | --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | ESP32 -> ROS 2 | `best_effort` | 原始线加速度/角速度，加上 STM32 四元数姿态 |
| `/imu/filtered` | `sensor_msgs/msg/Imu` | ESP32 -> ROS 2 | `best_effort` | 线加速度/角速度经过一阶低通，姿态四元数保持不变 |
| `/robot/state` | `std_msgs/msg/Int32` | ESP32 -> ROS 2 | `best_effort` | STM32 `AlgTask` 的 RMS 状态判别结果 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 -> ESP32 | `reliable` subscriber on ESP32 | 人工控制输入，经 UART 转成 `CMDVEL` 发往 STM32 |
| `/motor/target_rpm` | `std_msgs/msg/Float32` | ROS 2 -> ESP32 | `reliable` subscriber on ESP32 | ESP32 本地单电机控制目标转速入口 |
| `/motor/actual_rpm` | `std_msgs/msg/Float32` | ESP32 -> ROS 2 | `best_effort` | 当前由 mock motor response 生成，后续替换为编码器反馈 |
| `/motor/state` | `std_msgs/msg/String` | ESP32 -> ROS 2 | `best_effort` | 当前 JSON 字符串，包含 target / actual / error / timeout 等状态 |

## UART Protocol

### STM32 -> ESP32

- `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
- `IMU,ax,ay,az,gx,gy,gz,temp`
- `State:<n>`
- `DBG:...`

说明：

- `IMUQ` 是当前主格式。
- 旧 `IMU` 格式仍兼容，但没有真实姿态四元数。
- `DBG:` 只作为调试行，不发布成 ROS 2 话题。
- 训练 CSV 默认不会被当作正式 IMU 输入，因为 ESP32 配置里 `kAcceptTrainCsvAsImu = false`。

### ESP32 -> STM32

- `CMDVEL,<linear_x>,<angular_z>\n`
- `G\n`：ESP32 启动时会主动发一次，请求 STM32 切到 GAZEBO 模式

## Notes

- STM32 采样周期当前是 `10 ms`，因此原始传感器读数是约 `100 Hz`。
- `IMUQ` 输出被分频到约 `50 Hz`，以减轻 UART 和桥接端压力。
- `State:<n>` 由 `10` 个样本组成一个窗口，因此当前状态输出约为 `10 Hz`。
- Gazebo 默认消费 `/imu/filtered`，并通过 `lock_yaw:=true` 固定初始 yaw，减少 6 轴方案的漂移观感。
- ESP32 的 `/motor/target_rpm -> /motor/actual_rpm / /motor/state` 当前是无硬件 mock 链路，用于在 N20 编码器、CPR/PPR、PWM/DIR 和 PID 参数实测前验证 ROS 2 topic 与双核任务数据流。
