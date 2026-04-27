# Debug Log - ESP32/STM32 micro-ROS Bridge Project

## 当前基线

- ESP32-S3 通过 WiFi UDP 连接 `micro_ros_agent`
- STM32 通过 UART 上报 `IMUQ` / `IMU` / `State` / `DBG` 文本帧
- ESP32 发布 `/imu/data`、`/imu/filtered`、`/robot/state`
- ESP32 订阅 `/cmd_vel` 并下发 `CMDVEL,<linear_x>,<angular_z>`
- 当前默认使用简化连接模型，不在运行态周期性 `ping` Agent

## 最近联调结论

### 上下行链路都已打通

- `STM32 -> ESP32 -> ROS 2` 已稳定运行
- `ROS 2 -> ESP32 -> STM32 -> TB6612` 已现场验证可驱动电机
- `/imu/data`、`/imu/filtered`、`/robot/state` 当前都能正常出现
- `ros2 topic info -v /cmd_vel` 可见订阅者 `stm32_bridge`
- 实测 `ros2 topic pub --once /cmd_vel ...` 后，ESP32 会打印：

```text
[CMDVEL] vx=0.200 wz=0.000
```

### 当前剩余边界

- 还没有结构化的 `/motor/state` 执行反馈
- 还没有对 `PWMA`、`AIN1/AIN2`、`STBY` 做示波器级别确认

## 常用检查项

### 1. 先看启动日志

正常应至少看到：

```text
ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
Requested STM32 GAZEBO mode
WiFi Connected! IP: ...
micro-ROS WiFi transport configured
Connecting to micro-ROS Agent at ...:8888
micro-ROS connected!
```

### 2. 检查 ROS 2 话题

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
ros2 topic info -v /cmd_vel
```

### 3. 验证下行控制

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}, angular: {z: 0.00}}"
```

如果链路正常，ESP32 串口会打印 `[CMDVEL] ...`。

## 常见问题

### USB 串口没有日志

优先检查：

- `platformio.ini` 里 `board_build.cdc_on_boot = yes`
- `build_flags` 里 `-D ARDUINO_USB_CDC_ON_BOOT=1`
- 上传后是否重新拔插了开发板

### 只有 `/parameter_events` 和 `/rosout`

这通常说明业务节点没真正初始化成功。优先检查：

- `micro_ros_agent` 是否已启动
- ESP32 启动日志里是否出现 `micro-ROS connected!`
- 板子上是否还是旧固件

### 串口出现 XRCE 乱码

这通常意味着板子跑的不是当前 WiFi-only 固件，或者 transport 走错了。重新编译并上传当前工程后，再确认日志里出现 `micro-ROS WiFi transport configured`。

### `/robot/state` 有数据但 `/imu/data` 没有

优先检查 STM32 上行帧是否仍是下面两种之一：

- `IMUQ,...`
- `IMU,...`

如果 STM32 只在输出其他训练 CSV，ESP32 不会把它们当正式 IMU 输入。
