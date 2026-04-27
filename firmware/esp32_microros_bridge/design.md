# ESP32 Bridge Design

## 1. 目标

这个工作区只负责 ESP32-S3 micro-ROS 网桥，职责是把 STM32 和 ROS 2 主机接起来：

- 上行：`STM32 UART -> ESP32 -> /imu/data /imu/filtered /robot/state`
- 下行：`ROS 2 /cmd_vel -> ESP32 -> UART CMDVEL -> STM32 -> TB6612`

它不是电机闭环控制器，也不直接做运动学；真正的执行、超时保护和急停仍由 STM32 负责。

## 2. 当前架构

### 2.1 运行角色

- STM32：IMU 采样、状态识别、`CMDVEL` 解析、TB6612 执行
- ESP32：WiFi UDP transport、micro-ROS client、UART 文本桥接
- PC / ROS 2：`micro_ros_agent`、话题可视化、`rqt_robot_steering`

### 2.2 代码分层

- `src/app_config.h`
  WiFi、Agent、串口、日志、初始化重试参数
- `src/main.cpp`
  WiFi 管理、micro-ROS 初始化、串口桥接、运行期日志
- `src/uros_core.[h/cpp]`
  `rclc support / node / allocator`
- `src/uros_pub.[h/cpp]`
  `/imu/data`、`/imu/filtered`、`/robot/state`
- `src/uros_sub.[h/cpp]`
  `/cmd_vel` subscriber 与 executor
- `src/stm32_serial_parser.[h/cpp]`
  STM32 上行文本协议解析

## 3. 连接策略

当前默认策略已经简化，不再在运行态周期性 `ping` Agent：

1. ESP32 上电后先连 WiFi
2. WiFi 连上后配置 UDP transport
3. 周期性尝试一次 `createMicroRosEntities()`
4. 一旦初始化成功，后续只做正常 `spin`
5. 只有在 WiFi 真掉线时，才销毁 entities 并等待 WiFi 恢复后重建

这样做的原因是：现场链路更容易被高频 IMU 上行负载扰动，持续 `ping_agent()` 反而更容易把会话误判成掉线。

## 4. 串口协议

### 4.1 STM32 -> ESP32

- `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
- `IMU,ax,ay,az,gx,gy,gz,temp`
- `State:<n>`

### 4.2 ESP32 -> STM32

- `CMDVEL,<linear_x>,<angular_z>\n`
- 预留：`ESTOP`

## 5. 电机链路现状

### 5.1 已确认的软件链路

这轮联调已经确认下面这条链是通的：

```text
ROS 2 /cmd_vel
  -> ESP32 subscriber
  -> ESP32 串口下发 CMDVEL
  -> STM32 UART 行解析
  -> STM32 更新 g_motor_linear_x / g_motor_angular_z / g_motor_estop
  -> MotorTask 驱动 TB6612 A 路
```

依据有两类：

- ROS 图里 `/cmd_vel` 的订阅者 `stm32_bridge` 在线
- 实测 `ros2 topic pub --once /cmd_vel ...` 后，ESP32 串口出现：

```text
[CMDVEL] vx=0.200 wz=0.000
```

### 5.2 STM32 执行层代码现状

结合 `robot-state-monitor` 工程当前代码：

- `sensor_task.c` 已用 `strtof` 解析 `CMDVEL`
- 收到合法 `CMDVEL` 后会：
  - 更新 `g_motor_linear_x`
  - 更新 `g_motor_angular_z`
  - 更新时间戳 `g_motor_cmd_recv_tick_ms`
  - 清除 `g_motor_estop`
- `MotorTask` 每 `10 ms` 执行一次
- `CMDVEL` 超时保护是 `200 ms`
- `TB6612_Init()` 会启动 `TIM3 CH1`
- `TB6612_EmergencyStop()` 会拉低 `STBY`

当前映射策略是：

- 先优先用 `linear_x`
- 若线速度接近 0，再退化使用 `angular_z`
- 当前只驱动 TB6612 A 路单电机

### 5.3 当前验证边界

当前已经确认两件事：

- `ros2 topic pub --once /cmd_vel ...` 后，ESP32 串口会打印 `[CMDVEL] ...`
- 电机物理转动已经被现场观察确认

因此现在可以下的结论是：

- 软件链路已经打通
- 单电机 A 路物理执行已经跑通

当前还没有结构化确认的是：

- `PWMA` 实际占空比波形
- `AIN1/AIN2/STBY` 的示波器级别波形
- `/motor/state` 这种可回放的执行反馈话题

也就是说，当前系统已经具备“能跑”的执行闭环，但还没有把执行层反馈做成正式 ROS 2 观测接口。

## 6. 推荐联调顺序

1. 先启动 host `micro_ros_agent`
2. 烧录并启动 ESP32
3. 确认串口出现 `micro-ROS connected!`
4. 确认 `ros2 topic list` 中有 `/cmd_vel`
5. 用 `ros2 topic pub --once /cmd_vel ...` 或 `rqt_robot_steering` 发命令
6. 观察 ESP32 是否打印 `[CMDVEL] ...`
7. 若要继续压缩定位范围，再临时打开 STM32 的：
   - `APP_DEBUG_CMDVEL_RX`
   - `APP_DEBUG_RUNTIME_STATUS`

## 7. 当前结论

当前工作区更适合作为一个稳定、简单的 ESP32 bridge：

- 默认不做 Agent 运行态 watchdog
- 默认保留 WiFi 掉线后的本地重建
- 默认把 `/cmd_vel` 直接桥接成 STM32 文本命令

后续如果真的需要更强的掉线恢复能力，建议在确认上行负载和内存占用稳定后，再谨慎把 Agent watchdog 加回来。
