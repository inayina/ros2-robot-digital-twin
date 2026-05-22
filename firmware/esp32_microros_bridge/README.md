# ESP32 micro-ROS Bridge

本仓库是 **ESP32-S3 DevKitC-1** 的 micro-ROS 网桥工程，并逐步承接 N20 编码器电机闭环控制主线。
它位于 STM32 传感节点、电机控制链路与 ROS 2 主机之间，当前负责两件事：

1. 把 STM32 上行串口数据解析后发布到 ROS 2
2. 订阅 ROS 2 `/cmd_vel`，再通过串口下发给 STM32

当前项目角色是：

- **STM32**：MPU6050 采样、姿态解算、robot state 输出、既有数字孪生链路
- **ESP32**：WiFi + micro-ROS + 串口协议转换，后续作为 N20 motor controller
- **PC / ROS 2**：Agent、目标速度下发、状态可视化、系统集成

## 当前状态

当前代码已经覆盖以下链路：

- 上行：
  `STM32 UART -> ESP32 解析 -> /imu/data + /imu/filtered + /robot/state`
- 下行：
  `ROS 2 /cmd_vel -> ESP32 subscriber -> UART CMDVEL -> STM32`

当前运行时已经拆成双核骨架：

- Core 0：`ros_comm_task`
  负责 WiFi、micro-ROS、STM32 串口解析、`/cmd_vel` legacy bridge、`/motor/target_rpm` 和 `/motor/cmd` 接收
- Core 1：`motor_control_task`
  负责电机控制循环、`enable/max_pwm/timeout/stop` 安全检查、mock rpm 响应和共享状态更新

当前已实现的话题：

- 发布 `/imu/data`：`sensor_msgs/msg/Imu`
- 发布 `/imu/filtered`：`sensor_msgs/msg/Imu`
- 发布 `/robot/state`：`std_msgs/msg/Int32`
- 发布 `/motor/status`：`std_msgs/msg/String`
- 发布 `/motor/actual_rpm`：`std_msgs/msg/Float32`
- 发布 `/motor/state`：`std_msgs/msg/String`
- 订阅 `/cmd_vel`：`geometry_msgs/msg/Twist`
- 订阅 `/motor/target_rpm`：`std_msgs/msg/Float32`
- 订阅 `/motor/cmd`：`std_msgs/msg/String`

## 当前路线

电机闭环控制主线迁移到 ESP32：当前 TB6612 A 通道保留给 130 普通电机做驱动通道辅助验证，TB6612 B 通道承担单 6V N20 编码器减速电机闭环主线。单 N20 跑通后，再购买第二个同规格 N20，把 A 通道从 130 切换为 N20 扩展双轮差速。STM32 既有 open-loop motor control 仅保留为 legacy experiment / early validation，不再继续扩展为编码器读取、PID 调速、双轮差速或 `ros2_control` 主线。详细架构见 [docs/design.md](docs/design.md) 和 [docs/motor_control_design.md](docs/motor_control_design.md)。

当前代码已经实现双核任务框架、`/motor/target_rpm` 与 `/motor/cmd` 命令入口、`/motor/status` 与 `/motor/state` 发布，以及 `enable`、`max_pwm`、命令超时和 `stop` 优先级安全约束。当前 `/motor/cmd` 常规路径已经接入单 N20 bench 接线下的真实编码器滤波 `actual_rpm` 和本地 PI 输出，但参数仍以台架保守联调为主，不代表完整底盘调速完成。普通启动默认仍保持 `kEnableMotorHardwareOutputs = false`、`kEnableN20ClosedLoopBench = false`，上电后不会自动输出真实 TB6612 PWM。

当前固件同时支持 bench 运行时解锁，不需要为了单次台架联调反复改这两个编译期开关：

- 收到 `/motor/cmd` 且 `enabled=true`、`stop=false` 时，会在运行时打开真实硬件输出路径
- 收到 `stop=true` 或 `enabled=false` 时，会在运行时关闭真实硬件输出路径
- `/motor/status` / `motor_state` 会额外带出 `hardware_outputs_enabled`
- 如需显式覆盖，也支持在 `/motor/cmd` JSON 中携带 `hardware_enable`

这条运行时解锁只面向已确认安全的固定 bench 摆位，不改变普通启动默认值。

130 普通电机 A 通道 bench test 已保留为可开关调试项：

- `kEnableTb6612ChannelABenchTest = false`
- `GPIO4 -> PWMA`
- `GPIO5 -> AIN1`
- `GPIO6 -> AIN2`
- `GPIO18 -> STBY`
- 打开后，上电会低占空比正转短脉冲、coast、低占空比反转短脉冲、最后禁用 STBY

当前接线规划是 TB6612 A 通道保留 130、TB6612 B 通道先做 N20；ESP32 具体 GPIO 到 TB6612 / 编码器的规划见 [docs/motor_control_design.md](docs/motor_control_design.md)。

单 N20 编码器速度闭环 bench 也已落地为本地可开关测试能力：

- `kEnableN20ClosedLoopBench = false`
- 只占用 TB6612 B 通道和 `GPIO10/GPIO11` 编码器输入
- 不接 `ROS 2 /cmd_vel`
- 不改已有 micro-ROS topic / 串口协议
- 打开后按时间 profile 自动做速度阶跃，并输出 CSV 风格日志
- 当前测试与调参记录见 [docs/n20_closed_loop_bench_tuning.md](docs/n20_closed_loop_bench_tuning.md)

## 代码结构

`src/` 当前按职责拆成了这些模块入口：

- `main.cpp`
  程序入口，负责 task 启动和系统装配
- `config/app_config.h`
  ESP32 侧集中配置入口，放 WiFi、Agent、串口参数、初始化重试周期、调试开关
- `ros/uros_core.[h/cpp]`
  micro-ROS 基础上下文，负责 `support / node / allocator`
- `ros/uros_pub.[h/cpp]`
  上行 publisher，负责 IMU / robot state 和 motor state 发布
- `ros/uros_sub.[h/cpp]`
  下行 subscriber，负责 `/cmd_vel`、`/motor/target_rpm`、`/motor/cmd` 和 executor
- `bridge/stm32_serial_parser.[h/cpp]`
  STM32 上行串口协议解析
- `bridge/motor_command_parser.[h/cpp]`
  `/motor/cmd` JSON 解析，提取 `target_rpm`、`enabled`、`closed_loop`、`max_pwm`、`timeout_ms`、`stop`
- `motor/motor_control_shared.[h/cpp]`
  Core 0 / Core 1 之间的共享命令与状态快照
- `motor/motor_controller.[h/cpp]`
  Core 1 电机硬件输出与运行时 arm/disarm 控制
- `motor/motor_response_model.[h/cpp]`
  无硬件 mock motor response，用于在 N20 未到货时验证 `target_rpm -> actual_rpm` 数据流
- `motor/encoder_rpm_estimator.[h/cpp]`
  编码器计数到 rpm 的纯逻辑估算模块，等待 N20 CPR/PPR 实测后接入真实计数
- `motor/speed_pid.[h/cpp]`
  速度 PID 纯逻辑模块，包含输出限幅和积分限幅
- `motor/single_motor_control.[h/cpp]`
  单电机闭环状态组合模块，把 `target_rpm`、`actual_rpm`、PID 输出和 timeout 组合成控制状态
- `motor/tb6612_driver.[h/cpp]`
  TB6612 A/B 双通道 PWM/DIR/STBY 驱动边界，当前只提供可配置接口，不写死 GPIO、PWM channel、频率或分辨率

这样拆开后：

- `uros_pub` 不再混订阅逻辑
- `uros_sub` 现在已经承接 `/motor/cmd`，后续继续扩更细的下行接口时，优先保持 JSON 字段兼容
- `uros_core` 统一管理 node/support，避免 pub/sub 各自重复持有

## 硬件连接

- UART：
  `GPIO16 (RX) <- STM32 TX`
  `GPIO17 (TX) -> STM32 RX`
- 波特率：`921600`
- USB CDC：已开启
- WiFi SSID：由本地 `include/wifi_config.h` 提供，上传仓库只保留 `include/wifi_config.example.h`
- micro-ROS Agent：由本地 `include/wifi_config.h` 提供

这些参数现在统一收口在：

- [include/wifi_config.example.h](include/wifi_config.example.h)
- [app_config.h](src/config/app_config.h)

STM32 上电后，ESP32 会主动发送：

```text
G\n
```

用于请求 STM32 进入 `GAZEBO` / ROS 输出模式。

## 配置与调试

ESP32 侧的配置分两层：

- `include/wifi_config.h`：本地 WiFi SSID / Password、Agent IP / Port，不提交到 Git
- [app_config.h](src/config/app_config.h)：ROS 节点名、串口、IMU 滤波、重试周期、任务栈和调试开关

默认可开关的 ESP32 高频调试包括：

- `[IMU ...]`
- `[CMDVEL] ...`
- `[STM32] ...`
- `[RUNTIME] ...`

当前 ROS 发布限频也集中在 [app_config.h](src/config/app_config.h)：

- `/imu/data`：默认 `50 Hz`
- `/imu/filtered`：默认 `25 Hz`
- `/robot/state`：默认 `10 Hz`
- `/motor/status` JSON：默认 `5 Hz`
- `/motor/actual_rpm`：默认 `20 Hz`
- `/motor/state` JSON：默认 `5 Hz`
- `[RUNTIME]` 统计日志：默认 `10 s`

### 单 N20 编码器闭环阶跃 bench

当前 bench 相关配置也集中在 [app_config.h](src/config/app_config.h)：

- `kEnableN20ClosedLoopBench`
- `kN20ClosedLoopBenchControlPeriodMs`
- `kN20ClosedLoopBenchPrintIntervalMs`
- `kN20ClosedLoopBenchMaxDurationMs`
- `kN20ClosedLoopBenchMaxPwm`
- `kN20ClosedLoopBenchMaxTargetRpm`
- `kN20ClosedLoopBenchMinEffectivePwm`
- `kN20ClosedLoopBenchEncoderPulsesPerRev / GearRatio / EdgesPerPulse`
- `kN20ClosedLoopBenchWheelDiameterM`
- `kN20ClosedLoopBenchInvertEncoderDirection`
- `kN20ClosedLoopBenchKp / Ki / Kd`
- `kN20ClosedLoopBenchIntegralMin / Max`
- `kN20ClosedLoopBenchProfileStep*StartMs`
- `kN20ClosedLoopBenchProfileStep*TargetRpm`

默认阶跃 profile：

- `0-1s`：`0 rpm`
- `1-4s`：`40 rpm`
- `4-7s`：`60 rpm`
- `7-10s`：`80 rpm`
- `10-13s`：`50 rpm`
- `13-15s`：`0 rpm`
- `15s` 后自动 stop，硬上限 `20s`

串口日志输出为 CSV：

```text
timestamp_ms,target_rpm,actual_rpm,error_rpm,pwm,direction,encoder_delta,status,raw_rpm,filtered_rpm,integral,output_saturated,bench_phase,invalid_transitions
1050,40.000,5.091,34.909,0.120,1,16,run,14.545,5.091,3.745,0,1050,0
```

运行方法：

1. 在 [app_config.h](src/config/app_config.h) 中把 `kEnableN20ClosedLoopBench` 临时改为 `true`
2. 运行 `python3 -m platformio run`
3. 运行 `python3 -m platformio run --target upload --upload-port /dev/ttyACM0`
4. 打开 `python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0`
5. 从 CSV header 开始复制日志到 `.csv` 文件或表格做阶跃响应曲线
5. 测试结束后把 `kEnableN20ClosedLoopBench` 改回 `false` 并重新烧录安全版

说明：

- 当前硬件链路是 `ESP32-S3 -> TB6612FNG -> single N20 motor with encoder`
- 当前控制边界是 single motor `target_rpm / actual_rpm` 闭环，不是完整双轮底盘，不是 robot linear velocity，也不是 `ros2_control`
- 当前实测在 `max_pwm = 0.25` 下更适合 `40/60/80/50 rpm` 保守 profile；`120/180 rpm` 会长期大误差或接近饱和，不作为默认录屏 profile
- `direction = 1` 表示当前软件定义 forward，`encoder_delta` 和 `actual_rpm` 应与 forward 保持同号；如果反了，优先改 `kN20ClosedLoopBenchInvertEncoderDirection`
- `wheel_diameter_m` 只作为后续轮速显示/换算准备，当前 PID 仍以电机输出轴 rpm 为控制量
- bench 模式会压掉本地 IMU / runtime 高频调试，尽量让串口以 CSV 为主
- 如果没启动 micro-ROS Agent，启动阶段仍可能看到少量 WiFi / micro-ROS 重连提示，这些不影响 CSV 行本身

STM32 侧串口调试开关集中在：

- [app_debug.h](../stm32_sensor_node/User/App/app_debug.h)

默认关闭的 STM32 高频调试包括：

- `DBG:CMDVEL_RX,...`
- `DBG:STAT,...`
- `DBG:ESTOP_RX`

## 串口协议

### STM32 -> ESP32

当前支持以下上行格式：

- `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
- `IMU,ax,ay,az,gx,gy,gz,temp`
- `State:<n>`

说明：

- `IMUQ` 是当前主格式，包含四元数姿态
- 旧 `IMU` 格式仍兼容，但会用单位四元数占位
- 训练 CSV 目前默认 **不** 当作正式 IMU 话题输入
  因为代码里 `ACCEPT_TRAIN_CSV_AS_IMU = false`

### ESP32 -> STM32

当前下行格式：

```text
CMDVEL,<linear_x>,<angular_z>\n
```

例如：

```text
CMDVEL,0.200,0.000
CMDVEL,0.000,1.000
```

当前还没有在 ESP32 侧单独实现 ROS 话题式 `ESTOP`，如果后续要扩，可优先放进 `uros_sub`。

## ROS 2 行为

### 发布话题

- `/imu/data`
  原始解析后的 IMU 数据
- `/imu/filtered`
  线加速度与角速度的一阶低通滤波结果
- `/robot/state`
  STM32 推理得到的状态值
- `/motor/status`
  当前主电机状态字符串，包含 target / measured / pwm / enabled / fault 等字段
- `/motor/actual_rpm`
  当前单 N20 bench 接线下的编码器滤波实际转速
- `/motor/state`
  兼容旧调试链路的电机状态字符串，字段与 `/motor/status` 保持一致

### 订阅话题

- `/cmd_vel`
  类型：`geometry_msgs/msg/Twist`
- `/motor/target_rpm`
  类型：`std_msgs/msg/Float32`
- `/motor/cmd`
  类型：`std_msgs/msg/String`

收到 `/cmd_vel` 后，ESP32 会把：

- `linear.x`
- `angular.z`

编码成 `CMDVEL` 文本帧，经 `UART1` 发给 STM32。

收到 `/motor/target_rpm` 后，ESP32 会把目标转速写入 Core 0 / Core 1 共享命令。当前 `motor_control_task` 会在单 N20 bench 接线下使用真实 encoder A/B 计数、滤波后的 `actual_rpm` 和本地 PI 输出追踪 `target_rpm`，并通过 `/motor/status`、`/motor/actual_rpm` 和 `/motor/state` 回传状态。

收到 `/motor/cmd` 后，ESP32 会解析 JSON 里的 `target_rpm`、`enabled`、`closed_loop`、`max_pwm`、`timeout_ms` 和 `stop`，再写入 Core 0 / Core 1 共享命令，供 `motor_control_task` 在本地 encoder/PID 反馈下执行安全约束后的控制输出。当前 `20 rpm` 量级已能接近目标；`40/60/80 rpm` 仍是保守台架 tune，可能存在明显稳态误差。

## 连接恢复

ESP32 当前使用简化连接模型：

- 上电后先连 WiFi
- WiFi 连上后配置 UDP transport
- 周期性尝试 `createMicroRosEntities()`
- 初始化成功后只保持正常 `spin`

当前默认不会在运行态周期性 `ping` Agent。只有在 WiFi 真掉线时，ESP32 才会：

- 销毁当前 micro-ROS entities
- 重新连 WiFi
- 在 WiFi 恢复后重新初始化 node / publishers / subscriber / executor

## 依赖

- PlatformIO
- `micro_ros_arduino` `jazzy`
- ESP32 Arduino framework
- ROS 2 Jazzy
- micro-ROS Agent `udp4`

## 构建

如果你的 shell 里已经有 `pio`：

```bash
pio run
```

如果没有把 PlatformIO 加进 PATH，可以直接用：

```bash
~/.platformio/penv/bin/pio run
```

本仓库也保留了 Python venv，可以这样运行：

```bash
./.venv/bin/python3 -m platformio run
```

上传：

```bash
~/.platformio/penv/bin/pio run --target upload --upload-port /dev/ttyACM0
```

串口监视：

```bash
~/.platformio/penv/bin/pio device monitor -b 921600 --port /dev/ttyACM0
```

## 启动顺序

1. 启动 micro-ROS Agent

```bash
source /opt/ros/jazzy/setup.bash
source /home/ina/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

2. 烧录并启动 ESP32

```bash
cd firmware/esp32_microros_bridge
~/.platformio/penv/bin/pio run --target upload --upload-port /dev/ttyACM0
```

3. 打开串口监视器，确认日志包含：

```text
ESP32-S3 micro-ROS Bridge v1.2 dual-core - Starting...
Connecting to WiFi: ...
WiFi Connected! IP: ...
micro-ROS WiFi transport configured
Connecting to micro-ROS Agent at 192.168.1.8:8888
micro-ROS connected!
```

4. 在 ROS 2 主机查看话题：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /imu/data
ros2 topic echo /robot/state
```

如果已经烧录并复位板子，推荐直接跑仓库里的整链检查脚本：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/check_real_hw_chain.sh
```

它会检查：

- `/imu/data`、`/imu/filtered`、`/robot/state`
- `/motor/status`、`/motor/actual_rpm`、`/motor/state`
- `/cmd_vel`、`/motor/target_rpm`、`/motor/cmd` 的下行订阅是否在线

如果还要把 dashboard backend / MQTT 链路一起验掉，可以追加：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/check_real_hw_chain.sh --dashboard
```

## 用 rqt 控制

当前软件链路已经实现 `CMDVEL` 接收与下发，可以直接用：

```bash
source /opt/ros/jazzy/setup.bash
rqt_robot_steering
```

在 `rqt_robot_steering` 里设置：

- topic：`/cmd_vel`

也可以直接命令行测试：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

如果串口监视器里看到类似日志：

```text
[CMDVEL] vx=0.200 wz=0.000
```

说明 ESP32 已经收到 `/cmd_vel` 并开始向 STM32 下发控制帧。

## Legacy 电机链路现状

当前已经确认的早期验证链路是：

```text
/cmd_vel
  -> ESP32 subscriber
  -> UART CMDVEL
  -> STM32 CMDVEL 解析
  -> MotorTask
  -> TB6612 A 路
```

这轮实测里，`ros2 topic pub --once /cmd_vel ...` 后，ESP32 已打印：

```text
[CMDVEL] vx=0.200 wz=0.000
```

结合 STM32 当前代码实现，可以确认：

- `sensor_task.c` 已使用 `strtof` 解析 `CMDVEL`
- 合法命令会清除 `g_motor_estop`
- `MotorTask` 每 `10 ms` 执行一次
- `CMDVEL` 超时保护为 `200 ms`
- `TB6612` 当前驱动的是 A 路单电机

这条链路证明 ROS 2 到真实下位机执行层已经打通，但它不再作为新的电机闭环主线。当前还没有直接量到 `PWMA/AIN1/AIN2/STBY` 的示波器级波形，所以更准确的说法是：

- 软件链路已经打通
- 单电机 A 路的物理转动已经现场确认
- 当前 ESP32 侧已经有 mock `/motor/state`，后续剩下的是把执行反馈替换成真实硬件观测

## 已知限制

- 当前只桥接 `/cmd_vel`，还没有独立的 ROS `ESTOP` 话题
- STM32 侧电机控制语义仅作为 legacy experiment 保留
- WiFi、Agent、串口任一链路异常时，真正的安全停车仍需要 STM32 执行层负责
- `IMUQ` 是当前推荐上行格式，旧 `IMU` 仅作兼容保留

## 故障排除

- 如果只看到 `/parameter_events` 和 `/rosout`，优先检查 Agent 是否已启动，以及 ESP32 是否打印 `Connecting to micro-ROS Agent at ...`
- 如果 `/robot/state` 有数据但 `/imu/data` 没有，检查 STM32 串口输出是否仍是 `IMUQ` 或 `IMU`
- 如果 `rqt` 在发 `/cmd_vel`，但串口监视器没有 `[CMDVEL]` 日志，优先检查 subscriber 是否成功初始化
- 如果需要继续查 STM32 收命令问题，优先临时打开 [app_debug.h](../stm32_sensor_node/User/App/app_debug.h) 里的 `DBG` 开关
- 如果 `/dev/ttyACM0` 消失，关闭监视器后重新插拔开发板
- 如果 `pio` 不在 PATH，改用 `~/.platformio/penv/bin/pio`
- 当前架构说明见 [docs/design.md](docs/design.md)
- 更多排障记录见 [Debug.md](Debug.md)

## 版本信息

- micro-ROS：jazzy
- ROS 2：Jazzy Jalisco
- 平台：PlatformIO `espressif32`
- 板卡：`esp32-s3-devkitc-1`
- 固件标识：`ESP32-S3 micro-ROS Bridge v1.2 dual-core`
