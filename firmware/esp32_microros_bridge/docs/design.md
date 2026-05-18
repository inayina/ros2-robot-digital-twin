# ESP32 System Design

## 1. Scope

本文件描述的是 **整个 ESP32-S3 节点的系统级架构**，而不只是电机控制子系统。

当前 ESP32 在项目里的角色有两层：

- 现阶段已经落地的 **micro-ROS communication bridge**
- 后续逐步承接的 **motor controller**

因此，这份设计文档需要同时覆盖：

- ESP32 与 STM32、ROS 2 PC、可选 MQTT 的系统关系
- 当前已经实现的上行 / 下行桥接链路
- ESP32 软件模块划分
- 后续迁移到 ESP32 的电机闭环主线

电机控制细节、双核任务分工、Topic 设计和 Phase 1-8 路线，见 [motor_control_design.md](../docs/motor_control_design.md)。

## 2. System Role

当前系统角色如下：

- **STM32F411 = sensor/state controller**
- **ESP32-S3 = communication bridge + future motor controller**
- **ROS 2 PC = supervisor / integration layer**
- **MQTT = optional low-rate status mirror**

这意味着 ESP32 不是单纯 WiFi 网卡，也不是只写电机控制的副控，而是整个 ROS 2 接入层和后续电机执行层的核心节点。

## 3. Architecture Overview

### 3.1 STM32F411

STM32F411 保留以下职责：

- MPU6050 采样
- 姿态解算
- IMU / robot state 输出
- 原有数字孪生链路
- 与 ESP32 的串口数据通道

STM32F411 不再作为新的电机闭环主控制器，但它仍然是当前传感状态链路的源头。

### 3.2 ESP32-S3

ESP32-S3 是本仓库当前的主体，职责分两部分。

当前已实现职责：

- WiFi 接入
- micro-ROS client 生命周期管理
- Core 0 `ros_comm_task` 运行时调度
- STM32 串口上行数据解析
- ROS 2 IMU / state 发布
- ROS 2 `/cmd_vel` 订阅
- ROS 2 `/motor/target_rpm` 订阅
- mock `/motor/actual_rpm` 发布
- mock `/motor/state` 发布
- `/cmd_vel` 到 STM32 `CMDVEL` 文本命令桥接
- 运行日志和连接恢复

后续扩展职责：

- N20 编码器电机 PWM/DIR 控制
- 编码器 A/B 读取
- rpm 计算
- PID 速度闭环
- `target_rpm` 接收
- `actual_rpm` / `motor_state` 回传
- 双轮差速和 `/cmd_vel` 解算

### 3.3 ROS 2 PC

ROS 2 PC 负责：

- 运行 `micro_ros_agent`
- 接收 IMU / robot state / motor state
- 发布 `/cmd_vel` 或后续 `target_rpm`
- 用 `rqt_plot`、日志、rviz、dashboard 等工具观察系统行为
- 后续承接 `ros2_control` 上位集成

### 3.4 MQTT

MQTT 不是当前主控制链路的一部分，只是可选辅助链路，用于：

- dashboard 展示
- robot health
- motor state mirror
- 远程调试

MQTT 不进入实时 PID 闭环，不直接控制电机。

## 4. Data Paths

### 4.1 Upstream: STM32 to ROS 2

当前已经实现的上行链路：

```text
STM32 UART
  -> ESP32 serial parser
  -> /imu/data
  -> /imu/filtered
  -> /robot/state
```

这条链路是当前系统最稳定、最完整的已实现部分。

### 4.2 Downstream: ROS 2 to STM32 Legacy Control Path

当前已经实现的下行链路：

```text
ROS 2 /cmd_vel
  -> ESP32 micro-ROS subscriber
  -> UART CMDVEL,<linear_x>,<angular_z>
  -> STM32 parser
  -> STM32 legacy open-loop motor execution
```

这条链路证明了 ROS 2 到真实下位机执行层已经打通，但它只代表当前的 legacy 验证路径，不代表后续主线。

### 4.3 Local Motor-Control Path on ESP32

当前已先落地 mock motor response，用于验证 ROS 2 topic 和双核数据流；真实硬件反馈会在 N20 到货并实测后替换 mock：

```text
ROS 2 target_rpm
  -> ESP32 local control
  -> mock motor response
  -> ESP32 actual_rpm / motor_state
  -> ROS 2
```

真实闭环阶段会把中间的 mock response 替换为 `PID -> PWM/DIR -> TB6612 -> motor -> encoder A/B -> rpm calculation`。这个路径表示实时闭环迁移到 ESP32 本地完成，ROS 2 负责目标下发和状态观测。

## 5. ESP32 Software Architecture

当前代码已经形成比较清晰的 ESP32 软件分层。

### 5.1 Connectivity and Runtime

- [src/main.cpp](../src/main.cpp)
  负责启动流程、WiFi 管理、串口桥接、连接恢复和主循环调度
- [src/config/app_config.h](../src/config/app_config.h)
  集中管理 WiFi、Agent、串口、日志和初始化参数

### 5.2 micro-ROS Core

- [src/ros/uros_core.cpp](../src/ros/uros_core.cpp)
- [src/ros/uros_core.h](../src/ros/uros_core.h)

负责 `support / node / allocator` 生命周期，是 ESP32 作为 ROS 2 节点的基础层。

### 5.3 ROS Publish Layer

- [src/ros/uros_pub.cpp](../src/ros/uros_pub.cpp)
- [src/ros/uros_pub.h](../src/ros/uros_pub.h)

负责发布：

- `/imu/data`
- `/imu/filtered`
- `/robot/state`
- `/motor/actual_rpm`
- `/motor/state`

### 5.4 ROS Subscribe Layer

- [src/ros/uros_sub.cpp](../src/ros/uros_sub.cpp)
- [src/ros/uros_sub.h](../src/ros/uros_sub.h)

当前负责：

- `/cmd_vel` subscriber
- `/motor/target_rpm` subscriber
- executor spin

后续如果扩展电机闭环，它也会成为 `target_rpm` 或 `/cmd_vel` 控制接口进入 ESP32 的入口层。

### 5.5 Serial Protocol Layer

- [src/bridge/stm32_serial_parser.cpp](../src/bridge/stm32_serial_parser.cpp)
- [src/bridge/stm32_serial_parser.h](../src/bridge/stm32_serial_parser.h)

负责 STM32 上行文本协议解析，是 ESP32 连接 STM32 传感状态链路的边界层。

### 5.6 Motor Shared-State Layer

- [src/motor/motor_control_shared.cpp](../src/motor/motor_control_shared.cpp)
- [src/motor/motor_control_shared.h](../src/motor/motor_control_shared.h)

负责 `ros_comm_task` 和 `motor_control_task` 之间的共享命令 / 状态快照，是当前双核架构的最小数据交接层。

### 5.7 Motor Controller Skeleton

- [src/motor/motor_controller.cpp](../src/motor/motor_controller.cpp)
- [src/motor/motor_controller.h](../src/motor/motor_controller.h)

负责 Core 1 电机控制骨架。当前使用 mock motor response 模拟 `actual_rpm` 追踪 `target_rpm`，并预留真实 PWM/DIR 输出、encoder A/B 读取和 PID 接口。

### 5.8 Motor Control Logic

- [src/motor/motor_response_model.cpp](../src/motor/motor_response_model.cpp)
- [src/motor/motor_response_model.h](../src/motor/motor_response_model.h)
- [src/motor/encoder_rpm_estimator.cpp](../src/motor/encoder_rpm_estimator.cpp)
- [src/motor/encoder_rpm_estimator.h](../src/motor/encoder_rpm_estimator.h)
- [src/motor/speed_pid.cpp](../src/motor/speed_pid.cpp)
- [src/motor/speed_pid.h](../src/motor/speed_pid.h)
- [src/motor/single_motor_control.cpp](../src/motor/single_motor_control.cpp)
- [src/motor/single_motor_control.h](../src/motor/single_motor_control.h)

负责 N20 未到货阶段的代码级闭环设计：mock response、encoder count 到 rpm、速度 PID、单电机控制状态组合。这些模块不访问 GPIO/PWM，可通过 host unit tests 验证。

### 5.9 TB6612 Driver Boundary

- [src/motor/tb6612_driver.cpp](../src/motor/tb6612_driver.cpp)
- [src/motor/tb6612_driver.h](../src/motor/tb6612_driver.h)

负责 TB6612 A/B 双通道 PWM/DIR/STBY 驱动边界。当前文件已经落地并可编译，但还没有绑定真实 GPIO、PWM channel、频率或分辨率，也没有在 `motor_control_task` 中启用硬件输出。

## 6. Current Implemented Baseline

截至目前，代码实际已经实现的是：

- ESP32 连 WiFi
- ESP32 连接 `micro_ros_agent`
- ESP32 解析 STM32 上行 IMU / state
- ESP32 发布 `/imu/data`、`/imu/filtered`、`/robot/state`
- ESP32 订阅 ROS 2 `/cmd_vel`
- ESP32 订阅 ROS 2 `/motor/target_rpm`
- ESP32 发布 ROS 2 `/motor/actual_rpm`
- ESP32 发布 ROS 2 `/motor/state`
- ESP32 将 `/cmd_vel` 桥接为 STM32 `CMDVEL`
- ESP32 运行 Core 0 `ros_comm_task`
- ESP32 运行 Core 1 `motor_control_task`
- 两个 task 通过 shared state 传递控制命令和运行状态
- Core 1 使用 mock motor response 模拟 `actual_rpm` 追踪 `target_rpm`
- encoder rpm estimator、speed PID、single motor control 纯逻辑模块已落地并有 host tests
- TB6612 driver 文件已落地，当前仍保持未配置、未启用真实输出

也就是说，**当前已实现的是 ESP32 communication bridge + dual-core motor-control skeleton**。

尚未接入真实硬件运行的是：

- ESP32 本地 PWM/DIR 电机驱动
- 编码器读取
- 基于真实 encoder count 的 rpm 计算
- 基于真实 encoder feedback 的 PID 速度闭环
- 基于真实编码器的 `actual_rpm` / `motor_state` ROS 发布
- 基于真实电机参数的控制闭环
- 双核架构下的真实 PWM / encoder / PID 执行链

这些属于下一阶段的 ESP32 motor-control extension，而不是当前代码基线。

## 7. Motor-Control Extension Direction

电机闭环控制主线迁移到 ESP32，但它在系统设计里只是一个重要子系统，不应覆盖整份架构文档。

这一扩展方向的目标是：

- 保留 STM32 作为 sensor/state controller
- 让 ESP32 同时承担 ROS 2 bridge 和 motor controller
- 把实时 PID 闭环留在 ESP32 本地
- 把 ROS 2 保持为控制与状态链路

当前硬件阶段：

- 已购买 6V N20 编码器减速电机 x 1
- 手头有 130 普通电机 x 1，仅用于驱动通道辅助验证
- 当前 TB6612 A 通道保留给 130 普通电机
- 当前 TB6612 B 通道作为单 N20 主验证通道
- 当前不做完整底盘，不追求地面移动
- 当前目标是桌面级单 N20 闭环验证
- 单 N20 跑通后，再购买第二个同规格 N20，并把 A 通道从 130 切换为 N20

更细的双核分工、安全策略、实时控制边界和阶段路线，统一放在 [motor_control_design.md](../docs/motor_control_design.md)。

当前 ESP32 到 TB6612 / 编码器的具体 GPIO 接线规划也统一写在 [motor_control_design.md](../docs/motor_control_design.md)。

## 8. Legacy STM32 Motor Path

STM32 既有 open-loop motor control 保留，不删除。

它的定位是：

- legacy experiment
- early validation
- ROS 2 到真实下位机执行链路 proof-of-concept

它不再继续扩展为：

- 编码器读取主线
- PID 调速主线
- 双轮差速主线
- `ros2_control` 主线

等 ESP32 + N20 单电机闭环真实验证完成后，再考虑归档或删除。

## 9. MQTT Boundary

正确边界：

```text
ROS 2 target_rpm -> ESP32 local PID -> PWM/DIR -> motor
ESP32 motor_state -> ROS 2 / optional MQTT dashboard
```

不做：

```text
MQTT -> realtime PID -> motor control loop
Web dashboard -> realtime motor control loop
```

换句话说，MQTT 是监控镜像和运维入口，不是实时控制入口。

## 10. Roadmap Summary

系统级路线可以概括为两条并行演进：

1. 保持当前 ESP32 bridge 稳定，继续服务 STM32 IMU / state 上下行链路。
2. 在不破坏现有 bridge 的前提下，把新的电机闭环主线逐步迁移到 ESP32。

因此，本仓库的整体方向不是“只做 bridge”或“只做 motor”，而是：

**先稳定 bridge，再在 ESP32 上叠加 motor-control 能力，最终形成一个兼具 ROS 2 接入和本地执行能力的节点。**

## 11. Design Constraints

- 不删除 STM32 既有开环电机代码
- 不破坏已有 STM32 IMU / micro-ROS / Gazebo 数字孪生链路
- 当前阶段只做设计文档和软件骨架，不做大规模硬件工程改动
- 不写死真实 GPIO、PWM、编码器 CPR、减速比或 PID 参数
- MQTT 和 Web dashboard 不作为实时控制链路
- 电机细节不在本文件展开过深，避免系统设计文档退化成单一子系统文档
