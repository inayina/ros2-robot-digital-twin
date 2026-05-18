# ESP32 System Design Entry

> 主设计文档已迁移到 [docs/design.md](docs/design.md)。本文件保留为根目录入口，避免旧链接失效。

## Current Architecture

当前项目架构决策：

- **STM32F411 = sensor/state controller**
- **ESP32-S3 = communication bridge + future motor controller**
- **ROS 2 PC = supervisor / integration layer**

当前 `docs/design.md` 描述的是整个 ESP32 系统架构：既包括已经实现的 micro-ROS bridge，也包括后续迁移到 ESP32 的 motor-control extension。电机闭环是其中一个重要子系统，但不是这份主设计文档的全部。

当前代码已经落地双核任务骨架：Core 0 运行 `ros_comm_task`，Core 1 运行 `motor_control_task`。但真实 PWM / encoder / PID 硬件闭环仍在下一阶段实现。

## Responsibility Boundary

STM32F411 保留：

- MPU6050 采样
- 姿态解算
- IMU / robot state 输出
- 原有数字孪生链路

ESP32-S3 当前与后续共同职责：

- WiFi + micro-ROS 接入
- STM32 串口协议桥接
- IMU / robot state 上行发布
- `/cmd_vel` 下行桥接
- N20 编码器减速电机 PWM/DIR 控制
- 编码器 A/B 读取
- rpm 计算
- PID 速度闭环
- ROS 2 / micro-ROS 目标速度接收和状态回传
- 后续双轮差速、`/cmd_vel` 解算和 `ros2_control` 接入

ROS 2 PC 负责：

- 发布 `target_rpm`，后续发布 `/cmd_vel`
- 接收 `actual_rpm` / `motor_state`
- 用 `rqt_plot`、日志和可视化验证闭环
- 后续作为 `ros2_control hardware_interface` 的上位接口

## Legacy STM32 Motor Code

STM32 既有 open-loop motor control 不删除，但只保留为 legacy experiment / early validation。

它不再继续扩展为编码器读取、PID 调速、双轮差速或 `ros2_control` 主线。等 ESP32 + N20 单电机闭环真实验证完成后，再考虑归档或删除。

## Current Direction

当前硬件阶段：

- 已购买 6V N20 编码器减速电机 x 1
- 手头有 130 普通电机 x 1，仅用于驱动通道辅助验证
- 当前 TB6612 A 通道保留给 130 普通电机
- 当前 TB6612 B 通道作为单 N20 主验证通道
- 当前不做完整底盘，不追求地面移动
- 当前目标是桌面级单电机闭环验证
- 单 N20 跑通后，再购买第二个同规格 N20，并把 A 通道从 130 切换为 N20 扩展双轮差速

正确实时控制链路：

```text
ROS 2 target_rpm -> ESP32 local PID -> PWM/DIR -> motor
ESP32 motor_state -> ROS 2 / optional MQTT dashboard
```

MQTT 只用于低频状态上报、dashboard 展示、robot health、motor state mirror 和远程调试，不进入实时 PID 闭环。

## Documentation

- 系统级架构：[docs/design.md](docs/design.md)
- 电机控制细节：[docs/motor_control_design.md](docs/motor_control_design.md)
- 既有编码器差速设计记录：[docs/encoder_differential_drive_design.md](docs/encoder_differential_drive_design.md)
