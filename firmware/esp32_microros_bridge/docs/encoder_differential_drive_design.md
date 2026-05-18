# Encoder Differential Drive Design

> 当前电机控制主线的 canonical 文档是 [motor_control_design.md](../docs/motor_control_design.md)。本文件保留为早期编码器 / 差速路线设计记录，核心架构应与 canonical 文档保持一致。

## 1. Scope

本阶段目标是基于 ESP32 构建桌面级 N20 编码器减速电机闭环验证链路。

当前不追求完整移动底盘，不要求实车落地运行。重点验证：

- ROS 2 / micro-ROS 目标速度下发
- ESP32 PWM/DIR 电机驱动
- N20 编码器 A/B 相读取
- rpm 计算
- PID 调速
- `actual_rpm` 状态回传
- 后续扩展双轮差速和 `ros2_control`

本阶段不应描述为：

- 已完成完整双轮差速底盘
- 已完成可稳定输出 `/odom` 的移动底盘
- 已完成导航级实车运动控制

## 2. Architecture Decision

电机闭环控制主线迁移到 ESP32。

原因：

- STM32F411 当前已经承担 MPU6050 采样、姿态解算、FreeRTOS 多任务和状态输出。
- 电机闭环会新增 PWM、DIR、编码器中断、PID 控制周期和 ROS 2 状态回传。
- 为避免 F411 栈、队列、中断优先级和调度复杂度继续上升，电机控制不再放到 F411。
- ESP32 更适合作为 motor controller + ROS 2 / micro-ROS bridge。
- STM32F411 保持 sensor/state controller 角色。

因此，后续 N20 编码器电机闭环不再沿用“STM32 TB6612 B 路闭环”的路线。STM32 侧此前 A 路电机验证作为已完成的执行链路 proof-of-concept 保留，但不再作为下一阶段闭环控制主线。

## 3. Module Responsibility

### 3.1 ESP32

- 接收 ROS 2 目标速度
- 输出 PWM/DIR 到电机驱动
- 读取 N20 编码器 A/B 相
- 计算 `actual_rpm`
- 执行 PID 调速
- 发布 `motor_state`
- 后续扩展左右轮差速控制

### 3.2 STM32F411

- MPU6050 采样
- 姿态解算
- IMU 四元数输出
- `robot_state` / health status
- 不再作为电机闭环主控制器

### 3.3 ROS 2 PC

- 发布 `target_rpm`，后续再发布 `/cmd_vel`
- 订阅 `actual_rpm` / `motor_state`
- 使用 `rqt_plot`、日志记录和可视化工具观察闭环效果
- 后续扩展 `ros2_control hardware_interface`

## 4. Current Hardware Stage

当前硬件阶段：

- 已购买 6V N20 编码器减速电机 x 1
- 手头有 130 普通电机 x 1，仅用于驱动通道辅助验证
- 当前 TB6612 A 通道保留给 130 普通电机
- 当前 TB6612 B 通道作为单 N20 主验证通道
- 暂不做完整底盘
- 先做单电机桌面闭环验证
- 单 N20 跑通后再购买第二个同规格 N20，并把 A 通道从 130 切换为 N20

N20 参数待确认项：

- 编码器每电机轴或输出轴每转脉冲数
- 减速比
- 编码器供电电压和输出电平
- 6V 下空载电流、启动电流和堵转电流
- 推荐 PWM 频率范围
- 空载 100 RPM 是否为输出轴标称值

在这些参数未确认前，`encoder_resolution`、`gear_ratio`、`max_motor_rpm` 都按待标定参数处理。

## 5. Control Chain

```text
ROS 2 / micro-ROS
    ↓ target_rpm
ESP32 motor controller
    ↓ PWM/DIR
Motor driver
    ↓
6V N20 encoder gear motor
    ↑ encoder A/B
ESP32 rpm calculation + PID
    ↓ actual_rpm / motor_state
ROS 2 visualization / logging
```

第一阶段控制接口使用 `target_rpm`，避免一开始就引入底盘运动学。等单电机闭环稳定后，再把接口扩展到 `/cmd_vel -> left/right target rpm`。

## 6. Current Completed Baseline

此前已经完成并可作为项目背景表述的内容：

- ROS 2 `/cmd_vel` 已经由 ESP32-S3 micro-ROS subscriber 接收。
- ESP32 已经通过 UART 向 STM32 下发 `CMDVEL,<linear_x>,<angular_z>`。
- STM32 已完成 UART 行解析，并由 `MotorTask` 控制 TB6612FNG A 路。
- TB6612FNG A 路单电机已经完成实物转动验证。
- ESP32 侧已落地 TB6612 driver 边界、mock motor response、encoder rpm estimator、speed PID 和 single motor control 纯逻辑模块。
- 当前这些 N20 相关模块仍以 host tests 和 mock feedback 验证为主，尚未接入真实 N20 编码器、真实 PWM 闭环或实测 PID 参数。

这些内容证明“ROS 2 到真实下位机执行链路”已经跑通过。新的 N20 闭环阶段则以 ESP32 为电机控制主控重新展开，不再继续增加 STM32F411 的电机控制负担。

## 7. Planned Extension

### 7.1 V1: ESP32 Open-Loop Motor Bring-up

目标是先验证 ESP32 能稳定控制电机驱动输出。

计划内容：

- 选择 ESP32 PWM 引脚和 DIR 引脚
- 先用 A 通道的 130 普通电机做驱动通道辅助验证
- 再用 B 通道的单个 N20 做主验证
- 连接电机驱动模块
- 验证正转、反转、停止
- 验证 PWM 占空比与转速的粗略关系

初始约束：

- N20 使用 6V 电机电源
- 初始 PWM 不超过 30%
- 每次方向切换前先输出 0 PWM
- 电机电源与 ESP32 逻辑电源分离并共地

### 7.2 V2: Encoder A/B Acquisition

目标是让 ESP32 可靠读取 N20 编码器。

计划内容：

- 接入编码器 A/B 相
- 使用 GPIO interrupt 或 PCNT 计数
- 维护 encoder count
- 计算固定周期内的 delta count
- 判断方向
- 校验正反转方向和计数方向一致

预期输出：

- `encoder_count`
- `delta_count`
- `direction`
- `sample_period_ms`

代码层面当前对应 `src/motor/encoder_rpm_estimator.[h/cpp]`，真实 N20 到货前只验证计数到 rpm 的纯逻辑。

### 7.3 V3: RPM Calculation

目标是把编码器计数转换为 `actual_rpm`。

计算依赖参数：

- `encoder_ppr`
- `gear_ratio`
- `counts_per_output_rev`
- `sample_period_ms`

建议输出：

- `actual_rpm`
- `filtered_rpm`
- `encoder_count`
- `delta_count`

如果低速下 `delta_count` 抖动明显，可以先使用滑动平均或一阶低通滤波，避免 PID 输入过于跳变。

### 7.4 V4: Single-Motor PID Closed Loop

目标是完成单 N20 电机速度闭环。

控制逻辑：

- 接收 `target_rpm`
- 读取 `actual_rpm`
- 计算 `error = target_rpm - actual_rpm`
- PID 输出 PWM
- PWM 输出限幅
- 发布 `motor_state`

建议状态字段：

- `target_rpm`
- `actual_rpm`
- `error_rpm`
- `pwm_duty`
- `direction`
- `control_enabled`
- `saturated`
- `timeout`
- `estop`

初始调试建议：

- 先只开 P 控制
- 稳定后再加入 I 控制
- D 项默认不启用
- 目标速度从 20 到 60 RPM 开始
- `max_motor_rpm` 初始按 100 RPM 处理，实测后修正
- 积分项必须有 anti-windup

代码层面当前对应 `src/motor/speed_pid.[h/cpp]` 和 `src/motor/single_motor_control.[h/cpp]`，真实 PID 参数保持待实测状态。

### 7.5 V5: ROS 2 Interface Stabilization

目标是把 ESP32 电机控制器变成稳定可观测的 ROS 2 节点。

建议接口：

- 订阅：`/motor/target_rpm`
- 发布：`/motor/actual_rpm`
- 发布：`/motor/state`
- 后续订阅：`/cmd_vel`

建议先用简单标量消息完成单电机验证，避免过早设计复杂自定义消息。等双电机和状态字段稳定后，再决定是否引入自定义 `MotorState` 消息。

### 7.6 V6: Dual-Motor Differential Extension

目标是在单 N20 闭环稳定后，购买第二个同规格 N20，并扩展为左右轮差速控制。

计划内容：

- 把 A 通道从 130 普通电机切换为第二个 N20
- 增加第二路编码器 A/B
- 分别计算 `left_actual_rpm` 和 `right_actual_rpm`
- 分别执行左右轮 PID
- 实现 `/cmd_vel` 到左右轮目标速度转换

需要预留参数：

- `wheel_radius`
- `wheel_base`
- `encoder_resolution`
- `gear_ratio`
- `max_linear_speed`
- `max_angular_speed`
- `max_wheel_rpm`
- `control_period_ms`

### 7.7 V7: ros2_control Alignment

目标是在闭环控制和状态反馈稳定后，再考虑与 `ros2_control` 对齐。

对齐方向：

- `hardware_interface`
- velocity command interface
- velocity state interface
- encoder-derived wheel state
- 后续 `/odom` 推导

这一阶段不应提前开始。只有当单电机和双电机底层闭环都可稳定复现后，`ros2_control` 才有实际工程意义。

## 8. Safety Requirements

以下安全要求应视为硬约束：

- PWM 限幅
- target timeout 后停车
- ESTOP 覆盖普通速度命令
- 方向切换保护
- 上电默认 0 PWM
- 电机电源和逻辑电源分离但共地
- 调试阶段优先使用限流电源

### 8.1 PWM Limit

- 初始调试不超过 30% duty
- PID 输出必须限幅
- 低速卡滞时不允许无限增加积分输出
- 需要记录 PWM 饱和状态

### 8.2 Command Timeout

- `/motor/target_rpm` 超时后必须输出 0 PWM
- timeout 状态需要进入 `motor_state`
- timeout 解除后必须收到新命令才允许恢复运动

### 8.3 Direction Change Protection

- 禁止非零 PWM 下直接正反切换
- 方向切换前先进入 0 PWM
- 必要时插入短暂 dead time

### 8.4 ESTOP

- ESTOP 必须立即覆盖 PID 输出
- ESTOP 状态下 PWM 输出为 0
- ESTOP 释放后不应自动恢复旧速度命令

## 9. Test Strategy

### 9.1 Software-only Test

不接电机，验证控制算法和状态机：

- target timeout 状态机
- PWM 限幅
- PID 输出计算
- anti-windup
- rpm 计算函数
- 方向切换逻辑

### 9.2 Board-level Test

连接 ESP32 和电机驱动，但不挂载高风险负载：

- PWM 引脚波形
- DIR 引脚状态
- target_rpm 接收
- motor_state 发布
- timeout 后 PWM 归零
- ESTOP 后 PWM 归零

### 9.3 Hardware-in-the-loop Test

连接 N20、驱动、电源和编码器：

- 开环正反转
- PWM-speed 关系
- 编码器计数方向
- rpm 计算
- 单电机 P 控制
- 单电机 PI 控制
- target timeout 实际停车
- ESTOP 实际停车

## 10. Interview / Resume Wording

当前阶段可安全表述为：

“完成 ROS 2 `/cmd_vel` 到双 MCU 执行链路联调，基于 ESP32-S3 micro-ROS 和 STM32 UART 控制实现 TB6612FNG A 路单电机实物驱动验证。”

下一阶段可表述为：

“正在将电机闭环控制主线迁移到 ESP32，基于 6V N20 编码器减速电机构建桌面级速度闭环验证链路，实现 PWM/DIR 驱动、编码器 A/B 相读取、rpm 计算、PID 调速和 `actual_rpm` 回传。”

后续扩展可表述为：

“单电机闭环稳定后，将扩展到双 N20 差速控制，并进一步对齐 `/cmd_vel`、`/motor/state`、`/odom` 和 `ros2_control hardware_interface`。”

这样的表达既保留了已经完成的真实链路，也不会把当前阶段夸大成完整移动底盘。

## 11. Summary

当前项目已经完成 ROS 2 到 STM32 执行层的 proof-of-concept：ESP32 接收 `/cmd_vel`，通过 UART 下发到 STM32，STM32 驱动 TB6612FNG A 路单电机完成实物转动。

下一阶段的主线调整为 ESP32 motor controller：

1. ESP32 开环 PWM/DIR 驱动电机
2. ESP32 读取 N20 编码器 A/B 相
3. 计算 `actual_rpm`
4. 实现单电机 PID 闭环
5. 发布 `actual_rpm` / `motor_state`
6. 单 N20 稳定后购买第二个同规格 N20
7. 后续扩展双轮差速和 `ros2_control`

这条路线能把 STM32F411 保持在 sensor/state controller 角色，同时让 ESP32 承担 motor controller + micro-ROS bridge，更适合当前桌面级闭环验证阶段。
