# Motor Control Design

本文件是顶层电机控制设计说明，面向 Robot Ops Dashboard 和系统集成。ESP32 侧更详细的实现设计见 [../firmware/esp32_microros_bridge/docs/motor_control_design.md](../firmware/esp32_microros_bridge/docs/motor_control_design.md)。

## Scope

当前目标是单个 6V N20 编码器减速电机的桌面级闭环 bench，以及 dashboard 可观察的电机状态链路。

当前不是：

- 完整双轮差速底盘
- `/odom` 或导航级运动控制
- `ros2_control`
- dashboard 侧实时闭环控制器

## Architecture

```text
Dashboard / ROS 2 command
  -> /motor/cmd or /motor/target_rpm
  -> ESP32 ros_comm_task
  -> shared motor command
  -> ESP32 motor_control_task
  -> safety gate
  -> speed controller
  -> PWM / DIR / STBY
  -> TB6612
  -> N20 motor
  -> encoder A/B
  -> rpm estimator
  -> /motor/status and /motor/actual_rpm
```

职责划分：

- STM32 保留 IMU、姿态、状态判别和 legacy `/cmd_vel` 执行链路。
- ESP32 是 N20 编码器电机控制主线，负责 PWM、方向、编码器读取、速度估算、闭环控制和安全停止。
- ROS 2 / MQTT / dashboard 只做集成、低频命令、低频状态和可视化，不进入高频 PID 循环。

## PWM And Direction Output

当前 ESP32 规划通过 TB6612 驱动电机：

- `PWMA/PWMB` 表示占空比。
- `AIN1/AIN2`、`BIN1/BIN2` 表示方向、coast 或 brake。
- `STBY` 作为驱动待机控制，由软件掌控。
- `motor_control_task` 是唯一允许更新 PWM/DIR/STBY 的任务。

当前通道规划：

- TB6612 A 通道：保留给 130 普通电机做驱动通道辅助验证。
- TB6612 B 通道：当前单 N20 编码器电机闭环主验证通道。

默认安全边界：

- 普通启动默认不输出真实 TB6612 PWM。
- `/motor/cmd` 中的 `max_pwm` 需要被固件限幅。
- 不在未确认电机真实速度能力和供电能力前提高目标速度。

## Speed Feedback

N20 编码器 A/B 直接接 ESP32，不经过 TB6612。

反馈链路：

```text
N20 encoder A/B
  -> ESP32 GPIO interrupt / pulse counting
  -> encoder count / delta
  -> rpm estimator
  -> raw_rpm / filtered_rpm
  -> actual_rpm / measured_rpm
```

调试时必须重点观察：

- `encoder_count`
- `encoder_delta`
- `invalid_transitions`
- `raw_rpm`
- `filtered_rpm`
- `actual_rpm` / `measured_rpm`

如果 PWM 增大但 RPM 没有合理变化，优先排查接线、供电、方向、编码器计数和开环响应，不要直接增加 `Ki`。

## Closed-Loop Control

当前闭环以速度为目标：

```text
target_rpm - actual_rpm
  -> error_rpm
  -> P / PI / PID controller
  -> pwm command
  -> max_pwm clamp
  -> direction and PWM output
```

调参原则：

- 先开环 PWM sweep，再闭环。
- 先 P-only，再小幅加入 I。
- 输出饱和时不要盲目加 `Ki`。
- `target_rpm = 0` 时必须清输出并回到 coast / standby 安全状态。
- 方向反了时先修方向或编码器符号，不用 PID 参数掩盖。

建议按照 [motor_closed_loop_tuning_process.md](motor_closed_loop_tuning_process.md) 和 [skills/motor_closed_loop_tuning_skill.md](skills/motor_closed_loop_tuning_skill.md) 执行。

## Command Interfaces

### `/motor/cmd`

Dashboard 主命令入口，类型为 `std_msgs/msg/String`，内容为 JSON。当前字段边界包括：

- `target_rpm`
- `enabled`
- `closed_loop`
- `max_pwm`
- `timeout_ms`
- `stop`
- `hardware_enable`，仅用于已确认安全的 bench 摆位

安全优先级：

1. `stop=true` 最高优先级。
2. `enabled=false` 必须关闭输出。
3. 命令超时必须停车。
4. `max_pwm` 必须钳位。

### `/motor/target_rpm`

直接 RPM 调试入口，类型为 `std_msgs/msg/Float32`。它适合 ROS 2 层 bench 和开发验证，不作为 dashboard 主命令接口。

## Status Interfaces

### `/motor/status`

Dashboard 主状态输出，类型为 `std_msgs/msg/String`，JSON 字段建议包含：

- `target_rpm`
- `actual_rpm`
- `measured_rpm`
- `error_rpm`
- `pwm`
- `pwm_duty`
- `max_pwm`
- `enabled`
- `control_enabled`
- `closed_loop`
- `hardware_outputs_enabled`
- `fault`
- `timeout`
- `estop`
- `motor_state`
- `last_update_time`

### `/motor/actual_rpm` And `/motor/state`

这两个 topic 保留给 bench、曲线和旧调试链路。dashboard 主链路优先消费 `/motor/status -> robot/motor/status`。

## Safety Stop

必须保留的安全停止条件：

- `stop=true`
- `enabled=false`
- 命令超时
- `target_rpm = 0`
- 运行时 fault / estop
- bench profile 结束
- Agent / 命令链路失效时不继续保持高 PWM

停止动作至少要做到：

- PWM 输出回到 `0`
- 方向输出进入 coast 或明确安全态
- 必要时拉低 `STBY`
- 清理或冻结积分，避免恢复时暴冲
- 状态里明确报告 timeout / fault / enabled / hardware output 状态

## Dashboard Boundary

Dashboard 可以做：

- 显示 target / actual RPM
- 显示 PWM、限幅、超时和 fault
- 发低频人工命令
- 触发 stop
- 辅助观察调参曲线

Dashboard 不可以做：

- 直接连接 ESP32
- 直接连接 ROS 2 graph
- 直接进入高频 PID
- 跳过 backend 直接控制硬件
- 把 MQTT 当成实时控制总线
