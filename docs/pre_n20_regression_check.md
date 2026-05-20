# Pre-N20 Regression Check

本检查用于在接入真实 N20 编码器电机前，确认 ESP32 双核 skeleton 不会复发旧版本的通信拥塞问题。本轮只验证运行时负载、限频策略和 mock motor telemetry；不要接真实 N20，不要改 GPIO，不要启用真实 PWM / encoder / PID。

## 背景问题

旧版本还没有代码层面的双核任务拆分，IMU parse、micro-ROS publish、subscriber spin 和 motor telemetry 挤在同一执行路径。上行 IMU 以 `100 Hz` 运行时，再加入 motor telemetry 会出现不稳定；降到 `50 Hz` 后通信链路恢复稳定。

当前规避思路是把职责拆开：

- Core 0：`ros_comm_task`，负责 WiFi、micro-ROS、STM32 串口解析、ROS publish/subscriber spin。
- Core 1：`motor_control_task`，负责本地电机控制周期、command timeout、mock motor response、共享状态更新。
- 控制频率和状态发布频率解耦：电机本地控制可以保持 `10 ms / 100 Hz`，ROS 2 状态回传按 topic 降频。

接入 N20 前必须先验证：通信负载增加时，Core 0 的 publish/subscriber 压力不会明显影响 Core 1 的 `motor_control_task` 周期和 jitter。

## 当前代码审计

审计日期：2026-05-19。

| 检查项 | 当前状态 | 代码位置 |
| --- | --- | --- |
| `ros_comm_task` pinned to Core 0 | 是，使用 `app_config::kRosCommTaskCore = 0` | `firmware/esp32_microros_bridge/src/config/app_config.h`, `src/main.cpp` |
| `motor_control_task` pinned to Core 1 | 是，使用 `app_config::kMotorControlTaskCore = 1` | `firmware/esp32_microros_bridge/src/config/app_config.h`, `src/main.cpp` |
| `motor_control_task` 使用 `vTaskDelayUntil` 保持 `10 ms` 周期 | 是，周期来自 `app_config::kMotorControlTaskPeriodMs = 10` | `firmware/esp32_microros_bridge/src/main.cpp` |
| micro-ROS publish 只发生在通信侧 | 是，`urosPub*` 只从 `ros_comm_task` 路径调用 | `firmware/esp32_microros_bridge/src/main.cpp`, `src/ros/uros_pub.cpp` |
| `motor_control_task` 不直接 publish ROS topic | 是，只更新 shared motor state，不调用 `urosPub*` | `firmware/esp32_microros_bridge/src/main.cpp`, `src/motor/motor_control_shared.cpp` |
| `target_rpm` 和 `motor_state` 通过共享模块传递 | 是，使用 `motor_control_shared.[h/cpp]`，没有散落裸写全局变量 | `firmware/esp32_microros_bridge/src/motor/motor_control_shared.*` |
| 当前仍保持 mock motor response | 是，Core 1 调用 `motorControllerUpdateMock` | `firmware/esp32_microros_bridge/src/main.cpp`, `src/motor/motor_controller.cpp` |

## 默认限频策略

发布频率集中在 `firmware/esp32_microros_bridge/src/config/app_config.h`：

| Topic / log | Config | Default |
| --- | --- | --- |
| `/imu/data` | `kImuPublishIntervalMs` | `20 ms` / `50 Hz` |
| `/imu/filtered` | `kFilteredImuPublishIntervalMs` | `40 ms` / `25 Hz` |
| `/robot/state` | `kRobotStatePublishIntervalMs` | `100 ms` / `10 Hz` |
| `/motor/status` JSON | `kMotorStateJsonPublishIntervalMs` | `200 ms` / `5 Hz` |
| `/motor/actual_rpm` | `kMotorActualRpmPublishIntervalMs` | `50 ms` / `20 Hz` |
| `/motor/state` JSON | `kMotorStateJsonPublishIntervalMs` | `200 ms` / `5 Hz` |
| runtime stats print | `kRuntimeStatsPrintIntervalMs` | `10000 ms` / `10 s` |

`/motor/status` 和 `/motor/state` 当前共用低频 JSON 发布节奏；`/motor/actual_rpm` 可以稍高频用于曲线观察。

## 运行时统计日志

默认每 `10 s` 打印一次 `[RUNTIME]` 窗口统计：

- `ros_loop`：`ros_comm_task` loop count。
- `ros_max_interval_ms`：窗口内 `ros_comm_task` 最大 loop interval。
- `motor_loop`：`motor_control_task` loop count。
- `motor_max_jitter_ms`：窗口内 `motor_control_task` 相对 `10 ms` 周期的最大 jitter。
- `stm32_imu_frames`：窗口内 STM32 IMU frames received。
- `robot_state_frames`：窗口内 robot state frames received。
- `pub_ok imu / filtered / robot_state / motor_actual / motor_state / motor_status`：窗口内成功发布计数。
- `pub_err imu / filtered / robot_state / motor_actual / motor_state / motor_status`：窗口内按 topic 统计的发布错误计数。
- `free_heap`：ESP32 free heap。
- `ros_stack_hwm_bytes`：`ros_comm_task` stack high water mark。
- `motor_stack_hwm_bytes`：`motor_control_task` stack high water mark。

示例日志形态：

```text
[RUNTIME] window_ms=10000 ros_loop=... ros_max_interval_ms=... motor_loop=... motor_max_jitter_ms=...
[RUNTIME] stm32_imu_frames=... robot_state_frames=... stm32_dropped_total=...
[RUNTIME] pub_ok imu=... filtered=... robot_state=... motor_actual=... motor_state=... motor_status=...
[RUNTIME] pub_err imu=... filtered=... robot_state=... motor_actual=... motor_state=... motor_status=...
[RUNTIME] free_heap=... ros_stack_hwm_bytes=... motor_stack_hwm_bytes=...
```

## Pre-N20 验证步骤

1. 不接真实 N20，不修改 TB6612 / encoder GPIO，保持 `kEnableTb6612ChannelABenchTest = false`。
2. 启动 micro-ROS Agent。
3. 编译并烧录 ESP32，打开串口监视。
4. 确认启动日志包含：
   - `ros_comm_task started on core 0`
   - `motor_control_task started on core 1`
   - `micro-ROS connected!`
5. 发布 `/motor/target_rpm` 阶跃或慢速变化目标，观察 mock `/motor/status`、`/motor/actual_rpm` 和 `/motor/state` 是否稳定。
6. 同时观察 `/imu/data`、`/imu/filtered`、`/robot/state` 是否仍稳定。
7. 连续观察至少数个 `[RUNTIME]` 窗口，重点看：
   - `motor_loop` 每 `10 s` 约为 `1000`。
   - `motor_max_jitter_ms` 不应随 Core 0 publish 压力明显变大。
   - `pub_err ...` 应保持为 `0` 或可解释的短暂连接错误。
   - `ros_stack_hwm_bytes` 和 `motor_stack_hwm_bytes` 不应持续逼近 `0`。
   - `free_heap` 不应持续下降。

## 通过条件

接入真实 N20 前，至少应满足：

- `/motor/target_rpm` 订阅稳定。
- mock `/motor/status` JSON 发布稳定且低频。
- mock `/motor/actual_rpm` 发布稳定。
- mock `/motor/state` JSON 发布稳定且低频。
- IMU 数据仍稳定。
- `motor_control_task` 在 Core 1 保持 `10 ms` 周期，jitter 没有因为 Core 0 发布压力明显变大。

## 频率边界原则

- STM32 可以继续 `100 Hz` 采样。
- ESP32 不需要把所有数据都 `100 Hz` 发布到 ROS 2。
- 电机本地控制可以 `100 Hz`。
- ROS / MQTT / dashboard 状态回传必须降频。
- 真实 N20 接入前必须先通过本 regression check。
