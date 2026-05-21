# Dashboard Integration

## Boundary

Dashboard frontend 不直接连接 ROS 2、ESP32 或 micro-ROS runtime。

当前建议的边界是：

- ROS 2 侧负责采集和整理底层 topic
- ROS 2 -> MQTT bridge 负责把低频状态镜像到 dashboard topic
- dashboard backend 负责统一订阅 MQTT、对外提供 HTTP API 与 `/ws/status`
- dashboard frontend 只调用 dashboard backend

这样可以把 ROS graph、串口、QoS、Agent 与设备细节都收口在 backend 之前。

这个仓库已经和 `robot-ops-dashboard` 做过一轮联调，所以当前文档不再按“监控优先”表述。更准确的口径是：`2` 条交互链路 + `1` 条只读链路。

## Current Main Path

当前已经收束到下面三条对外链路：

```text
Interactive 1:
amr_warehouse_sim Mock WMS
  -> /home/ina/ros2_ws/src/amr_warehouse_sim
  -> SQLite / CLI / HTTP API
  -> mock_wms_executor / mock_wms_task_runner
  -> dashboard/backend task-state interaction

Interactive 2:
frontend
  -> dashboard backend POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> ros2/robot_mqtt_bridge
  -> ROS 2 /motor/cmd
  -> ESP32 motor_control_task

Read-only:
ROS 2 /imu/data or /imu/filtered
  -> robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py
  -> MQTT robot/imu
  -> dashboard backend
  -> frontend
ROS 2 /motor/status
  -> ros2/robot_mqtt_bridge
  -> MQTT robot/motor/status
  -> dashboard backend
  -> frontend
```

补充：

- `ROS 2 /cmd_vel -> ESP32 -> STM32` 仍保留为 legacy 本地控制 / 验证链路
- 但它不再作为这里的 “Interactive 1”

其中：

- `robot/imu` 的 bridge 当前维护在 `robot-ops-dashboard` 仓库
- `robot/motor/status` 的 bridge 当前维护在本仓库 `ros2/robot_mqtt_bridge`
- WMS 任务态链路当前来自 `/home/ina/ros2_ws/src/amr_warehouse_sim`，不是一个未落地的占位概念
- 已归档的 `archive/robot_status_api_bridge_legacy/` 不再是当前主链路

## Topics And Ownership

当前 dashboard 相关来源建议按“2 交互 + 1 只读”理解：

- `amr_warehouse_sim Mock WMS -> backend`：任务态 / 调度态交互入口
- `/motor/cmd`：主电机控制交互入口
- `/imu/data`、`/imu/filtered`：只读 IMU 数据源
- `/motor/status`：ESP32 发布的主电机状态 JSON
- `/motor/actual_rpm`、`/motor/state`：兼容旧调试与 bench 验证链路
- `robot/imu`：dashboard backend 消费的 IMU MQTT topic
- `robot/motor/status`：dashboard backend 消费的 motor MQTT topic
- `robot/motor/cmd`：dashboard backend 发布的 motor MQTT command topic
- `/cmd_vel`：额外保留的 legacy 本地控制入口，不计入当前对外 `2` 条交互链路

推荐由 backend 统一聚合这些数据面：

- `amr_task_status`
- `robot_state`
- `imu_state`
- `motor_state`

frontend 面向的是业务稳定接口，不需要理解 ROS 2 话题名、QoS 或 micro-ROS Agent。

## Motor Payload

`robot/motor/status` 当前建议至少包含这些字段：

- `actual_rpm`
- `measured_rpm`
- `target_rpm`
- `error_rpm`
- `pwm`
- `pwm_duty`
- `enabled`
- `control_enabled`
- `closed_loop`
- `fault`
- `timeout`
- `estop`
- `motor_state`
- `last_update_time`

其中 `motor_state` 建议保留为结构化对象，方便 dashboard 继续兼容现有容错解析逻辑。

## Transport Strategy

当前 dashboard 主链路采用 MQTT + backend HTTP/WebSocket：

```text
STM32 / ESP32
  -> ROS 2 topics
  -> ROS 2 -> MQTT bridge
  -> MQTT broker
  -> dashboard backend
  -> HTTP API + /ws/status
  -> dashboard frontend
```

这意味着：

- dashboard frontend 不直接消费 ROS 2
- backend 仍是统一汇聚层
- MQTT 承载低频状态镜像和低频人工命令，不参与实时 PID 闭环
- 电机控制命令必须经过 backend，不从 frontend 直连 ROS 2

## Archived Path

`archive/robot_status_api_bridge_legacy/` 已归档，原因是：

- 旧链路以 `latest_robot_status.json` 为中心
- 当前 `robot-ops-dashboard` 的 IMU 前端链路已经不依赖它
- 继续在旧包里混合 JSON 文件、HTTP POST、MQTT 状态镜像，会让职责边界继续发散

如果需要回看旧实验或兼容旧 JSON 轮询方案，可以查归档目录；当前新开发不再往该目录追加功能。

## Handoff

如果接下来要对接 `robot-ops-dashboard`，优先阅读：

- [motor_dashboard_interface.md](motor_dashboard_interface.md)
- [motor_dashboard_progress_2026_05_20.md](motor_dashboard_progress_2026_05_20.md)
- [robot_ops_dashboard_handoff.md](robot_ops_dashboard_handoff.md)
- [data-flow.md](data-flow.md)
- [ros2/robot_mqtt_bridge/README.md](../ros2/robot_mqtt_bridge/README.md)
