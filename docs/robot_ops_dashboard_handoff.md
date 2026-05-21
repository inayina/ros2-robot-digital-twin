# Robot Ops Dashboard Handoff

这份文档面向 `robot-ops-dashboard` 对接，目标是让 dashboard/backend 同学不用先读固件代码，就能知道当前该从哪里接、哪些链路已经切换到 MQTT、哪些目录已经归档。

这个仓库已经做过一轮联调。当前对外口径不再写“监控优先”，而是按访问形态收口为 `2` 条交互链路和 `1` 条只读链路。

```text
交互 1:
amr_warehouse_sim Mock WMS
  -> /home/ina/ros2_ws/src/amr_warehouse_sim
  -> SQLite / CLI / HTTP API
  -> mock_wms_executor / mock_wms_task_runner
  -> dashboard/backend task flow

交互 2:
POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> ROS 2 /motor/cmd
  -> ESP32

只读:
robot/imu
robot/motor/status
```

## Recommended Ingestion Path

当前推荐接入链路：

```text
IMU:
ROS 2 /imu/data or /imu/filtered
  -> robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py
  -> MQTT robot/imu
  -> dashboard backend

Motor:
ROS 2 /motor/status
  -> ros2/robot_mqtt_bridge
  -> MQTT robot/motor/status
  -> dashboard backend
```

不推荐：

- dashboard frontend 直接读 ROS 2 topic
- dashboard frontend 直接连 ESP32
- 继续把 `archive/robot_status_api_bridge_legacy/` 当当前主链路

## Stable Inputs

当前长期受保护的 ROS 2 topic：

- `/imu/data`
- `/imu/filtered`
- `/robot/state`
- `/cmd_vel`
- `/motor/target_rpm`
- `/motor/status`
- `/motor/actual_rpm`
- `/motor/state`

其中 dashboard 当前主消费的是只读链路：

- `robot/imu`
- `robot/motor/status`

两条交互链路分别是：

- `/home/ina/ros2_ws/src/amr_warehouse_sim` 的 Mock WMS 任务态 / 调度态交互链路
- `POST /api/robot/motor/cmd -> robot/motor/cmd -> /motor/cmd` 主电机命令链路

补充保留链路：

- `/cmd_vel` 仍保留为 legacy 本地控制 / 验证链路，但不再作为这里的“交互 1”

## Motor MQTT Fields To Consume

建议 backend 至少消费这些字段：

- `last_update_time`
- `status`
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

说明：

- `motor_state` 当前建议保留为结构化对象，便于 frontend 复用已有容错解析逻辑
- `/motor/actual_rpm` 与 `/motor/state` 仍保留给 bench/调试，但不再作为 dashboard 主入口

## Field Semantics

### robot_state

当前 `robot_state.value` 语义：

- `0` = `normal`
- `1` = `vibration`
- `2` = `collision`
- `3` = `tip_over`

### IMU Source

IMU bridge 同时允许从 `/imu/data` 或 `/imu/filtered` 镜像到 `robot/imu`。
backend 不要假设前端永远只来自某一个 ROS topic。

### Motor Status

`/motor/status` 当前来自 ESP32 发布的 JSON 字符串，主要字段已经对齐 dashboard 展示需要：

- `target_rpm`
- `measured_rpm`
- `actual_rpm`
- `error_rpm`
- `pwm`
- `enabled`
- `closed_loop`
- `fault`

在当前 skeleton 阶段，这些字段仍可能包含 mock response，但 topic 形态已经按正式 dashboard 链路收口。

## Archived Path

`archive/robot_status_api_bridge_legacy/` 已经归档，原因是：

1. `robot-ops-dashboard` 的 IMU 前端链路已经走 `robot/imu`
2. motor 状态当前也已切到 `/motor/status -> robot/motor/status`
3. 旧 JSON 文件聚合桥继续扩展会混淆职责边界

如果需要回看历史方案，可查归档目录；当前不再把它作为 active package 构建或对接。

## Startup Checklist

联调时建议按这个顺序：

1. 直接运行 `./scripts/start_motor_dashboard_stack.sh` 启动整链。
2. 复位 ESP32。
3. 优先运行 `./scripts/check_real_hw_chain.sh --dashboard` 做实机整链检查：
   - ESP32 串口设备是否已枚举
   - micro-ROS topic 和 `/cmd_vel`、`/motor/target_rpm`、`/motor/cmd` 下行订阅是否在线
   - `robot/imu` 是否到达 backend
   - `POST /api/robot/motor/cmd` 是否到达 ROS 2 `/motor/cmd`
   - `/motor/status` 是否已经由实机固件发布
4. 如果只想检查 dashboard backend / MQTT 侧，不额外检查板子串口枚举与 micro-ROS topic，可单独运行 `./scripts/check_motor_dashboard_loop.sh`，它会检查：
   - `robot/imu` 是否到达 backend
   - `POST /api/robot/motor/cmd` 是否到达 ROS 2 `/motor/cmd`
   - `/motor/status` 是否已经由实机固件发布
5. 如果脚本提示缺少 `/motor/status`，先刷当前仓库里的 ESP32 固件，再重跑检查。

2026-05-20 的现场联调状态见：

- [motor_dashboard_progress_2026_05_20.md](motor_dashboard_progress_2026_05_20.md)

WMS 侧如果需要回看之前的调试与验证证据，优先参考：

- `/home/ina/ros2_ws/src/amr_warehouse_sim/README.md`
- `/home/ina/ros2_ws/src/amr_warehouse_sim/docs/wms/reports/`

## Related Docs

- [dashboard-integration.md](dashboard-integration.md)
- [data-flow.md](data-flow.md)
- [ros2/robot_mqtt_bridge/README.md](../ros2/robot_mqtt_bridge/README.md)
- [archive/robot_status_api_bridge_legacy/README.md](../archive/robot_status_api_bridge_legacy/README.md)
