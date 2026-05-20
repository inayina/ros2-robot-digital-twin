# Robot Ops Dashboard Handoff

这份文档面向 `robot-ops-dashboard` 对接，目标是让 dashboard/backend 同学不用先读固件代码，就能知道当前该从哪里接、哪些链路已经切换到 MQTT、哪些目录已经归档。

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

其中 dashboard 当前主消费的是：

- `robot/imu`
- `robot/motor/status`

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
3. 运行 `./scripts/check_motor_dashboard_loop.sh` 检查：
   - `robot/imu` 是否到达 backend
   - `POST /api/robot/motor/cmd` 是否到达 ROS 2 `/motor/cmd`
   - `/motor/status` 是否已经由实机固件发布
4. 如果脚本提示缺少 `/motor/status`，先刷当前仓库里的 ESP32 固件，再重跑检查。

2026-05-20 的现场联调状态见：

- [motor_dashboard_progress_2026_05_20.md](motor_dashboard_progress_2026_05_20.md)

## Related Docs

- [dashboard-integration.md](dashboard-integration.md)
- [data-flow.md](data-flow.md)
- [ros2/robot_mqtt_bridge/README.md](../ros2/robot_mqtt_bridge/README.md)
- [archive/robot_status_api_bridge_legacy/README.md](../archive/robot_status_api_bridge_legacy/README.md)
