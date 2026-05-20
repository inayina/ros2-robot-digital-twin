# Motor Dashboard Progress - 2026-05-20

本文记录电机状态与 dashboard 对接阶段的当前进度，重点区分“主机侧软件链路已验证”与“实机固件仍待更新”的边界。

## 已完成

### 1. 主链路已经收束

当前 active path 已统一为：

```text
ESP32/micro-ROS <-> ROS 2 topic <-> ROS2-MQTT bridge <-> MQTT <-> robot-ops-dashboard backend <-> frontend
```

旧的 `ros2/robot_status_api_bridge` 已归档到：

- `archive/robot_status_api_bridge_legacy/`

不再继续扩旧 `api` 目录。

### 2. 控制下发链路已完成软件闭环

已落地：

- frontend -> `POST /api/robot/motor/cmd`
- backend -> MQTT `robot/motor/cmd`
- `robot_mqtt_bridge` -> ROS 2 `/motor/cmd`
- ESP32 固件侧已实现 `/motor/cmd` JSON 解析和安全约束逻辑

2026-05-20 实机联调中，已现场验证：

- dashboard backend 返回了已发布命令 payload
- ROS 2 `/motor/cmd` 上实际看到了该命令 JSON

### 3. IMU 上报链路已现场贯通

2026-05-20 实机联调中，已现场验证：

- micro-ROS Agent 实际收到 ESP32 连接
- ROS 2 `/imu/data` 持续有实机数据
- `robot-ops-dashboard/scripts/microros_imu_to_mqtt_bridge.py` 可把 `/imu/data` 转到 `robot/imu`
- dashboard backend 的 `/api/robot/status` 已能看到最新 IMU 数据

### 4. `/motor/status` 软件桥已具备

已落地：

- ESP32 固件会发布 `/motor/status`
- `robot_mqtt_bridge` 会把 `/motor/status` 转到 `robot/motor/status`
- dashboard backend/fronted 已能消费这些字段

## 当前阻塞

### 1. 板子仍在旧固件上

2026-05-20 的实机 topic 现状是：

- 可见：`/imu/data`、`/imu/filtered`、`/robot/state`、`/motor/actual_rpm`、`/motor/state`
- 缺失：`/motor/status`

这说明现场 ESP32 仍在运行旧固件，尚未刷入当前仓库里带有 `/motor/status` 和 `/motor/cmd` 的版本。

### 2. 因此“状态上报实机闭环”还差最后一步刷机

当前已经确认：

- 主机侧 bridge 代码、backend、frontend、命令链路都在
- 实机 IMU 主链路在

但由于板子还是旧固件，`/motor/status -> robot/motor/status -> /api/robot/status -> frontend` 这条“实机电机状态链”还不能算最终完成。

## 新增资产

### 启动脚本

- `scripts/start_motor_dashboard_stack.sh`

用途：

- 一键启动 MQTT broker、micro-ROS Agent、IMU bridge、motor status bridge、motor cmd bridge、dashboard backend、frontend

### 联调检查脚本

- `scripts/check_motor_dashboard_loop.sh`

用途：

- 检查 ROS 2 topic 是否齐全
- 检查 backend `/health` 和 `/api/robot/status`
- 检查 `robot/imu`
- 检查 `POST /api/robot/motor/cmd -> /motor/cmd`
- 明确报告 `/motor/status` 缺失时的旧固件阻塞

### 接口文档

- `docs/motor_dashboard_interface.md`

## 下一步

1. 用当前仓库重新编译并刷写 ESP32 固件
2. 复位板子后确认 `/motor/status` 与 `/motor/cmd` 出现在 ROS 2 graph
3. 重跑 `./scripts/check_motor_dashboard_loop.sh`
4. 在 dashboard 页面做一次真实 `enable / target_rpm / stop` bench 验证

## 备注

当前固件仍保持安全默认：

- `kEnableMotorHardwareOutputs = false`

这意味着即使刷到新固件，TB6612 B 路真实驱动默认仍不会直接输出，仍需 bench 确认后再打开。
