# Motor Dashboard Interface

本文收口当前电机状态上报与控制下发主链路的接口契约，面向固件、ROS 2 bridge、dashboard backend 和 frontend 联调。

当前主链路：

```text
ESP32/micro-ROS <-> ROS 2 topic <-> ROS2-MQTT bridge <-> MQTT <-> robot-ops-dashboard backend <-> frontend
```

## 1. ROS 2 Topics

### `/motor/status`

- 类型：`std_msgs/msg/String`
- 方向：`ESP32 -> ROS 2`
- QoS：`best_effort`
- 用途：当前电机主状态 JSON，上报给 `robot_mqtt_bridge`

建议字段：

```json
{
  "target_rpm": 120.0,
  "measured_rpm": 118.5,
  "actual_rpm": 118.5,
  "error_rpm": 1.5,
  "pwm": 0.31,
  "pwm_duty": 0.31,
  "enabled": true,
  "closed_loop": true,
  "fault": false,
  "timeout_ms": 800,
  "max_pwm": 0.25,
  "stop": false,
  "source": "motor_cmd"
}
```

dashboard 最少依赖这些字段：

- `target_rpm`
- `measured_rpm`
- `pwm`
- `error_rpm`
- `enabled`
- `closed_loop`
- `fault`

### `/motor/cmd`

- 类型：`std_msgs/msg/String`
- 方向：`ROS 2 -> ESP32`
- QoS：`reliable`
- 用途：dashboard 下发的电机控制主命令

命令 JSON：

```json
{
  "robot_id": "amr-001",
  "source": "dashboard_backend",
  "command_id": "motor_cmd_20260520T124101771601Z",
  "issued_at": "2026-05-20T12:41:01.771658+00:00",
  "target_rpm": 150.0,
  "enabled": true,
  "closed_loop": true,
  "max_pwm": 0.35,
  "timeout_ms": 900,
  "stop": false
}
```

安全约束：

- `stop` 优先级最高
- `enabled=false` 时不允许持续驱动
- `max_pwm` backend 限一次，ESP32 再限一次
- `timeout_ms` backend 限一次，ESP32 再限一次

## 2. MQTT Topics

### `robot/motor/status`

- 方向：`ROS 2 bridge -> MQTT -> backend`
- 发布方：`ros2/robot_mqtt_bridge`
- 消费方：`robot-ops-dashboard` backend

bridge 会把 `/motor/status` 归一化为 dashboard 友好的 payload，并补齐：

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
- `stop`
- `last_update_time`
- `motor_state`

### `robot/motor/cmd`

- 方向：`backend -> MQTT -> ROS 2 bridge`
- 发布方：dashboard backend `POST /api/robot/motor/cmd`
- 消费方：`ros2/robot_mqtt_bridge`

## 3. Dashboard Backend API

### `GET /api/robot/status`

返回当前缓存的 IMU、电机和告警状态快照。电机状态位于：

- `topics["robot/motor/status"]`
- `robot.motor_status`
- `robot.devices[]` 中的 `subsystem = "motor_driver"`

### `POST /api/robot/motor/cmd`

输入：

```json
{
  "target_rpm": 150.0,
  "enabled": true,
  "closed_loop": true,
  "max_pwm": 0.35,
  "timeout_ms": 900,
  "stop": false
}
```

返回：

```json
{
  "topic": "robot/motor/cmd",
  "published_at": "2026-05-20T12:41:01.772457+00:00",
  "payload": {
    "robot_id": "amr-001",
    "source": "dashboard_backend",
    "command_id": "motor_cmd_20260520T124101771601Z",
    "issued_at": "2026-05-20T12:41:01.771658+00:00",
    "target_rpm": 150.0,
    "enabled": true,
    "closed_loop": true,
    "max_pwm": 0.35,
    "timeout_ms": 900,
    "stop": false
  }
}
```

### `GET /ws/status`

前端通过 websocket 接收统一状态快照，电机字段与 `GET /api/robot/status` 对齐。

## 4. Frontend Fields

Motor / Encoder 卡片当前展示：

- `target_rpm`
- `measured_rpm`
- `pwm`
- `error_rpm`
- `enabled`
- `closed_loop`
- `fault`
- `max_pwm`
- `timeout_ms`

控制入口：

- `enable`
- `target_rpm`
- `max_pwm`
- `timeout_ms`
- `Apply`
- `Stop`

## 5. Bring-Up

建议直接使用：

```bash
./scripts/start_motor_dashboard_stack.sh
./scripts/check_motor_dashboard_loop.sh
```

如果 `check_motor_dashboard_loop.sh` 报 `/motor/status` 缺失，通常表示板子还在旧固件上，只发布旧的 `/motor/actual_rpm` 和 `/motor/state`，需要先刷当前仓库里的 ESP32 固件。
