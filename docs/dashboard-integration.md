# Dashboard Integration

## Boundary

Dashboard frontend 不直接连接 ROS 2、ESP32 或 MQTT。

当前建议的边界是：

- ROS 2 侧负责采集和聚合底层机器人状态
- dashboard backend 负责统一接入 AMR task status、`robot_state`、
  `imu_state`、`motor_state`
- dashboard frontend 只调用 dashboard backend 提供的 HTTP API

这样可以把机器人侧通信细节、topic/QoS、串口和 bridge 逻辑都收口在
backend 之前，避免把 frontend 绑死在 ROS 2 runtime 上。

## Current Phase

当前阶段使用 `ros2/robot_status_api_bridge/` 作为 ROS 2 到 dashboard
backend 的最小状态桥。

它订阅：

- `/imu/data`
- `/imu/filtered`
- `/robot/state`

它输出：

- `data/dashboard_state/latest_robot_status.json`

这个 JSON 文件是当前阶段给 dashboard backend 的最小集成面。backend
可以轮询读取，或者在后续版本中由可选 HTTP POST 接口接收。

## Data Ownership

dashboard backend 是统一汇聚层，负责把多路机器人与业务状态整理成稳定的
前端接口。当前建议由 backend 统一聚合这些数据面：

- `amr_task_status`
- `robot_state`
- `imu_state`
- `motor_state`

其中：

- `robot_state` 当前来自 STM32 `AlgTask` 的低频状态判别，经 ESP32
  micro-ROS bridge 发布为 `/robot/state`
- `imu_state` 当前来自 `/imu/data` 和 `/imu/filtered`
- `motor_state` 后续可从 `/motor/state`、`/motor/actual_rpm` 接入

这样 frontend 面向的是业务稳定接口，不需要理解 ROS 2 话题命名、QoS、
topic 类型或设备在线细节。

## Transport Strategy

当前阶段前后端仍使用 HTTP polling。

建议链路如下：

```text
STM32 / ESP32
  -> ROS 2 topics
  -> robot_status_api_bridge
  -> latest_robot_status.json
  -> dashboard backend
  -> HTTP API
  -> dashboard frontend
```

原因是：

- 当前目标是先把状态聚合链路跑通，而不是提前引入实时推送复杂度
- polling 更容易调试、回放和做容错
- backend 可以在读取 JSON 后再补充 task status、业务状态和鉴权逻辑

## Future Extensions

WebSocket 是后续的实时推送增强，而不是当前阶段的默认前后端接口。

推荐演进顺序是：

1. 先稳定 ROS 2 -> backend 状态聚合和 HTTP polling
2. 再在 backend 层增加 WebSocket 推送，用于更低延迟的 dashboard 刷新
3. 再根据需要增加 MQTT 低频状态镜像

MQTT 只作为未来低频状态镜像，不进入实时控制链路。

这意味着：

- 不把 MQTT 作为当前 dashboard 主链路
- 不让 dashboard 或 MQTT 参与实时控制闭环
- 不把 `cmd_vel`、电机控制、急停等链路依赖在 MQTT 上

## Non-Goals

当前阶段明确不做这些事情：

- 不修改 `firmware/stm32_sensor_node`
- 不修改 `firmware/esp32_microros_bridge` 的稳定 IMU 主链路
- 不把 dashboard bridge 放进 `imu_data_logger`
- 不让 dashboard frontend 直接连接 ROS 2
- 不把 MQTT 引入为当前主链路
