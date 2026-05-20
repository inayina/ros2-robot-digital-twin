# robot_status_api_bridge

> Archived on 2026-05-20. This directory is kept only for reference and is no longer part of the active ROS 2 build path.

`robot_status_api_bridge` 是一个 ROS 2 Python 包，用于把现有 ROS 2
观测话题聚合成 dashboard backend 更容易消费的低频 `robot_status`
快照。

当前阶段它做三类可选输出：

- 订阅 `/imu/data`、`/imu/filtered`、`/robot/state`
- 订阅 `/motor/actual_rpm`、`/motor/state`
- 周期性写出 `data/dashboard_state/latest_robot_status.json`
- 可选 HTTP POST 到 dashboard backend，默认关闭
- 可选 MQTT publish 到本地 broker，默认关闭

这个包故意不把 dashboard frontend 直接接到 ROS 2。HTTP POST 和 MQTT
publish 都只是低频状态输出模式，默认关闭，不参与实时控制闭环。

## 当前输出

输出 JSON 会包含这些主字段：

- `latest_imu_timestamp`
- `quaternion`
- `rpy`
- `robot_state`
- `motor`
- `health_status`
- `last_update_time`
- `http_post`
- `mqtt_publish`

同时会带上每个来源 topic 的 freshness 信息。`selected_imu_topic`
表示当前快照实际选用了哪一路 IMU；`/motor/state`、
`/motor/actual_rpm` 尚未发布时，聚合快照会保留 `motor` 字段，主 JSON
中的 freshness 会显示为 `missing`，而 MQTT 的 `robot/motor/status`
会把这种状态归纳为 `reserved`。

## JSON Contract

当前默认写出的 JSON 快照包含这些 top-level 字段：

| Field | Type | Meaning |
| --- | --- | --- |
| `schema_version` | `int` | 当前快照版本，现为 `1` |
| `latest_imu_timestamp` | `float \| null` | 当前选中 IMU 的 ROS header 时间戳（秒） |
| `selected_imu_topic` | `string \| null` | 本次快照选中的 IMU topic，通常是 `/imu/filtered` 或 `/imu/data` |
| `quaternion` | `object \| null` | 当前选中 IMU 的四元数 |
| `rpy` | `object \| null` | 由四元数计算的 roll/pitch/yaw，包含 rad 和 deg |
| `robot_state` | `object` | `{value, label}` |
| `motor` | `object` | `{actual_rpm, motor_state}` |
| `health_status` | `object` | 低频健康度汇总和各来源 freshness |
| `last_update_time` | `string` | 快照生成时间，ISO8601 UTC |
| `output_mode` | `string` | 当前固定为 `json_file` |
| `http_post` | `object` | HTTP POST 输出配置快照 |
| `mqtt_publish` | `object` | MQTT 输出配置快照 |

### robot_state labels

当前 `robot_state.value` 与 `label` 的映射是：

- `0` -> `normal`
- `1` -> `vibration`
- `2` -> `collision`
- `3` -> `tip_over`
- 其他值 -> `unknown`

### health_status meanings

`health_status.status` 当前取值为：

- `waiting`
- `ok`
- `degraded`
- `stale`

`health_status.sources` 下每个来源都会带：

- `topic`
- `status`
- `age_sec`
- `last_received_time`
- `last_header_stamp`

其中主 JSON `health_status.sources[*].status` 可能为：

- `ok`
- `stale`
- `missing`

`reserved` 不会出现在主 JSON 的 freshness 里；它主要用于 MQTT 的
`robot/motor/status.status`，表示 motor 相关 topic 在当前阶段尚未进入正式硬件闭环。

## Example Snapshot

典型输出形态：

```json
{
  "schema_version": 1,
  "selected_imu_topic": "/imu/filtered",
  "robot_state": {
    "value": 0,
    "label": "normal"
  },
  "motor": {
    "actual_rpm": 0.0,
    "motor_state": "{\"target_rpm\":0.0,\"actual_rpm\":0.0}"
  },
  "health_status": {
    "status": "ok",
    "summary": "Both IMU topics and robot_state are fresh."
  }
}
```

注意：

- 示例只展示主要字段，真实快照会包含完整 freshness 和时间戳信息。
- `motor.motor_state` 当前是原始字符串，不是 bridge 再次展开后的对象。

## 构建

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select robot_status_api_bridge \
  --symlink-install
source install/setup.bash
```

## 运行

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_status_api_bridge robot_status_bridge
```

默认会写到：

```text
data/dashboard_state/latest_robot_status.json
```

如果进程不是从仓库根目录启动，节点会优先尝试定位当前仓库根目录；如果没
找到仓库根目录，再退回到当前工作目录下的相对路径。

## 主要参数

- `raw_imu_topic`
  默认值：`/imu/data`
- `filtered_imu_topic`
  默认值：`/imu/filtered`
- `robot_state_topic`
  默认值：`/robot/state`
- `output_path`
  默认值：`data/dashboard_state/latest_robot_status.json`
- `publish_rate_hz`
  默认值：`2.0`
- `imu_stale_timeout_sec`
  默认值：`1.5`
- `robot_state_stale_timeout_sec`
  默认值：`2.5`
- `enable_http_post`
  默认值：`false`
- `http_endpoint`
  默认值：``
- `http_timeout_sec`
  默认值：`2.0`
- `enable_mqtt_publish`
  默认值：`false`
- `mqtt_host`
  默认值：`127.0.0.1`
- `mqtt_port`
  默认值：`1883`
- `mqtt_topic_prefix`
  默认值：`robot`
- `mqtt_publish_rate_hz`
  默认值：与 `publish_rate_hz` 相同

## 输出模式

### JSON 文件输出

JSON 文件输出始终保留，默认路径：

```text
data/dashboard_state/latest_robot_status.json
```

它用于当前 dashboard backend 通过文件轮询读取低频状态快照。

### HTTP POST 输出

HTTP POST 默认关闭。启用示例：

```bash
ros2 run robot_status_api_bridge robot_status_bridge --ros-args \
  -p enable_http_post:=true \
  -p http_endpoint:=http://127.0.0.1:9000/api/robot/status-ingest
```

HTTP POST 使用 Python 标准库 `urllib`，不会改变 JSON 文件输出。

### MQTT publish 输出

MQTT publish 默认关闭。启用示例：

```bash
ros2 run robot_status_api_bridge robot_status_bridge --ros-args \
  -p enable_mqtt_publish:=true \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p mqtt_topic_prefix:=robot
```

依赖说明：`package.xml` 声明 `python3-paho-mqtt`，`setup.py` 声明
`paho-mqtt`。如果运行环境缺少该库，节点仍可继续执行 JSON 文件输出和
HTTP POST；只有启用 MQTT publish 时会跳过 MQTT 输出并记录 warning。

MQTT 输出 topic：

- `robot/status`：完整聚合快照，与 JSON 文件主体保持一致
- `robot/imu`：当前选中的 IMU 姿态、四元数、RPY 与 freshness
- `robot/state`：`robot_state` 的 `value`、`label` 与 freshness
- `robot/motor/status`：`actual_rpm`、`motor_state` 与 freshness；如果
  motor topic 还没有消息，则状态为 `reserved` / `missing`

说明：

- `mqtt_topic_prefix` 可修改 topic 前缀；例如设为 `amr01` 后输出到
  `amr01/status`、`amr01/imu`、`amr01/state`、
  `amr01/motor/status`。
- MQTT publish 只做状态镜像，不订阅 MQTT topic，不下发电机控制。
- 该包不修改 ESP32 固件，也不修改 `/motor/target_rpm`、`/cmd_vel` 等控制链路。

## 设计取舍

- `/imu/filtered` 和 `/imu/data` 都会被接入；输出时优先选择“最新收到”的
  IMU 姿态作为 `robot_status` 当前姿态。
- `health_status` 不参与实时控制，只用于 dashboard/backend 的低频状态判定。
- 可选 HTTP POST 使用 Python 标准库 `urllib` 实现，避免引入额外依赖。
- 可选 MQTT publish 使用 `paho-mqtt`，只发布状态，不做 MQTT 控制订阅。
