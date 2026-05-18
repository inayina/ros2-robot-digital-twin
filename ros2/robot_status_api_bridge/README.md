# robot_status_api_bridge

`robot_status_api_bridge` 是一个 ROS 2 Python 包，用于把现有 ROS 2
观测话题聚合成 dashboard backend 更容易消费的低频 `robot_status`
快照。

当前阶段它只做两件事：

- 订阅 `/imu/data`、`/imu/filtered`、`/robot/state`
- 周期性写出 `data/dashboard_state/latest_robot_status.json`

这个包故意不把 dashboard frontend 直接接到 ROS 2，也不把 MQTT 引入为
当前主链路。HTTP POST 接口只做预留，默认关闭。

## 当前输出

输出 JSON 会包含这些主字段：

- `latest_imu_timestamp`
- `quaternion`
- `rpy`
- `robot_state`
- `health_status`
- `last_update_time`

同时会带上每个来源 topic 的 freshness 信息，以及为
`/motor/state`、`/motor/actual_rpm` 预留的占位字段。

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

## 设计取舍

- `/imu/filtered` 和 `/imu/data` 都会被接入；输出时优先选择“最新收到”的
  IMU 姿态作为 `robot_status` 当前姿态。
- `health_status` 不参与实时控制，只用于 dashboard/backend 的低频状态判定。
- 可选 HTTP POST 使用 Python 标准库 `urllib` 实现，避免引入额外依赖。
