# IMU Data Logger

`imu_data_logger` 是一个 ROS 2 Python 包，用于记录 IMU 相关数据，并对原始 IMU 与滤波后 IMU 数据进行实时可视化。

当前包含两个节点：

- `imu_logger`：记录 `/imu/data`、`/imu/filtered` 和动态发现的 `/robot/state`
- `imu_live_plot`：实时绘制 raw/filtered IMU 的线加速度、角速度和姿态角

## 主要能力

- 将 `/imu/data` 记录为 CSV
- 将 `/imu/filtered` 记录为 CSV
- 动态发现 `/robot/state` 的消息类型，并逐行写入 `JSON Lines`
- 使用 `matplotlib` 实时比较原始与滤波后的 IMU 数据
- 对 IMU 使用 `qos_profile_sensor_data`
- 对动态创建的 `/robot/state` 订阅使用 `QoSProfile(depth=10, reliability=BEST_EFFORT)`

## 构建

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imu_data_logger
source install/setup.bash
```

## 记录数据

```bash
ros2 run imu_data_logger imu_logger --ros-args \
  -p output_dir:=/tmp/imu_logs \
  -p raw_imu_topic:=/imu/data \
  -p filtered_imu_topic:=/imu/filtered \
  -p robot_state_topic:=/robot/state
```

推荐显式设置 `output_dir`，这样输出文件路径会更清晰。

输出目录示例：

```text
/tmp/imu_logs/
  imu_data.csv
  imu_filtered.csv
  robot_state.jsonl
```

## 实时绘图

```bash
ros2 run imu_data_logger imu_live_plot --ros-args \
  -p raw_imu_topic:=/imu/data \
  -p filtered_imu_topic:=/imu/filtered \
  -p time_window:=20.0 \
  -p update_interval:=0.05 \
  -p use_header_stamp:=true
```

关闭 matplotlib 窗口或按 `Ctrl+C` 可以停止节点。

## 输出文件说明

### `imu_data.csv` / `imu_filtered.csv`

每一行包含：

- 消息时间戳
- 本地接收时间
- `frame_id`
- 姿态四元数
- 角速度
- 线加速度
- 三组展平后的协方差数组

### `robot_state.jsonl`

每一行是一条完整消息记录，包含：

- `timestamp`
- `received_time`
- `topic`
- `type`
- `message`

这种格式适合记录运行时才知道具体消息类型的话题。

## 常用参数

### `imu_logger`

- `output_dir`：输出目录，默认 `.`
- `raw_imu_topic`：默认 `/imu/data`
- `filtered_imu_topic`：默认 `/imu/filtered`
- `robot_state_topic`：默认 `/robot/state`

### `imu_live_plot`

- `raw_imu_topic`：默认 `/imu/data`
- `filtered_imu_topic`：默认 `/imu/filtered`
- `time_window`：默认 `20.0`
- `update_interval`：默认 `0.05`
- `use_header_stamp`：默认 `true`

## 本地检查

```bash
pytest -q
```

## 注意事项

- `robot_state_topic` 对应的消息类型必须能在当前 ROS 2 环境中导入
- 如果同一 topic 被发现有多个消息类型，节点会使用 ROS graph 返回的第一个类型
- `BEST_EFFORT` 配置更适合高频传感器流，也更容易与 ESP32 侧发布 QoS 对齐
