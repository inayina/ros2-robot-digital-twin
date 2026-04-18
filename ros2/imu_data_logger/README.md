# IMU Data Logger

ROS 2 Python 分析包，用于记录和实时查看 RSM V2 的 IMU 与状态话题。

## Nodes

- `imu_logger`：订阅 `/imu/data`、`/imu/filtered`，分别写入 CSV；动态发现 `/robot/state` 的消息类型并写入 JSON Lines。
- `imu_live_plot`：实时绘制 raw/filtered 线加速度、角速度和由四元数换算出的 roll/pitch/yaw。

## Build

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select imu_data_logger
source install/setup.bash
```

## Record Data

```bash
ros2 run imu_data_logger imu_logger --ros-args -p output_dir:=./data/imu_run_001
```

默认输出：

```text
data/imu_run_001/
  imu_data.csv
  imu_filtered.csv
  robot_state.jsonl
```

可配置参数：

- `output_dir`：输出目录，默认当前目录
- `raw_imu_topic`：默认 `/imu/data`
- `filtered_imu_topic`：默认 `/imu/filtered`
- `robot_state_topic`：默认 `/robot/state`

## Live Plot

```bash
ros2 run imu_data_logger imu_live_plot
```

可配置参数：

- `raw_imu_topic`：默认 `/imu/data`
- `filtered_imu_topic`：默认 `/imu/filtered`
- `time_window`：显示时间窗口，默认 `20.0` 秒
- `update_interval`：Matplotlib 刷新间隔，默认 `0.05` 秒
- `use_header_stamp`：优先使用 ROS 消息时间戳，默认 `true`
