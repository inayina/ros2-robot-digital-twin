# imu_data_logger

`imu_data_logger` 是一个 ROS 2 Python 包，用于记录 IMU 相关数据，并对原始 IMU 与滤波后 IMU 数据进行实时可视化。

当前提供两个节点：

- `imu_logger`：订阅 IMU 和 robot state 话题并落盘
- `imu_live_plot`：使用 matplotlib 实时绘制原始与滤波后的 IMU 数据

## 功能概览

- 将 `/imu/data` 记录为 CSV
- 将 `/imu/filtered` 记录为 CSV
- 动态发现 `/robot/state` 的消息类型，并将消息逐行写入 JSON Lines 文件
- 实时显示原始 IMU 和滤波后 IMU 的：
  - 线加速度
  - 角速度
  - 姿态角 `roll`、`pitch`、`yaw`
- IMU 订阅使用 `qos_profile_sensor_data`
- `robot_state` 动态订阅使用 `BEST_EFFORT`，队列深度为 `10`

## 依赖

本包依赖以下组件：

- `rclpy`
- `sensor_msgs`
- `rosidl_runtime_py`
- `python3-matplotlib`

该包使用 `ament_python` 构建，适合放在 ROS 2 工作区中使用。

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select imu_data_logger
source install/setup.bash
```

## 快速开始

### 1. 记录 IMU 和 robot state 数据

```bash
ros2 run imu_data_logger imu_logger --ros-args \
  -p output_dir:=/tmp/imu_logs \
  -p raw_imu_topic:=/imu/data \
  -p filtered_imu_topic:=/imu/filtered \
  -p robot_state_topic:=/robot/state
```

建议始终显式设置 `output_dir`，这样输出文件会写到一个明确的位置。

### 2. 实时绘图查看原始与滤波后 IMU

```bash
ros2 run imu_data_logger imu_live_plot --ros-args \
  -p raw_imu_topic:=/imu/data \
  -p filtered_imu_topic:=/imu/filtered \
  -p time_window:=20.0 \
  -p update_interval:=0.05 \
  -p use_header_stamp:=true
```

关闭 matplotlib 窗口或按 `Ctrl+C` 可以停止绘图节点。

## 节点说明

### `imu_logger`

这个节点会生成：

- 原始 IMU 对应的 CSV 文件
- 滤波后 IMU 对应的 CSV 文件
- `robot_state.jsonl` 文件

行为说明：

- 原始 IMU 和滤波后 IMU 使用 `qos_profile_sensor_data` 订阅
- `robot_state_topic` 会通过 ROS graph 动态发现类型
- 发现类型后，节点会动态导入对应消息类型并建立订阅
- `robot_state` 订阅使用 `QoSProfile(depth=10, reliability=BEST_EFFORT)`
- 如果 `robot_state` 消息包含 `header`，则使用 `header.stamp` 作为时间戳
- 如果消息没有 `header`，则使用本地接收时间作为时间戳

参数：

- `output_dir`
  
  默认值：`.`
  
  输出目录。

- `raw_imu_topic`
  
  默认值：`/imu/data`
  
  原始 IMU 话题名。

- `filtered_imu_topic`
  
  默认值：`/imu/filtered`
  
  滤波后 IMU 话题名。

- `robot_state_topic`
  
  默认值：`/robot/state`
  
  需要动态发现并记录的 robot state 话题名。

### `imu_live_plot`

这个节点订阅原始 IMU 和滤波后 IMU，并使用 matplotlib 实时绘图。图中包含三组信号：

- 线加速度
- 角速度
- 由四元数转换得到的姿态角

参数：

- `raw_imu_topic`
  
  默认值：`/imu/data`
  
  原始 IMU 话题名。

- `filtered_imu_topic`
  
  默认值：`/imu/filtered`
  
  滤波后 IMU 话题名。

- `time_window`
  
  默认值：`20.0`
  
  图中显示的时间窗口，单位为秒。

- `update_interval`
  
  默认值：`0.05`
  
  图像刷新周期，单位为秒。

- `use_header_stamp`
  
  默认值：`true`
  
  如果为 `true`，优先使用 `msg.header.stamp` 作为绘图时间；如果时间戳无效，则退回到本地接收时间。

## 输出文件

### IMU CSV 文件

IMU 日志文件名会根据话题名自动生成，例如：

- `/imu/data` -> `imu_data.csv`
- `/imu/filtered` -> `imu_filtered.csv`

每一行包含：

- 消息时间戳
- 本地接收时间
- `frame_id`
- 姿态四元数
- 角速度
- 线加速度
- 展平后的协方差数组：
  - `orientation_covariance`
  - `angular_velocity_covariance`
  - `linear_acceleration_covariance`

### `robot_state.jsonl`

`robot_state` 日志采用 JSON Lines 格式，每行一条消息，例如：

```json
{
  "timestamp": 1777008510.626171,
  "received_time": 1777008510.6273706,
  "topic": "/robot/state",
  "type": "std_msgs/msg/Int32",
  "message": {
    "...": "..."
  }
}
```

这种格式适合记录“类型运行时才知道”的 topic，也方便后续脚本逐行处理。

## QoS 说明

- `imu_logger` 对 `/imu/data` 和 `/imu/filtered` 使用 `qos_profile_sensor_data`
- `imu_live_plot` 对两个 IMU 输入也使用 `qos_profile_sensor_data`
- 动态创建的 `robot_state` 订阅使用 `BEST_EFFORT`，深度为 `10`

这个配置更适合高频传感器流，优先保证低延迟和兼容性。

## 开发与检查

运行本地检查：

```bash
pytest -q
```

## 注意事项

- `robot_state_topic` 对应的消息类型必须在当前 ROS 2 环境中可导入，否则节点无法建立订阅
- 如果同一个 `robot_state_topic` 被发现有多个类型，节点会使用 ROS graph 返回的第一个类型
- 当前 `package.xml` 和 `setup.py` 中的包描述与许可证字段仍然是占位内容，后续可以补全
