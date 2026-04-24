# Robot State Monitor

用于在 Gazebo Harmonic 中可视化真实 MPU6050 数据流的 ROS 2 Jazzy 软件包。

这个软件包会订阅 `/imu/filtered`，在 Gazebo Sim 中生成一个可见的 `mpu6050` 标记模型，并通过 Gazebo Harmonic 服务更新模型位姿。启动文件会启动 `ros_gz_bridge parameter_bridge`，让 ROS 2 可以调用 Gazebo Transport 服务。

硬件演示时，ESP32 固件会发布滤波后的姿态四元数。Gazebo bridge 在更新可见标记模型位姿前，会先归一化这个四元数。默认会锁定初始 yaw，这样 MPU6050 的陀螺仪漂移就不会让标记模型慢慢自转。

## 目录结构

- `robot_state_monitor/robot_gazebo_bridge.py`：把 `/imu/filtered` 桥接到 Gazebo 模型位姿更新的 ROS 2 节点
- `models/mpu6050.sdf`：带 RGB 坐标轴条的可见静态 MPU6050 标记模型
- `worlds/empty.sdf`：最小 Gazebo Harmonic world，包含物理、场景广播器和地面
- `launch/mpu6050_gazebo.launch.py`：稳定的无头 Gazebo server、服务桥和位姿桥
- `launch/mpu6050_gazebo_gui.launch.py`：Gazebo GUI、服务桥和位姿桥
- `start_static.sh`：无头 smoke test 脚本

## 构建

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor
source install/setup.bash
```

## 先运行无头模式

先用这个流程验证 Gazebo 服务、模型生成和 bridge 是否正常工作，不引入 GUI 渲染问题。

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

在另一个终端中确认模型已经存在于 Gazebo：

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

如果真实 ESP32/micro-ROS 数据流还没有运行，可以发布一个静态姿态：

```bash
ros2 topic pub --once /imu/filtered sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

## 硬件演示

先启动 micro-ROS Agent 和 ESP32，然后确认 IMU 数据流已经在线：

```bash
ros2 topic list | grep imu
ros2 topic echo --once /imu/filtered
```

然后启动 Gazebo：

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

真实 MPU6050 板子倾斜时，标记模型应该跟着旋转。位置会故意固定在 `(x, y, z)`，因为没有额外定位信息时，原始 MPU6050 加速度不适合用来可靠地跟踪世界坐标位置。

默认会锁定 yaw，因为 MPU6050 没有磁力计修正。演示时 roll 和 pitch 应该是最稳定的两个轴。

## 使用 GUI 运行

无头模式确认正常后，再尝试 GUI 启动：

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
```

如果 GUI 因为 EGL/NVIDIA 渲染问题闪退，可以保持无头启动继续运行，然后单独连接 GUI：

```bash
gz sim -g --render-engine-gui ogre
```

## 预期的 ROS 2 Topic

ESP32 micro-ROS 节点运行时，应该能看到这些 topic：

```text
/imu/data
/imu/filtered
/robot/state
```

Gazebo bridge 默认使用 `/imu/filtered`，并使用 `best_effort` QoS 订阅，以匹配硬件 IMU 数据流。如果要使用其他 topic：

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py imu_topic:=/imu/data
```

## 常用检查命令

```bash
ros2 node list | grep gazebo_bridge
ros2 service list | grep /world/default
gz topic -l | grep /world/default
gz topic -e -t /world/default/pose/info | grep mpu6050
```

不要使用 `ros2 topic echo /world/default/pose`，除非你已经明确把这个 Gazebo topic 桥接到了 ROS 2。在 Gazebo Harmonic 中，`/world/default/pose/info` 默认是 Gazebo Transport topic，不是 ROS 2 topic。

如果 `gz topic -e -t /world/default/pose/info | grep mpu6050` 没有输出，检查下面三个进程是否都在运行：

```bash
ros2 node list | grep gazebo_service_bridge
ros2 node list | grep gazebo_bridge
pgrep -a gz
```

## 参数

- `world_name`：Gazebo world 名称，默认 `default`
- `imu_topic`：要订阅的 IMU topic，默认 `/imu/filtered`
- `model_name`：Gazebo entity 名称，默认 `mpu6050`
- `model_file`：可选的自定义 SDF 路径
- `x`, `y`, `z`：生成和更新时使用的位置，默认 `(0.0, 0.0, 0.35)`
- `update_rate`：位姿更新频率，默认 `15.0`
- `lock_yaw`：保持初始 yaw 固定，避免陀螺仪漂移导致自转，默认 `true`
