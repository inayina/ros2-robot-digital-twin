# 调试日志 - Robot State Monitor Gazebo Bridge

## 当前状态

- ROS 2 发行版：Jazzy
- Gazebo 版本系列：Harmonic / Gazebo Sim
- 桥接接口：`ros_gz_interfaces`
- 服务桥：`ros_gz_bridge parameter_bridge`
- IMU 输入 topic：`/imu/filtered`
- IMU 订阅 QoS：`best_effort`
- Gazebo 模型：`mpu6050`
- World 名称：`default`

## 问题记录

### 1. 看不到 MPU6050 模型

**现象**：

- Gazebo 已经运行，但看不到模拟的 MPU6050 方块。

**根因**：

- `models/mpu6050.sdf` 定义了 IMU sensor，但没有定义可见的 `visual`。
- bridge 会更新位姿，但之前没有稳定地先生成 `mpu6050` 模型。
- `setup.py` 没有把 `models/` 目录安装到 package share 路径。

**修复**：

- 在 `models/mpu6050.sdf` 中添加 box `visual` 和 `collision`。
- 更新 `robot_gazebo_bridge.py`，先调用 `/world/default/create`，再调用 `/world/default/set_pose`。
- 在 `setup.py` 中加入 `models/*.sdf` 安装。

**验证**：

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

### 2. 避免 Classic Gazebo 服务不匹配

**风险现象**：

- 使用 `gazebo_msgs` 或 `/spawn_entity` 会走 Gazebo Classic 的模式，不适用于当前 Jazzy + Harmonic 配置。
- 如果没有运行 `ros_gz_bridge parameter_bridge`，直接等待 ROS 2 服务 `/world/default/create` 和 `/world/default/set_pose` 会失败。

**决定**：

- 保留 `ros_gz_interfaces.srv.SpawnEntity` 和 `ros_gz_interfaces.srv.SetEntityPose`。
- 使用 Gazebo Sim 服务 `/world/default/create` 和 `/world/default/set_pose`。
- 在 launch 文件里为这两个服务启动 `ros_gz_bridge parameter_bridge`：
  ```bash
  /world/default/create@ros_gz_interfaces/srv/SpawnEntity
  /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose
  ```

**验证**：

```bash
ros2 service list | grep /world/default/create
ros2 service list | grep /world/default/set_pose
```

### 3. `ros2 topic echo /world/default/pose` 不工作

**现象**：

```text
WARNING: topic [/world/default/pose] does not appear to be published yet
Could not determine the type for the passed topic
```

**根因**：

- `/world/default/pose` 不会自动桥接到 ROS 2。
- Gazebo Harmonic 会把原生 Gazebo Transport 位姿数据流发布为 `/world/default/pose/info`。

**正确检查方式**：

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

### 3.1 `gz topic ... | grep mpu6050` 没有输出

**2026-04-11 找到的根因**：

- 缺少 `setup.cfg`，所以 `ros2 launch` 在 `lib/robot_state_monitor` 下找不到 `gazebo_bridge` 可执行文件。
- 修复之后，`gazebo_bridge` 仍然一直等待 `/world/default/create`，原因是缺少 ROS-Gazebo 服务桥。

**修复**：

- 添加 `setup.cfg`：
  ```ini
  [develop]
  script_dir=$base/lib/robot_state_monitor

  [install]
  install_scripts=$base/lib/robot_state_monitor
  ```
- 在两个 launch 文件中都添加 `ros_gz_bridge parameter_bridge` 节点。

**已验证日志**：

```text
Creating ROS->GZ service bridge [/world/default/create ...]
Creating ROS->GZ service bridge [/world/default/set_pose ...]
Spawned Gazebo model: mpu6050
Gazebo bridge node started: /imu/filtered -> mpu6050
```

### 4. Gazebo GUI 闪退

**观察到的日志**：

```text
libEGL warning: egl: failed to create dri2 screen
```

**可能原因**：

- GUI 渲染、EGL 或 NVIDIA 驱动路径在 GUI 保持打开前失败。
- 这不一定表示 Gazebo server、服务或 bridge 有问题。

**规避方式**：

- 先使用无头启动：
  ```bash
  ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
  ```
- 然后单独尝试 GUI：
  ```bash
  gz sim -g --render-engine-gui ogre
  ```
- 如有需要，再使用 GUI launch：
  ```bash
  ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
  ```

### 5. MPU6050 标记模型闪烁，并且不跟随硬件倾斜

**现象**：

- Gazebo 中的小方块快速闪烁。
- 演示时，模型没有清晰地跟随真实 MPU6050 板子的姿态。

**根因**：

- 模型原来是 dynamic，Gazebo 物理仿真可能会和频繁的 `/world/default/set_pose` 更新互相冲突。
- bridge 发送位姿更新太快，不适合视觉演示。
- ESP32 的 `/imu/filtered` 消息之前发布了真实加速度和角速度，但姿态四元数仍可能是占位用的单位四元数。

**修复**：

- 把 `models/mpu6050.sdf` 改成静态、更大、可见性更高的标记模型。
- 添加红、绿、蓝三色坐标轴条，让旋转方向更明显。
- 将默认位姿更新频率降低到 `15 Hz`。
- 后续把四元数计算和简单滤波移到了 ESP32 固件中；Gazebo bridge 现在直接使用消息中的四元数。
- Gazebo bridge 默认锁定初始 yaw，避免 MPU6050 陀螺仪漂移导致模型慢慢自转。

**演示预期**：

- Roll 和 pitch 应该能明显跟随真实板子的倾斜。
- Yaw 默认固定，因为 MPU6050 没有磁力计。
- 世界坐标位置会故意保持固定；仅靠原始 IMU 加速度不足以可靠地进行位置跟踪。

### 6. 硬件 IMU QoS 从 reliable 改为 best_effort

**现象**：

- 硬件端改成 `best_effort` 后，如果 ROS 2 订阅端仍使用默认 reliable QoS，可能收不到 `/imu/filtered` 数据。

**根因**：

- ROS 2 发布端和订阅端的 QoS reliability 不匹配。
- `rclpy` 使用裸队列深度创建订阅时会使用默认 QoS，默认 reliability 通常是 reliable。

**修复**：

- 在 `robot_gazebo_bridge.py` 中为 IMU 订阅显式创建 `QoSProfile`。
- 将 `reliability` 设置为 `QoSReliabilityPolicy.BEST_EFFORT`。
- 启动日志中标出 `qos=best_effort`，方便确认当前配置。

**验证**：

```bash
python3 -m py_compile robot_state_monitor/robot_gazebo_bridge.py
source /opt/ros/jazzy/setup.bash && python3 -c "from rclpy.qos import QoSProfile, QoSReliabilityPolicy; print(QoSReliabilityPolicy.BEST_EFFORT)"
```

## 操作手册

### 构建

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor
source install/setup.bash
```

### 无头模式验证

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

在第二个终端中运行：

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

### 尝试 GUI

```bash
ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
```

### 静态 IMU 示例

```bash
ros2 topic pub --once /imu/filtered sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

## 待办事项

- 在目标机器上确认 EGL/NVIDIA 警告之后 GUI 渲染是否正常。
- 如果模型已经生成但不在初始视角内，添加 camera pose 或 GUI config 的 launch 选项。
- 在较长时间演示中确认硬件端滤波后的四元数始终保持归一化。
