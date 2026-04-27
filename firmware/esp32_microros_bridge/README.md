# ESP32 micro-ROS Bridge

这个子工程是 `robot-state-monitor-v1` 里的 ESP32-S3 网桥固件。它位于 STM32 传感节点与 ROS 2 主机之间，负责两条链路：

1. `STM32 UART -> ESP32 -> /imu/data /imu/filtered /robot/state`
2. `ROS 2 /cmd_vel -> ESP32 -> UART CMDVEL -> STM32`

当前代码已经对齐到这次联调后的结构：

- 发布 `/imu/data`：`sensor_msgs/msg/Imu`
- 发布 `/imu/filtered`：`sensor_msgs/msg/Imu`
- 发布 `/robot/state`：`std_msgs/msg/Int32`
- 订阅 `/cmd_vel`：`geometry_msgs/msg/Twist`

## 代码结构

- `src/app_config.h`
  发布版配置入口，放串口参数、重试周期和调试开关；WiFi/Agent 地址来自 `include/wifi_config.h`
- `src/main.cpp`
  WiFi 管理、micro-ROS 初始化、串口桥接和运行期日志
- `src/uros_core.[h/cpp]`
  `rclc support / node / allocator`
- `src/uros_pub.[h/cpp]`
  `/imu/data`、`/imu/filtered`、`/robot/state`
- `src/uros_sub.[h/cpp]`
  `/cmd_vel` subscriber 与 executor
- `src/stm32_serial_parser.[h/cpp]`
  STM32 上行文本协议解析

## 本地配置

真实 WiFi 凭据不要提交到 Git。第一次构建前先复制示例文件：

```bash
cp include/wifi_config.example.h include/wifi_config.h
```

然后编辑 `include/wifi_config.h`：

```cpp
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define AGENT_IP "192.168.1.8"
#define AGENT_PORT 8888
```

`src/app_config.h` 会从这个文件读取：

- WiFi SSID / Password
- Agent IP / Port

下面这些仍然在 `src/app_config.h` 里维护：

- ROS 节点名
- IMU 滤波参数
- WiFi 重试周期、micro-ROS 初始化重试周期
- `[IMU ...]`、`[CMDVEL]`、`[STM32]`、运行期状态日志开关

## 串口协议

### STM32 -> ESP32

- `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
- `IMU,ax,ay,az,gx,gy,gz,temp`
- `State:<n>`
- `DBG:...`

说明：

- `IMUQ` 是当前主格式，包含四元数姿态
- 旧 `IMU` 格式仍兼容，但会用单位四元数占位
- `DBG:` 行会被当作调试文本转发到 USB 串口，不会发布成 ROS 话题
- 训练 CSV 默认不会被当作正式 IMU 输入，因为 `kAcceptTrainCsvAsImu = false`

### ESP32 -> STM32

```text
CMDVEL,<linear_x>,<angular_z>\n
```

例如：

```text
CMDVEL,0.200,0.000
CMDVEL,0.000,1.000
```

## 连接模型

当前默认是简化连接模型：

1. 上电后先连 WiFi
2. WiFi 连上后配置 UDP transport
3. 周期性尝试 `createMicroRosEntities()`
4. 初始化成功后持续 `spin`
5. 只有 WiFi 真掉线时才销毁 entities 并重建

运行态默认不再周期性 `ping` Agent，这样更适合高频 IMU 上行的现场链路。

## 构建与上传

```bash
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

如果你的环境里已经有 `pio`，也可以直接用 `pio run`。

## 预期启动日志

```text
ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
Requested STM32 GAZEBO mode
WiFi Connected! IP: ...
micro-ROS WiFi transport configured
Connecting to micro-ROS Agent at ...:8888
micro-ROS connected!
```

## 常用检查

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
ros2 topic info -v /cmd_vel
```

## 相关文档

- `design.md`
- `Debug.md`
