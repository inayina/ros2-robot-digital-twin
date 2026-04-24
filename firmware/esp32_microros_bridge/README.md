# ESP32 micro-ROS Bridge

ESP32-S3 DevKitC-1 作为 micro-ROS 客户端，通过 UART 接收 STM32 的 IMU/状态数据，再通过 WiFi UDP 连接 micro-ROS Agent，把数据发布到 ROS 2 Jazzy。

## 当前实现

- 启动后向 STM32 发送 `G`，请求进入 Gazebo/ROS 输出模式
- 启动前先 ping Agent，避免在 `rclc_support_init` 阶段卡死
- 发布 ROS 2 话题：
  - `/imu/data`
  - `/imu/filtered`
  - `/robot/state`
- `/imu/filtered` 对线加速度和角速度做一阶低通滤波
- 三个发布器都使用 `best_effort` QoS

## 代码结构

- `src/main.cpp`：WiFi 连接、看门狗、任务启动、调试输出
- `src/stm32_serial_parser.cpp`：STM32 串口协议解析
- `src/uros_pub.cpp`：micro-ROS 初始化与话题发布
- `include/wifi_config.example.h`：本地 WiFi 配置模板

## 支持的 STM32 串口帧

- `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
- `IMU,ax,ay,az,gx,gy,gz,temp`
- `State:0`

说明：

- `IMUQ` 中的四元数会写入 `/imu/data.orientation`
- `/imu/filtered` 保留同一姿态，只滤波角速度和线加速度
- 训练 CSV 默认不会被当作 IMU 帧发布；如需临时兼容，可在 `src/main.cpp` 中把 `ACCEPT_TRAIN_CSV_AS_IMU` 改成 `true`

## 本地 WiFi 配置

真实 SSID 和密码不要提交到 Git。第一次构建前复制示例文件：

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

如果 Agent IP 改了，也同步修改 `platformio.ini` 中的传输配置。

## 依赖

- PlatformIO
- `micro_ros_arduino` `jazzy`
- ESP32 Arduino framework
- ROS 2 Jazzy
- micro-ROS Agent `udp4`

## 构建与上传

```bash
python3 -m platformio pkg install
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

## 推荐启动顺序

### 1. 启动 Agent

```bash
cd /home/ina/microros_ws
source /opt/ros/jazzy/setup.bash
./build/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v 6
```

### 2. 上传 ESP32 固件

```bash
cd firmware/esp32_microros_bridge
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
```

### 3. 查看启动日志

```bash
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

期望日志：

```text
ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
Requested STM32 GAZEBO mode
Setting micro-ROS WiFi transports...
Pinging micro-ROS Agent over WiFi UDP...
micro-ROS Agent reachable
micro-ROS connected!
```

## 常见检查

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo --once /imu/data
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
```

## 故障排除

- 如果串口里出现 `Setting micro-ROS transports...` 而不是 `Setting micro-ROS WiFi transports...`，说明板子还在运行旧固件
- 如果只看到 `/parameter_events` 和 `/rosout`，先确认 Agent 已启动，再检查 ESP32 是否已经打印 `micro-ROS Agent reachable`
- 如果 `/robot/state` 有数据但 `/imu/data` 没数据，检查 STM32 是否已经进入 `G` 模式，并确认串口帧是 `IMUQ,...` 或 `IMU,...`
- 如果 `/dev/ttyACM0` 消失，先关闭串口监视器，再重新插拔开发板或进入 BOOT 模式重传

更多排障记录见 `Debug.md`。
