# ESP32 micro-ROS Bridge

ESP32-S3 DevKitC-1 作为 micro-ROS 客户端，通过 WiFi UDP 连接到 micro-ROS Agent，实现 STM32 数据的 ROS2 发布。

当前状态: 已验证可用。ROS2 中可以看到 `/imu/data`、`/imu/filtered`、`/robot/state`，并且 `/imu/data`、`/robot/state` 均已有数据输出。

## 功能

- 通过 UART 接收 STM32 发送的 IMU 和状态数据
- WiFi UDP 连接到 micro-ROS Agent，默认示例地址为 `192.168.1.8:8888`
- 发布 ROS2 话题:
  - `/imu/data` (sensor_msgs/Imu): 原始 IMU 数据，兼容 `IMUQ` 四元数姿态
  - `/imu/filtered` (sensor_msgs/Imu): 一阶低通滤波后的 IMU 数据，用于 Gazebo/分析
  - `/robot/state` (std_msgs/Int32): 机器人状态 (0-3)
- micro-ROS 初始化前会 ping Agent，Agent 不可达时不会继续卡在 `rclc_support_init`
- 启动后向 STM32 发送 `G`，请求 Gazebo/ROS 数据输出模式
- 使用行缓冲解析 STM32 串口，避免 `readStringUntil()` 阻塞主循环

## 硬件连接

- UART: GPIO16 (RX) <- STM32 TX, GPIO17 (TX) -> STM32 RX
- 波特率: 921600
- micro-ROS Agent: 默认示例地址 `192.168.1.8:8888`

## 本地 WiFi 配置

真实 SSID 和密码不要提交到 Git。第一次构建前复制示例配置:

```bash
cp include/wifi_config.example.h include/wifi_config.h
```

然后编辑 `include/wifi_config.h` 中的 `WIFI_SSID`、`WIFI_PASS`、`AGENT_IP` 和 `AGENT_PORT`。
如果 Agent IP 改了，也同步修改 `platformio.ini` 里的 `CONFIG_MICRO_ROS_TRANSPORT_UDP_NAME`。

## 依赖

- PlatformIO
- micro-ROS Arduino (jazzy)
- ESP32 Arduino 框架
- ROS2 Jazzy
- micro-ROS Agent (UDP4)

## 构建与运行

1. 安装依赖:
   ```bash
   python3 -m platformio pkg install
   ```

2. 编译:
   ```bash
   python3 -m platformio run
   ```

3. 上传:
   ```bash
   python3 -m platformio run --target upload --upload-port /dev/ttyACM0
   ```

4. 监控:
   ```bash
   python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
   ```

## 启动顺序

1. 启动 micro-ROS Agent:
   ```bash
   cd /home/ina/microros_ws
   source /opt/ros/jazzy/setup.bash
   ./build/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v 6
   ```

2. 上传 ESP32 固件:
   ```bash
   cd firmware/esp32_microros_bridge
   python3 -m platformio run --target upload --upload-port /dev/ttyACM0
   ```

3. 监控 ESP32 启动日志:
   ```bash
   python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
   ```

   正常新固件应出现:
   ```text
   ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
   Setting micro-ROS WiFi transports...
   Pinging micro-ROS Agent over WiFi UDP...
   micro-ROS Agent reachable
   micro-ROS connected!
   ```

4. 查看 ROS2 话题:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list
   ros2 topic echo /imu/data
   ros2 topic echo /imu/filtered
   ros2 topic echo /robot/state
   ```

## 已验证结果

`ros2 topic list` 应包含:

```text
/imu/data
/imu/filtered
/parameter_events
/robot/state
/rosout
```

`/imu/data` 已验证输出 IMU 数据，例如:

```text
angular_velocity:
  x: -0.0189
  y: -0.0391
  z: -0.0723
linear_acceleration:
  x: -0.282
  y: -0.187
  z: 9.934
```

## 数据格式

STM32 发送格式:

- 首选 IMU: `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp\n`
- 兼容旧 IMU: `IMU,ax,ay,az,gx,gy,gz,temp\n`
- State: `State:0\n` (0=正常, 1=警告, 2=警报, 3=严重)

`IMUQ` 中的四元数会写入 `/imu/data.orientation`，`/imu/filtered` 保留同一姿态并只滤波线加速度和角速度。旧 `IMU` 格式没有姿态，orientation 使用单位四元数占位。

训练 CSV 默认不会被当作 IMU 发布，避免把采集数据误送进 ROS 话题；如需临时兼容，可在 `src/main.cpp` 中打开 `ACCEPT_TRAIN_CSV_AS_IMU`。

## 故障排除

- 如果串口出现 `~...XRCE...` 乱码，并且日志是 `Setting micro-ROS transports...`，说明 ESP32 仍在运行旧固件或 Serial transport 固件。重新上传后必须看到 `v1.1 WiFi-only`。
- 如果只看到 `/parameter_events` 和 `/rosout`，说明 micro-ROS 节点没有成功注册到 ROS2。先确认 Agent 已启动，再看 ESP32 是否打印 `micro-ROS Agent reachable`。
- 如果 `/robot/state` 有数据但 `/imu/data` 没数据，检查 STM32 是否处于 `G` 模式，并确认串口行是 `IMUQ,...` 或 `IMU,...`。
- 如果 `/dev/ttyACM0` 消失，关闭串口监视器后重新插拔 ESP32。必要时按住 `BOOT`，点按 `RESET/EN`，松开 `BOOT` 后重新上传。
- 更多排障记录见 `Debug.md`

## 版本

- micro-ROS: jazzy
- ROS2: Jazzy Jalisco
- ESP32: Arduino framework via PlatformIO espressif32
- 固件标识: `ESP32-S3 micro-ROS Bridge v1.1 WiFi-only`
