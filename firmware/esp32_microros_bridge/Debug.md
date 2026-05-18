# Debug Log - ESP32/STM32 micro-ROS Bridge Project

## 项目概述

- ESP32-S3 DevKitC-1 作为 micro-ROS 客户端，通过 WiFi UDP 连接到 micro-ROS Agent
- STM32 作为传感器节点，通过 UART 发送 IMU 和状态数据到 ESP32
- ESP32 接收数据并发布到 ROS2 话题

## 当前状态

### micro-ROS 桥已跑通 (2026-04-11)

**验证结果**:

- ESP32 通过 WiFi 连接到 `192.168.1.8:8888` 的 micro-ROS Agent
- ROS2 中已经出现 `/imu/data`、`/imu/filtered`、`/robot/state`
- `/imu/data` 已输出 IMU 数据，`linear_acceleration.z` 约为 `9.93`
- `/robot/state` 已确认有数据

**ROS2 话题列表**:

```text
/imu/data
/imu/filtered
/parameter_events
/robot/state
/rosout
```

**关键确认点**:

- 新固件启动日志必须包含 `ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...`
- 当前版本会打印 `Connecting to WiFi: ...` 和 `micro-ROS WiFi transport configured`
- 如果看到 `Setting micro-ROS transports...`，说明板子仍在运行旧固件或错误固件

### 当前默认基线：简化连接模型 + 电机链路核对完成 (2026-04-27)

**本轮完成**:

- ESP32 WiFi / Agent / 调试参数已集中到 [app_config.h](src/config/app_config.h)
- ESP32 已切回简化连接模型，不再在运行态周期性 `ping` Agent
- ESP32 当前只保留 WiFi 掉线后的本地销毁 / 重连，以及 `createMicroRosEntities()` 的初始化重试
- STM32 高频 `DBG` 输出已收口到 [app_debug.h](../stm32_sensor_node/User/App/app_debug.h)
- 主机侧已从 `/home/ina/microros_ws/install` 启动 `micro_ros_agent udp4 --port 8888`
- ESP32 已重新烧录到 `/dev/ttyACM0`

**当前默认行为**:

- 高频调试日志默认关闭
- WiFi 掉线后会自动重连
- 不再运行 Agent watchdog / 周期性 ping

**当前实机现象**:

- Agent 端已看到 `stm32_bridge` client session 建立
- ESP32 串口已看到：
  ```text
  Initializing rclc support...
  rclc support initialized
  ROS node created
  System running normally imu=...
  ```
- `ros2 topic info -v /cmd_vel` 已确认订阅者 `stm32_bridge` 在线
- 实测执行：
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}, angular: {z: 0.00}}"
  ```
  后，ESP32 串口已打印：
  ```text
  [CMDVEL] vx=0.200 wz=0.000
  ```

**当前判断**:

- `ROS 2 -> ESP32 subscriber -> UART CMDVEL` 这段链路已实机打通
- 结合 STM32 当前代码，`CMDVEL -> g_motor_* -> MotorTask -> TB6612 A 路` 软件路径没有明显逻辑问题
- 现场已经观察到电机物理转动，说明单电机 A 路执行层也已跑通
- 当前剩余缺口主要是执行反馈还没有形成 `/motor/state`

### 上下行链路联合验证完成 (2026-04-28)

**新增确认**:

- 电机物理转动已现场确认
- ROS 2 上行链路当前正常：
  - `/imu/data` 约 `47.5 Hz`
  - `/imu/filtered` 约 `47.6 Hz`
  - `/robot/state` 约 `9.5 Hz`
- `/imu/data` 与 `/imu/filtered` 的消息内容连续、数值正常
- `/robot/state` 当前稳定输出 `0`

**当前结论**:

- `ROS 2 -> ESP32 -> STM32 -> TB6612` 已经打通
- `STM32 -> ESP32 -> ROS 2` 已经打通
- 当前系统已经具备稳定的最小闭环联调能力

### `/imu/data` 无数据，`/robot/state` 正常 (2026-04-18)

**现象**:

- `ros2 topic list` 中有 `/imu/data`、`/imu/filtered`、`/robot/state`
- `/robot/state` 可以 echo 到 `data: 0`
- `/imu/data` echo 没有返回

**根本原因**:

- STM32 当前发送的 IMU 行是裸 CSV，例如 `1405324,0.8398,-0.2560,9.8909,-1.2595,-2.0305,-4.2672,0`
- ESP32 旧 parser 只识别 `IMU,ax,ay,az,gx,gy,gz,temp`
- 因此 micro-ROS bridge 已连接，但裸 CSV 的 IMU 行被忽略，只有 `State:0` 行被发布到 ROS2

**解决方案**:

- ESP32 parser 已改为同时兼容:
  ```text
  IMU,ax,ay,az,gx,gy,gz,temp
  timestamp_ms,ax,ay,az,gx,gy,gz,state
  ```
- 已重新编译并上传到 `/dev/ttyACM0`

**验证**:

- `ros2 topic echo --once /imu/data` 已返回 IMU 消息
- `ros2 topic hz /imu/data` 约 `8 Hz`

### ESP32 `/imu/filtered` 一阶低通滤波 (2026-04-18)

**实现位置**:

- `src/main.cpp`
- `/imu/data` 继续发布原始解析后的 IMU 数据
- `/imu/filtered` 对角速度和线加速度发布一阶低通滤波结果

**滤波参数**:

```text
alpha = 0.2
filtered = alpha * raw + (1 - alpha) * previous_filtered
```

**当前限制**:

- 当前主链路 `IMUQ` 的 orientation 已来自 STM32 侧姿态解算，不再是单位四元数占位
- 旧 `IMU` 兼容格式仍会使用单位四元数占位
- covariance 仍未根据实测噪声填写

**验证**:

- ESP32 固件已重新编译并上传到 `/dev/ttyACM0`
- `ros2 topic echo --once /imu/filtered` 已返回 IMU 消息

## 问题日志

### 1. ACM0 串口无数据 (2026-04-10)

**问题描述**: PlatformIO device monitor 显示无数据输出。

**根本原因**: ESP32-S3 USB CDC 未启用，默认使用 USB-Serial/JTAG 模式。

**解决方案**:

- 在 `platformio.ini` 中添加 `board_build.cdc_on_boot = yes`
- 确保 build_flags 中有 `-D ARDUINO_USB_CDC_ON_BOOT=1`
- 重新编译上传 ESP32 固件
- 重启 ESP32，断开重连 USB

**验证**: ESP32 启动日志正常显示。

### 2. micro-ROS 初始化失败 (2026-04-10)

**问题描述**: ESP32 WiFi 连接成功，但卡在 `Initializing rclc support...`，ROS 话题列表为空。

**根本原因**:

- micro-ROS Agent 未运行
- 版本不匹配，ESP32 与 Agent 需要同为 Jazzy

**解决方案**:

- 启动 micro-ROS Agent:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /home/ina/microros_ws/install/setup.bash
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  ```
- 更新 ESP32 库到 Jazzy: `lib_deps = https://github.com/micro-ROS/micro_ros_arduino.git#jazzy`
- 重新编译上传 ESP32

**验证**: Agent 显示连接日志，ROS 话题 `/imu/data` 和 `/robot/state` 出现。

### 3. 串口出现 `~...XRCE...` 乱码，ROS2 没有业务话题 (2026-04-11)

**问题描述**:

- ESP32 串口打印到 `Initializing rclc support...` 后出现 `~...XRCE...` 二进制乱码
- `ros2 topic list` 只有 `/parameter_events` 和 `/rosout`
- 日志中出现 `Setting micro-ROS transports...`

**根本原因**:

- 当前烧录到 ESP32 的固件不是本工程最新 WiFi-only 固件
- 旧固件或默认 transport 会把 micro-ROS XRCE-DDS 数据包写到 USB Serial，导致串口出现 `XRCE` 乱码
- 这时 ROS2 Agent 收不到来自 WiFi UDP 的 micro-ROS 节点创建请求，所以业务话题不会出现

**解决方案**:

- 重新编译并上传当前工程固件:
  ```bash
  cd firmware/esp32_microros_bridge
  ./.venv/bin/python3 -m platformio run --target upload --upload-port /dev/ttyACM0
  ```
- 如果 `/dev/ttyACM0` 消失，关闭串口监视器，重新插拔 ESP32
- 必要时按住 `BOOT`，点按 `RESET/EN`，松开 `BOOT` 后再上传
- 上传后重新打开监视器，确认启动日志包含:
  ```text
  ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
  micro-ROS WiFi transport configured
  Connecting to micro-ROS Agent at 192.168.1.8:8888
  ```

**最终验证**:

- `ros2 topic list` 出现 `/imu/data`、`/imu/filtered`、`/robot/state`
- `ros2 topic echo /imu/data` 有 IMU 数据
- `ros2 topic echo /robot/state` 有状态数据

### 4. STM32 不发送 State 数据 (2026-04-10)

**问题描述**: ESP32 只接收 IMU 数据，没有 State 数据。

**根本原因**: STM32 代码只在 printf 中输出 State，未通过 UART 发送。

**解决方案**:

- 修改 `algo_task.c`，添加 UART 发送 State 数据:
  ```c
  char state_buffer[32];
  int state_len = snprintf(state_buffer, sizeof(state_buffer), "State:%d\n", features.anomaly_state);
  HAL_UART_Transmit(&huart1, (uint8_t*)state_buffer, state_len, HAL_MAX_DELAY);
  ```
- 重新编译上传 STM32 固件

**验证**: ESP32 接收到 `State:X` 格式数据，ROS2 `/robot/state` 有数据。

### 5. USB 连接不稳定 (2026-04-10)

**问题描述**: ESP32 经常断开连接，monitor 显示 `device disconnected`。

**可能原因**:

- USB 线缆接触不良
- 供电不足
- USB 端口问题

**解决方案**:

- 使用高质量 USB 数据线
- 尝试不同 USB 端口
- 检查 ESP32 供电，必要时考虑外部电源

**状态**: 链路已跑通，但 USB 线缆/供电仍建议保持稳定。

### 6. ACM0 进入下载模式或上传失败 (2026-04-11)

**问题描述**:

- 串口输出 `boot:0x23 (DOWNLOAD(USB/UART0))`
- 显示 `waiting for download`
- 或上传时报 `/dev/ttyACM0` 不存在

**原因分析**:

- ESP32-S3 复位时进入下载模式，不是 `main.cpp` 崩溃
- 串口监视器的 DTR/RTS 控制线可能触发自动复位/下载
- USB 断开后设备节点会消失，需要重新枚举

**解决方案**:

- 在 `platformio.ini` 中固定:
  ```ini
  monitor_dtr = 0
  monitor_rts = 0
  ```
- 上传前关闭所有串口监视器
- 必要时手动进入下载模式: 按住 `BOOT`，点按 `RESET/EN`，松开 `BOOT`
- 上传成功后按 `RESET/EN` 运行应用固件

### 7. `/cmd_vel` 在 ROS 2 中正常，但 ESP32 不执行回调，串口出现 `WiFiUdp endPacket(): could not send data: 12` (2026-04-27)

**现象**:

- `rqt_robot_steering` 已确认持续发布 `/cmd_vel`
- `ros2 topic info -v /cmd_vel` 显示 `stm32_bridge` 订阅者在线
- `/imu/data`、`/imu/filtered`、`/robot/state` 仍然存在，说明 micro-ROS 会话没有完全消失
- ESP32 串口持续打印 `[IMU IMUQ] ...`，说明 `STM32 -> ESP32` 上行串口仍在工作
- 但发 `/cmd_vel` 时看不到 ESP32 侧应有的 `[CMDVEL] vx=...`，也看不到 STM32 回执 `[STM32] CMDVEL_RX,...`
- 串口反复出现:
  ```text
  WiFiUdp endPacket(): could not send data: 12
  ```

**证据链**:

- ESP32 Arduino `WiFiUDP::endPacket()` 内部使用 `sendto(...)`；如果发送失败就打印 `could not send data: %d`
- 在 ESP32S3 的 lwIP 头文件里，错误码 `12` 对应 `ENOMEM`，即 UDP 发送侧资源不足 / 内存不足
- STM32 `SensorTask` 每 `10 ms` 发送一帧 `IMUQ`，见 `robot-state-monitor/User/App/sensor_task.c`
- STM32 `AlgTask` 每 10 个样本输出一次 `State:<n>`，约 `10 Hz`，见 `robot-state-monitor/User/App/algo_task.c`
- ESP32 `handleParsedImuSample()` 对每一帧 IMU 都立即发布 `/imu/data` 和 `/imu/filtered` 两个 topic，见 `src/main.cpp` 与 `src/uros_pub.cpp`
- 因此当前理论上行发布负载约为:
  ```text
  100 Hz /imu/data
  100 Hz /imu/filtered
   10 Hz /robot/state
  -------------------
  约 210 次 publish / 秒
  ```
- 现场实测 `/imu/data` 在 ROS 2 侧只有约 `8 ~ 25 Hz`，远低于 STM32 设计的 `100 Hz`，说明上行已经存在明显丢包或拥塞
- `micro_ros_arduino` 自带 `rcl/publisher.h` 明确写了 `rcl_publish()` 是 `potentially blocking call`
- 当前项目里:
  - `src/uros_core.cpp` 只在初始化成功时把 `connected = true`，之后没有掉线探测，也没有自动重连
  - `src/uros_sub.cpp` 对 `rclc_executor_spin_some()` 的返回值做了 soft check，但没有打印错误
  - `src/uros_pub.cpp` 对 `rcl_publish()` 的返回值也直接忽略，没有暴露发送失败
- `micro_ros_arduino` 官方 `micro-ros_reconnection_example` 使用了:
  - `WAITING_AGENT`
  - `AGENT_AVAILABLE`
  - `AGENT_CONNECTED`
  - `AGENT_DISCONNECTED`
  并通过周期性 `rmw_uros_ping_agent()` 销毁 / 重建实体；当前工程还没有这套状态机

**当前判断**:

- 当前更像是 ESP32 micro-ROS 运行时健壮性问题，而不是 TB6612 或电机接线问题
- 上行发布负载偏高，再叠加 WiFi UDP `ENOMEM`，会让会话进入一种“ROS 图里节点和订阅者还在，但实际回调越来越不及时甚至不再执行”的半失效状态
- 这可以解释为什么 `/imu/data` 偶尔还有数据，但 `/cmd_vel` 回调和 STM32 的 `CMDVEL_RX` 回执都没有出现

**优先修复顺序**:

1. 按官方 reconnection example 给 ESP32 加上 `ping -> create_entities -> connected -> destroy_entities -> reconnect` 状态机
2. 先把上行负载降下来再复测:
   - 暂时只保留 `/imu/data`
   - 或把 `/imu/filtered` 改成低频发布
   - 或把 STM32 的 `IMUQ` 输出频率从 `100 Hz` 降到 `20 ~ 50 Hz`
3. 给 `rcl_publish()` 和 `rclc_executor_spin_some()` 补上错误日志，不再静默吞掉失败
4. 保留 STM32 侧 `DBG:CMDVEL_RX,...` 回执，继续作为端到端判断依据

**当前已落地的缓解动作**:

- STM32 已先改为仅在 `GAZEBO` 模式下每 2 帧发送 1 帧 `IMUQ`，即外发约 `50 Hz`
- 板上姿态解算与 `SensorTask` 主循环仍保持原有 `100 Hz` 左右节奏，变化的是 UART 上行外发频率，不是本地解算频率

**最快确认办法**:

- 临时停掉 `/imu/filtered`，或只发布每 4 帧中的 1 帧 IMU
- 如果此后 `[CMDVEL] vx=...` 恢复，且 `WiFiUdp endPacket(): could not send data: 12` 明显减少，就基本可以确认是发送侧拥塞 / 资源耗尽
- 如果降载后仍没有 `[CMDVEL]`，下一步就优先实现 Agent 掉线探测与实体重建

### 8. STM32 实际收到了 `/cmd_vel` 文本帧，但 `CMDVEL` 一直不生效 (2026-04-27)

**现象**:

- ESP32 串口已经稳定打印 `[CMDVEL] vx=... wz=...`
- 但早期 STM32 一直没有返回 `DBG:CMDVEL_RX,...`
- 电机不动，`STAT` 里长期是:
  ```text
  estop=1,vx=0.000,wz=0.000
  ```
- 同时新增的 STM32 统计又显示:
  ```text
  rxb=..., rxl=..., rxerr=0, drop=0
  ```
  说明 STM32 实际已经收到了大量完整文本行，不是“压根没收到”

**根本原因**:

- `sensor_task.c` 里最初使用:
  ```c
  sscanf(line, "CMDVEL,%f,%f", &linear_x, &angular_z)
  ```
- 工程链接选项里启用了 `-u _printf_float`，但没有启用 `_scanf_float`
- 在这种配置下，`snprintf(..., "%.3f")` 能正常工作，所以上行 `IMUQ` 没问题
- 但 `sscanf(..., "%f")` 在 STM32 裸机/`newlib-nano` 场景里不可靠，结果就是:
  - 串口行收到了
  - `CMDVEL` 浮点解析失败
  - `g_motor_estop` 不会被清掉
  - `vx/wz` 保持为 0
  - 电机当然不会动

**修复**:

- 没有改波特率
- 没有改 ROS 2 话题格式
- 直接把 `CMDVEL` 解析从 `sscanf("%f,%f")` 改成了 `strtof` 手动分段解析
- 同时保留了 UART 运行期诊断:
  - `DBG:STAT,...`
  - `DBG:CMDVEL_RX,...`
  - `HAL_UART_ErrorCallback()` 错误计数

**修复后验证**:

- STM32 已开始稳定返回:
  ```text
  [STM32] CMDVEL_RX,0.111,0.258,...
  ```
- `DBG:STAT` 已变成:
  ```text
  estop=0,vx=0.111,wz=0.258
  ```
- 这说明以下链路已经被实机证实打通:
  ```text
  rqt_robot_steering
    -> /cmd_vel
    -> ESP32 subscriber
    -> UART 文本帧
    -> STM32 行接收
    -> STM32 CMDVEL 解析
  ```

**当前剩余定位**:

- 如果电机此时仍不动，问题就已经不在 ROS、ESP32、UART 文本解析
- 剩余范围应收敛到:
  - `MotorTask` 实际 PWM/方向输出
  - TB6612 `STBY/AIN1/AIN2/PWMA`
  - 电机供电或接线

## 配置总结

### ESP32 (platformio.ini)

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 921600
upload_speed = 921600

monitor_port = /dev/ttyACM0
monitor_dtr = 0
monitor_rts = 0

board_build.cdc_on_boot = yes

lib_deps =
    https://github.com/micro-ROS/micro_ros_arduino.git#jazzy

build_flags =
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D ARDUINO_ARCH_ESP32
    -D ESP32
    -D RMW_UXRCE_TRANSPORT_UDP
    -D RMW_UXRCE_TRANSPORT_IPV4
    -D CONFIG_MICRO_ROS_TRANSPORT_UDP_NAME=\"192.168.1.8\"
    -D CONFIG_MICRO_ROS_TRANSPORT_UDP_PORT=8888
    -L.pio/libdeps/esp32-s3-devkitc-1/micro_ros_arduino/src/esp32
    -lmicroros
    -D CONFIG_ESP_TASK_WDT_TIMEOUT_S=60
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_HAS_PSRAM
```

### WiFi 配置 (main.cpp)

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AGENT_IP = "192.168.1.8";
const uint16_t AGENT_PORT = 8888;
```

### UART 配置

- ESP32 RX: GPIO16, TX: GPIO17, Baud: 921600
- STM32 UART1: 连接到 ESP32 UART

## 运行步骤

1. 启动 micro-ROS Agent:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/ina/microros_ws/install/setup.bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 6
   ```

2. 上传 ESP32 固件:
   ```bash
   cd firmware/esp32_microros_bridge
   ./.venv/bin/python3 -m platformio run --target upload --upload-port /dev/ttyACM0
   ```

3. 监控 ESP32:
   ```bash
   ./.venv/bin/python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
   ```

   正常启动日志应包含:
   ```text
   ESP32-S3 micro-ROS Bridge v1.1 WiFi-only - Starting...
   micro-ROS WiFi transport configured
   Connecting to micro-ROS Agent at 192.168.1.8:8888
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

## 待解决

- 根据实际 IMU 噪声填写 covariance
- 如果后续还需要兼容旧 `IMU` 上行格式，可考虑补充更清晰的“无姿态解算”标记或单独 topic，避免与 `IMUQ` 主链路混淆
- 如果要把“电机物理已转动”也纳入证据链，建议下一轮临时打开 STM32 的 `APP_DEBUG_CMDVEL_RX` 与 `APP_DEBUG_RUNTIME_STATUS`
- USB 线缆/供电仍建议保持稳定，避免 `/dev/ttyACM0` 枚举中断

## 版本信息

- ESP32: micro-ROS Arduino jazzy
- Agent: micro-ROS Agent jazzy
- ROS2: Jazzy Jalisco
- PlatformIO: espressif32
- 固件标识: `ESP32-S3 micro-ROS Bridge v1.1 WiFi-only`
