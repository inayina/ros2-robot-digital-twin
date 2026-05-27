# Troubleshooting Log

本文件记录本仓库常见联调问题和排查顺序，重点覆盖串口、QoS、micro-ROS Agent、I2C、供电和电机 bench。它是排障入口，不替代实际日志。

## Quick Triage

先按下面顺序缩小问题范围：

1. STM32 是否在运行：`DEBUG_LED` / PC13 是否闪烁。
2. MPU6050 是否启动成功：串口是否有 `MPU6050 OK` 或 `WHO_AM_I` 正常日志。
3. STM32 -> ESP32 UART 是否有 `IMUQ,...`、`State:<n>`。
4. ESP32 是否连上 WiFi，并打印 `micro-ROS connected!`。
5. ROS 2 主机是否启动 micro-ROS Agent UDP `8888`。
6. ROS 2 topic 是否存在：`/imu/data`、`/imu/filtered`、`/robot/state`。
7. dashboard MQTT bridge 是否在运行：`robot/imu`、`robot/motor/status`、`robot/motor/cmd`。
8. 电机 bench 前先确认 `hardware_outputs_enabled`、`max_pwm`、`target_rpm`、`encoder_count`、`invalid_transitions`。

## Serial / UART

### Symptom: ROS 2 没有 IMU topic

可能原因：

- STM32 没有进入 Gazebo / ROS 输出模式。
- ESP32 没有发送或 STM32 没有收到 `G\n`。
- UART RX/TX 接反。
- 波特率不是 `921600`。
- STM32 或 ESP32 没有共地。
- 串口日志被高频 debug 淹没或解析器无法识别行。

检查：

```bash
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

期望看到：

```text
Requested STM32 GAZEBO mode
micro-ROS connected!
```

STM32 上行应包含：

```text
IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp
State:<n>
```

处理建议：

- 先确认交叉接线：STM32 TX -> ESP32 RX，STM32 RX <- ESP32 TX。
- 只保留必要 debug，避免 UART 行被打碎。
- 确认 `G\n` 后 STM32 输出 `IMUQ`，而不是训练 CSV。

### Symptom: `/cmd_vel` 发了但 STM32 没有动作

可能原因：

- `/cmd_vel` QoS 或 executor 没有收到。
- ESP32 没有转发 `CMDVEL,<linear_x>,<angular_z>\n`。
- STM32 UART parser 没有收到完整行。
- STM32 MotorTask 超时急停。
- TB6612 legacy A 通道没有接好或 `STBY` 未使能。

处理建议：

- 先看 ESP32 串口是否有 `[CMDVEL]` 或类似转发日志。
- 再看 STM32 是否打开 `DBG:CMDVEL_RX` 类调试。
- 不要把 `/cmd_vel` 当作 dashboard 主电机控制链路；dashboard 主链路是 `/motor/cmd`。

## QoS

### Symptom: `ros2 topic echo /imu/data` 看不到数据

IMU topic 当前按传感器流使用 `best_effort`。部分命令或节点默认 reliable 时可能不匹配。

检查：

```bash
ros2 topic info -v /imu/data
ros2 topic info -v /imu/filtered
ros2 topic info -v /robot/state
```

建议：

- 对 IMU / state 使用 sensor-data 风格或 best-effort 订阅。
- `/cmd_vel`、`/motor/target_rpm`、`/motor/cmd` 在 ESP32 侧是 reliable subscriber，命令链路不要随意改 QoS。
- 修改 QoS 前先确认是谁发布、谁订阅、类型是否一致。

## micro-ROS Agent

### Symptom: ESP32 WiFi 已连接但没有 ROS 2 topic

可能原因：

- Agent 没启动。
- Agent 端口不是 `8888`。
- ESP32 `include/wifi_config.h` 里的 Agent IP 不对。
- 主机防火墙或网络隔离阻断 UDP。
- ESP32 和主机不在同一网络。

启动 Agent：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/start_microros_agent_udp.sh
```

或手动：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

处理建议：

- 先启动 Agent，再复位 ESP32。
- 确认 ESP32 串口打印的 Agent IP / port。
- 用 `./scripts/check_microros_topics.sh` 检查基本 topic。

## I2C / MPU6050

### Symptom: STM32 没有 IMU 数据

可能原因：

- MPU6050 供电或 GND 问题。
- `PB6/PB7` I2C 接线错误。
- SCL / SDA 没有有效上拉。
- MPU6050 `AD0` 漂浮导致地址变化。
- `SensorTask` 栈太小，姿态解算或浮点格式化导致异常。

处理建议：

- 确认启动日志中的 I2C scan / `WHO_AM_I`。
- 固定 `AD0`，优先使用 `0x68`。
- 确认 CubeMX 里 `SensorTask` 栈不是旧的 `256 words`。
- 如果重新生成 STM32 代码，检查 `SensorTask` 栈和任务配置是否被覆盖。

## Power / Ground

### Symptom: 电机一动，ESP32 重启或 IMU 数据异常

可能原因：

- 电机电源从 ESP32 逻辑电源取电。
- TB6612 `VM` 供电不足或无电流限制。
- ESP32、STM32、TB6612、电机电源没有共地。
- 电机噪声导致 UART / I2C / 编码器输入异常。

处理建议：

- `VM` 使用独立电机电源，调试阶段加限流。
- ESP32 / STM32 / TB6612 / 电机电源负极共地。
- 先低占空比开环验证，再进入闭环。
- 编码器线和电机线尽量分开布线，必要时加滤波和更稳的接地。

## Motor Bench

### Symptom: PWM 增大但 `actual_rpm` 没有变好

优先判断：

- 电机没有真实转动。
- TB6612 `STBY`、方向脚或 PWM 脚接错。
- `max_pwm` 被命令侧或固件侧钳住。
- 编码器 A/B 没有计数或方向反了。
- 供电不足、堵转或机械阻力过大。

处理顺序：

1. 回到 open-loop PWM sweep。
2. 看 `encoder_count` 和 `encoder_delta` 是否随方向变化。
3. 看 `invalid_transitions` 是否持续增加。
4. 确认 `target_rpm = 0` 时 `pwm = 0`。
5. 再决定是否调整 P / I。

不要直接加 `Ki`，也不要直接把 `max_pwm` 放到 `1.0`。

### Symptom: `actual_rpm` 符号反了

处理建议：

- 先确认电机 forward 定义。
- 再确认编码器 A/B 顺序。
- 必要时调整编码器方向配置或交换电机线。
- 不要用 PID 参数掩盖方向错误。

### Symptom: `target_rpm = 0` 但仍有 PWM

这是安全问题，必须先修。

期望行为：

- PWM 立即回到 `0`。
- 方向输出进入 coast 或明确安全态。
- 积分清理或冻结。
- 状态里能看出 stop / timeout / enabled / fault。

## Dashboard Bridge

### Symptom: ROS 2 有 `/motor/status`，Dashboard 没有电机数据

可能原因：

- `robot_mqtt_bridge motor_status_bridge` 没启动。
- MQTT broker 地址或端口不对。
- dashboard backend 没订阅 `robot/motor/status`。
- payload 字段和 backend 容错解析不一致。

检查 bridge：

```bash
ros2 run robot_mqtt_bridge motor_status_bridge --ros-args \
  -p motor_status_topic:=/motor/status \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p mqtt_topic:=robot/motor/status
```

### Symptom: Dashboard 下发命令但 ESP32 没反应

检查链路：

```text
frontend
  -> backend POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> robot_mqtt_bridge motor_cmd_bridge
  -> ROS 2 /motor/cmd
  -> ESP32 subscriber
```

处理建议：

- 先确认 MQTT `robot/motor/cmd` 有消息。
- 再确认 ROS 2 `/motor/cmd` 有消息。
- 再看 ESP32 `/motor/status` 里的 `enabled`、`stop`、`timeout`、`hardware_outputs_enabled`。
- Dashboard frontend 不应绕过 backend 直接连 ROS 2 或 ESP32。
