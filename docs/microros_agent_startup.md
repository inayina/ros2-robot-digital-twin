# micro-ROS Agent Startup

本仓库不包含 micro-ROS Agent 源码，也不 vendor `micro_ros_setup`。Agent 是 ROS 2 主机侧的外部依赖，建议单独放在外部 workspace 中构建和维护。

## Recommended Workspace

推荐使用以下外部 workspace 之一：

- `~/uros_ws`
- `~/micro_ros_ws`

构建完成后，本仓库脚本会查找：

```bash
~/uros_ws/install/local_setup.bash
~/micro_ros_ws/install/local_setup.bash
```

如果你的路径不同，设置：

```bash
export MICRO_ROS_AGENT_SETUP=/path/to/your/micro_ros_ws/install/local_setup.bash
```

## Start Agent

默认 UDP 端口是 `8888`，需要和 ESP32 本地 WiFi 配置中的 Agent 端口保持一致。

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/start_microros_agent_udp.sh
```

脚本会依次 source：

1. `/opt/ros/jazzy/setup.bash`
2. `MICRO_ROS_AGENT_SETUP` 指向的外部 workspace，或常见路径中的 `local_setup.bash`

然后启动：

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 4
```

## Startup Order

启动顺序固定为：

1. 启动 micro-ROS Agent。
2. 复位或重新上电 ESP32。
3. 等待 ESP32 日志出现 `micro-ROS connected!`。
4. 检查 ROS 2 topic 是否出现。

先启动 Agent 再复位 ESP32，可以避免 ESP32 上电时找不到 Agent 而进入重试路径。

## Check Topics

打开另一个终端运行：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/check_microros_topics.sh
```

脚本会用 `ros2 topic list` 检查这些 topic：

- `/imu/data`
- `/imu/filtered`
- `/robot/state`
- `/motor/status`
- `/motor/actual_rpm`
- `/motor/state`

如果 topic 已出现，脚本会继续用 `ros2 topic hz` 做短时间频率采样。接真实 N20 前，必须先确认 micro-ROS Agent 已连接，且这些 topic 能正常出现。

## Troubleshooting

- Agent 没启动：先运行 `./scripts/start_microros_agent_udp.sh`，看到 UDP Agent 正在监听后再复位 ESP32。
- ESP32 IP / 端口不一致：检查 `firmware/esp32_microros_bridge/include/wifi_config.h` 中的 Agent IP 和端口，端口默认应为 `8888`。
- `ROS_DOMAIN_ID` 不一致：启动 Agent、检查 topic、运行其他 ROS 2 节点的终端应使用相同的 `ROS_DOMAIN_ID`。
- Wi-Fi 不通：确认 ESP32 和运行 Agent 的主机在同一可达网络内，SSID、密码、主机 IP 都正确。
