# robot_mqtt_bridge

`robot_mqtt_bridge` 是当前主链路里的 ROS 2 <-> MQTT bridge 包。

当前收口两条电机链路：

- 上行状态：
  - 订阅 ROS 2 `/motor/status`
  - 解析 ESP32 发布的 JSON 字符串
  - 归一化为 dashboard 兼容 payload
  - 发布到 MQTT `robot/motor/status`
- 下行命令：
  - 订阅 MQTT `robot/motor/cmd`
  - 归一化 `target_rpm / enabled / closed_loop / max_pwm / timeout_ms / stop`
  - 发布到 ROS 2 `/motor/cmd`

它不做 dashboard API 聚合，不写 `latest_robot_status.json`，也不承担旧
`robot_status_api_bridge` 的低频快照职责。

## 构建

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select robot_mqtt_bridge \
  --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 run robot_mqtt_bridge motor_status_bridge --ros-args \
  -p motor_status_topic:=/motor/status \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p mqtt_topic:=robot/motor/status \
  -p robot_id:=amr-001

ros2 run robot_mqtt_bridge motor_cmd_bridge --ros-args \
  -p ros_cmd_topic:=/motor/cmd \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p mqtt_topic:=robot/motor/cmd \
  -p robot_id:=amr-001
```

默认输入输出：

- ROS 2 topic：`/motor/status`
- MQTT topic：`robot/motor/status`
- ROS 2 topic：`/motor/cmd`
- MQTT topic：`robot/motor/cmd`

如果要和 dashboard 一起整链启动，优先使用仓库根目录脚本：

```bash
./scripts/start_motor_dashboard_stack.sh
./scripts/check_motor_dashboard_loop.sh
```

## MQTT Payload

bridge 会保留 ESP32 原始字段，并补充 dashboard 常用字段：

- `actual_rpm`
- `measured_rpm`
- `target_rpm`
- `error_rpm`
- `pwm`
- `pwm_duty`
- `enabled`
- `control_enabled`
- `closed_loop`
- `fault`
- `timeout`
- `estop`
- `motor_state`
- `last_update_time`

其中 `motor_state` 是展开后的结构化对象，便于 `robot-ops-dashboard`
继续兼容现有前端展示逻辑。

## Motor Command Payload

bridge 会把 dashboard backend 发出的 `robot/motor/cmd` 归一化后再转成 ROS 2
字符串，主要字段包括：

- `target_rpm`
- `enabled`
- `closed_loop`
- `max_pwm`
- `timeout_ms`
- `stop`
