# N20 Motor Closed-loop Tuning Process

## 1. 当前调试目标

这份文档记录当前仓库里已经落地的单个 N20 编码器电机闭环调参流程，目标不是宣称“已经完成底盘控制”，而是把台架阶段真正可复现的工作方法记下来。

当前关注的是这条单电机速度闭环：

- `target_rpm` 下发
- 编码器反馈 `measured_rpm` / `actual_rpm`
- ESP32 本地 PI 输出 `pwm`
- 观察 `error_rpm` 是否收敛
- 通过 ROS 2、MQTT 和 dashboard 看状态

边界要说清楚：

- 当前是 single N20 bench / 调参阶段
- 当前不是完整双轮差速底盘
- 当前不是 `ros2_control`
- Dashboard 只是低频观察和人工下发入口，不参与实时 PID 闭环

当前仓库里存在两种互补的调试入口：

- 本地 bench mode：`kEnableN20ClosedLoopBench = true`，ESP32 按本地 step profile 运行，并从 USB 串口输出 CSV
- 普通 `/motor/cmd` 路径：dashboard/backend/MQTT/ROS 2 下发命令，ESP32 在当前 bench 接线下用真实 encoder + PI 回传 `/motor/status`

## 2. 当前硬件与链路

当前台架涉及的硬件和软件节点：

- ESP32-S3
- TB6612FNG
- 单个 6V N20 编码器减速电机
- 编码器 A/B 相
- micro-ROS Agent
- ROS 2 topic
- `robot_mqtt_bridge`
- `robot-ops-dashboard`

当前接线和职责边界以 `firmware/esp32_microros_bridge/src/main.cpp`、`firmware/esp32_microros_bridge/src/config/app_config.h`、`firmware/esp32_microros_bridge/docs/motor_control_design.md` 为准。当前主验证通道是：

- TB6612 B 通道驱动 N20
- `GPIO10/GPIO11` 读 N20 编码器 A/B
- `GPIO7/GPIO8/GPIO9/GPIO18` 负责 TB6612 B 通道 PWM/DIR/STBY

当前 dashboard-facing 数据流可以概括为：

```text
Dashboard motor command
  -> backend POST /api/robot/motor/cmd
  -> MQTT robot/motor/cmd
  -> ros2/robot_mqtt_bridge
  -> ROS 2 /motor/cmd
  -> ESP32 ros_comm_task / executor
  -> ESP32 motor control task
  -> TB6612 PWM/DIR
  -> N20 motor
  -> encoder feedback
  -> ESP32 speed estimate
  -> ROS 2 /motor/status
  -> MQTT robot/motor/status
  -> Dashboard motor card / tuning curve
```

补充说明：

- bench mode 下，最适合看的不是 dashboard，而是 USB 串口 CSV
- `/motor/status` 是当前主状态 JSON
- `/motor/actual_rpm` 和 `/motor/state` 仍保留给 bench / 调试兼容链路

## 3. 安全限幅策略

先区分代码里的已固化事实和现场调参时的工作边界。

当前代码里的已固化事实：

- `kN20ClosedLoopBenchMaxPwm = 0.25`
- `kMotorControlDefaultMaxPwm = 0.25`
- `kMotorControlCommandMaxPwm = 0.50`
- 普通启动默认 `kEnableMotorHardwareOutputs = false`
- 普通启动默认 `kEnableN20ClosedLoopBench = false`

当前调参文档采用的工作边界：

- `max_pwm = 0.25`
  初始防暴冲值，用来先确认方向、编码器、反馈正负号和 stop/timeout 是否工作正常
- `max_pwm = 0.35`
  当前建议的人工调参值；当 `0.25` 已经明显不够时，优先先把命令侧 `max_pwm` 提到这一档再看响应，不要先盲目把 PI 参数拧大
- `max_pwm = 0.45`
  只作为短时间人工观察响应裕量的上探边界，不写成默认连续运行值；仓库里当前没有把它固化成代码默认值
- 不建议直接放开到 `1.0`
- 超过 `0.45` 前，至少要先观察电机、TB6612、电源温升，以及是否出现堵转风险

还要注意一个事实：

- 代码侧的 dashboard / `/motor/cmd` 最大命令钳位目前是 `0.50`
- 因此文档里把 `0.45` 写成“短时人工观察上限”，本质上是在总钳位之下保留一点安全余量，而不是宣称它已经被完整实测验证为默认值

判断标准：

如果 `pwm` 长时间贴着 `max_pwm`，但 `actual_rpm` 仍明显低于 `target_rpm`，优先判断：

- 当前 `max_pwm` 不足
- 当前目标速度过高
- 当前机械/电源条件不支持

这时不要继续盲目增加 `Kp` / `Ki`。先解决限幅或目标设定问题，再谈 PI 微调。

## 4. 调参前检查清单

每次开始前至少确认下面这些点：

- TB6612 `STBY`、`BIN1/BIN2`、`PWMB` 接线确认
- 电机正反转方向确认
- 编码器计数方向确认
- `encoder_count` 在低占空比脉冲下能正常变化
- `invalid_transitions` 不持续快速增加
- `target_rpm = 0` 时，`pwm` 必须回到 `0`
- 方向切换时要经过 coast / direction hold，不直接硬切
- micro-ROS Agent 已连接
- `/motor/status` topic 正常刷新
- dashboard 能看到 `robot/motor/status`
- 如果走 `/motor/cmd` 路径，要确认 `hardware_outputs_enabled` 已经被允许

其中和当前代码直接对应的安全点有：

- `target_rpm = 0` 会清积分并把输出拉回 coast
- 方向切换会进入 `direction_change_coast_ms`
- bench 和普通 `/motor/cmd` 路径都复用了同一套 N20 编码器方向 / PI / direction hold 逻辑

## 5. 调参步骤

### Step 1: 开环 / 低占空比方向验证

目标：先确认电机转向和编码器方向一致，不要一上来就进闭环。

当前仓库里已经有一个很适合做这步的入口：

- `kEnableTb6612ChannelBBenchTest = true`

它会执行：

```text
coast -> forward low-duty pulse -> coast -> reverse low-duty pulse -> coast
```

当前默认低占空比脉冲是：

- `kTb6612BenchTestDuty = 0.18`

这一步重点观察：

- `encoder_count` 是否变化
- 正转 / 反转的 `delta` 是否符号相反
- `invalid_transitions` 是否异常增加
- forward 时，编码器方向和软件定义的 forward 是否一致

如果方向不一致，先修方向定义或编码器反向配置，再进入闭环。

### Step 2: `0.25 max_pwm` 闭环初测

目标：确认闭环没有变成正反馈，没有暴冲，stop/timeout 也还正常。

建议先从低一点的 `target_rpm` 开始，不要第一次就直接上较高目标。

这一步重点观察：

- `target_rpm`
- `actual_rpm`
- `error_rpm`
- `pwm`
- `output_saturated` 或 `/motor/status` 里的 `saturated`
- `invalid_transitions`

正常现象应该是：

- 电机能起转，但不会突然冲满
- `actual_rpm` 会往 `target_rpm` 方向走
- `target_rpm = 0` 时，输出能快速回到 `0`

异常现象包括：

- `actual_rpm` 越调越反方向
- 一进闭环就大幅暴冲
- `pwm` 长时间顶在上限但 `actual_rpm` 上不去
- `invalid_transitions` 很快攀升

### Step 3: 提高到 `0.35`

如果 `0.25` 已经明显不够，就不要继续硬调 PI，先把 `max_pwm` 提到大约 `0.35` 再看。

这一步的目的不是“加大火力”，而是先排除限幅过紧导致的假性跟踪差。

重点观察：

- `actual_rpm` 是否开始更接近 `target_rpm`
- `pwm` 是否还长期饱和
- `error_rpm` 是否开始收敛

如果从 `0.25` 提到 `0.35` 后，`pwm` 仍然长期贴边，那主要矛盾通常还不是 PI 太弱，而是：

- 目标速度仍偏高
- 供电或机械条件不足
- `min_effective_pwm`、方向、编码器链路还有别的问题

### Step 4: 短时间试 `0.45`

这一步只用于确认响应裕量，不是无条件默认值。

当前更合理的理解是：

- `0.45` 是在固件总钳位 `0.50` 之下的短时人工试探边界
- 适合短时间观察
- 不适合直接写成“已经稳定可长期运行”

这一步重点观察：

- 温升
- 震荡
- 超调
- 稳态误差
- 编码器异常

如果这一步里看到明显发热、卡滞、异响或 `invalid_transitions` 明显上升，就该回退，而不是继续把 `max_pwm` 往 `0.50` 顶。

### Step 5: PI 参数微调

当前仓库里的 PI 起点参数在 `firmware/esp32_microros_bridge/src/config/app_config.h`：

- `Kp = 0.0018`
- `Ki = 0.0012`
- `Kd = 0.0`

当前微调原则：

- 先调 `Kp`
- 先让 `actual_rpm` 能跟随 `target_rpm`
- 但不要为了“跟得更快”直接把系统调到振荡
- `Ki` 先小一点，或者暂时关闭
- `Kp` 合理后，再慢慢加 `Ki` 去消稳态误差
- 如果 `pwm` 已经饱和，就不要继续盲目加 `Kp` / `Ki`

当前 `speed_pid.cpp` 已经有 anti-windup：

- 输出饱和且误差继续同向时，会回退本周期积分累积

这一步的意义是：

- 避免输出已经顶死，但积分还继续堆积
- 否则一旦脱离饱和区，容易出现更大的超调和恢复慢

## 6. 曲线观察标准

当前 bench CSV 或后续 dashboard 调参曲线，至少值得看这些量：

- `target_rpm` vs `actual_rpm`
- `error_rpm`
- `pwm / max_pwm`
- `output_saturated` 或 `saturated`
- `encoder_delta`
- `invalid_transitions`

观察解释：

- `actual_rpm` 慢慢追上 `target_rpm`
  这是正常闭环收敛
- `pwm` 长期顶到 `max_pwm`
  先看限幅或目标速度，不要立刻怪 PI
- `actual_rpm` 方向反了
  优先查电机方向或编码器方向
- `error_rpm` 不收敛但 `pwm` 没顶满
  PI 参数可能偏弱，或 `min_effective_pwm` / 估速滤波仍待调整
- `error_rpm` 来回震荡
  `Kp` 可能偏大，或者估速滤波 / 采样周期要重新看
- `target_rpm = 0` 但 `pwm` 不为 `0`
  这是安全问题，先修这个

还要区分观测层：

- bench CSV 最适合看 `encoder_delta`、`raw_rpm`、`filtered_rpm`、`integral`
- `/motor/status` 最适合看当前闭环是否在工作、是否超时、是否 stop、是否饱和
- dashboard 当前更适合作为联调观察页，不是高频控制器

## 7. 当前 CSV / telemetry 字段

当前字段要按“bench CSV”和“dashboard 状态链路”分开理解。

| 字段 | bench CSV | `/motor/status` 固件 JSON | `robot/motor/status` MQTT / dashboard | 说明 |
| --- | --- | --- | --- | --- |
| `timestamp_ms` | yes | yes | yes | ESP32 单调毫秒时间，适合作为 dashboard 曲线 x 轴 |
| `target_rpm` | yes | yes | yes | 当前目标转速 |
| `actual_rpm` | yes | yes | yes | 当前闭环实际使用的是滤波后的速度估计 |
| `measured_rpm` | no | yes | yes | 当前固件里与 `actual_rpm` 同值，给 dashboard 兼容消费 |
| `error_rpm` | yes | yes | yes | `target_rpm - actual_rpm` |
| `pwm` | yes | yes | yes | 输出 duty；固件 JSON 里 `pwm` 与 `pwm_duty` 同值 |
| `direction` | yes | yes | yes | 当前软件定义方向，`1/-1/0` |
| `encoder_delta` | yes | no | no | bench CSV available，dashboard payload TODO |
| `raw_rpm` | yes | no | no | bench CSV available，dashboard payload TODO |
| `filtered_rpm` | yes | no | no | bench CSV available；dashboard 当前直接看 `actual_rpm` |
| `integral` | yes | no | no | bench CSV available |
| `output_saturated` | yes | yes | yes | 与 `saturated` 同义，便于和 bench CSV 字段对齐 |
| `invalid_transitions` | yes | no | no | bench CSV available，dashboard payload TODO |

当前 `/motor/status` 和 `/motor/state` 额外还会带这些状态字段：

- `status`
- `schema_version`
- `timestamp_ms`
- `publish_ms`
- `sample_age_ms`
- `abs_error_rpm`
- `pwm_ratio`
- `max_pwm`
- `command_timeout_ms`
- `control_enabled`
- `enabled`
- `hardware_outputs_enabled`
- `closed_loop`
- `saturated`
- `output_saturated`
- `timeout`
- `stop`
- `estop`
- `fault`
- `source`
- `loop`
- `numeric_valid`

当前 `robot_mqtt_bridge` 会把 `/motor/status` 归一化成 dashboard 友好的 payload，并补齐：

- `actual_rpm`
- `measured_rpm`
- `target_rpm`
- `error_rpm`
- `pwm`
- `pwm_duty`
- `timestamp_ms`
- `publish_ms`
- `sample_age_ms`
- `abs_error_rpm`
- `pwm_ratio`
- `direction`
- `saturated`
- `output_saturated`
- `enabled`
- `control_enabled`
- `closed_loop`
- `fault`
- `timeout`
- `estop`
- `stop`
- `last_update_time`
- `motor_state`

## 8. 当前已知问题与下一步

这部分按工程记录写，不按“失败总结”写。

当前状态：

- `0.25 max_pwm` 对安全初测很合适，但对更高目标转速明显偏保守
- 当前 bench 文档和实测记录已经显示，在 `0.25` 下 `80 rpm` 段仍有明显稳态误差
- 当前 `/motor/cmd` 常规路径已经接入真实 encoder + PI，但普通启动仍然默认不上真实 PWM

下一步建议：

- 先把人工调参观察值提高到 `0.35` 附近，再看 `pwm` 是否仍长期饱和
- 用调参曲线确认主要矛盾是不是“输出长期贴边”
- 让 dashboard frontend / backend / firmware 对 `max_pwm` 的提示和限幅口径保持一致
- 把 bench / tuning 曲线更明确地接到 `robot-ops-dashboard`
- 双轮差速、`ros2_control` 或完整底盘控制留到后续，不在这份文档范围内

当前风险：

- bench 调好的参数不能直接等同于上车参数
- 无负载、悬空和接地带负载的表现会不同
- 编码器噪声、供电和机械固定状态会显著影响结论

## 9. 验证命令

下面只列当前仓库里已经存在、或脚本里已经明确写出的命令，不代表这些命令已经在本次文档改动里全部跑过。

### Firmware build / flash / monitor

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1/firmware/esp32_microros_bridge
python3 -m platformio run
python3 -m platformio run --target upload --upload-port /dev/ttyACM0
python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0
```

### micro-ROS Agent

优先用仓库脚本：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/start_microros_agent_udp.sh
```

如果已经有 ROS 2 和 micro-ROS Agent workspace：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/uros_ws/install/local_setup.bash"
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 4
```

### ROS 2 topic check

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/check_microros_topics.sh
```

或手工看关键 topic：

```bash
source /opt/ros/jazzy/setup.bash
source /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1/install/setup.bash
ros2 topic echo --once /motor/status
ros2 topic echo --once /motor/actual_rpm
ros2 topic echo --once /motor/state
ros2 topic info -v /motor/cmd
ros2 topic info -v /motor/target_rpm
```

### MQTT observe

```bash
timeout 5s mosquitto_sub -h 127.0.0.1 -p 1883 -t robot/motor/status -C 1
timeout 5s mosquitto_sub -h 127.0.0.1 -p 1883 -t robot/motor/cmd -C 1
```

### Dashboard integration check

一键启动当前主链路：

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/start_motor_dashboard_stack.sh
./scripts/check_motor_dashboard_loop.sh
```

脚本里当前对应的 backend / frontend 启动命令分别是：

```bash
source /home/ina/workspace/robot-ops-dashboard/.venv/bin/activate
export PYTHONPATH=/home/ina/workspace/robot-ops-dashboard
export MQTT_BROKER_URL=mqtt://127.0.0.1:1883
export ROBOT_OPS_TASK_SOURCE=mock_json
cd /home/ina/workspace/robot-ops-dashboard
uvicorn backend.app.main:app --host 127.0.0.1 --port 9000
```

```bash
cd /home/ina/workspace/robot-ops-dashboard
python3 -m http.server 8001 --bind 127.0.0.1
```

`node --check frontend/app.js` 属于 `robot-ops-dashboard` 仓库侧检查，不属于本仓库构建命令：

```bash
cd /home/ina/workspace/robot-ops-dashboard
node --check frontend/app.js
```

### Real-hardware end-to-end check

```bash
cd /home/ina/Documents/PlatformIO/Projects/robot-state-monitor-v1
./scripts/check_real_hw_chain.sh
./scripts/check_real_hw_chain.sh --dashboard
```

## 10. 文档写法约束

这份文档按当前仓库里能看到的实现和记录来写，刻意保持下面这些边界：

- 语气按现场工程师调试记录写
- 明确区分 current status、TODO 和 risk
- 不写“已经完全稳定”
- 不把 dashboard 写成实时控制器
- 不把 single-motor bench 写成完整底盘能力
- 不引入 `ros2_control`

后续如果出现新的 bench 记录、曲线或 dashboard 页面能力，再单独补充，不在这份文档里提前透支结论。
