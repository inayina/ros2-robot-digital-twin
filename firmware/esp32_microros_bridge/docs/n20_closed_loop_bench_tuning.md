# N20 Closed-Loop Bench Tuning Record

## Scope

本记录用于跟踪 `kEnableN20ClosedLoopBench` 对应的 single N20 encoder motor bench 调参过程。

当前硬件链路：

```text
ESP32-S3 -> TB6612FNG -> single N20 motor with encoder
```

约束：

- 只测单个 N20 编码器电机
- 只用 TB6612 B 通道和 `GPIO10/GPIO11` 编码器输入
- 只做 `target_rpm / actual_rpm` 闭环
- 不接 `ROS 2 /cmd_vel`
- 不接 `ros2_control`
- 不声明 robot linear velocity 或完整双轮底盘能力
- 不修改既有 micro-ROS topic / STM32 串口协议
- bench 默认保持 `false`

## Safety Boundary

默认安全状态：

- `kEnableN20ClosedLoopBench = false`
- `kEnableMotorHardwareOutputs = false`
- 普通启动不会输出 TB6612 PWM
- TB6612 `STBY` 在 init / stop / target 0 时保持关闭
- 启动阶段先写 `PWM = 0` 和 coast，再根据 bench profile 决定是否输出

自动停机条件：

- profile 到 `kN20ClosedLoopBenchProfileStopMs` 后自动 stop
- 超过 `kN20ClosedLoopBenchMaxDurationMs` 后强制 stop
- 控制周期异常过大或 `dt = 0` 时 stop
- TB6612 写入失败时 stop
- 编码器非法跳变超过 `kN20ClosedLoopBenchMaxInvalidEncoderTransitions` 时 stop

输出限制：

- `target_rpm` 经过 `kN20ClosedLoopBenchMaxTargetRpm` 限幅
- PI 输出经过 `kN20ClosedLoopBenchMaxPwm` 限幅
- 积分项经过 `kN20ClosedLoopBenchIntegralMin / Max` 限幅
- `speed_pid.cpp` 保留 anti-windup：输出饱和且误差继续同向时回退本周期积分
- stop 和 target 0 时清积分
- 方向切换前先 coast `kN20ClosedLoopBenchDirectionChangeCoastMs`

当前 bench profile 只默认测试 forward，reverse 逻辑保留但不纳入默认 profile。

## CSV Columns

串口日志列：

```text
timestamp_ms,target_rpm,actual_rpm,error_rpm,pwm,direction,encoder_delta,status,raw_rpm,filtered_rpm,integral,output_saturated,bench_phase,invalid_transitions
```

含义：

- `timestamp_ms`：相对 bench 启动时间
- `target_rpm`：当前阶跃目标，已过最大目标限幅
- `actual_rpm`：当前用于 PI 的 filtered rpm
- `error_rpm`：`target_rpm - actual_rpm`
- `pwm`：输出到 TB6612 B 通道的 signed duty，正数为 forward
- `direction`：当前输出方向，`1` forward，`-1` reverse，`0` stop/coast
- `encoder_delta`：本控制周期编码器计数增量，已应用 `invert_encoder_direction`
- `status`：`run`、`stop`、`direction_hold` 或 `encoder_warmup`
- `raw_rpm`：本周期未滤波 rpm
- `filtered_rpm`：一阶低通后的 rpm
- `integral`：PI 内部积分状态
- `output_saturated`：PI 或最终输出限幅是否触发
- `bench_phase`：当前 elapsed ms，便于画 profile 背景
- `invalid_transitions`：A/B 相非法跳变计数

CSV header 只在 bench 开始后打印一次。bench 模式下会抑制本地 IMU / runtime 高频调试，便于直接复制 CSV。

## Current Profile

当前默认保守阶跃 profile：

- `0-1s`：`0 rpm`
- `1-4s`：`40 rpm`
- `4-7s`：`60 rpm`
- `7-10s`：`80 rpm`
- `10-13s`：`50 rpm`
- `13-15s`：`0 rpm`
- `15s` 后自动 stop
- 硬上限：`20s`

2026-05-21 实测显示，在 `max_pwm = 0.25` 下这颗 bench 电机更适合 `40/60/80/50 rpm` 保守 profile；`120/180 rpm` 会长期大误差或接近饱和，不作为默认录屏 profile。

## Current Parameters

当前 `app_config.h` bench 参数：

```text
kN20ClosedLoopBenchControlPeriodMs = 50
kN20ClosedLoopBenchPrintIntervalMs = 100
kN20ClosedLoopBenchMaxDurationMs = 20000
kN20ClosedLoopBenchDirectionChangeCoastMs = 200
kN20ClosedLoopBenchWheelDiameterM = 0.043
kN20ClosedLoopBenchEncoderPulsesPerRev = 11.0
kN20ClosedLoopBenchGearRatio = 30.0
kN20ClosedLoopBenchEdgesPerPulse = 4.0
kN20ClosedLoopBenchEncoderFilterAlpha = 0.35
kN20ClosedLoopBenchInvertEncoderDirection = true
kN20ClosedLoopBenchMaxPwm = 0.25
kN20ClosedLoopBenchMinEffectivePwm = 0.12
kN20ClosedLoopBenchMaxTargetRpm = 80.0
kN20ClosedLoopBenchDeadbandRpm = 0.0
kN20ClosedLoopBenchMaxInvalidEncoderTransitions = 100
kN20ClosedLoopBenchKp = 0.0018
kN20ClosedLoopBenchKi = 0.0012
kN20ClosedLoopBenchKd = 0.0
kN20ClosedLoopBenchIntegralMin = -160.0
kN20ClosedLoopBenchIntegralMax = 160.0
```

`wheel_diameter_m` 当前不参与 PID，只为后续 dashboard wheel-speed 显示或线速度换算准备。后续 Dashboard speed slider 只能作为 target wheel speed / target rpm 的上位输入，不应直接控制 PWM。

## Test Method

1. 确认电机悬空固定，TB6612 `VM` 使用限流电源，ESP32 / TB6612 / 电机电源共地。
2. 在 [../src/config/app_config.h](../src/config/app_config.h) 中把 `kEnableN20ClosedLoopBench` 临时改为 `true`。
3. 编译：`python3 -m platformio run`
4. 烧录：`python3 -m platformio run --target upload --upload-port /dev/ttyACM0`
5. 串口：`python3 -m platformio device monitor -b 921600 --port /dev/ttyACM0`
6. 从 CSV header 开始复制日志到文件，例如 `n20_bench_YYYYMMDD.csv`。
7. 测试结束后把 `kEnableN20ClosedLoopBench` 改回 `false`，重新编译烧录安全版。

可接受现象：

- 普通启动不转
- bench 打开后按 profile 阶跃转动
- `target_rpm / actual_rpm / pwm / error_rpm` 可解释
- `actual_rpm` 能大致跟随 `target_rpm`
- `13s` 回到 `0 rpm` 后能停下，`15s` 后自动 stop
- `output_saturated` 只在限幅需要时出现，`integral` 不持续发散
- forward 时 `direction`、`encoder_delta`、`actual_rpm` 符号一致

## Test Record

### 2026-05-20 ticks/s step-response bench

旧版 bench 使用 `ticks/s` profile。观察到编码器累计计数、50ms 速度估算、PI 输出、PWM 限幅和积分限幅均正常工作，但口径不利于和 N20 输出轴转速对齐。

### 2026-05-21 RPM bench cleanup

- Mode: initial RPM profile (`0 -> 60 -> 120 -> 180 -> 90 -> 0 rpm`)
- Change:
  - CSV 改为 `target_rpm / actual_rpm / error_rpm`
  - 编码器配置集中到 `app_config.h`
  - actual rpm 使用 `encoder_rpm_estimator` 一阶低通
  - 加入 max duration、profile stop、invalid encoder transition stop、direction-change coast
  - 默认真实硬件输出保持关闭，只有 bench flag 显式打开时输出
- Status:
  - 已完成代码级和构建级验证
  - 实机第一轮显示 `120/180 rpm` 在 `max_pwm = 0.25` 下不可作为默认录屏 profile：`180 rpm` 段实际约 `77 rpm`，并因非法编码器跳变超过旧阈值触发安全 stop

### 2026-05-21 conservative hardware bench

- Mode: adjusted RPM profile (`0 -> 40 -> 60 -> 80 -> 50 -> 0 rpm`)
- Result:
  - profile 完整跑完并打印 `profile complete`
  - `invalid_transitions` 最终约 `36`，低于当前安全阈值 `100`
  - `output_saturated = 0`，未出现 PI 输出饱和发散
  - `integral` 在 `80 rpm` 段达到 `160` 上限，但没有继续发散
  - `40 rpm` 段约稳定在 `33 rpm`
  - `60 rpm` 段约爬升到 `42 rpm`
  - `80 rpm` 段约爬升到 `66 rpm`
  - 回落 `50 rpm` 段约稳定在 `52-53 rpm`
  - `13s` 切到 `0 rpm` 后 PWM 立即归零，约 `14.4s` filtered rpm 衰减到接近 `0`
- Conclusion:
  - 当前参数适合安全录屏和观察阶跃响应，但还不是精确调速
  - 若后续要让 `80 rpm` 更贴近目标，需要在确认电源/温升/机械固定安全后提高 `max_pwm` 或加入更明确的速度前馈
  - 如果继续出现较多 `invalid_transitions`，优先检查 encoder A/B 接线、上拉、电平和线缆噪声，再考虑 PCNT 或更稳健的解码方式

## Known Issues

- 低速可能存在 TB6612 / N20 静摩擦死区，必要时再调整 `min_effective_pwm`
- 编码器噪声可能导致 `raw_rpm` 抖动，应优先看 `filtered_rpm / actual_rpm`
- 当前是 single motor bench，不能代表整车速度或双轮同步表现
- 当前 PI 参数是保守起点，不是最终生产参数
- 未接负载时调好的参数，装到车体或轮子接地后需要重新调
