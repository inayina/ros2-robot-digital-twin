# N20 Closed-Loop Bench Tuning Record

## Scope

本记录用于跟踪 `kEnableN20ClosedLoopBench` 对应的单 N20 编码器速度闭环 bench 调参过程。

约束：

- 只测单个 N20
- 只用 TB6612 B 通道
- 不接 `ROS 2 /cmd_vel`
- 不修改既有 micro-ROS 主链路
- bench 默认保持 `false`

## CSV Columns

串口日志列：

```text
timestamp_ms,target_ticks_per_sec,measured_ticks_per_sec,pwm,error,encoder_count,invalid_transitions
```

含义：

- `timestamp_ms`：相对 bench 启动时间
- `target_ticks_per_sec`：当前阶跃目标
- `measured_ticks_per_sec`：当前估算速度
- `pwm`：输出到 TB6612 B 通道的占空比
- `error`：目标与测量差
- `encoder_count`：累计编码器计数
- `invalid_transitions`：A/B 相非法跳变计数

## Current Profile

当前默认阶跃 profile：

- `0-2s`：`0 ticks/s`
- `2-6s`：`500 ticks/s`
- `6-10s`：`800 ticks/s`
- `10-14s`：`1000 ticks/s`
- `14-18s`：`500 ticks/s`
- `18s` 后：`0 ticks/s`

## Current Parameters

当前 `app_config.h` bench 参数：

```text
kN20ClosedLoopBenchControlPeriodMs = 50
kN20ClosedLoopBenchPrintIntervalMs = 100
kN20ClosedLoopBenchMaxPwm = 0.25
kN20ClosedLoopBenchInvertMeasuredTicks = true
kN20ClosedLoopBenchKp = 0.00018
kN20ClosedLoopBenchKi = 0.00005
kN20ClosedLoopBenchKd = 0.0
kN20ClosedLoopBenchIntegralMin = -400.0
kN20ClosedLoopBenchIntegralMax = 400.0
```

## Test Record

### 2026-05-20 Step-response bench

- Mode: default 6-stage profile (`0 -> 500 -> 800 -> 1000 -> 500 -> 0 ticks/s`)
- Observation:
  `0-2s` 静止段保持 `PWM=0`
  `500 ticks/s` 段实测大多在 `240-320 ticks/s`
  `800 ticks/s` 段实测大多在 `420-480 ticks/s`
  `1000 ticks/s` 段实测大多在 `540-600 ticks/s`
  `14s` 回落到 `500 ticks/s` 时能看到明显降速响应
  `18s` 后自动 `coast` 并打印 `profile complete`
  `invalid_transitions = 0`
- Conclusion:
  编码器累计计数、50ms 速度估算、PI 输出、PWM 限幅和积分限幅均正常工作
  当前参数下阶跃响应稳定，但中高目标段存在明显稳态误差，后续优先调 `max_pwm / kp / ki`

典型日志：

```text
timestamp_ms,target_ticks_per_sec,measured_ticks_per_sec,pwm,error,encoder_count,invalid_transitions
59,0.000,0.000,0.000,0.000,0,0
2059,500.000,160.000,0.063,340.000,-8,0
6059,800.000,520.000,0.070,280.000,-1108,0
10159,1000.000,560.000,0.099,440.000,-2990,0
14059,500.000,180.000,0.078,320.000,-5204,0
17959,500.000,260.000,0.063,240.000,-6314,0
```

补充：

- bench 模式下已抑制本地 IMU / runtime 高频调试，便于复制 CSV
- 若未启动 micro-ROS Agent，串口中仍可能出现少量连接重试提示；做离线调参时可直接忽略这些非 CSV 行

## Next Tuning Ideas

- 如果 `1000 ticks/s` 长时间停在 `540-600`，优先增大 `kN20ClosedLoopBenchMaxPwm`
- 如果上升太慢但没有明显振荡，优先小幅提高 `kN20ClosedLoopBenchKp`
- 如果稳态误差长期偏大，再缓慢提高 `kN20ClosedLoopBenchKi`
- 如果低速段抖动或堵转明显，先降低 `Ki`，再观察积分限幅是否需要收紧
