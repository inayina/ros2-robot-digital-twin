# Motor Closed-loop Tuning Skill

本 skill 用于本仓库 N20 编码器电机闭环调参。它的目标是保护现有 micro-ROS、MQTT、Dashboard 和电机安全边界，避免在未诊断清楚前盲目修改 PI/PID 参数。

## 1. 工作原则

- 先诊断，再调参
- 先开环，再闭环
- 先 P-only，再加 I
- PWM 饱和时不要盲目加 Ki
- target=0 时 pwm 必须为 0
- 测试结束必须 coast / standby

## 2. 必看信号

- target_rpm
- actual_rpm / measured_rpm
- error_rpm
- pwm / max_pwm
- encoder_delta
- encoder_count
- invalid_transitions
- output_saturated
- loop_count
- hardware_outputs_enabled

## 3. 快速判断表

| 现象 | 优先判断 | 下一步 |
| --- | --- | --- |
| PWM 增大但 actual_rpm 不变 | 硬件/PWM/供电/估速问题 | 做 open-loop PWM sweep |
| PWM 长期顶到 max_pwm 且 rpm 跟不上 | 目标过高或能力不足 | 降 target 或提高安全限幅 |
| PWM 没顶满但 rpm 上不去 | Kp 太小或输出被清零 | 检查 deadband/min_pwm/control_enabled |
| actual_rpm 方向反了 | 编码器方向或电机方向反了 | 检查 invert_encoder_direction |
| invalid_transitions 增加 | 编码器接线/中断/抖动问题 | 暂停闭环，先查编码器 |
| target=0 但 pwm 不为0 | 安全逻辑错误 | 必须先修 |

## 4. 标准调参流程

### Step 1: open-loop PWM sweep

固定 PWM 阶梯：

```text
0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45
```

每档保持 2 秒，记录：

```text
timestamp_ms, mode, pwm, actual_rpm, raw_rpm, filtered_rpm, encoder_delta, encoder_count, invalid_transitions, direction, status
```

### Step 2: 确定真实速度能力

如果 0.45 下最高只有 80 rpm，则闭环 target_rpm 不要从 100 rpm 开始。
建议先选最大能力的 50%~70%。

### Step 3: P-only

Ki=0, Kd=0，只调 Kp。
目标是 actual_rpm 能朝 target_rpm 靠近，不要求立即消除稳态误差。

### Step 4: 小幅加入 Ki

只有在 P 控制已经基本能跟随后，再加很小 Ki。
如果 pwm 已经饱和，不要加 Ki。

### Step 5: Dashboard 曲线观察

Dashboard 只做观察，不参与实时控制。
重点看 target_rpm、actual_rpm、error_rpm、pwm/max_pwm。

## 5. 禁止行为

- 不要一上来把 max_pwm 放到 1.0
- 不要在不知道电机真实能力前提高 target_rpm
- 不要看到误差大就直接加 Ki
- 不要忽略 invalid_transitions
- 不要删除现有安全收尾逻辑
- 不要让前端直接控制高频闭环

## 6. 修改输出要求

每次 Codex 完成调参相关修改后，必须输出：

- 修改文件列表
- 改了哪些安全边界
- 是否改变 max_pwm / target_rpm / Kp / Ki / deadband / min_pwm
- 运行了哪些检查
- 哪些部分还需要真实硬件验证
