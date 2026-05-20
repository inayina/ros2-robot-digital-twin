# Motor Control Design

## 1. Scope

本设计文档描述 ESP32 侧 N20 编码器减速电机控制主线。

当前阶段目标是桌面级单电机闭环验证：

- ESP32 输出 PWM/DIR
- ESP32 读取 N20 编码器 A/B
- ESP32 计算 rpm
- ESP32 本地执行 PID 速度闭环
- ROS 2 / micro-ROS 下发 `target_rpm`
- ESP32 回传 `actual_rpm` / `motor_state`

当前不做完整底盘，不追求地面移动，不声明已经完成双轮差速、`/odom` 或导航级实车运动控制。

## 2. Architecture Decision

电机闭环控制主线迁移到 ESP32。

系统角色：

- **STM32F411**：sensor/state controller
- **ESP32-S3**：motor controller + communication bridge
- **ROS 2 PC**：supervisor / integration layer

STM32F411 保留 MPU6050 采样、姿态解算、IMU / robot state 输出和原有数字孪生链路。它不再作为 N20 编码器电机闭环主控制器。

ESP32-S3 负责 N20 PWM/DIR、编码器读取、rpm 计算、PID 速度闭环、ROS 2 / micro-ROS 目标速度接收和状态回传。后续双轮差速、`/cmd_vel` 解算和 `ros2_control` 接入都沿 ESP32 主线扩展。

ROS 2 PC 发布目标速度、接收状态、做可视化验证，并在后续作为 `ros2_control hardware_interface` 的上位集成层。

## 3. Current Hardware Stage

当前硬件状态：

- 已购买 6V N20 编码器减速电机 x 1
- 手头有 130 普通电机 x 1，仅用于驱动通道辅助验证
- 当前 TB6612 A 通道保留给 130 普通电机
- 当前 TB6612 B 通道作为单 N20 主验证通道
- 当前不做完整底盘
- 当前不追求地面移动
- 当前目标是桌面级单 N20 闭环验证
- 单 N20 跑通后，再购买第二个同规格 N20，并把 TB6612 A 通道从 130 切换为 N20，扩展为双轮差速

待硬件到货和实测后再确认：

- 真实 GPIO 分配
- PWM 频率和分辨率
- 编码器 CPR / PPR
- 减速比
- 供电电压和电平兼容性
- 空载 rpm、启动电流、堵转电流
- PID 初始参数和限幅参数

在这些参数未确认前，文档和软件骨架只保留可配置入口，不写死真实数值。

## 4. Legacy STM32 Motor Code

STM32 既有 open-loop motor control 不删除。

它的定位是：

- legacy experiment
- early validation
- ROS 2 到真实下位机执行链路的 proof-of-concept

它不再继续扩展为：

- 编码器读取
- PID 调速
- 双轮差速
- `ros2_control` 主线

等 ESP32 + N20 单电机闭环真实验证完成后，再考虑归档或删除 STM32 旧电机代码。当前阶段不能因为迁移电机主线而破坏 STM32 IMU / micro-ROS / Gazebo 数字孪生链路。

## 5. ESP32 Control Chain

单电机闭环链路：

```text
ROS 2 / micro-ROS target_rpm
  -> ESP32 ros_comm_task
  -> target_rpm shared state / queue
  -> ESP32 motor_control_task
  -> local PID
  -> PWM/DIR
  -> TB6612 motor driver
  -> 6V N20 encoder gear motor
  -> encoder A/B
  -> ESP32 rpm calculation
  -> motor_state shared state / queue
  -> ROS 2 / optional MQTT dashboard
```

第一版使用 `target_rpm`，避免过早引入底盘运动学。单 N20 闭环稳定后，再扩展到 `/cmd_vel -> left/right rpm`。

### 5.1 ESP32 to TB6612 Wiring Plan

当前电机驱动模块按 **TB6612 A / B 分工规划**：

- A 通道保留给 130 普通电机做驱动通道辅助验证
- B 通道作为当前单 N20 编码器减速电机的主验证通道
- 后续购买第二个同规格 N20 后，再把 A 通道从 130 切换为 N20

本小节描述的是 **信号级接线规划**，不是已经锁定的最终 GPIO 表。真实 GPIO 号、PWM channel / timer 和编码器输入脚位，等硬件上板和实测后再在配置里固化。

#### A. TB6612 Logic / Power Side

- ESP32 `3.3V logic` -> TB6612 `VCC`
- ESP32 `GND` -> TB6612 `GND`
- 外部 `6V motor supply` -> TB6612 `VM`
- 电机电源负极 -> TB6612 `GND`
- ESP32 地、TB6612 地、电机电源地必须共地

约束：

- `VM` 只接电机电源，不接 ESP32 `3.3V`
- `VCC` 只接逻辑电源，不直接拿来驱动电机
- 调试阶段优先使用独立限流电源给 `VM`

#### B. ESP32 Control Signals to TB6612

当前推荐把 A / B 两路都预留成独立控制接口：

- ESP32 `channel_a_pwm_out` -> TB6612 `PWMA`
- ESP32 `channel_a_dir_in1` -> TB6612 `AIN1`
- ESP32 `channel_a_dir_in2` -> TB6612 `AIN2`
- ESP32 `channel_b_pwm_out` -> TB6612 `PWMB`
- ESP32 `channel_b_dir_in1` -> TB6612 `BIN1`
- ESP32 `channel_b_dir_in2` -> TB6612 `BIN2`
- ESP32 `motor_standby_out` -> TB6612 `STBY`

规划原则：

- `PWMA/PWMB` 都由 `motor_control_task` 独占更新
- `AIN1/AIN2/BIN1/BIN2` 只表达方向和制动状态
- `STBY` 默认由软件控制，不建议长期硬拉高后失去可控待机能力
- ROS 通信任务不直接改 `PWMA/PWMB/AIN1/AIN2/BIN1/BIN2/STBY`

#### B.1 Current Planned ESP32 GPIO Assignment

以下是当前 **ESP32-S3 DevKitC-1 接线规划**，用于桌面级 bring-up 和后续代码落地。这里写的是当前 planned mapping，后续如果实物布线、EMI、PCNT 稳定性或调试便利性有问题，可以再调整，但当前文档和后续代码应先以这组 GPIO 为主。

- `GPIO4`  -> TB6612 `PWMA`   : A 通道 PWM，保留给 130 普通电机
- `GPIO5`  -> TB6612 `AIN1`   : A 通道方向 1
- `GPIO6`  -> TB6612 `AIN2`   : A 通道方向 2
- `GPIO7`  -> TB6612 `PWMB`   : B 通道 PWM，当前单 N20 主通道
- `GPIO8`  -> TB6612 `BIN1`   : B 通道方向 1
- `GPIO9`  -> TB6612 `BIN2`   : B 通道方向 2
- `GPIO18` -> TB6612 `STBY`   : 驱动待机控制
- `GPIO10` -> N20 encoder `A` : 当前单 N20 编码器 A 相输入
- `GPIO11` -> N20 encoder `B` : 当前单 N20 编码器 B 相输入
- `GPIO12` -> future encoder `A` : 预留给后续 A 通道 N20 编码器 A 相
- `GPIO13` -> future encoder `B` : 预留给后续 A 通道 N20 编码器 B 相

补充说明：

- `GPIO16/GPIO17` 已经用于当前 STM32 UART，不再分给电机侧
- `GPIO19/GPIO20` 默认用于 ESP32-S3 USB D- / D+，不占用到电机侧
- `GPIO0/GPIO3/GPIO45/GPIO46` 属于 strapping pins，当前不作为电机控制引脚
- `GPIO26-GPIO32` 通常与 SPI flash / PSRAM 相关，当前不纳入规划

因此，当前这套规划优先使用 `GPIO4-13` 和 `GPIO18` 这组比较干净、连续、便于布线和调试的引脚。

#### C. TB6612 Output to Motor

- TB6612 `A01/AO1` -> 130 motor terminal 1
- TB6612 `A02/AO2` -> 130 motor terminal 2
- TB6612 `B01/BO1` -> N20 motor terminal 1
- TB6612 `B02/BO2` -> N20 motor terminal 2

说明：

- 不预先假定哪一端一定是正极
- 如果实测方向与软件定义相反，优先通过交换对应通道的两根电机线或调整方向映射修正
- 当前 130 普通电机固定在 A 通道做辅助验证，当前单 N20 主线固定在 B 通道

#### D. Encoder Wiring Boundary

N20 编码器 **不经过 TB6612**，编码器线直接回 ESP32：

- encoder `VCC` -> 按实测兼容电压接逻辑电源
- encoder `GND` -> 系统共地
- 当前单 N20 encoder `A` -> ESP32 `GPIO10`
- 当前单 N20 encoder `B` -> ESP32 `GPIO11`
- 后续第二个 N20 encoder `A` -> ESP32 `GPIO12`
- 后续第二个 N20 encoder `B` -> ESP32 `GPIO13`

约束：

- 编码器输出电平未确认前，不默认直接上 ESP32
- 如果编码器输出可能到 `5V`，必须先确认是否需要分压、限流或电平转换
- 编码器 A/B 输入脚位后续优先按 PCNT 友好方式选型

#### E. Future A-Channel Migration

当前 B 通道先承担单 N20 主验证路径。后续第二个同规格 N20 到货后：

- 130 普通电机从 A 通道退出
- A 通道改接第二个 N20
- B 通道继续保留当前第一颗 N20
- 两个 N20 再进入双轮差速阶段

在第二个 N20 到货前：

- A 通道仍允许只承担 130 辅助验证
- 当前不要求把 A 通道提前接成 N20
- 当前不要求第一版代码立即完成双电机控制

#### F. Recommended Bring-Up Order

1. 先接 `VCC / GND / VM / STBY`
2. 先接 A 通道 `PWMA / AIN1 / AIN2`，用 130 普通电机做驱动通道带电验证
3. 再接 B 通道 `PWMB / BIN1 / BIN2`，做单个 N20 开环验证
4. 再接入 N20 编码器 `A/B` 做 rpm 计算和闭环
5. 后续购买第二个同规格 N20，再把 A 通道从 130 切换成第二颗 N20

#### G. Wiring Summary

```text
ESP32 3.3V logic  -> TB6612 VCC
ESP32 GND         -> TB6612 GND
6V motor supply   -> TB6612 VM

ESP32 GPIO4       -> TB6612 PWMA
ESP32 GPIO5       -> TB6612 AIN1
ESP32 GPIO6       -> TB6612 AIN2
ESP32 GPIO7       -> TB6612 PWMB
ESP32 GPIO8       -> TB6612 BIN1
ESP32 GPIO9       -> TB6612 BIN2
ESP32 GPIO18      -> TB6612 STBY

TB6612 A01/AO1    -> 130 motor terminal 1
TB6612 A02/AO2    -> 130 motor terminal 2
TB6612 B01/BO1    -> N20 motor terminal 1
TB6612 B02/BO2    -> N20 motor terminal 2

N20 encoder A     -> ESP32 GPIO10
N20 encoder B     -> ESP32 GPIO11
N20 encoder power -> logic-side power after voltage-level confirmation
```

## 6. ESP32 Task and Core Assignment

双核分工是设计目标，第一版仍以单 N20 最小闭环为目标，不为了双核分工引入复杂并发框架。

当前代码状态：

- Core 0 `ros_comm_task` 已实现并固定到通信侧运行时
- Core 1 `motor_control_task` 已实现并固定到电机控制侧运行时
- `ros_comm_task` 和 `motor_control_task` 已通过 shared state 交换命令 / 状态快照
- 当前 Core 1 使用 mock motor response 模拟 `actual_rpm` 追踪 `target_rpm`
- encoder rpm estimator、speed PID、single motor control 纯逻辑模块已落地
- TB6612 driver 文件已落地，提供 A/B 双通道 PWM/DIR/STBY 可配置接口
- 已保留 130 普通电机 A 通道 bench test，用于验证 TB6612 `PWMA/AIN1/AIN2/STBY`，默认关闭
- 已新增单 N20 编码器速度闭环 step-response bench，本地读取 `GPIO10/GPIO11`、驱动 TB6612 B 通道、输出 CSV 调参日志，默认关闭
- 当前仍未接入真实 PWM、编码器和 PID 执行

### 6.1 Core 1: Motor-Control Core

Core 1 规划运行 `motor_control_task`，负责实时性更高的本地控制：

- 当前 mock `actual_rpm` response
- encoder pulse counting / PCNT
- rpm calculation
- PID speed control
- PWM/DIR update
- safety stop / timeout protection

`motor_control_task` 是唯一允许更新 PWM/DIR 的任务。

### 6.2 Core 0: Communication Core

Core 0 规划运行 `ros_comm_task`，负责通信和低频状态输出：

- micro-ROS command subscription
- `motor_state` publishing
- optional MQTT status bridge
- debug logging

`ros_comm_task` 不直接修改 PWM，不直接进入 PID 计算，也不直接控制电机方向。

### 6.3 Inter-Task Data Flow

`motor_control_task` 和 `ros_comm_task` 之间通过简单机制传递数据：

- queue
- shared state
- atomic variable

建议数据方向：

- `ros_comm_task -> motor_control_task`：`target_rpm`、enable、estop
- `motor_control_task -> ros_comm_task`：`actual_rpm`、`pwm_duty`、direction、timeout、fault、control state

第一版应保持数据结构简单，先完成可复现的单电机闭环，再决定是否抽象为更完整的 motor controller module。

## 7. ROS 2 / micro-ROS Topics

单 N20 阶段建议接口：

- Subscribe: `/motor/target_rpm`
- Publish: `/motor/actual_rpm`
- Publish: `/motor/state`

当前代码已实现：

- Subscribe: `/motor/target_rpm`
- Subscribe: `/cmd_vel`
- Publish: `/motor/actual_rpm`
- Publish: `/motor/state`

当前 `/motor/actual_rpm` 和 `/motor/state` 使用 mock motor response 数据；N20 到货并实测后，再替换为真实编码器 rpm 和 PID 状态。

第一版用简单标量消息完成闭环验证：

- `/motor/target_rpm`：目标 rpm
- `/motor/actual_rpm`：当前为 mock actual rpm，真实闭环阶段改为编码器实测 rpm

`/motor/state` 当前先用 `std_msgs/msg/String` 的易调试 JSON 风格字符串表达，字段包括：

- `target_rpm`
- `actual_rpm`
- `error_rpm`
- `pwm_duty`
- `direction`
- `control_enabled`
- `saturated`
- `timeout`
- `estop`
- `fault`

当前代码状态字段结构位于 [motor_control_shared.h](../src/motor/motor_control_shared.h)，mock 控制骨架位于 [motor_controller.h](../src/motor/motor_controller.h) / [motor_controller.cpp](../src/motor/motor_controller.cpp)。TB6612 驱动边界位于 [tb6612_driver.h](../src/motor/tb6612_driver.h) / [tb6612_driver.cpp](../src/motor/tb6612_driver.cpp)，当前只提供可配置接口，不写死真实 GPIO、PWM channel、频率或分辨率。真实硬件阶段再把预留的 PWM/DIR、encoder A/B、PID 接口落到具体实现里。

双轮差速阶段再扩展：

- Subscribe: `/cmd_vel`
- Publish: `/motor/left_actual_rpm`
- Publish: `/motor/right_actual_rpm`
- Publish: `/motor/state`

自定义 `MotorState` 消息等字段稳定后再引入，避免第一版把接口设计得过重。

## 8. MQTT Boundary

MQTT 仅用于低频状态上报和运维展示：

- dashboard 展示
- robot health
- motor state mirror
- 远程调试
- 低频运行状态上报

MQTT 不进入实时 PID 闭环。

正确链路：

```text
ROS 2 target_rpm -> ESP32 local PID -> PWM/DIR -> motor
ESP32 motor_state -> ROS 2 / optional MQTT dashboard
```

不做：

```text
MQTT -> realtime PID -> motor control loop
Web dashboard -> realtime motor control loop
```

实时电机闭环由 ESP32 本地完成。ROS 2 / micro-ROS 用于控制指令和机器人状态链路，MQTT 只镜像低频状态或辅助远程调试。

## 9. Safety Requirements

第一版单电机闭环也必须包含安全边界：

- 上电默认 0 PWM
- `target_rpm` timeout 后停车
- ESTOP 覆盖普通速度命令
- ESTOP 释放后不自动恢复旧速度
- PWM 输出限幅
- PID 积分 anti-windup
- 方向切换前先输出 0 PWM
- 记录 saturated / timeout / estop / fault 状态
- 调试阶段优先使用限流电源
- 电机电源与 ESP32 逻辑电源分离并共地

安全逻辑应由 `motor_control_task` 执行，不能依赖 ROS 2、MQTT 或 dashboard 的实时响应。

当前代码已经落地的安全骨架包括：

- `target_rpm` 命令共享状态
- Core 1 本地 command timeout 检查
- timeout 后本地 control state 置为 disabled

但真实 PWM 归零、方向切换保护和 ESTOP 输出仍要等硬件控制层接入后完成。

## 10. Phased Roadmap

### Phase 1: ESP32 Single N20 Open-Loop PWM/DIR

目标：验证 ESP32 能稳定控制驱动通道。

- 使用 `tb6612_driver` 配置真实 PWM/DIR/STBY 引脚和 PWM 参数
- 先用 TB6612 A 通道上的 130 普通电机做低风险开环测试
- 再用 TB6612 B 通道上的单个 N20 做开环测试
- 验证正转、反转、停止
- 验证 PWM duty 与转速的粗略关系
- 验证方向切换前归零

当前 130 A 通道 bench test 参数集中在 [app_config.h](../src/config/app_config.h)，临时使用：

- `kEnableTb6612ChannelABenchTest`
- `GPIO4 -> PWMA`
- `GPIO5 -> AIN1`
- `GPIO6 -> AIN2`
- `GPIO18 -> STBY`
- `kTb6612BenchTestDuty = 0.18`
- 正转短脉冲、coast、反转短脉冲、最后禁用 STBY

这个 bench test 只验证 130 / TB6612 A 通道，不代表 N20 闭环已经完成。测试结束后普通运行应保持 `kEnableTb6612ChannelABenchTest = false`。

### Phase 2: Encoder A/B Reading and RPM Calculation

目标：可靠读取编码器并计算 rpm。

- 接入编码器 A/B
- 使用 PCNT 或 GPIO interrupt 计数
- 维护 encoder count 和 delta count
- 判断方向
- 计算 `actual_rpm`
- 必要时加入简单滤波

代码层面已经落地 [encoder_rpm_estimator.h](../src/motor/encoder_rpm_estimator.h) / [encoder_rpm_estimator.cpp](../src/motor/encoder_rpm_estimator.cpp)，当前使用可配置 `counts_per_output_rev` 和 filter alpha，不写死 N20 CPR/PPR。真实 N20 到货后，应先实测编码器计数方向和每输出轴一圈计数，再把真实计数源接入该模块。

### Phase 3: Single-Motor PID Closed Loop

目标：完成 `target_rpm -> PID -> PWM` 单电机闭环。

- 先只开 P 控制
- 稳定后加入 I 控制
- D 项默认不启用
- 加入 PWM 限幅和 anti-windup
- 发布闭环状态

代码层面已经落地 [speed_pid.h](../src/motor/speed_pid.h) / [speed_pid.cpp](../src/motor/speed_pid.cpp)，以及 [single_motor_control.h](../src/motor/single_motor_control.h) / [single_motor_control.cpp](../src/motor/single_motor_control.cpp)。当前只作为可测试纯逻辑，不绑定真实 PID 参数、不启用真实 PWM 输出。

### Phase 4: ROS 2 / micro-ROS Command and Feedback

目标：完成 ROS 2 下发与状态回传。

- ROS 2 发布 `/motor/target_rpm`
- ESP32 回传 `/motor/actual_rpm`
- ESP32 回传 `/motor/state`
- 使用 `rqt_plot` / 日志验证响应曲线

### Phase 5: Buy Second Matching N20 and Extend Differential Drive

目标：单 N20 稳定后扩展到双电机。

- 购买第二个同规格 N20
- 把 TB6612 A 通道从 130 普通电机切换为第二个 N20
- 为第二个 N20 接入编码器 A/B
- 左右轮分别计算 rpm
- 左右轮分别执行 PID

### Phase 6: `/cmd_vel` to Left / Right RPM

目标：加入差速运动学。

- 引入 `wheel_radius`
- 引入 `wheel_base`
- 计算 left/right target rpm
- 对左右轮目标做限幅

### Phase 7: `ros2_control diff_drive_controller` Planning

目标：规划 `ros2_control` 对齐。

- velocity command interface
- velocity state interface
- encoder-derived wheel state
- 后续 `/odom` 推导
- `diff_drive_controller` 接入方式评估

这一阶段不应提前开始，必须等单电机和双电机底层闭环都能稳定复现。

### Phase 8: AMR Simulation / Dashboard / Ops Integration

目标：与既有 AMR 仿真、dashboard 和运维链路合流。

- 与 AMR 仿真中的控制抽象对齐
- 将 `motor_state` 镜像到 dashboard
- 用 MQTT 做低频 health / state 展示
- 保持 ROS 2 为控制和机器人状态主链路

## 11. Test Strategy

### 11.1 Software-Only Test

- timeout 状态机
- PID 输出计算
- PWM 限幅
- anti-windup
- rpm 计算函数
- 方向切换逻辑

### 11.2 Board-Level Test

- PWM 引脚波形
- DIR 引脚状态
- `target_rpm` 接收
- `motor_state` 发布
- timeout 后 PWM 归零
- ESTOP 后 PWM 归零

### 11.3 Hardware-In-The-Loop Test

- 开环正反转
- PWM-speed 关系
- 编码器计数方向
- rpm 计算
- 单电机 P 控制
- 单电机 PI 控制
- target timeout 实际停车
- ESTOP 实际停车

## 12. Summary

当前电机控制路线是：保留 STM32 既有 open-loop 电机验证代码作为 legacy experiment，把新的 N20 编码器电机闭环主线迁移到 ESP32。

当前 TB6612 A 通道保留给 130 普通电机做辅助验证，TB6612 B 通道承担单 N20 桌面闭环主线：`target_rpm -> ESP32 local PID -> PWM/DIR -> TB6612 B -> N20`，再把 `actual_rpm` / `motor_state` 回传 ROS 2。单 N20 闭环稳定后，再购买第二个同规格 N20，把 A 通道从 130 切换为 N20，扩展双轮差速、`/cmd_vel` 和 `ros2_control diff_drive_controller`。
