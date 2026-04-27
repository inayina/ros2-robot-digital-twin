# 🤖 Project: Robot-State-Monitor 
**文档基准：** 以当前仓库代码状态（2026-04-28）为准  
**项目定位：** 基于嵌入式边缘计算与 ROS 2 数字孪生的无线监测系统  
**核心环境：** P14s (Ubuntu 24.04 + ROS 2 Jazzy Native)  
**硬件组合：** STM32F411CEU6 (计算与执行核心) + ESP32-S3 (无线 micro-ROS 网桥)

---

## 1. 系统架构 (System Architecture)

当前系统已经形成稳定的**上行观测链路**和**下行电机控制链路**：

```text
上行（已实现）
MPU6050 / 状态推理
    -> STM32 SensorTask / AlgTask
    -> USART1 ASCII 文本帧
    -> ESP32 串口解析
    -> micro-ROS over Wi-Fi UDP
    -> micro-ROS Agent
    -> ROS 2 topics: /imu/data /imu/filtered /robot/state

下行（已实现，单电机 A 路已联调）
rqt_robot_steering / ros2 topic pub
    -> ROS 2 topic: /cmd_vel
    -> ESP32 micro-ROS subscriber
    -> USART1 CMDVEL 控制帧
    -> STM32 MotorTask
    -> TIM3_CH1 + AIN1/AIN2 + STBY
    -> TB6612 A 路
    -> 电机物理转动已确认
```

系统角色分工如下：

1. **STM32 (Master)**：负责 100 Hz 传感器采集、6 轴姿态解算、状态判别、本地 LED 报警，以及电机执行控制。
2. **ESP32 (Bridge)**：负责串口桥接、micro-ROS 建链、ROS 2 上行发布，以及 `/cmd_vel` 下行订阅转发。
3. **PC (Digital Twin + HMI)**：负责运行 micro-ROS Agent、Gazebo / RViz 可视化，以及通过 `rqt` 或命令行下发人工控制指令。

---

## 2. 硬件资源与连接 (Hardware Map)

### 2.1 当前物理连接
* **I2C1 (STM32)**：连接 MPU6050。
* **UART1 (STM32 ↔ ESP32)**：
  * 波特率：**921600 bps**
  * 连线：STM32 TX/PA9 -> ESP32 RX/GPIO16；STM32 RX/PA10 -> ESP32 TX/GPIO17；**必须共地 (GND)**
* **ESP32 Wi-Fi**：连接到 PC 所在局域网，通过 UDP 接入 micro-ROS Agent。

### 2.2 当前外设分配
| 外设 | 关键配置 | 当前用途 |
| :--- | :--- | :--- |
| **RCC** | HSE -> 100 MHz HCLK | 保证姿态解算和算法任务实时性 |
| **USART1** | 921600, 8N1 | STM32 与 ESP32 双向数据链路 |
| **TIM2 PWM** | 已启用 | 本地状态报警 LED |
| **SYS / SWD** | Serial Wire | 调试与下载 |

### 2.3 电机控制接口现状（TB6612 方案）
电机链路当前已经直接挂在 STM32。运动学映射、超时停车和 `STBY` 安全逻辑都在 STM32 执行层，不放在 ESP32。  
当前代码已经落地 **TB6612 A 路单电机**，同时为后续双电机差速保留接口：

* **2 路 PWM**：`PWMA`、`PWMB`
* **4 路方向 GPIO**：`AIN1`、`AIN2`、`BIN1`、`BIN2`
* **1 路使能 GPIO**：`STBY`

结合当前 `robot-state-monitor.ioc`，推荐一版不和现有资源冲突的分配：

| TB6612 引脚 | STM32 建议引脚 | CubeMX 建议功能 |
| :--- | :--- | :--- |
| `PWMA` | `PA6` | `TIM3_CH1` PWM |
| `PWMB` | `PA7` | `TIM3_CH2` PWM |
| `AIN1` | `PB0` | GPIO Output |
| `AIN2` | `PB1` | GPIO Output |
| `BIN1` | `PB12` | GPIO Output |
| `BIN2` | `PB13` | GPIO Output |
| `STBY` | `PB14` | GPIO Output |

当前实际已落地的是：

* 已启用 **A 路**：`PWMA + AIN1 + AIN2 + STBY`
* `PWMA` 接 `PA6 / TIM3_CH1`
* `AIN1/AIN2` 接 `PB0/PB1`
* `STBY` 接 `PB14`
* **B 路仍预留未启用**：`PA7 / PB12 / PB13`

说明：

1. **TIM2 已经被 LED 报警占用**，不要再拿来做电机 PWM。
2. **TIM3_CH1 当前已经完成 PWM 落地**，并且已经用于 TB6612 A 路；`CH2` 仍可继续作为后续双电机扩展位。
3. 现在只调一路时，不必强行把 `PWMB/BIN1/BIN2` 也一起打开；保持预留即可。
4. 如果你实物接线已经焊好，也可以换成别的空闲 PWM/GPIO，只要保留“2 路 PWM + 4 路 DIR + 1 路 STBY”这个结构即可。

---

## 3. 软件逻辑与任务规划 (Task Management)

### 3.1 STM32 端：当前真实任务结构

`CommsTask` 已删除，当前串口发送不再由独立任务承担，而是直接内嵌到业务任务中。

| 模块 / 任务 | 状态 | 作用 |
| :--- | :--- | :--- |
| **SensorTask** | 已实现 | 100 Hz 读取 MPU6050，完成姿态解算，输出 `IMUQ,...` 或训练 CSV，并把 `SensorData_t` 写入 `SensorQueue` |
| **AlgTask** | 已实现 | 从 `SensorQueue` 取窗口数据，做 RMS 状态判别，输出 `State:x`，同时更新 LED 报警 |
| **MotorTask** | 已实现 | 每 `10 ms` 消费最新电机命令，做超时急停、`STBY` 使能、方向控制和 `TIM3_CH1` PWM 输出 |
| **DefaultTask** | 已实现 | 心跳任务，翻转调试 LED，便于判断系统是否卡死 |
| **HAL_UART_RxCpltCallback + 行缓冲解析** | 已实现 | 当前同时处理 `T/G/0~3/S/?` 单字节命令，以及 `CMDVEL,...` / `ESTOP` 文本下行控制帧 |

### 3.2 STM32 端：电机控制当前实现

当前驱动器是 **TB6612FNG**，并且已经按“单个 `MotorTask`”方案落地，不再是规划状态。

| 模块 / 任务 | 状态 | 当前职责 |
| :--- | :--- | :--- |
| **MotorTask** | 已实现 | 读取 `g_motor_linear_x / g_motor_angular_z / g_motor_cmd_recv_tick_ms / g_motor_estop`，做限幅、TB6612 方向控制、PWM 输出和超时急停 |

当前版本就是单电机执行基线：

- 只驱动 **A 路**
- **B 路** 固定为关闭并保持预留
- `STBY` 作为整颗 TB6612 的总使能

配套逻辑建议保持轻量：

1. **不要新增 `CommsTask`**
2. **不要新增专门的 `ParserTask`**
3. USART1 接收仍然放在 `HAL_UART_RxCpltCallback` 或行缓冲解析函数里
4. 串口解析完成后，只更新一份最新电机命令，由 `MotorTask` 周期性消费

推荐优先级关系：

```text
SensorTask   >   MotorTask   >   AlgTask   >=   DefaultTask
```

原因：

1. 采样仍然是全系统时基源。
2. 电机控制需要比状态推理更快地响应人工控制。
3. `AlgTask` 允许窗口化处理，实时性要求低于电机执行。

### 3.3 ESP32 端：当前实现

当前 ESP32 侧已经完成下行桥接，不再是“后续规划”：

1. `main.cpp` 负责 Wi-Fi 连接、UDP transport 配置、micro-ROS 初始化与主循环。
2. `uros_sub` 已实现 `/cmd_vel` subscriber。
3. `rclc_executor_spin_some()` 已在运行态调度。
4. `/cmd_vel` 会被编码成 `CMDVEL,<linear_x>,<angular_z>\n` 通过 UART 下发给 STM32。

ESP32 当前仍然只做**桥接和协议转换**，不把差速解算、急停、失联保护等安全逻辑放到桥上。

---

## 4. ROS 2 接口定义 (ROS Interface)

### 4.1 当前已实现话题
| 方向 | 话题 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| 上行 | `/imu/data` | `sensor_msgs/msg/Imu` | STM32 `IMUQ` 数据，经 ESP32 发布 |
| 上行 | `/imu/filtered` | `sensor_msgs/msg/Imu` | ESP32 对线加速度和角速度做一阶低通滤波后的结果 |
| 上行 | `/robot/state` | `std_msgs/msg/Int32` | `AlgTask` 输出的状态分类结果 |

### 4.2 当前下行与扩展话题
| 方向 | 话题 | 类型 | 状态 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 下行 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 已实现 | 已用于 `rqt_robot_steering` 或 `ros2 topic pub` 下发控制命令 |
| 上行 | `/motor/state` | `std_msgs/msg/Int32` 或自定义消息 | 可选 | 电机使能、故障、急停等状态反馈 |

为什么主链路推荐 `/cmd_vel`：

1. `rqt_robot_steering` 可直接使用，不需要自定义 GUI。
2. ROS 2 生态里更通用，后续换手柄、导航栈、遥操作节点都能复用。
3. 运动学和电机映射仍可放在 STM32 内部实现，不会把桥接层做重。

---

## 5. 串口协议定义 (UART Protocol)

### 5.1 当前 STM32 -> ESP32 上行协议

1. **Gazebo / ROS 模式**
   * `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp\n`
2. **状态输出**
   * `State:<n>\n`
3. **训练模式**
   * `timestamp,ax,ay,az,gx,gy,gz,label\n`

说明：

* 当前 ESP32 默认只把 `IMUQ` / `State:` 解析成 ROS 2 话题。
* `TRAIN` CSV 在 STM32 端仍然保留，但目前 `microros_node` 中 `ACCEPT_TRAIN_CSV_AS_IMU = false`，所以它**还不是正式 ROS 2 训练链路**。

### 5.2 当前 ESP32 -> STM32 下行协议

当前已经沿用 **ASCII 文本行协议**，便于调试与串口直观观察。

推荐定义：

```text
CMDVEL,<linear_x>,<angular_z>\n
ESTOP\n
```

示例：

```text
CMDVEL,0.20,0.00
CMDVEL,0.00,1.20
ESTOP
```

当前职责划分：

1. **ROS / rqt** 负责产生命令：`/cmd_vel`
2. **ESP32** 负责把 `Twist.linear.x` 与 `Twist.angular.z` 编码成 `CMDVEL`
3. **STM32** 负责把 `CMDVEL` 换算成左右轮 PWM / 单电机目标值

当前代码里 `ESTOP` 文本命令的 STM32 解析入口已经存在，但 ESP32 侧还没有单独暴露 ROS `ESTOP` 话题。  
如果后续只是做底层驱动调试，也可以额外预留一个非主链路命令：

```text
CMDPWM,<left_pwm>,<right_pwm>\n
```

但这更适合作为调试口，不建议取代 `/cmd_vel` 主控制链路。

---

## 6. 下行电机控制链路现状 (rqt -> Motor)

### 6.1 当前主链路

```text
rqt_robot_steering
    -> /cmd_vel (geometry_msgs/Twist)
    -> ESP32 subscriber (reliable QoS)
    -> UART: CMDVEL,vx,wz
    -> STM32 UART Rx line parser
    -> latest g_motor_* command state
    -> MotorTask
    -> TIM3_CH1 + AIN1/AIN2 + STBY
    -> TB6612 A 路
    -> 电机物理转动已确认
```

### 6.2 STM32 侧当前命令状态

```c
volatile float g_motor_linear_x;
volatile float g_motor_angular_z;
volatile uint32_t g_motor_cmd_recv_tick_ms;
volatile uint8_t g_motor_estop;
```

### 6.3 `MotorTask` 当前行为

当前实现保留了 `MotorTask` 的集中执行思路：

1. 周期性检查 **超时停车逻辑**
2. 在任务内做 **限幅和速度到 PWM 的映射**
3. 在任务内统一处理 **ESTOP / STBY**

推荐执行节拍：

* `MotorTask` 周期：**10 ms**
* `cmd_vel` 超时急停：**200 ms**
* 上电默认：`STBY=0`、`PWMA=0`

当前单电机版本的执行策略是：

* `MotorTask` 只更新 `AIN1/AIN2 + PWMA`
* `PA7 / BIN1 / BIN2` 保持预留未使用
* `STBY` 仍然作为整颗 TB6612 的总使能

任务内部建议流程：

1. 读取最新命令状态
2. 若 `estop=1` 或超时，则立即拉低 `STBY`
3. 若命令有效，则先拉高 `STBY`
4. 将 `linear_x` 优先映射成目标值；若线速度接近 0，则退化使用 `angular_z`
5. 输出 `AIN1/AIN2 + PWMA`

如果当前只是单电机首版：

1. 第 4 步可以先简化成“把目标速度映射成 A 路 PWM”
2. 第 5 步只输出 `AIN1/AIN2 + PWMA`
3. `BIN1/BIN2/PWMB` 保持关闭即可

### 6.4 TB6612 控制约定

首版建议在文档和代码里统一以下约定：

* **前进**：方向脚设置为正转组合，PWM 输出占空比
* **后退**：方向脚设置为反转组合，PWM 输出占空比
* **停车**：`PWMA/PWMB = 0`
* **急停 / 失联**：`STBY = 0`

这样做的好处是：

1. 平时停车和急停语义分开，调试更清楚。
2. `STBY=0` 可以作为总保险，任何异常都能一键断输出。
3. 后续如果要加编码器闭环，也不用改下位执行层的总结构。

### 6.5 CubeMX / MX 修改步骤

下面这套步骤是按你当前工程状态写的。当前代码已经按**单电机 A 路**落地，同时为 B 路保留扩展位。

#### 6.5.1 Pinout 里怎么改

1. 保持现有外设不动：
   * `USART1`: `PA9 / PA10`
   * `TIM2_CH1/2/3`: `PA0 / PA1 / PA2`，继续给 LED 用
   * `I2C1`: `PB6 / PB7`
2. 打开 `TIM3`
3. 把 `TIM3` 配成：
   * 当前先启用 `PWM Generation CH1` -> `PA6`
   * `PWM Generation CH2` -> `PA7` 作为后续双电机扩展预留
4. 当前至少新增 3 个 GPIO Output：
   * `PB0` -> `AIN1`
   * `PB1` -> `AIN2`
   * `PB14` -> `STBY`
5. 如果你想一次把未来小车接口也占好，可以额外预留：
   * `PB12` -> `BIN1`
   * `PB13` -> `BIN2`
6. 这些 GPIO 的默认输出建议都设成 **Low**

#### 6.5.2 TIM3 里怎么改

在 `Pinout & Configuration -> Timers -> TIM3` 中：

1. 使能 `PWM Generation CH1`
2. 当前首版可以先不启用 `PWM Generation CH2`；如果你想提前把未来接口配好，也可以一并开出来但保持占空比为 0
3. 推荐参数：

```text
Prescaler = 4
Counter Period = 999
Pulse CH1 = 0
Pulse CH2 = 0   (仅在启用 CH2 预留时配置)
Auto Reload Preload = Enable
```

按当前 100 MHz 定时器时钟，这样约等于 **20 kHz PWM**，更适合 TB6612，也能尽量避开电机啸叫声。

#### 6.5.3 GPIO 里怎么改

在 `System Core -> GPIO` 中检查：

1. 当前至少保证 `AIN1/AIN2/STBY` 是 `GPIO_Output`
2. 如果你决定把未来小车接口一并占好，再把 `BIN1/BIN2` 也配成 `GPIO_Output`
3. `GPIO output level` 默认设为 `Low`
4. 输出速度保持 `Low` 或 `Medium` 即可，首版不用拉太高

推荐当前单电机版本上电默认值：

```text
AIN1 = 0
AIN2 = 0
STBY = 0
```

如果你同时把 B 路也预留出来，则继续保持：

```text
BIN1 = 0
BIN2 = 0
```

这样上电不会误转。

#### 6.5.4 NVIC 里怎么改

如果 `TIM3 global interrupt` 目前是开启的，首版**可以关掉**，因为这里只把 `TIM3` 当普通 PWM 用，不需要定时器中断。

保留：

* `USART1_IRQn`
* 现有 FreeRTOS 相关中断

#### 6.5.5 FreeRTOS 里怎么改

在 `Middleware and Software Packs -> FREERTOS -> Tasks and Queues` 中：

1. 新增一个任务：`MotorTask`
2. 建议参数：

```text
Task Name   : MotorTask
Priority    : osPriorityAboveNormal
Stack Size  : 512 words
```

建议关系：

```text
SensorTask  >  MotorTask  >  AlgTask  >=  DefaultTask
```

首版**不必**在 CubeMX 里新增 `CommsTask`、`ParserTask` 或额外定时器任务。

如果你想用消息队列，也可以额外加一个：

```text
Queue Name  : MotorCmdQueue
Length      : 1
Item Size   : sizeof(MotorCmd_t)
```

但如果你准备先做最小实现，也可以不在 MX 里建队列，而是在用户代码里维护一份最新命令结构体。

#### 6.5.6 生成代码后要检查什么

CubeMX 生成完成后，优先检查这几项：

1. 当前单电机首版至少要确认 `tim.c` 里真的生成了 `TIM3_CH1`
2. 如果你一并预留了第二路，再确认 `TIM3_CH2` 也存在
3. `gpio.c` 里是否生成了 `AIN1/AIN2/STBY` 输出初始化
4. 如果你占了未来接口，再检查 `BIN1/BIN2` 是否也正确生成
5. `freertos.c` 里是否生成了 `MotorTask`
6. `TIM2` 的 LED 通道有没有被误改
7. `USART1` 的 `PA9/PA10` 有没有被别的功能抢走

### 6.6 为什么不建议恢复 `CommsTask`

原因不是“不能有通信逻辑”，而是当前系统更适合把链路拆成三层：

1. **STM32 业务任务产生或消费数据**
2. **UART Rx/Tx 负责轻量协议搬运**
3. **ESP32 只做 ROS 桥接**

这样比单独维护一个“大而全的 `CommsTask`”更清晰，也更符合现在代码已经形成的结构。

---

## 7. PC 端运行方式 (Host Setup)

### 7.1 当前上行链路
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
ros2 topic echo /imu/data
ros2 topic echo /robot/state
```

### 7.2 当前下行链路
```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

推荐配置：

* topic：`/cmd_vel`
* 类型：`geometry_msgs/msg/Twist`

如果只想做最小联调，也可以直接用命令行：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

---

## 8. 开发 Roadmap (更新版)

1. **保持当前最小闭环稳定**：继续使用 `/imu/data`、`/imu/filtered`、`/robot/state` 和 `/cmd_vel` 完成双向联调。
2. **补执行反馈闭环**：按需要新增 `/motor/state`，把使能、急停、故障或目标值状态发布出来。
3. **扩展 B 路与双电机差速**：在保留 A 路基线稳定的前提下，启用 `PA7 / PB12 / PB13`。
4. **补更强的执行层观测**：如编码器、电流、故障脚或示波器验证。
5. **继续数字孪生和数据集工作流**：在现有双向链路稳定后再迭代 Gazebo / 录包 / 训练数据链路。

---

## 9. 避坑指南 (Critical Notes)

* **`CommsTask` 已不再是当前架构的一部分**：文档、代码、CubeMX 任务规划都应以 `SensorTask / AlgTask / DefaultTask` 为基准。
* **上行与下行 QoS 不同**：IMU / 状态可继续 `best effort`；`/cmd_vel` 建议使用 `reliable`。
* **下行链路一定要做超时停车**：Wi-Fi、Agent、`rqt` 任一环节中断时，电机必须自动归零。
* **TB6612 首版建议用 `TIM3` 做 PWM**：不要动已经给 LED 用的 `TIM2`。
* **上电默认先拉低 `STBY`**：等收到第一条有效命令后再使能驱动，能明显减少误动作风险。
* **首版协议先用文本帧**：当前串口链路已经是文本行解析，先跑通 `CMDVEL`，后续再考虑 CRC / 二进制协议。
* **训练模式不要和当前 Gazebo 链路混为一谈**：STM32 虽然保留 `TRAIN` CSV 输出，但 ESP32 默认并未把它接成正式 ROS 训练数据流。
