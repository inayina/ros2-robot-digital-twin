# Robot-State-Monitor (RSM) v2

基于嵌入式边缘计算与 ROS 2 数字孪生的无线状态监测系统。  
本仓库当前是 **STM32F411 主控固件工程**，用于完成采样、特征提取、状态判别和本地报警；已与 ESP32 micro-ROS 网桥联调成功。

## 项目目标

按照 `design.md` 的定义，系统最终由三部分组成：

1. STM32（Master）：100Hz 采样、滤波/推理、本地报警
2. ESP32（Bridge）：串口收包 + Wi-Fi + micro-ROS 发布
3. PC（Digital Twin）：micro-ROS Agent、数据录制、Gazebo 同步

当前代码处于 **STM32 + ESP32 micro-ROS 桥接已联调成功** 阶段。

## 当前实现状态（以代码为准）

已完成：

1. FreeRTOS 三任务链路
   - `SensorTask`：读取 MPU6050，打包 `SensorData_t`，写入 `SensorQueue`
   - `AlgTask`：窗口特征提取（RMS），本地更新报警灯，并通过 USART1 输出 `State:x`
   - `DefaultTask`：翻转 PC13/`DEBUG_LED`，用于判断系统是否仍在运行
2. LED 状态报警（TIM2 PWM，三色 LED）
   - 绿色 (TIM_CHANNEL_1)：正常状态常亮
   - 黄色 (TIM_CHANNEL_3)：警告状态闪烁
   - 红色 (TIM_CHANNEL_2)：警报闪烁，严重常亮
3. 串口交互与双模式输出
   - `GAZEBO`：输出 `IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp`
   - `TRAIN`：输出 `timestamp + 6轴 + label` 的 CSV
4. 启动期 I2C 设备探测
   - 启动时打印 I2C 状态和 MPU6050 `WHO_AM_I`
5. **Gazebo 数字孪生桥接**
   - 订阅 `/imu/filtered` 话题
   - 通过 Gazebo 服务更新模型姿态
   - 实现机器人运动的可视化同步
6. **micro-ROS 版本匹配**
   - ESP32 和 Agent 均使用 jazzy 版本

未完成（设计目标已定义）：

1. AHT20/BMP280 接入
2. XGBoost 嵌入式推理替换当前 RMS 阈值基线
3. 更完整的数据集采集、训练和评估流程

## 代码结构

```text
Core/
  Src/                 # CubeMX 生成外设、时钟、FreeRTOS初始化
User/
  App/
    sensor_task.c      # 传感器采样、串口命令、数据输出
    algo_task.c        # 窗口特征提取 + 状态判别
    attitude_estimator.c # 6轴姿态解算，输出四元数
  Bsp/
    mpu6050.c          # MPU6050 驱动
    led_alarm.c        # TIM2 PWM 报警灯控制
design.md              # 项目总体设计白皮书
```

## ESP32 micro-ROS 桥接项目

位于 `../esp32_microros_bridge/` 目录，使用 PlatformIO + Arduino 框架。

### 主要组件
- `src/main.cpp`：Wi-Fi 连接、UART 接收 STM32 数据、micro-ROS 初始化和发布
- `platformio.ini`：ESP32-S3 配置，启用 USB CDC
- `Debug.md`：问题排查日志

### 硬件连接
- ESP32 UART (GPIO16 RX, GPIO17 TX) 连接 STM32 USART1
- ESP32 Wi-Fi 连接到 micro-ROS Agent (192.168.1.8:8888)

### 运行步骤
1. 启动 Agent：`./build/micro_ros_agent/micro_ros_agent udp4 --port 8888`
2. 上传 ESP32：`pio run --target upload`
3. 查看话题：`ros2 topic list` 显示 `/imu/data` 和 `/robot/state`

## 数据结构

`SensorData_t`（原始/预处理输入）：

- 加速度 `acc[3]`（m/s²）
- 角速度 `gyro[3]`（°/s）
- 姿态四元数 `quat[4]`（x, y, z, w）
- `mpu_temp`
- 预留：`env_temp`、`env_humid`、`pressure`
- `timestamp`、`sensor_status[4]`

`FeatureData_t`（算法输出）：

- `anomaly_state`：0 正常 / 1 震动 / 2 碰撞 / 3 倾倒
- `confidence`：当前为固定占位值
- `filtered_rms`：窗口特征值
- `process_time`：当前未填充

## 串口命令（USART1）

波特率：`921600`

- `T`：切到训练输出模式
- `G`：切到 Gazebo 输出模式
- `0~3`：设置训练标签
- `S`：查看当前模式和标签
- `?` / `h`：帮助

## 姿态解算与 CubeMX 注意事项

当前姿态解算放在 `User/App/attitude_estimator.c`，由 `SensorTask` 每次采样后调用。它不是新的 FreeRTOS 任务，只是普通 C 算法模块。

为什么不放在 `algo_task.c`：

1. 姿态解算需要逐帧积分陀螺仪，依赖稳定 `dt`
2. `sensor_task.c` 最靠近 MPU6050 采样点，时间戳最直接
3. `algo_task.c` 当前是窗口 RMS/状态判断任务，攒 10 帧后处理，不适合连续姿态积分
4. 让 `algo_task.c` 专注状态识别，姿态解算单独成模块，后续更容易替换 Mahony/Madgwick 参数

当前 MPU6050 只有加速度计和陀螺仪，没有磁力计。因此这是 6 轴姿态解算：

- roll/pitch 会被重力方向修正
- yaw 只能靠陀螺仪积分，长时间会漂移
- 输出四元数用于 ROS `sensor_msgs/Imu.orientation`

### CubeMX 必改项

姿态解算和 `IMUQ` 串口输出会增加 `SensorTask` 的栈压力，尤其是 `snprintf` 浮点格式化很吃栈。不要直接手改 `Core/Src/freertos.c`，否则 CubeMX 重新生成后可能被覆盖。

请在 CubeMX 中手动修改：

```text
Middleware and Software Packs
  -> FREERTOS
  -> Tasks and Queues
  -> SensorTask
  -> Stack Size
```

建议值：

```text
1024 words  最小建议值，对应生成代码 .stack_size = 1024 * 4
2048 words  更稳，调试阶段推荐
```

当前真正的 micro-ROS 节点运行在 ESP32，STM32 不再保留旧的 `MicroROSTask` 占位任务，也不再创建 `FeatureQueue`。STM32 只负责采样、姿态解算、状态判断和串口输出，ROS 发布交给 ESP32。

推荐任务栈配置：

```text
SensorTask    1024 words  姿态解算 + IMUQ 串口输出，优先保证
AlgTask        512 words  RMS/状态判断，当前够用
DefaultTask    128 words  默认保持
```

CubeMX 中对应只保留 `SensorTask`、`AlgTask`、`DefaultTask` 和 `SensorQueue`。如果以后重新生成代码，确认不要把旧的 `MicroROSTask` / `FeatureQueue` 加回来。

如果上电后 `DEBUG_LED` / PC13 不闪，或者串口没有 `MPU6050 OK`、没有 IMU 数据输出，优先检查：

1. `SensorTask` 栈是否仍是 `256 words`
2. CubeMX 重新生成后 `Core/Src/freertos.c` 是否又变回 `.stack_size = 256 * 4`
3. 是否已经重新编译并烧录 STM32
4. MPU6050 I2C 是否正常，启动日志是否有 `MPU6050 found`

### 新串口输出格式

Gazebo/ROS 模式下，STM32 输出带姿态四元数的 `IMUQ` 行：

```text
IMUQ,ax,ay,az,gx,gy,gz,qx,qy,qz,qw,temp
```

ESP32 `esp32_microros_bridge` 已兼容该格式，并会把 `qx,qy,qz,qw` 写入 ROS `/imu/data` 和 `/imu/filtered` 的 `orientation` 字段。旧格式 `IMU,ax,ay,az,gx,gy,gz,temp` 仍可兼容，但旧格式没有真实姿态，只会使用单位四元数占位。

## 硬件连接（当前 STM32 部分）

1. `I2C1`（PB6/PB7）连接 MPU6050（地址 `0x68`）
2. `USART1`（PA9/PA10）用于日志与后续 ESP32 通讯
3. `TIM2_CH2`（PA1）驱动报警 LED PWM
4. 调试 LED：`DEBUG_LED`

提示：

1. I2C 引脚当前为 `GPIO_NOPULL`，需确认外部上拉有效
2. MPU6050 的 AD0 建议固定，避免地址漂移

## 构建与烧录

### 依赖

1. `arm-none-eabi-gcc`
2. `cmake >= 3.22`
3. `ninja`
4. **ESP32 项目**: PlatformIO, Python 3.8+

### STM32 编译

```bash
cmake --preset Debug
cmake --build --preset Debug
```

输出文件位于：

- `build/Debug/robot-state-monitor.elf`
- `build/Debug/robot-state-monitor.map`

烧录可使用 STM32CubeIDE / OpenOCD / ST-Link 工具链。

### ESP32 编译

```bash
cd ../esp32_microros_bridge
pio run
pio run --target upload
```

## 运行验证（最小闭环）

1. 上电后串口应看到 I2C 探测结果和 `MPU6050 found`
2. 持续打印 `State:x RMS:y`（算法任务）
3. 输入 `S` 可回显当前模式
4. 切换 `T/G` 后输出格式变化
5. 运动状态变化时 LED 亮度随 `anomaly_state` 变化
6. **ESP32 桥接**: 连接 Wi-Fi 后发布 ROS 话题

## 与 design.md 的对应关系

1. 已落地：STM32 采样 + 算法 + 报警主链路
2. 已落地：ESP32 Wi-Fi + micro-ROS 桥接
3. 已落地：ESP32 micro-ROS 网桥、PC Agent、ROS 2 话题和 Gazebo 可视化闭环
4. 已落地：训练数据输出接口（串口 CSV）
5. 已新增：PC 端 `imu_data_logger` 数据记录和实时分析节点

## Roadmap（建议）

1. Phase 1：接入 ESP32 串口解析与 UDP/Wi-Fi micro-ROS 发布（已完成）
2. Phase 2：发布 `/imu/data` 与 `/imu/filtered`，并写入姿态四元数（已完成）
3. Phase 3：将 `sensor_status/process_time` 真实化，完善错误处理
4. Phase 4：部署 XGBoost C 推理代码，替换 RMS 基线
5. Phase 5：完善 rosbag/CSV 数据集采集和算法评估
