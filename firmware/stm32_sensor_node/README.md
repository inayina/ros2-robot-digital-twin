# STM32 Sensor Node

基于嵌入式边缘计算与 ROS 2 数字孪生的无线状态监测系统。  
本目录是 **STM32F411 主控固件工程**，用于完成采样、特征提取、状态判别和本地报警；V1 已与 ESP32 micro-ROS 网桥、ROS 2 Jazzy 和 Gazebo Harmonic 联调成功。

## 项目目标

按照 `design.md` 的定义，系统最终由三部分组成：

1. STM32（Master）：100Hz 采样、滤波/推理、本地报警
2. ESP32（Bridge）：串口收包 + Wi-Fi + micro-ROS 发布
3. PC（Digital Twin）：micro-ROS Agent、数据录制、Gazebo 同步

当前代码处于 **STM32 + ESP32 micro-ROS 桥接已联调成功** 阶段。

## 当前实现状态

已完成：

1. FreeRTOS 三任务链路
   - `SensorTask`：读取 MPU6050，打包 `SensorData_t`，写入 `SensorQueue`
   - `AlgTask`：窗口特征提取（RMS），输出 `FeatureData_t` 到 `FeatureQueue`
   - `MicroROSTask`：消费特征并串口打印发布占位信息
2. LED 状态报警（TIM2 PWM，三色 LED）
   - 绿色 (TIM_CHANNEL_1)：正常状态常亮
   - 黄色 (TIM_CHANNEL_3)：警告状态闪烁
   - 红色 (TIM_CHANNEL_2)：警报闪烁，严重常亮
3. 串口交互与双模式输出
   - `GAZEBO`：输出 `IMU,ax,ay,az,gx,gy,gz,temp`
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
3. 将更完整的滤波/姿态估计下沉到嵌入式侧
4. rosbag 数据集采集和算法评估流程

## 代码结构

```text
Core/
  Src/                 # CubeMX 生成外设、时钟、FreeRTOS初始化
User/
  App/
    sensor_task.c      # 传感器采样、串口命令、数据输出
    algo_task.c        # 窗口特征提取 + 状态判别
    ros_bridge.c       # micro-ROS 发布占位任务
  Bsp/
    mpu6050.c          # MPU6050 驱动
    led_alarm.c        # TIM2 PWM 报警灯控制
design.md              # 项目总体设计白皮书
```

## ESP32 micro-ROS 桥接项目

位于 `../microros_node/` 目录，使用 PlatformIO + Arduino 框架。

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
cd ../microros_node
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
3. 已落地：ROS 2 话题接入与 Gazebo 姿态可视化
4. 部分落地：训练数据输出接口（串口 CSV）
5. 待增强：算法模型、数据集采集和环境传感器接入

## Roadmap（建议）

1. Phase 1：接入 ESP32 串口解析与 UDP/Wi-Fi micro-ROS 发布（已完成）
2. Phase 2：发布 `/imu/data`、`/imu/filtered`、`/robot/state`（已完成）
3. Phase 3：完成 Gazebo 姿态可视化闭环（已完成）
4. Phase 4：将 `sensor_status/process_time` 真实化，完善错误处理
5. Phase 5：部署 XGBoost C 推理代码，替换 RMS 基线
6. Phase 6：完成 rosbag 数据集采集和算法评估
