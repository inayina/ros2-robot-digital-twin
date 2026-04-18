# Project: Robot-State-Monitor (RSM) V2
**项目定位：** 基于嵌入式边缘计算与 ROS 2 数字孪生的无线监测系统
**核心环境：** P14s (Ubuntu 24.04 + ROS 2 Jazzy Native)
**硬件组合：** STM32F411CEU6 (计算核心) + ESP32 (无线网桥)

---

## 1. 系统架构 (System Architecture)

系统由**感知推理、无线传输、数字孪生**三个维度构成：

1.  **STM32 (Master)**：负责 100Hz MPU6050 采样、6 轴姿态解算、RMS 状态判别及本地 LED 报警。
2.  **ESP32 (Bridge)**：通过串口接收 STM32 `IMUQ` / `State` 帧，利用 micro-ROS 将数据通过 Wi-Fi 发布至 PC 端。
3.  **PC (Digital Twin)**：运行原生 micro-ROS Agent、Gazebo 数字孪生、数据记录和实时曲线分析节点。

---

## 2. 硬件资源与连接 (Hardware Map)

### 2.1 物理连接
* **I2C1 (STM32)**：连接 MPU6050 (IMU)、AHT20 (温湿度)、BMP280 (气压)。
* **UART1 (STM32 ↔ ESP32)**：
    * 波特率：**921600 bps** (支持高频数据流)。
    * 连线：STM32(TX/PA9) -> ESP32(RX/GPIO16)；STM32(RX/PA10) -> ESP32(TX/GPIO17)；**必须共地 (GND)**。
* **电源**：ESP32 开启 Wi-Fi 时电流较大，建议通过 Type-C 独立供电。

### 2.2 外设参数
| 外设 | 接口 | 关键配置 | 作用 |
| :--- | :--- | :--- | :--- |
| **RCC** | HSE | 25MHz -> 100MHz HCLK | 核心主频，保证 FPU 推理性能 |
| **TIM2** | PA1 | PWM Mode (1KHz) | 状态指示灯，亮度映射状态 |
| **SYS** | SWD | Serial Wire | 调试与下载 |

---

## 3. 软件逻辑与任务规划 (Task Management)

### 3.1 STM32 端 (FreeRTOS)
| 任务名称 | 优先级 | 功能描述 |
| :--- | :--- | :--- |
| **SensorTask** | 48 | 100Hz 采集 MPU6050，调用 `attitude_estimator` 解算四元数，并在 Gazebo/ROS 模式输出 `IMUQ`。 |
| **AlgTask** | 32 | 基于滑动窗口 RMS 做状态判别，更新 LED 报警并输出 `State:x`。 |
| **DefaultTask** | 24 | 翻转调试 LED，用于确认系统仍在运行。 |

### 3.2 ESP32 端 (PlatformIO/Arduino)
* **任务**：解析串口帧，通过 `micro_ros_arduino` 发布话题。
* **网络**：Station 模式连接 Wi-Fi，目标 IP 指向 P14s。

---

## 4. 数据双轨制逻辑 (Data Pipeline)

为了同时兼顾“模型训练”与“仿真稳定性”，数据在发送前进行分流：

1.  **原始轨 (Raw Track)**：
    * **数据内容**：STM32 输出的加速度、角速度和四元数姿态。
    * **目的**：ESP32 发布至 `/imu/data`，供 `imu_data_logger` 记录和后续训练分析。
2.  **滤波轨 (Filtered Track)**：
    * **数据内容**：ESP32 对线加速度和角速度做一阶低通滤波，姿态四元数保持 STM32 解算结果。
    * **目的**：发布至 `/imu/filtered`，实时驱动 Gazebo 模型并提供更平滑的分析曲线。

---

## 5. PC 端配置 (Native Host Setup)

### 5.1 原生 Agent 安装 (不使用 Docker)
```bash
# 工作空间构建
mkdir -p ~/uros_ws/src && cd ~/uros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ros-agent.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/local_setup.bash
```

### 5.2 运行指令
* **启动代理**：`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
* **防火墙放行**：`sudo ufw allow 8888/udp`
* **录制 rosbag**：`ros2 bag record /imu/data /imu/filtered /robot/state`
* **记录 CSV/JSONL**：`ros2 run imu_data_logger imu_logger --ros-args -p output_dir:=./data/imu_run_001`

---

## 6. 开发 Roadmap (五阶段)

1.  **无线打通**：ESP32 成功在 P14s 的 micro-ROS Agent 上注册节点并发布测试 Topic。（已完成）
2.  **协议联调**：STM32 发送 `IMUQ` / `State` 帧，ESP32 完整解析并发布 ROS 2 话题。（已完成）
3.  **数据采集**：使用 `imu_data_logger` 采集 CSV/JSONL，形成训练与评估数据集。（已完成）
4.  **模型部署**：将 `m2cgen` 转换的 C 算法代码刷入 STM32，验证本地 LED 响应速度。
5.  **仿真闭环**：在 Gazebo 中加载模型，订阅 `/imu/filtered` 实现物理-数字同频。（已完成）

---

## 7. 避坑指南 (Critical Notes)
* **内存管理**：ESP32 micro-ROS 默认缓冲区较小，若发布频率过高导致频繁掉线，需调整 `rmw_options`。
* **浮点支持**：STM32 编译时务必开启 `-u _printf_float` 并在配置中选择 `Hard VFP`。
* **频率同步**：ESP32 的发布频率建议略低于 STM32 的采样频率（如 50Hz），以缓解 Wi-Fi 传输压力
