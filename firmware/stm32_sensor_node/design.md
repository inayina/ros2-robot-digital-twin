这份 **Robot-State-Monitor (RSM) v2.7** 完整版项目白皮书，集成了你所有的核心需求：**双板架构**、**Wi-Fi 传输**、**数据双轨制（原始+滤波）**以及 **Native（原生）环境运行**。

你可以直接将其作为项目的核心文档（`design.md`），指导后续所有开发工作。

---

# 🤖 Project: Robot-State-Monitor (RSM) v2.7
**项目定位：** 基于嵌入式边缘计算与 ROS 2 数字孪生的无线监测系统
**核心环境：** P14s (Ubuntu 24.04 + ROS 2 Jazzy Native)
**硬件组合：** STM32F411CEU6 (计算核心) + ESP32 (无线网桥)

---

## 1. 系统架构 (System Architecture)

系统由**感知推理、无线传输、数字孪生**三个维度构成：

1.  **STM32 (Master)**：负责 100Hz 传感器采集、互补滤波算法、XGBoost 状态推理及本地 LED 报警。
2.  **ESP32 (Bridge)**：通过串口接收 STM32 封装帧，利用 micro-ROS 将数据通过 Wi-Fi 发布至 PC 端。
3.  **PC (Digital Twin)**：运行原生 micro-ROS Agent，实现数据录制（训练集）与 Gazebo 实时姿态同步。

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
| **SensorTask** | 48 | 100Hz 采集传感器原始数据。 |
| **AlgTask** | 32 | 1. **互补滤波**计算姿态四元数；2. **XGBoost** 识别运动状态。 |
| **CommsTask** | 24 | 按协议打包 `[Raw + Filtered + Inference]` 帧，通过串口推给 ESP32。 |

### 3.2 ESP32 端 (PlatformIO/Arduino)
* **任务**：解析串口帧，通过 `micro_ros_arduino` 发布话题。
* **网络**：Station 模式连接 Wi-Fi，目标 IP 指向 P14s。

---

## 4. 数据双轨制逻辑 (Data Pipeline)

为了同时兼顾“模型训练”与“仿真稳定性”，数据在发送前进行分流：

1.  **原始轨 (Raw Track)**：
    * **数据内容**：未经过滤的 $accel$ 和 $gyro$ 原始数值。
    * **目的**：发布至 `/imu/raw`，通过 `ros2 bag record` 录制，作为 P14s 上 Python 训练 XGBoost 的数据集。
2.  **滤波轨 (Filtered Track)**：
    * **数据内容**：经过 STM32 互补滤波计算后的姿态角/四元数。
    * **目的**：发布至 `/imu/filtered`，实时驱动 Gazebo 中的方块模型，确保仿真平滑无抖动。

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
* **录制数据**：`ros2 bag record /imu/raw`

---

## 6. 开发 Roadmap (五阶段)

1.  **无线打通**：ESP32 成功在 P14s 的 micro-ROS Agent 上注册节点并发布测试 Topic。
2.  **协议联调**：STM32 发送封装帧，ESP32 能够完整解析并校验 CRC8。
3.  **数据采集**：手持设备模拟不同状态，PC 录制数据并导出 CSV，完成 XGBoost 模型训练。
4.  **模型部署**：将 `m2cgen` 转换的 C 算法代码刷入 STM32，验证本地 LED 响应速度。
5.  **仿真闭环**：在 Gazebo 中加载模型，订阅 `/imu/filtered` 实现物理-数字同频。

---

## 7. 避坑指南 (Critical Notes)
* **内存管理**：ESP32 micro-ROS 默认缓冲区较小，若发布频率过高导致频繁掉线，需调整 `rmw_options`。
* **浮点支持**：STM32 编译时务必开启 `-u _printf_float` 并在配置中选择 `Hard VFP`。
* **频率同步**：ESP32 的发布频率建议略低于 STM32 的采样频率（如 50Hz），以缓解 Wi-Fi 传输压力。

---

这份文档就是你后续工程的“导航图”。明天硬件到手后，建议先从 **Phase 1 (ESP32 Wi-Fi 通讯)** 开始，那是整条链路最容易卡壳的地方。需要针对具体的 ESP32 micro-ROS 代码配置进行深入讨论吗？