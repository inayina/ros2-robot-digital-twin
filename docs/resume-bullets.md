# Resume Bullets

## 中文版

机器人状态监测与数字孪生系统 V2 | STM32, ESP32, micro-ROS, ROS 2, Gazebo

- 设计并实现 STM32F411 + MPU6050 状态监测节点，基于 FreeRTOS 搭建传感器采样、姿态解算、状态判别和 LED 报警任务链路。
- 将 6 轴姿态解算下沉到 STM32，输出 `IMUQ` 四元数串口帧，使 ROS 2 `sensor_msgs/Imu.orientation` 可使用真实姿态数据。
- 开发 ESP32-S3 micro-ROS 无线桥接模块，模块化重构 UART 帧解析与 ROS 2 发布逻辑，并以 `best_effort` QoS 发布 `/imu/data`、`/imu/filtered`、`/robot/state`。
- 在 ESP32 桥接端实现 IMU 一阶低通滤波，保留 STM32 姿态四元数并输出 raw/filtered 双路传感器流，提升硬件数据流的可视化与分析稳定性。
- 在 ROS 2 Jazzy 与 Gazebo Harmonic 中实现数字孪生可视化节点，直接使用实机 IMU 四元数驱动 MPU6050 模型，并通过归一化与 `lock_yaw` 抑制 yaw 漂移影响。
- 新增 Python 分析包 `imu_data_logger`，支持 IMU CSV、状态 JSONL 记录和 raw/filtered 实时曲线对比，支撑后续训练数据采集。

## English Version

Robot State Monitoring and Digital Twin System V2 | STM32, ESP32, micro-ROS, ROS 2, Gazebo

- Built an STM32F411 + MPU6050 monitoring node with FreeRTOS tasks for sensor sampling, attitude estimation, state classification, and LED alarm output.
- Moved 6-axis attitude estimation onto the STM32 and emitted `IMUQ` quaternion UART frames so ROS 2 `sensor_msgs/Imu.orientation` carries real pose data.
- Implemented an ESP32-S3 micro-ROS bridge with modular UART parsing and ROS 2 publishing, using `best_effort` QoS for `/imu/data`, `/imu/filtered`, and `/robot/state` over WiFi UDP.
- Added first-order IMU low-pass filtering on the ESP32 bridge while preserving STM32 quaternion orientation, producing raw/filtered sensor streams for steadier visualization and analysis.
- Developed a ROS 2 Jazzy and Gazebo Harmonic digital twin node that drives an MPU6050 model from live IMU quaternions, with normalization and `lock_yaw` handling to reduce yaw-drift artifacts.
- Added the `imu_data_logger` Python analysis package for IMU CSV logging, robot-state JSONL logging, and live raw/filtered telemetry plotting.
