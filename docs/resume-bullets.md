# Resume Bullets

## 中文版

机器人状态监测与数字孪生系统 | STM32, ESP32, micro-ROS, ROS 2, Gazebo

- 设计并实现 STM32F411 + MPU6050 状态监测节点，基于 FreeRTOS 搭建传感器采样、特征计算、状态判别和 LED 报警任务链路。
- 开发 ESP32-S3 micro-ROS 无线桥接模块，通过 UART 接收 STM32 IMU/状态帧，并经 WiFi UDP 发布 ROS 2 话题 `/imu/data`、`/imu/filtered`、`/robot/state`。
- 在 ROS 2 Jazzy 与 Gazebo Harmonic 中实现数字孪生可视化节点，订阅实机 IMU 数据并通过 Gazebo 服务驱动 MPU6050 模型姿态同步。
- 完成从真实传感器采集、嵌入式边缘处理、micro-ROS 通信到 Gazebo 可视化的 V1 闭环验证，并整理调试日志与复现实验流程。

## English Version

Robot State Monitoring and Digital Twin System | STM32, ESP32, micro-ROS, ROS 2, Gazebo

- Built an STM32F411 + MPU6050 monitoring node with FreeRTOS tasks for sensor sampling, feature extraction, state classification, and LED alarm output.
- Implemented an ESP32-S3 micro-ROS bridge that receives STM32 UART frames and publishes `/imu/data`, `/imu/filtered`, and `/robot/state` to ROS 2 over WiFi UDP.
