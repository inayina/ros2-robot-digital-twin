# Hardware Wiring

本文件整理当前实机状态与电机控制子系统的硬件连接关系。它用于说明职责和接线边界，不替代实际原理图；真实上板前仍要逐项核对电压、共地、引脚复用和供电能力。

## Hardware Blocks

- STM32F411 sensor node
- ESP32-S3 DevKitC-1 micro-ROS bridge
- MPU6050 IMU
- TB6612FNG motor driver
- 130 普通电机，当前作为 TB6612 A 通道辅助验证负载
- 6V N20 编码器减速电机，当前作为 TB6612 B 通道闭环 bench 主负载
- 外部电机电源
- ROS 2 / micro-ROS Agent 主机

## STM32 And MPU6050

当前 STM32 负责 MPU6050 采样、姿态解算和状态判别。

| Signal | STM32 | Device | Notes |
| --- | --- | --- | --- |
| I2C1 SCL | `PB6` | MPU6050 `SCL` | 需要确认外部上拉 |
| I2C1 SDA | `PB7` | MPU6050 `SDA` | 需要确认外部上拉 |
| Power | board dependent | MPU6050 `VCC` | 按模块额定电压接入 |
| Ground | `GND` | MPU6050 `GND` | 与 STM32 共地 |

MPU6050 默认地址为 `0x68`。`AD0` 建议固定，避免地址漂移。

## STM32 And ESP32 UART

STM32 与 ESP32 使用 USART/UART 文本协议通信，波特率 `921600`。

| Direction | STM32 | ESP32-S3 | Payload |
| --- | --- | --- | --- |
| STM32 -> ESP32 | `USART1 TX` / `PA9` | `GPIO16 RX` | `IMUQ,...`, `IMU,...`, `State:<n>`, `DBG:...` |
| ESP32 -> STM32 | `USART1 RX` / `PA10` | `GPIO17 TX` | `G\n`, `CMDVEL,<linear_x>,<angular_z>\n` |
| Ground | `GND` | `GND` | 必须共地 |

注意：

- ESP32 `G\n` 用于请求 STM32 进入 Gazebo / ROS 输出模式。
- `CMDVEL` 是 legacy 本地控制链路，不是当前 dashboard 主电机命令链路。
- 不要在未确认电平兼容前跨电压域直连。

## ESP32 And TB6612

ESP32 承接 N20 编码器电机控制主线。当前规划 TB6612 A 通道保留给 130 普通电机，B 通道用于单 N20 bench。

### Logic And Power

| Source | TB6612 | Notes |
| --- | --- | --- |
| ESP32 `3.3V` | `VCC` | 逻辑电源 |
| ESP32 `GND` | `GND` | 与电机电源负极共地 |
| external motor supply | `VM` | 电机电源，不接 ESP32 `3.3V` |
| ESP32 `GPIO18` | `STBY` | 软件控制待机，不建议永久硬拉高 |

约束：

- ESP32、TB6612、电机电源必须共地。
- `VM` 只接电机电源，不能从 ESP32 `3.3V` 取电驱动电机。
- 调试阶段优先使用限流电源，先从低占空比验证。

### TB6612 Control Pins

| ESP32-S3 GPIO | TB6612 Pin | Current Role |
| --- | --- | --- |
| `GPIO4` | `PWMA` | A 通道 PWM，保留给 130 普通电机 |
| `GPIO5` | `AIN1` | A 通道方向 1 |
| `GPIO6` | `AIN2` | A 通道方向 2 |
| `GPIO7` | `PWMB` | B 通道 PWM，当前单 N20 主通道 |
| `GPIO8` | `BIN1` | B 通道方向 1 |
| `GPIO9` | `BIN2` | B 通道方向 2 |
| `GPIO18` | `STBY` | 驱动待机控制 |

当前避开的引脚：

- `GPIO16/GPIO17` 已用于 STM32 UART。
- `GPIO19/GPIO20` 默认用于 ESP32-S3 USB D- / D+。
- `GPIO0/GPIO3/GPIO45/GPIO46` 属于 strapping pins，不作为电机控制引脚。
- `GPIO26-GPIO32` 通常与 SPI flash / PSRAM 相关，当前不纳入规划。

## TB6612 Outputs And Motors

| TB6612 Output | Load | Notes |
| --- | --- | --- |
| `AO1` / `A01` | 130 motor terminal 1 | A 通道辅助验证 |
| `AO2` / `A02` | 130 motor terminal 2 | A 通道辅助验证 |
| `BO1` / `B01` | N20 motor terminal 1 | B 通道主验证 |
| `BO2` / `B02` | N20 motor terminal 2 | B 通道主验证 |

不要预先假定哪根电机线一定是正方向。如果实测方向与软件定义相反，优先通过交换电机线或调整方向映射修正。

## N20 Encoder Wiring

N20 编码器不经过 TB6612，编码器 A/B 直接回 ESP32。

| Signal | ESP32-S3 GPIO | Notes |
| --- | --- | --- |
| current N20 encoder A | `GPIO10` | 当前单 N20 主通道 |
| current N20 encoder B | `GPIO11` | 当前单 N20 主通道 |
| future second N20 encoder A | `GPIO12` | 后续 A 通道 N20 预留 |
| future second N20 encoder B | `GPIO13` | 后续 A 通道 N20 预留 |
| encoder VCC | logic-side power | 先确认编码器电压和输出电平 |
| encoder GND | system GND | 必须共地 |

注意：

- 如果编码器输出可能到 `5V`，必须先确认是否需要分压、限流或电平转换。
- `invalid_transitions` 持续增加时，先检查接线、供电、电平和干扰，再进入闭环调参。

## Recommended Bring-Up Order

1. 只接逻辑电源、共地、STM32 UART 和 MPU6050，确认 `/imu/data`、`/imu/filtered`、`/robot/state`。
2. 接 TB6612 `VCC`、`GND`、`VM`、`STBY`，不接电机或保持低占空比。
3. 接 A 通道和 130 普通电机，做低占空比正反转辅助验证。
4. 接 B 通道和单 N20，先做开环低占空比方向验证。
5. 接 N20 编码器 A/B，确认计数方向、`encoder_count` 和 `invalid_transitions`。
6. 进入低 `max_pwm` 闭环 bench，先确认 stop、timeout、target=0 输出为 0。

## Safety Checklist

- 所有控制链路共地。
- 电机电源具有限流或保险措施。
- `STBY` 由软件可控。
- 上电默认不输出真实 PWM。
- `target_rpm = 0` 时 PWM 必须回到 `0`。
- 调试结束后进入 coast / standby。
- Dashboard 不参与高频 PID，只做低频观察和命令入口。
