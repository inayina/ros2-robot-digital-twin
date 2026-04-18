#ifndef __ALGO_TASK_H
#define __ALGO_TASK_H

#include "main.h"
#include "cmsis_os.h"

extern osThreadId_t AlgTaskHandle;
extern osMessageQueueId_t SensorQueueHandle;

/* 1. 传感器原始数据流 (52 Bytes) */
typedef struct {
    float acc[3];           // 加速度 (ax, ay, az)
    float gyro[3];          // 陀螺仪 (gx, gy, gz)
    float quat[4];          // 姿态四元数 (x, y, z, w)
    float mpu_temp;         // MPU内部温度
    float env_temp;         // (预留) AHT20
    float env_humid;        // (预留) AHT20
    float pressure;         // (预留) BMP280
    uint32_t timestamp;    // 系统时间戳
    uint8_t sensor_status[4]; // 硬件健康标志位
} SensorData_t;

/* 2. 算法推理结论 (16 Bytes) */
typedef struct {
    uint8_t anomaly_state; // 分类状态 (0:正常, 1:震动, 2:碰撞, 3:倾倒)
    uint8_t padding[3];   // 内存对齐占位
    float confidence;      // 模型置信度
    float filtered_rms;    // 关键特征值
    uint32_t process_time; // 算法单次耗时
} FeatureData_t;

void StartAlgTask(void *argument);
void Extract_Features(const SensorData_t *data, uint32_t count, FeatureData_t *features);

#endif
