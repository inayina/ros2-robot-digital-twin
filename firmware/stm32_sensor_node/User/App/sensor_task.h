#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "mpu6050.h"
#include "algo_task.h"

extern osThreadId_t SensorTaskHandle;
extern osMessageQueueId_t SensorQueueHandle;

HAL_StatusTypeDef SensorTaskUartWrite(const uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef SensorTaskUartWriteString(const char *text, uint32_t timeout);
void StartSensorTask(void *argument);

#endif
