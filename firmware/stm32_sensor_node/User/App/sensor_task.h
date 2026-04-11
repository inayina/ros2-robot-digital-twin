#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "mpu6050.h"
#include "algo_task.h"

extern osThreadId_t SensorTaskHandle;
extern osMessageQueueId_t SensorQueueHandle;

void StartSensorTask(void *argument);

#endif