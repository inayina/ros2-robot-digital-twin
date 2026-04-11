#ifndef __ROS_BRIDGE_H
#define __ROS_BRIDGE_H

#include "main.h"
#include "cmsis_os.h"

extern osThreadId_t MicroROSTaskHandle;
extern osMessageQueueId_t FeatureQueueHandle;

void StartMicroROSTask(void *argument);

#endif