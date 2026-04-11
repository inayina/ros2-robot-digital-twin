#include "ros_bridge.h"
#include "algo_task.h"
#include <stdio.h>

extern osMessageQueueId_t FeatureQueueHandle;

void StartMicroROSTask(void *argument)
{
  printf("MicroROSTask started!\n");

  for (;;)
  {
    FeatureData_t features;
    osStatus_t status = osMessageQueueGet(FeatureQueueHandle, &features, NULL, 100);

    if (status == osOK) {
      printf("ROS Publish - State:%d Confidence:%.2f RMS:%.2f\n",
             features.anomaly_state,
             features.confidence,
             features.filtered_rms);
    }

    osDelay(100);
  }
}