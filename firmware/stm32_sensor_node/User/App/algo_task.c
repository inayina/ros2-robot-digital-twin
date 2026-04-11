#include "algo_task.h"
#include "led_alarm.h"
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t SensorQueueHandle;
extern osMessageQueueId_t FeatureQueueHandle;

#define WINDOW_SIZE 10

void Extract_Features(const SensorData_t *data, uint32_t count, FeatureData_t *features)
{
    if (count == 0 || data == NULL || features == NULL) {
        return;
    }

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sq_sum_ax = 0, sq_sum_ay = 0, sq_sum_az = 0;
    float sq_sum_gx = 0, sq_sum_gy = 0, sq_sum_gz = 0;

    for (uint32_t i = 0; i < count; i++) {
        sum_ax += data[i].acc[0];
        sum_ay += data[i].acc[1];
        sum_az += data[i].acc[2];
        sum_gx += data[i].gyro[0];
        sum_gy += data[i].gyro[1];
        sum_gz += data[i].gyro[2];

        sq_sum_ax += data[i].acc[0] * data[i].acc[0];
        sq_sum_ay += data[i].acc[1] * data[i].acc[1];
        sq_sum_az += data[i].acc[2] * data[i].acc[2];
        sq_sum_gx += data[i].gyro[0] * data[i].gyro[0];
        sq_sum_gy += data[i].gyro[1] * data[i].gyro[1];
        sq_sum_gz += data[i].gyro[2] * data[i].gyro[2];
    }

    float mean_ax = sum_ax / count;
    float mean_ay = sum_ay / count;
    float mean_az = sum_az / count;
    float mean_gx = sum_gx / count;
    float mean_gy = sum_gy / count;
    float mean_gz = sum_gz / count;

    float var_ax = (sq_sum_ax / count) - (mean_ax * mean_ax);
    float var_ay = (sq_sum_ay / count) - (mean_ay * mean_ay);
    float var_az = (sq_sum_az / count) - (mean_az * mean_az);
    float var_gx = (sq_sum_gx / count) - (mean_gx * mean_gx);
    float var_gy = (sq_sum_gy / count) - (mean_gy * mean_gy);
    float var_gz = (sq_sum_gz / count) - (mean_gz * mean_gz);

    float rms = sqrtf((var_ax + var_ay + var_az + var_gx + var_gy + var_gz) / 6.0f);
    
    float max_acc = 0;
    for (uint32_t i = 0; i < count; i++) {
        float acc_mag = sqrtf(data[i].acc[0]*data[i].acc[0] + 
                               data[i].acc[1]*data[i].acc[1] + 
                               data[i].acc[2]*data[i].acc[2]);
        if (acc_mag > max_acc) {
            max_acc = acc_mag;
        }
    }
    
    if (max_acc > 25.0f || rms > 8.0f) {
        features->anomaly_state = 3;
    } else if (rms > 5.0f) {
        features->anomaly_state = 2;
    } else if (rms > 2.0f) {
        features->anomaly_state = 1;
    } else {
        features->anomaly_state = 0;
    }

    features->confidence = 0.95f;
    features->filtered_rms = rms;
    features->process_time = 0;
}

void StartAlgTask(void *argument)
{
    SensorData_t buffer[WINDOW_SIZE];
    uint32_t index = 0;

    for (;;)
    {
        LED_Alarm_Update();
        
        SensorData_t data;
        osStatus_t status = osMessageQueueGet(SensorQueueHandle, &data, NULL, 100);

        if (status == osOK) {
            buffer[index] = data;
            index++;

            if (index >= WINDOW_SIZE) {
                FeatureData_t features;
                Extract_Features(buffer, WINDOW_SIZE, &features);

                printf("State:%d RMS:%.2f\n", features.anomaly_state, features.filtered_rms);

                // Send state to UART for ESP32
                char state_buffer[32];
                int state_len = snprintf(state_buffer, sizeof(state_buffer), "State:%d\n", features.anomaly_state);
                HAL_UART_Transmit(&huart1, (uint8_t*)state_buffer, state_len, HAL_MAX_DELAY);

                LED_Alarm_SetState(features.anomaly_state);

                if (osMessageQueuePut(FeatureQueueHandle, &features, 0, 10) != osOK) {
                    printf("FeatureQueue full or error, drop result\n");
                }

                index = 0;
            }
        }
        
        osDelay(10);
    }
}