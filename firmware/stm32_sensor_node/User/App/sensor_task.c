#include "sensor_task.h"
#include "algo_task.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t SensorQueueHandle;

typedef enum {
    OUTPUT_MODE_GAZEBO = 0,
    OUTPUT_MODE_TRAIN = 1
} OutputMode_t;

static OutputMode_t current_mode = OUTPUT_MODE_GAZEBO;
static uint8_t train_label = 0;
static uint8_t cmd_byte = 0;

static void OutputGazebo(const SensorData_t *data)
{
    char buffer[96];
    int len = snprintf(buffer, sizeof(buffer), "IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n",
                       data->acc[0], data->acc[1], data->acc[2],
                       data->gyro[0], data->gyro[1], data->gyro[2],
                       data->mpu_temp);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

static void OutputTrain(const SensorData_t *data)
{
    char buffer[128];
    uint32_t ts = data->timestamp;
    int len = snprintf(buffer, sizeof(buffer), "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n",
                       ts,
                       data->acc[0], data->acc[1], data->acc[2],
                       data->gyro[0], data->gyro[1], data->gyro[2],
                       train_label);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

static void HandleCommand(uint8_t cmd)
{
    char ack[64];
    int len;
    
    if (cmd == 't' || cmd == 'T') {
        current_mode = OUTPUT_MODE_TRAIN;
        len = snprintf(ack, sizeof(ack), "-> TRAIN mode. Label=%d\n", train_label);
    } else if (cmd == 'g' || cmd == 'G') {
        current_mode = OUTPUT_MODE_GAZEBO;
        len = snprintf(ack, sizeof(ack), "-> GAZEBO mode\n");
    } else if (cmd >= '0' && cmd <= '3') {
        train_label = cmd - '0';
        len = snprintf(ack, sizeof(ack), "Label=%d OK\n", train_label);
    } else if (cmd == 's' || cmd == 'S') {
        len = snprintf(ack, sizeof(ack), "Mode:%s Label:%d\n", 
                       current_mode == OUTPUT_MODE_TRAIN ? "TRAIN" : "GAZEBO",
                       train_label);
    } else if (cmd == '?' || cmd == 'h') {
        len = snprintf(ack, sizeof(ack), "T=Train G=Gazebo 0-3=Label S=Status ?=Help\n");
    } else {
        return;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)ack, len, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HandleCommand(cmd_byte);
        HAL_UART_Receive_IT(&huart1, &cmd_byte, 1);
    }
}

void StartSensorTask(void *argument)
{
  if (MPU6050_Init(&hi2c1) == 0) {
    printf("MPU6050 init FAIL!\n");
    while(1) osDelay(1000);
  }
  printf("MPU6050 OK\n");
  printf("CMD: T=Train G=Gazebo 0-3=Label S=Status ?=Help\n");

  cmd_byte = 0;
  HAL_UART_Receive_IT(&huart1, &cmd_byte, 1);

  for (;;)
  {
    SensorData_t data;
    MPU6050_Data raw_data;
    MPU6050_Read_RawData(&hi2c1, &raw_data);

    data.acc[0] = raw_data.ax / 16384.0f * 9.8f;
    data.acc[1] = raw_data.ay / 16384.0f * 9.8f;
    data.acc[2] = raw_data.az / 16384.0f * 9.8f;

    data.gyro[0] = raw_data.gx / 131.0f;
    data.gyro[1] = raw_data.gy / 131.0f;
    data.gyro[2] = raw_data.gz / 131.0f;

    data.mpu_temp = raw_data.temperature;
    data.env_temp = 0;
    data.env_humid = 0;
    data.pressure = 0;
    data.timestamp = osKernelGetTickCount();
    data.sensor_status[0] = 1;
    data.sensor_status[1] = 0;
    data.sensor_status[2] = 0;
    data.sensor_status[3] = 0;

    if (current_mode == OUTPUT_MODE_TRAIN) {
        OutputTrain(&data);
    } else {
        OutputGazebo(&data);
    }

    if (osMessageQueuePut(SensorQueueHandle, &data, 0, 10) != osOK) {
    }

    osDelay(10);
  }
}