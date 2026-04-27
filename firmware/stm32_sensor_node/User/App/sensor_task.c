#include "sensor_task.h"
#include "app_debug.h"
#include "algo_task.h"
#include "attitude_estimator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t SensorQueueHandle;
extern osSemaphoreId_t SensorSemaphoreHandle;

typedef enum {
    OUTPUT_MODE_GAZEBO = 0,
    OUTPUT_MODE_TRAIN = 1
} OutputMode_t;

static OutputMode_t current_mode = OUTPUT_MODE_GAZEBO;
static uint8_t train_label = 0;
static uint8_t cmd_byte = 0;
static AttitudeEstimator_t attitude_estimator;
static char uart_line_buffer[64];
static uint8_t uart_line_length = 0;
static char uart_pending_line[64];
static volatile uint8_t uart_pending_ready = 0U;
#if APP_DEBUG_CMDVEL_RX
static uint32_t last_cmd_debug_tick_ms = 0U;
#endif
static uint8_t gazebo_output_divider = 0U;
static uint32_t uart_rx_byte_count = 0U;
static uint32_t uart_rx_line_count = 0U;
static uint32_t uart_rx_error_count = 0U;
static uint32_t uart_rx_drop_count = 0U;
static uint32_t uart_rx_rearm_fail_count = 0U;
static uint32_t uart_last_error_code = 0U;
#if APP_DEBUG_RUNTIME_STATUS
static uint32_t last_runtime_debug_tick_ms = 0U;
#endif

volatile float g_motor_linear_x = 0.0f;
volatile float g_motor_angular_z = 0.0f;
volatile uint32_t g_motor_cmd_recv_tick_ms = 0;
volatile uint8_t g_motor_estop = 1;

static uint8_t SensorTaskTakeUartLock(uint32_t timeout)
{
    if (__get_IPSR() != 0U || osKernelGetState() != osKernelRunning || SensorSemaphoreHandle == NULL) {
        return 0U;
    }

    return (osSemaphoreAcquire(SensorSemaphoreHandle, timeout) == osOK) ? 1U : 0U;
}

static void SensorTaskGiveUartLock(uint8_t lock_taken)
{
    if (lock_taken != 0U && SensorSemaphoreHandle != NULL) {
        (void)osSemaphoreRelease(SensorSemaphoreHandle);
    }
}

HAL_StatusTypeDef SensorTaskUartWrite(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    HAL_StatusTypeDef status;
    uint8_t lock_taken;

    if (data == NULL || len == 0U) {
        return HAL_OK;
    }

    lock_taken = SensorTaskTakeUartLock(timeout);
    if (__get_IPSR() == 0U &&
        osKernelGetState() == osKernelRunning &&
        SensorSemaphoreHandle != NULL &&
        lock_taken == 0U) {
        return HAL_TIMEOUT;
    }

    status = HAL_UART_Transmit(&huart1, (uint8_t *)data, len, timeout);
    SensorTaskGiveUartLock(lock_taken);
    return status;
}

HAL_StatusTypeDef SensorTaskUartWriteString(const char *text, uint32_t timeout)
{
    if (text == NULL) {
        return HAL_ERROR;
    }

    return SensorTaskUartWrite((const uint8_t *)text, (uint16_t)strlen(text), timeout);
}

static void ReportCmdVelReceipt(float linear_x, float angular_z)
{
#if !APP_DEBUG_CMDVEL_RX
    (void)linear_x;
    (void)angular_z;
    return;
#else
    char buffer[64];
    uint32_t now_ms = HAL_GetTick();
    int len;

    if ((now_ms - last_cmd_debug_tick_ms) < 200U) {
        return;
    }

    len = snprintf(buffer, sizeof(buffer), "DBG:CMDVEL_RX,%.3f,%.3f,%lu\n",
                   linear_x, angular_z, (unsigned long)now_ms);
    if (len > 0) {
        (void)SensorTaskUartWrite((const uint8_t *)buffer, (uint16_t)len, 20U);
        last_cmd_debug_tick_ms = now_ms;
    }
#endif
}

static void EnsureUartReceptionActive(void)
{
    if (huart1.RxState != HAL_UART_STATE_READY) {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(&huart1);
    if (HAL_UART_Receive_IT(&huart1, &cmd_byte, 1) != HAL_OK) {
        uart_rx_rearm_fail_count++;
    }
}

static void ReportRuntimeStatus(void)
{
#if !APP_DEBUG_RUNTIME_STATUS
    return;
#else
    char buffer[160];
    uint32_t now_ms = HAL_GetTick();
    int len;

    if ((now_ms - last_runtime_debug_tick_ms) < 1000U) {
        return;
    }

    len = snprintf(buffer, sizeof(buffer),
                   "DBG:STAT,rxb=%lu,rxl=%lu,rxerr=%lu,drop=%lu,rearm=%lu,last=%lu,estop=%u,vx=%.3f,wz=%.3f\n",
                   (unsigned long)uart_rx_byte_count,
                   (unsigned long)uart_rx_line_count,
                   (unsigned long)uart_rx_error_count,
                   (unsigned long)uart_rx_drop_count,
                   (unsigned long)uart_rx_rearm_fail_count,
                   (unsigned long)uart_last_error_code,
                   (unsigned)g_motor_estop,
                   (double)g_motor_linear_x,
                   (double)g_motor_angular_z);
    if (len > 0) {
        (void)SensorTaskUartWrite((const uint8_t *)buffer, (uint16_t)len, 20U);
        last_runtime_debug_tick_ms = now_ms;
    }
#endif
}

static uint8_t FetchPendingUartLine(char *line, uint32_t line_size)
{
    uint8_t has_line = 0U;

    if (line == NULL || line_size == 0U) {
        return 0U;
    }

    __disable_irq();
    if (uart_pending_ready != 0U) {
        strncpy(line, uart_pending_line, line_size - 1U);
        line[line_size - 1U] = '\0';
        uart_pending_ready = 0U;
        has_line = 1U;
    }
    __enable_irq();

    return has_line;
}

static void OutputGazebo(const SensorData_t *data)
{
    char buffer[192];
    int len = snprintf(buffer, sizeof(buffer),
                       "IMUQ,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.1f\n",
                       data->acc[0], data->acc[1], data->acc[2],
                       data->gyro[0], data->gyro[1], data->gyro[2],
                       data->quat[0], data->quat[1], data->quat[2], data->quat[3],
                       data->mpu_temp);
    (void)SensorTaskUartWrite((const uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
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
    (void)SensorTaskUartWrite((const uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
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
    (void)SensorTaskUartWrite((const uint8_t *)ack, (uint16_t)len, 50U);
}

static uint8_t IsImmediateCommand(uint8_t cmd)
{
    return (cmd == 't' || cmd == 'T' ||
            cmd == 'g' || cmd == 'G' ||
            (cmd >= '0' && cmd <= '3') ||
            cmd == 's' || cmd == 'S' ||
            cmd == '?' || cmd == 'h');
}

static uint8_t ParseCmdVelLine(const char* line, float* linear_x, float* angular_z)
{
    char *endptr;
    float parsed_linear_x;
    float parsed_angular_z;

    if (line == NULL || linear_x == NULL || angular_z == NULL) {
        return 0U;
    }

    if (strncmp(line, "CMDVEL,", 7U) != 0) {
        return 0U;
    }

    parsed_linear_x = strtof(line + 7U, &endptr);
    if (endptr == (line + 7U) || *endptr != ',') {
        return 0U;
    }

    parsed_angular_z = strtof(endptr + 1, &endptr);
    if (endptr == NULL || (*endptr != '\0' && *endptr != '\r' && *endptr != '\n')) {
        return 0U;
    }

    *linear_x = parsed_linear_x;
    *angular_z = parsed_angular_z;
    return 1U;
}

static void HandleUartLine(const char* line)
{
    float linear_x = 0.0f;
    float angular_z = 0.0f;

    if (strcmp(line, "ESTOP") == 0) {
        g_motor_estop = 1;
#if APP_DEBUG_ESTOP_RX
        (void)SensorTaskUartWriteString("DBG:ESTOP_RX\n", 20U);
#endif
        return;
    }

    if (ParseCmdVelLine(line, &linear_x, &angular_z) != 0U) {
        g_motor_linear_x = linear_x;
        g_motor_angular_z = angular_z;
        g_motor_cmd_recv_tick_ms = HAL_GetTick();
        g_motor_estop = 0;
        ReportCmdVelReceipt(linear_x, angular_z);
        return;
    }

    if (strlen(line) == 1U && IsImmediateCommand((uint8_t)line[0])) {
        HandleCommand((uint8_t)line[0]);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_rx_byte_count++;
        if (cmd_byte == '\r' || cmd_byte == '\n') {
            if (uart_line_length > 0U) {
                uart_line_buffer[uart_line_length] = '\0';
                if (uart_pending_ready == 0U) {
                    memcpy(uart_pending_line, uart_line_buffer, uart_line_length + 1U);
                    uart_pending_ready = 1U;
                    uart_rx_line_count++;
                } else {
                    uart_rx_drop_count++;
                }
                uart_line_length = 0U;
            }
        } else if (uart_line_length < (sizeof(uart_line_buffer) - 1U)) {
            uart_line_buffer[uart_line_length++] = (char)cmd_byte;
        } else {
            uart_line_length = 0U;
            uart_rx_drop_count++;
        }

        if (HAL_UART_Receive_IT(&huart1, &cmd_byte, 1) != HAL_OK) {
            uart_rx_rearm_fail_count++;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    uart_rx_error_count++;
    uart_last_error_code = huart->ErrorCode;
    __HAL_UART_CLEAR_PEFLAG(huart);
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
  uart_line_length = 0U;
  uart_pending_ready = 0U;
  EnsureUartReceptionActive();
  AttitudeEstimator_Init(&attitude_estimator);
  uint32_t last_tick = osKernelGetTickCount();

  for (;;)
  {
    char pending_line[64];
    SensorData_t data;
    MPU6050_Data raw_data;

    EnsureUartReceptionActive();

    if (FetchPendingUartLine(pending_line, sizeof(pending_line)) != 0U) {
        HandleUartLine(pending_line);
    }

    MPU6050_Read_RawData(&hi2c1, &raw_data);

    data.acc[0] = raw_data.ax / 16384.0f * 9.8f;
    data.acc[1] = raw_data.ay / 16384.0f * 9.8f;
    data.acc[2] = raw_data.az / 16384.0f * 9.8f;

    data.gyro[0] = raw_data.gx / 131.0f;
    data.gyro[1] = raw_data.gy / 131.0f;
    data.gyro[2] = raw_data.gz / 131.0f;

    uint32_t now_tick = osKernelGetTickCount();
    float dt = (float)(now_tick - last_tick) / 1000.0f;
    last_tick = now_tick;
    AttitudeEstimator_Update(&attitude_estimator,
                             data.acc[0], data.acc[1], data.acc[2],
                             data.gyro[0], data.gyro[1], data.gyro[2],
                             dt);
    data.quat[0] = attitude_estimator.q1;
    data.quat[1] = attitude_estimator.q2;
    data.quat[2] = attitude_estimator.q3;
    data.quat[3] = attitude_estimator.q0;

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
        gazebo_output_divider ^= 1U;
        if (gazebo_output_divider == 0U) {
            OutputGazebo(&data);
        }
    }

    ReportRuntimeStatus();

    if (osMessageQueuePut(SensorQueueHandle, &data, 0, 10) != osOK) {
    }

    osDelay(10);
  }
}
