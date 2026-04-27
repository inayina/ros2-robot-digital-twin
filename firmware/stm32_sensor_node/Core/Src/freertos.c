/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor_task.h"
#include "led_alarm.h"
#include "tb6612.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern volatile float g_motor_linear_x;
extern volatile float g_motor_angular_z;
extern volatile uint32_t g_motor_cmd_recv_tick_ms;
extern volatile uint8_t g_motor_estop;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_TASK_PERIOD_MS             10U
#define MOTOR_CMD_TIMEOUT_MS             200U
#define MOTOR_LINEAR_X_FULL_SCALE_MPS    0.30f
#define MOTOR_ANGULAR_Z_FULL_SCALE_RADPS 1.50f
#define MOTOR_COMMAND_DEADBAND           0.05f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AlgTask */
osThreadId_t AlgTaskHandle;
const osThreadAttr_t AlgTask_attributes = {
  .name = "AlgTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DefaultTask */
osThreadId_t DefaultTaskHandle;
const osThreadAttr_t DefaultTask_attributes = {
  .name = "DefaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SensorQueue */
osMessageQueueId_t SensorQueueHandle;
const osMessageQueueAttr_t SensorQueue_attributes = {
  .name = "SensorQueue"
};
/* Definitions for SensorSemaphore */
osSemaphoreId_t SensorSemaphoreHandle;
const osSemaphoreAttr_t SensorSemaphore_attributes = {
  .name = "SensorSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static float clampMotorScalar(float value);
static int16_t mapCmdVelToMotorDuty(float linear_x, float angular_z);

/* USER CODE END FunctionPrototypes */

void StartSensorTask(void *argument);
void StartAlgTask(void *argument);
void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LED_Alarm_Init();
  if (!TB6612_Init()) {
    Error_Handler();
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SensorSemaphore */
  SensorSemaphoreHandle = osSemaphoreNew(1, 1, &SensorSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SensorQueue */
  SensorQueueHandle = osMessageQueueNew (10, sizeof(SensorData_t), &SensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of AlgTask */
  AlgTaskHandle = osThreadNew(StartAlgTask, NULL, &AlgTask_attributes);

  /* creation of DefaultTask */
  DefaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &DefaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
__weak void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartAlgTask */
/**
* @brief Function implementing the AlgTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlgTask */
__weak void StartAlgTask(void *argument)
{
  /* USER CODE BEGIN StartAlgTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAlgTask */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the DefaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
__weak void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  (void)argument;

  for(;;)
  {
    uint32_t now_ms = HAL_GetTick();
    uint32_t recv_tick_ms = g_motor_cmd_recv_tick_ms;
    uint8_t estop = g_motor_estop;

    if (estop || recv_tick_ms == 0U || (now_ms - recv_tick_ms) > MOTOR_CMD_TIMEOUT_MS) {
      TB6612_EmergencyStop();
      osDelay(MOTOR_TASK_PERIOD_MS);
      continue;
    }

    TB6612_SetStandby(1U);

    int16_t duty = mapCmdVelToMotorDuty(g_motor_linear_x, g_motor_angular_z);
    if (duty == 0) {
      TB6612_MotorA_Stop();
    } else {
      TB6612_MotorA_SetDuty(duty);
    }

    osDelay(MOTOR_TASK_PERIOD_MS);
  }
  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static float clampMotorScalar(float value)
{
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

static int16_t mapCmdVelToMotorDuty(float linear_x, float angular_z)
{
  float command = linear_x / MOTOR_LINEAR_X_FULL_SCALE_MPS;

  if (fabsf(command) < MOTOR_COMMAND_DEADBAND) {
    command = angular_z / MOTOR_ANGULAR_Z_FULL_SCALE_RADPS;
  }

  command = clampMotorScalar(command);
  if (fabsf(command) < MOTOR_COMMAND_DEADBAND) {
    return 0;
  }

  return (int16_t)(command * (float)TB6612_MotorA_GetMaxDuty());
}

/* USER CODE END Application */

