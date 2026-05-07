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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IMUTaskHandle;
osThreadId LEDTaskHandle;
osThreadId BeepTaskHandle;
osThreadId MotorTaskHandle;
osThreadId ErrorTaskHandle;
osThreadId RS485TaskHandle;
osThreadId SuperCapTaskHandle;
osThreadId JudgeTaskHandle;
osThreadId DetectTaskHandle;
osThreadId RcTaskHandle;
osThreadId ShooterTaskHandle;
osThreadId ChassisHandle;
osThreadId GimbalHandle;
osThreadId VisionTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void OS_IMUCallback(void const * argument);
void OS_LedCallback(void const * argument);
void OS_BeepCallback(void const * argument);
void OS_MotorCallback(void const * argument);
void OS_ErrorCallback(void const * argument);
void OS_Board2BoardCallback(void const * argument);
void OS_SuperCapCallback(void const * argument);
void OS_JudgeCallback(void const * argument);
void OS_DetectCallback(void const * argument);
void OS_RcCallback(void const * argument);
void OS_ShooterCallback(void const * argument);
void OS_ChassisCallback(void const * argument);
void OS_GimbalCallback(void const * argument);
void OS_VisionCallback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, OS_IMUCallback, osPriorityNormal, 0, 2048);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, OS_LedCallback, osPriorityBelowNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of BeepTask */
  osThreadDef(BeepTask, OS_BeepCallback, osPriorityNormal, 0, 256);
  BeepTaskHandle = osThreadCreate(osThread(BeepTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, OS_MotorCallback, osPriorityNormal, 0, 512);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of ErrorTask */
  osThreadDef(ErrorTask, OS_ErrorCallback, osPriorityRealtime, 0, 128);
  ErrorTaskHandle = osThreadCreate(osThread(ErrorTask), NULL);

  /* definition and creation of RS485Task */
  osThreadDef(RS485Task, OS_Board2BoardCallback, osPriorityHigh, 0, 512);
  RS485TaskHandle = osThreadCreate(osThread(RS485Task), NULL);

  /* definition and creation of SuperCapTask */
  osThreadDef(SuperCapTask, OS_SuperCapCallback, osPriorityNormal, 0, 128);
  SuperCapTaskHandle = osThreadCreate(osThread(SuperCapTask), NULL);

  /* definition and creation of JudgeTask */
  osThreadDef(JudgeTask, OS_JudgeCallback, osPriorityNormal, 0, 256);
  JudgeTaskHandle = osThreadCreate(osThread(JudgeTask), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, OS_DetectCallback, osPriorityNormal, 0, 256);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* definition and creation of RcTask */
  osThreadDef(RcTask, OS_RcCallback, osPriorityAboveNormal, 0, 256);
  RcTaskHandle = osThreadCreate(osThread(RcTask), NULL);

  /* definition and creation of ShooterTask */
  osThreadDef(ShooterTask, OS_ShooterCallback, osPriorityNormal, 0, 256);
  ShooterTaskHandle = osThreadCreate(osThread(ShooterTask), NULL);

  /* definition and creation of Chassis */
  osThreadDef(Chassis, OS_ChassisCallback, osPriorityNormal, 0, 256);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, OS_GimbalCallback, osPriorityNormal, 0, 256);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of VisionTask */
  osThreadDef(VisionTask, OS_VisionCallback, osPriorityNormal, 0, 512);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_OS_IMUCallback */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_IMUCallback */
__weak void OS_IMUCallback(void const * argument)
{
  /* USER CODE BEGIN OS_IMUCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_IMUCallback */
}

/* USER CODE BEGIN Header_OS_LedCallback */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_LedCallback */
__weak void OS_LedCallback(void const * argument)
{
  /* USER CODE BEGIN OS_LedCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_LedCallback */
}

/* USER CODE BEGIN Header_OS_BeepCallback */
/**
* @brief Function implementing the BeepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_BeepCallback */
__weak void OS_BeepCallback(void const * argument)
{
  /* USER CODE BEGIN OS_BeepCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_BeepCallback */
}

/* USER CODE BEGIN Header_OS_MotorCallback */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_MotorCallback */
__weak void OS_MotorCallback(void const * argument)
{
  /* USER CODE BEGIN OS_MotorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_MotorCallback */
}

/* USER CODE BEGIN Header_OS_ErrorCallback */
/**
* @brief Function implementing the ErrorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ErrorCallback */
__weak void OS_ErrorCallback(void const * argument)
{
  /* USER CODE BEGIN OS_ErrorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_ErrorCallback */
}

/* USER CODE BEGIN Header_OS_Board2BoardCallback */
/**
* @brief Function implementing the RS485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_Board2BoardCallback */
__weak void OS_Board2BoardCallback(void const * argument)
{
  /* USER CODE BEGIN OS_Board2BoardCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_Board2BoardCallback */
}

/* USER CODE BEGIN Header_OS_SuperCapCallback */
/**
* @brief Function implementing the SuperCapTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_SuperCapCallback */
__weak void OS_SuperCapCallback(void const * argument)
{
  /* USER CODE BEGIN OS_SuperCapCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_SuperCapCallback */
}

/* USER CODE BEGIN Header_OS_JudgeCallback */
/**
* @brief Function implementing the JudgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_JudgeCallback */
__weak void OS_JudgeCallback(void const * argument)
{
  /* USER CODE BEGIN OS_JudgeCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_JudgeCallback */
}

/* USER CODE BEGIN Header_OS_DetectCallback */
/**
* @brief Function implementing the DetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_DetectCallback */
__weak void OS_DetectCallback(void const * argument)
{
  /* USER CODE BEGIN OS_DetectCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_DetectCallback */
}

/* USER CODE BEGIN Header_OS_RcCallback */
/**
* @brief Function implementing the RcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_RcCallback */
__weak void OS_RcCallback(void const * argument)
{
  /* USER CODE BEGIN OS_RcCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_RcCallback */
}

/* USER CODE BEGIN Header_OS_ShooterCallback */
/**
* @brief Function implementing the ShooterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ShooterCallback */
__weak void OS_ShooterCallback(void const * argument)
{
  /* USER CODE BEGIN OS_ShooterCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_ShooterCallback */
}

/* USER CODE BEGIN Header_OS_ChassisCallback */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ChassisCallback */
__weak void OS_ChassisCallback(void const * argument)
{
  /* USER CODE BEGIN OS_ChassisCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_ChassisCallback */
}

/* USER CODE BEGIN Header_OS_GimbalCallback */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_GimbalCallback */
__weak void OS_GimbalCallback(void const * argument)
{
  /* USER CODE BEGIN OS_GimbalCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_GimbalCallback */
}

/* USER CODE BEGIN Header_OS_VisionCallback */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_VisionCallback */
__weak void OS_VisionCallback(void const * argument)
{
  /* USER CODE BEGIN OS_VisionCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_VisionCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
