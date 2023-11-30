/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
osThreadId StartINSTaskHandle;
osThreadId StartDetectTaskHandle;
osThreadId StartUsartTaskHandle;
osThreadId StartCANTxTaskHandle;
osThreadId StartChassis_TaHandle;
osThreadId StartGimbal_TasHandle;
osThreadId StartCAN2TxTaskHandle;
osThreadId StartGPIO_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void Detect_Task(void const * argument);
void Usart_Task(void const * argument);
void CAN1Tx_Task(void const * argument);
void Chassis_Task(void const * argument);
void gimbal_Task(void const * argument);
void CAN2Tx_Task(void const * argument);
void GPIO_Task(void const * argument);

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
  /* definition and creation of StartINSTask */
  osThreadDef(StartINSTask, INS_Task, osPriorityRealtime, 0, 256);
  StartINSTaskHandle = osThreadCreate(osThread(StartINSTask), NULL);

  /* definition and creation of StartDetectTask */
  osThreadDef(StartDetectTask, Detect_Task, osPriorityRealtime, 0, 256);
  StartDetectTaskHandle = osThreadCreate(osThread(StartDetectTask), NULL);

  /* definition and creation of StartUsartTask */
  osThreadDef(StartUsartTask, Usart_Task, osPriorityRealtime, 0, 256);
  StartUsartTaskHandle = osThreadCreate(osThread(StartUsartTask), NULL);

  /* definition and creation of StartCANTxTask */
  osThreadDef(StartCANTxTask, CAN1Tx_Task, osPriorityRealtime, 0, 256);
  StartCANTxTaskHandle = osThreadCreate(osThread(StartCANTxTask), NULL);

  /* definition and creation of StartChassis_Ta */
  osThreadDef(StartChassis_Ta, Chassis_Task, osPriorityRealtime, 0, 256);
  StartChassis_TaHandle = osThreadCreate(osThread(StartChassis_Ta), NULL);

  /* definition and creation of StartGimbal_Tas */
  osThreadDef(StartGimbal_Tas, gimbal_Task, osPriorityRealtime, 0, 256);
  StartGimbal_TasHandle = osThreadCreate(osThread(StartGimbal_Tas), NULL);

  /* definition and creation of StartCAN2TxTask */
  osThreadDef(StartCAN2TxTask, CAN2Tx_Task, osPriorityRealtime, 0, 256);
  StartCAN2TxTaskHandle = osThreadCreate(osThread(StartCAN2TxTask), NULL);

  /* definition and creation of StartGPIO_Task */
  osThreadDef(StartGPIO_Task, GPIO_Task, osPriorityRealtime, 0, 256);
  StartGPIO_TaskHandle = osThreadCreate(osThread(StartGPIO_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_Usart_Task */
/**
* @brief Function implementing the StartUsartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usart_Task */
__weak void Usart_Task(void const * argument)
{
  /* USER CODE BEGIN Usart_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Usart_Task */
}

/* USER CODE BEGIN Header_CAN1Tx_Task */
/**
* @brief Function implementing the StartCANTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN1Tx_Task */
__weak void CAN1Tx_Task(void const * argument)
{
  /* USER CODE BEGIN CAN1Tx_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN1Tx_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the StartChassis_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_gimbal_Task */
/**
* @brief Function implementing the StartGimbal_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_Task */
__weak void gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_Task */
}

/* USER CODE BEGIN Header_CAN2Tx_Task */
/**
* @brief Function implementing the StartCAN2TxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2Tx_Task */
__weak void CAN2Tx_Task(void const * argument)
{
  /* USER CODE BEGIN CAN2Tx_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN2Tx_Task */
}

/* USER CODE BEGIN Header_GPIO_Task */
/**
* @brief Function implementing the StartGPIO_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPIO_Task */
__weak void GPIO_Task(void const * argument)
{
  /* USER CODE BEGIN GPIO_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GPIO_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
