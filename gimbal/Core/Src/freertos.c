/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
osThreadId Board_uartTaskHandle;
/* USER CODE END Variables */
osThreadId ProteckTaskHandle;
osThreadId IMUTaskHandle;
osThreadId GimbleTaskHandle;
osThreadId ShootTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Board_uart_Fun(void const * argument);
/* USER CODE END FunctionPrototypes */

void Protecck_Fun(void const * argument);
void INS_task(void const * argument);
void Gimble_Fun(void const * argument);
void Shoot_Fun(void const * argument);

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
  /* definition and creation of ProteckTask */
  osThreadDef(ProteckTask, Protecck_Fun, osPriorityAboveNormal, 0, 128);
  ProteckTaskHandle = osThreadCreate(osThread(ProteckTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, INS_task, osPriorityHigh, 0, 512);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of GimbleTask */
  osThreadDef(GimbleTask, Gimble_Fun, osPriorityHigh, 0, 128);
  GimbleTaskHandle = osThreadCreate(osThread(GimbleTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, Shoot_Fun, osPriorityHigh, 0, 128);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
  osThreadDef(Board_uartTask, Board_uart_Fun, osPriorityHigh, 0, 128);
  ShootTaskHandle = osThreadCreate(osThread(Board_uartTask), NULL);	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Protecck_Fun */
/**
  * @brief  Function implementing the ProteckTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Protecck_Fun */
void Protecck_Fun(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Protecck_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Protecck_Fun */
}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the IMU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_Gimble_Fun */
/**
* @brief Function implementing the GimbleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimble_Fun */
__weak void Gimble_Fun(void const * argument)
{
  /* USER CODE BEGIN Gimble_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimble_Fun */
}

/* USER CODE BEGIN Header_Shoot_Fun */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Fun */
__weak void Shoot_Fun(void const * argument)
{
  /* USER CODE BEGIN Shoot_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
__weak void Board_uart_Fun(void const * argument)
{
  /* USER CODE BEGIN Shoot_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Fun */
}
/* USER CODE END Application */
