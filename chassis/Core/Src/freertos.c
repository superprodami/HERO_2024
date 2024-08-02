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

/* USER CODE END Variables */
osThreadId UITaskHandle;
osThreadId ShootTASKHandle;
osThreadId GimbleTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId Board_uartTaskHandle;
osThreadId Can_sendTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Board_uart_Fun(void const * argument);


/* USER CODE END FunctionPrototypes */

void UI_Fun(void const * argument);
void Shoot_Fun(void const * argument);
void Gimble_Fun(void const * argument);
void Chassis_Fun(void const * argument);
void Board_uart_Fun(void const * argument);
void Can_sendFun(void const * argument);

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
  /* definition and creation of UITask */
  osThreadDef(UITask, UI_Fun, osPriorityHigh, 0, 128);
  UITaskHandle = osThreadCreate(osThread(UITask), NULL);

  /* definition and creation of ShootTASK */
  osThreadDef(ShootTASK, Shoot_Fun, osPriorityHigh, 0, 128);
  ShootTASKHandle = osThreadCreate(osThread(ShootTASK), NULL);

  /* definition and creation of GimbleTask */
  osThreadDef(GimbleTask, Gimble_Fun, osPriorityHigh, 0, 128);
  GimbleTaskHandle = osThreadCreate(osThread(GimbleTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, Chassis_Fun, osPriorityHigh, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of Board_uartTask */
  osThreadDef(Board_uartTask, Board_uart_Fun, osPriorityHigh, 0, 128);
  Board_uartTaskHandle = osThreadCreate(osThread(Board_uartTask), NULL);

  /* definition and creation of Can_sendTask */
  osThreadDef(Can_sendTask, Can_sendFun, osPriorityHigh, 0, 128);
  Can_sendTaskHandle = osThreadCreate(osThread(Can_sendTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

 /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_UI_Fun */
/**
  * @brief  Function implementing the UITask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_UI_Fun */
__weak void UI_Fun(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN UI_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_Fun */
}

/* USER CODE BEGIN Header_Shoot_Fun */
/**
* @brief Function implementing the ShootTASK thread.
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

/* USER CODE BEGIN Header_Chassis_Fun */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Fun */
__weak void Chassis_Fun(void const * argument)
{
  /* USER CODE BEGIN Chassis_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Fun */
}

/* USER CODE BEGIN Header_Board_uart_Fun */
/**
* @brief Function implementing the Board_uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Board_uart_Fun */
__weak void Board_uart_Fun(void const * argument)
{
  /* USER CODE BEGIN Board_uart_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Board_uart_Fun */
}

/* USER CODE BEGIN Header_Can_sendFun */
/**
* @brief Function implementing the Can_sendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_sendFun */
__weak void Can_sendFun(void const * argument)
{
  /* USER CODE BEGIN Can_sendFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Can_sendFun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
