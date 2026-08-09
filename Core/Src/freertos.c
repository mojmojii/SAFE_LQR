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
#include "test_2.h"
#include "test_1.h"
#include "usart.h"
#include "oled_i2c.h"
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
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId myTask_1Handle;
osThreadId myTask_2Handle;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(myTask_1, StartTask_1, osPriorityIdle, 0, 512);
  myTask_1Handle = osThreadCreate(osThread(myTask_1), NULL);

  osThreadDef(myTask_2, StartTask_2, osPriorityIdle, 0, 512);
  myTask_2Handle = osThreadCreate(osThread(myTask_2), NULL);


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
uint8_t add;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint32_t last_display_ticks = UINT32_MAX;
  OLED_Init();
  /* Infinite loop */
  for(;;)
  {
    uint8_t adjustment_flash_finished = 0U;
    if (oled_stop_flash_requested)
    {
      oled_stop_flash_requested = 0U;
      OLED_SetInverted(1U);
      osDelay(1000);
      OLED_SetInverted(0U);
      if (lqr_adjustment_confirmed) adjustment_flash_finished = 1U;
    }

    if (lqr_adjustment_active || lqr_adjustment_confirmed)
    {
      int32_t quarter_cm = lqr_adjustment_confirmed
                             ? lqr_adjustment_final_quarter_cm
                             : lqr_adjustment_half_cm * 2;
      uint32_t magnitude = (uint32_t)((quarter_cm < 0) ? -quarter_cm : quarter_cm);
      OLED_Clear();
      OLED_SetCursor(0, 0);
      OLED_ShowString(lqr_adjustment_active ? "SET:" : "OK:");
      OLED_SetCursor(3, 0);
      OLED_ShowCharLarge((quarter_cm < 0) ? '-' : '+');
      OLED_ShowUIntLarge(magnitude / 4U);
      OLED_ShowCharLarge('.');
      uint32_t decimal = (magnitude % 4U) * 25U;
      OLED_ShowCharLarge((char)('0' + decimal / 10U));
      OLED_ShowCharLarge((char)('0' + decimal % 10U));
      OLED_Update();
      if (adjustment_flash_finished)
      {
        lqr_adjustment_confirm_until = HAL_GetTick() + 2000U;
        lqr_adjustment_flash_complete = 1U;
      }

      if (lqr_adjustment_confirmed &&
          (int32_t)(HAL_GetTick() - lqr_adjustment_confirm_until) >= 0)
      {
        lqr_adjustment_confirmed = 0U;
        last_display_ticks = UINT32_MAX;
      }
    }
    else if (oled_timing_enabled || oled_time_ticks != last_display_ticks)
    {
      uint32_t display_ticks = oled_time_ticks;
      last_display_ticks = display_ticks;
      OLED_Clear();
      OLED_SetCursor(0, 0);
      OLED_ShowString("TIME:");
      OLED_SetCursor(3, 0);
      OLED_ShowUIntLarge(display_ticks / 10U);
      OLED_ShowCharLarge('.');
      OLED_ShowUIntLarge(display_ticks % 10U);
      OLED_Update();
    }
    osDelay(500);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
