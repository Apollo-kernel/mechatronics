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
int iButtonCount_UKEY;
int iButtonCount_Button;
int iButtonFlag_UKEY;
int iButtonFlag_Button;
int g_iButtonState_UKEY=0;
int g_iButtonState_Button=0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OledTimer */
osTimerId_t OledTimerHandle;
const osTimerAttr_t OledTimer_attributes = {
  .name = "OledTimer"
};
/* Definitions for msTimer */
osTimerId_t msTimerHandle;
const osTimerAttr_t msTimer_attributes = {
  .name = "msTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void OledCallback(void *argument);
void msTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the timer(s) */
  /* creation of OledTimer */
  OledTimerHandle = osTimerNew(OledCallback, osTimerPeriodic, NULL, &OledTimer_attributes);

  /* creation of msTimer */
  msTimerHandle = osTimerNew(msTimerCallback, osTimerPeriodic, NULL, &msTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    if(g_iButtonState_UKEY == 1)
    {
      HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
      g_iButtonState_UKEY=0;
    }
    if(g_iButtonState_Button == 1)
    {
      HAL_GPIO_TogglePin(LED_D7_Red_GPIO_Port,LED_D7_Red_Pin);
      HAL_GPIO_TogglePin(LED_D6_Green_GPIO_Port,LED_D6_Green_Pin);
      osDelay(3000);
      HAL_GPIO_TogglePin(LED_D6_Green_GPIO_Port,LED_D6_Green_Pin);
      HAL_GPIO_TogglePin(LED_D5_Yellow_GPIO_Port,LED_D5_Yellow_Pin);
      osDelay(500);
      HAL_GPIO_TogglePin(LED_D5_Yellow_GPIO_Port,LED_D5_Yellow_Pin);
      osDelay(500);
      HAL_GPIO_TogglePin(LED_D5_Yellow_GPIO_Port,LED_D5_Yellow_Pin);
      osDelay(500);
      HAL_GPIO_TogglePin(LED_D5_Yellow_GPIO_Port,LED_D5_Yellow_Pin);
      osDelay(500);
      HAL_GPIO_TogglePin(LED_D5_Yellow_GPIO_Port,LED_D5_Yellow_Pin);
      osDelay(500);
      HAL_GPIO_TogglePin(LED_D5_Yellow_GPIO_Port,LED_D5_Yellow_Pin);
      osDelay(500);
      HAL_GPIO_TogglePin(LED_D7_Red_GPIO_Port,LED_D7_Red_Pin);
      g_iButtonState_Button=0;
    }

    if( HAL_GPIO_ReadPin(UKEY_GPIO_Port,UKEY_Pin) == GPIO_PIN_RESET )
    {
      iButtonCount_UKEY++;
      if(iButtonCount_UKEY>=7)
      {
        if(iButtonFlag_UKEY==0)
        {
          g_iButtonState_UKEY=1;
          iButtonCount_UKEY=0;
          iButtonFlag_UKEY=1;
        }
        else
          iButtonCount_UKEY=0;
      }
      else
        g_iButtonState_UKEY=0;
    }
    else
    {
      iButtonCount_UKEY=0;
      g_iButtonState_UKEY=0;
      iButtonFlag_UKEY=0;
    }

    if( HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) == GPIO_PIN_RESET )
    {
      iButtonCount_Button++;
      if(iButtonCount_Button>=7)
      {
        if(iButtonFlag_Button==0)
        {
          g_iButtonState_Button=1;
          iButtonCount_Button=0;
          iButtonFlag_Button=1;
        }
        else
          iButtonCount_Button=0;
      }
      else
        g_iButtonState_Button=0;
    }
    else
    {
      iButtonCount_Button=0;
      g_iButtonState_Button=0;
      iButtonFlag_Button=0;
    }


    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* OledCallback function */
void OledCallback(void *argument)
{
  /* USER CODE BEGIN OledCallback */

  /* USER CODE END OledCallback */
}

/* msTimerCallback function */
void msTimerCallback(void *argument)
{
  /* USER CODE BEGIN msTimerCallback */

  /* USER CODE END msTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

