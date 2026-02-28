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
#include "usart.h"
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
uint8_t rx_data[256] = {0};
uint8_t rx1_data[256] = {0};
uint8_t data_enable[10] = {0x01, 0xA0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F};
uint8_t data_speed_loop[10] = {0x01, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE4};
uint8_t data_30RPM[10] = {0x01, 0x64, 0x01, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6};
// +10 RPM  (10RPM -> 100 -> 0x0064)
uint8_t data_10RPM_CW[10]   = {0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F};

// -10 RPM  (-10RPM -> -100 -> 0xFF9C)
uint8_t data_10RPM_CCW[10]  = {0x01, 0x64, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A};

// (optional) -30 RPM  (-30RPM -> -300 -> 0xFED4)
uint8_t data_30RPM_CCW[10]  = {0x01, 0x64, 0xFE, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osTimerId myTimer01Handle;
osStaticTimerDef_t myTimer01ControlBlock;
osTimerId myTimer02Handle;
osStaticTimerDef_t myTimer02ControlBlock;
osTimerId myTimer03Handle;
osStaticTimerDef_t myTimer03ControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Callback01(void const * argument);
void Callback02(void const * argument);
void Callback03(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of myTimer01 */
  osTimerStaticDef(myTimer01, Callback01, &myTimer01ControlBlock);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* definition and creation of myTimer02 */
  osTimerStaticDef(myTimer02, Callback02, &myTimer02ControlBlock);
  myTimer02Handle = osTimerCreate(osTimer(myTimer02), osTimerPeriodic, NULL);

  /* definition and creation of myTimer03 */
  osTimerStaticDef(myTimer03, Callback03, &myTimer03ControlBlock);
  myTimer03Handle = osTimerCreate(osTimer(myTimer03), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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
  /* USER CODE BEGIN StartDefaultTask */
  HAL_GPIO_TogglePin(Power_5V_EN_GPIO_Port,Power_5V_EN_Pin);
  HAL_GPIO_TogglePin(Power_OUT2_EN_GPIO_Port,Power_OUT2_EN_Pin);
  HAL_GPIO_TogglePin(Power_OUT1_EN_GPIO_Port,Power_OUT1_EN_Pin);
  osDelay((3000));
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data, sizeof(rx_data));
  // __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx1_data, sizeof(rx1_data));
  // __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Transmit_DMA(&huart10, data_enable, sizeof(data_enable));
    // HAL_UART_Transmit_DMA(&huart1, data_enable, sizeof(data_enable));
    osDelay((100));
    HAL_UART_Transmit_DMA(&huart10, data_speed_loop, sizeof(data_speed_loop));
    // HAL_UART_Transmit_DMA(&huart1, data_speed_loop, sizeof(data_speed_loop));
    osDelay((100));
    HAL_UART_Transmit_DMA(&huart10, data_30RPM, sizeof(data_30RPM));
    // HAL_UART_Transmit_DMA(&huart1, data_30RPM, sizeof(data_30RPM));
     osDelay((1000));

    HAL_UART_Transmit_DMA(&huart10, data_30RPM_CCW, sizeof(data_30RPM_CCW));
    osDelay((1000));

    HAL_UART_Transmit_DMA(&huart10, data_10RPM_CW, sizeof(data_10RPM_CW));
    osDelay((1000));
    HAL_UART_Transmit_DMA(&huart10, data_10RPM_CCW, sizeof(data_10RPM_CCW));
    osDelay((3000));

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void const * argument)
{
  /* USER CODE BEGIN Callback02 */

  /* USER CODE END Callback02 */
}

/* Callback03 function */
void Callback03(void const * argument)
{
  /* USER CODE BEGIN Callback03 */

  /* USER CODE END Callback03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {

  //  if (huart->Instance == USART10)
  // {
  //   HAL_UART_Transmit_DMA(&huart1, rx1_data, Size);
  //   HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx1_data, sizeof(rx1_data));
  //   __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
  // }
  // if (huart->Instance == USART1)
  // {
  //   HAL_UART_Transmit_DMA(&huart10, rx_data, Size);
  //   // HAL_UART_Transmit_DMA(&huart1, rx_data, Size);
  //   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data, sizeof(rx_data));
  //   __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  // }
// }
/* USER CODE END Application */
