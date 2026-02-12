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
#include <stdio.h>
#include <string.h>
#include "oled_font.h"

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
float ELEC_inv=0.0f;
float ELEC_inc=0.0f;
float ELEC_outc=0.0f;
float ELEC_capv=0.0f;
float ELEC_capc=0.0f;
float ELEC_capp=0.0f;
float ELEC_inp=0.0f;
float ADC_VALUE=0.0f;
float ELEC_inp_capp=0.0f;
float ELEC_pe=0.0f;

typedef struct {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  min;
  uint8_t  sec;
} DateTime;

static DateTime g_time;


static char g_line2[17]; // y=2 date line (16 chars + '\0')
static char g_line4[17]; // y=4 time line (16 chars + '\0')


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
static uint8_t is_leap_year(uint16_t y);
static uint8_t days_in_month(uint16_t y, uint8_t m);

static void Clock_SetInitial(uint16_t y, uint8_t mo, uint8_t d,
                             uint8_t h, uint8_t mi, uint8_t s);
static void Clock_Tick1s(void);
static void Clock_DrawLine4(void);
static void Clock_DrawDateLine2(void);
static void Clock_DrawTimeLine4(void);


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
  // osTimerStart(OledTimerHandle,500);
  // osTimerStart(msTimerHandle,50);

  osTimerStart(OledTimerHandle,100);
  osTimerStart(msTimerHandle,10);


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

  // 放在 for(;;) 前面
  Clock_SetInitial(2026, 2, 11, 19, 30,58 );  // 你想要的初始时间：2026-02-10 12:00:00
  static uint32_t ms_Count = 0;

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

    if(++ms_Count >= 1000)
    {
      ms_Count = 0;

      Clock_Tick1s();
      taskENTER_CRITICAL();
      FPS = FPS_Count;
      FPS_Count = 0;
      taskEXIT_CRITICAL();

      Clock_DrawDateLine2();   // y=2
      Clock_DrawTimeLine4();   // y=4
      // FPS stays at y=6 from msTimerCallback(): DrawNum(80,6,...)
    }



  }
  /* USER CODE END StartDefaultTask */
}

/* OledCallback function */
void OledCallback(void *argument)
{
  /* USER CODE BEGIN OledCallback */
  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
  HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);



  /* USER CODE END OledCallback */
}

/* msTimerCallback function */
void msTimerCallback(void *argument)
{
  /* USER CODE BEGIN msTimerCallback */
  ++FPS_Count;
  DrawNum(80,6, FPS, 3);

  //
  // //			DrawNum(60,2, cap_v.valu[1], 5);
  // //			DrawNum(60,4, cap_c.valu[1], 5);
  // OLED_Showdecimal(9,0,debug_float[0],1,1);
  // OLED_Showdecimal(18,2,ELEC_inv,2,1);
  // OLED_Showdecimal(84,2,ELEC_capv,2,1);
  // OLED_Showdecimal(18,4,ELEC_inc,2,1);
  // OLED_Showdecimal(84,4,ELEC_capc,2,1);
  // //	OLED_Showdecimal(18,6,ELEC_outc,2,1);
  // OLED_Showdecimal(18,6,ELEC_inp,2,1);
  // OLED_Showdecimal(84,6,ELEC_capp,2,1);
  // OLED_Showdecimal(102,0,ELEC_inp_capp,1,1);
  // OLED_Showdecimal(48,0,ELEC_pe,2,1);

  //	HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
  //	HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
  HAL_I2C_Mem_Write_DMA(&hi2c3, OLED_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, ScreenBuffer[0], SCREEN_PAGE_NUM * SCREEN_PAGEDATA_NUM);



  /* USER CODE END msTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static uint8_t is_leap_year(uint16_t y)
{
  return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) ? 1 : 0;
}

static uint8_t days_in_month(uint16_t y, uint8_t m)
{
  static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return (uint8_t)(dim[1] + is_leap_year(y));
  if (m >= 1 && m <= 12) return dim[m - 1];
  return 31;
}

// 初始时间设置接口：你想设多少就设多少
static void Clock_SetInitial(uint16_t y, uint8_t mo, uint8_t d,
                             uint8_t h, uint8_t mi, uint8_t s)
{
  // 简单防呆（不合法就夹到合理范围）
  if (mo < 1) mo = 1; if (mo > 12) mo = 12;
  if (d < 1) d = 1;
  uint8_t maxd = days_in_month(y, mo);
  if (d > maxd) d = maxd;
  if (h > 23) h = 0;
  if (mi > 59) mi = 0;
  if (s > 59) s = 0;

  g_time.year  = y;
  g_time.month = mo;
  g_time.day   = d;
  g_time.hour  = h;
  g_time.min   = mi;
  g_time.sec   = s;
}

// 每秒调用一次：自增并处理进位（含日期进位、闰年）
static void Clock_Tick1s(void)
{
  g_time.sec++;
  if (g_time.sec >= 60) {
    g_time.sec = 0;
    g_time.min++;
    if (g_time.min >= 60) {
      g_time.min = 0;
      g_time.hour++;
      if (g_time.hour >= 24) {
        g_time.hour = 0;

        // 日期进位
        g_time.day++;
        uint8_t maxd = days_in_month(g_time.year, g_time.month);
        if (g_time.day > maxd) {
          g_time.day = 1;
          g_time.month++;
          if (g_time.month > 12) {
            g_time.month = 1;
            g_time.year++;
          }
        }
      }
    }
  }
}

// 生成显示字符串并画到(0,4)
static void Clock_DrawLine4(void)
{
  // 前面留三个空格，和你原来风格一致
  // OLED通常是等宽字体，建议固定宽度字段
  snprintf(g_line4, sizeof(g_line4),
           "   %04u-%02u-%02u %02u:%02u:%02u",
           g_time.year, g_time.month, g_time.day,
           g_time.hour, g_time.min, g_time.sec);

  DrawString(0, 4, g_line4);
}

static void Clock_DrawDateLine2(void)
{
  unsigned char old_size = GetFontSize();
  SetFontSize(0); // 8x16, 16 chars per line

  memset(g_line2, ' ', 16);
  g_line2[16] = '\0';

  // "   MM-DD-YYYY" (3 + 10 = 13 chars)
  uint16_t y = g_time.year;

  g_line2[3]  = '0' + (g_time.month / 10);
  g_line2[4]  = '0' + (g_time.month % 10);
  g_line2[5]  = '-';
  g_line2[6]  = '0' + (g_time.day / 10);
  g_line2[7]  = '0' + (g_time.day % 10);
  g_line2[8]  = '-';
  g_line2[9]  = '0' + (y / 1000) % 10;
  g_line2[10] = '0' + (y / 100)  % 10;
  g_line2[11] = '0' + (y / 10)   % 10;
  g_line2[12] = '0' + (y % 10);

  DrawString(0, 2, g_line2);

  SetFontSize(old_size);
}


static void Clock_DrawTimeLine4(void)
{
  unsigned char old_size = GetFontSize();
  SetFontSize(0); // 8x16, 16 chars per line

  memset(g_line4, ' ', 16);
  g_line4[16] = '\0';

  // "   HH:MM:SS" (3 + 8 = 11 chars)
  g_line4[3]  = '0' + (g_time.hour / 10);
  g_line4[4]  = '0' + (g_time.hour % 10);
  g_line4[5]  = ':';
  g_line4[6]  = '0' + (g_time.min / 10);
  g_line4[7]  = '0' + (g_time.min % 10);
  g_line4[8]  = ':';
  g_line4[9]  = '0' + (g_time.sec / 10);
  g_line4[10] = '0' + (g_time.sec % 10);

  DrawString(0, 4, g_line4);

  SetFontSize(old_size);
}


/* USER CODE END Application */

