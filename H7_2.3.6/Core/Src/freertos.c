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
#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  M0603_MODE_OPEN    = 0x00,
  M0603_MODE_CURRENT = 0x01,
  M0603_MODE_SPEED   = 0x02,
  M0603_MODE_ENABLE  = 0x08,
  M0603_MODE_DISABLE = 0x09,
} m0603_mode_t;

typedef enum
{
  AUTO_IDLE = 0,
  AUTO_START,
  AUTO_WAIT_ENABLE,
  AUTO_WAIT_SPEED_MODE,
  AUTO_WAIT_FWD,
  AUTO_HOLD_FWD,
  AUTO_WAIT_REV,
  AUTO_HOLD_REV,
  AUTO_WAIT_STOP,
  AUTO_WAIT_QPOS,
  AUTO_WAIT_QVER,
  AUTO_WAIT_DISABLE,
  AUTO_DONE,
  AUTO_FAIL,
} auto_test_state_t;

typedef struct
{
  int16_t  speed_drpm;      /* 0.1 rpm */
  int16_t  current_raw;     /* -32767..32767 => -4A..4A */
  uint8_t  accel_time;
  uint8_t  temperature_c;
  uint8_t  err_code;
  int32_t  total_turns;
  uint16_t position_raw;    /* 0..32767 => 0..360 deg */
  uint8_t  mode;
  uint8_t  ver_year;
  uint8_t  ver_month;
  uint8_t  ver_day;
  uint8_t  ver_model;
  uint8_t  ver_sw;
  uint8_t  ver_hw;
  uint32_t last_update_tick;
  uint32_t last_pos_tick;
  uint32_t last_mode_tick;
  uint32_t last_ver_tick;
} motor_feedback_t;

typedef struct
{
  uint8_t  data[10];
  uint8_t  expected_cmd;
  uint8_t  retries_left;
  uint8_t  used;
  uint16_t timeout_ms;
  char     note[20];
} motor_tx_item_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLI_UART                    huart1
#define MOTOR_UART                  huart10
#define MOTOR_ID_DEFAULT            0x01u
#define M0603_FRAME_LEN             10u
#define CLI_RX_DMA_BUF_SZ           256u
#define MOTOR_RX_DMA_BUF_SZ         256u
#define CLI_LINE_BUF_SZ             128u
#define MOTOR_STREAM_BUF_SZ         128u
#define MOTOR_TX_QUEUE_LEN          8u
#define MOTOR_MIN_TX_INTERVAL_MS    4u      /* 250Hz max */
#define MOTOR_DEFAULT_TIMEOUT_MS    80u
#define MOTOR_DEFAULT_RETRY         2u
#define AUTO_HOLD_MS                2000u

#define CLI_TX_RING_SZ              2048u
#define CLI_TX_DMA_CHUNK_SZ         128u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAY_SIZE(x) ((uint32_t)(sizeof(x) / sizeof((x)[0])))
typedef struct
{
  UART_HandleTypeDef *huart;
  const char *name;                 /* "m7" / "m10" */
  uint8_t motor_id;

  uint8_t rx_dma[MOTOR_RX_DMA_BUF_SZ];
  uint8_t stream[MOTOR_STREAM_BUF_SZ];
  uint16_t stream_len;

  volatile uint8_t tx_busy;
  volatile uint32_t last_tx_tick;

  motor_tx_item_t q[MOTOR_TX_QUEUE_LEN];
  uint8_t q_head;
  uint8_t q_tail;
  uint8_t q_count;

  volatile uint8_t wait_reply;
  uint8_t wait_expected;
  char wait_note[20];
  uint32_t wait_deadline;
  motor_tx_item_t active_item;

  motor_feedback_t fb;
  auto_test_state_t auto_state;
  uint32_t auto_state_tick;
  uint32_t last_status_tick;
} motor_ctx_t;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t cli_rx_dma[CLI_RX_DMA_BUF_SZ];
static volatile uint8_t cli_tx_busy = 0;

static uint8_t cli_tx_ring[CLI_TX_RING_SZ];
static volatile uint16_t cli_tx_head = 0u;
static volatile uint16_t cli_tx_tail = 0u;

static uint8_t cli_tx_dma_buf[CLI_TX_DMA_CHUNK_SZ];
static volatile uint16_t cli_tx_dma_len = 0u;
// static uint8_t motor_rx_dma[MOTOR_RX_DMA_BUF_SZ];
// static uint8_t motor_stream[MOTOR_STREAM_BUF_SZ];
// static uint16_t motor_stream_len = 0;
//
static char cli_line[CLI_LINE_BUF_SZ];
static uint16_t cli_line_len = 0;
static volatile uint8_t cli_line_ready = 0;
//
// static volatile uint8_t cli_tx_busy = 0;
// static volatile uint8_t motor_tx_busy = 0;
// static volatile uint32_t motor_last_tx_tick = 0;
//
// static motor_tx_item_t motor_q[MOTOR_TX_QUEUE_LEN];
// static uint8_t motor_q_head = 0;
// static uint8_t motor_q_tail = 0;
// static uint8_t motor_q_count = 0;
//
// static volatile uint8_t motor_wait_reply = 0;
// static uint8_t motor_wait_expected = 0;
// static uint8_t motor_wait_note[20];
// static uint32_t motor_wait_deadline = 0;
// static motor_tx_item_t motor_active_item;
//
// static uint8_t g_motor_id = MOTOR_ID_DEFAULT;
// static motor_feedback_t g_fb;
// static auto_test_state_t g_auto_state = AUTO_IDLE;
// static uint32_t g_auto_state_tick = 0;
// static uint32_t g_last_status_tick = 0;


static motor_ctx_t g_m7;
static motor_ctx_t g_m10;

static const uint8_t crc8_maxim_table[256] = {
    0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
    157,195,33,127,252,162,64,30,95,1,227,189,62,96,130,220,
    35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
    190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
    70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
    219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
    101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
    248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
    140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
    17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
    175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
    50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
    202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
    87,9,235,181,54,104,138,212,149,203,41,119,244,170,72,22,
    233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
    116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53
};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 2048 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId CLITxTaskHandle;
uint32_t CLITxTask_Buffer[ 512 ];
osStaticThreadDef_t CLITxTask_ControlBlock;
osMessageQId cliEvtQHandle;
uint8_t cliEvtQBuffer[ 32 * sizeof( void * ) ];
osStaticMessageQDef_t cliEvtQControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void UART1_StartRxDMA(void);
static void cli_tx_enqueue(const uint8_t *data, uint16_t len);
static void cli_tx_kick_dma(void);
// static void UART10_StartRxDMA(void);
// static void UART10_StopRxDMA(void);
static uint32_t m0603_pos_raw_to_mdeg(uint16_t pos_raw);
static void motor_ctx_init(motor_ctx_t *m, UART_HandleTypeDef *huart, const char *name, uint8_t motor_id);
static void Motor7_StartRxDMA(void);
static void Motor10_StartRxDMA(void);
static void Motor_StopRxDMA(motor_ctx_t *m);

static void motor_dump_frame_prefix(motor_ctx_t *m, const char *tag, const uint8_t *f, uint16_t len);
static void motor_print_status(motor_ctx_t *m);

static int  motor_queue_push(motor_ctx_t *m, const uint8_t *frame, uint8_t expected_cmd, uint16_t timeout_ms, uint8_t retry, const char *note);
static void motor_service_tx(motor_ctx_t *m);
static void motor_service_timeout(motor_ctx_t *m);
static void motor_rx_consume(motor_ctx_t *m, const uint8_t *data, uint16_t len);
static void motor_parse_stream(motor_ctx_t *m);
static void motor_handle_frame(motor_ctx_t *m, const uint8_t *f);

static void auto_test_start(motor_ctx_t *m);
static void auto_test_stop(motor_ctx_t *m, const char *reason);
static void auto_test_service(motor_ctx_t *m);

static void cli_printf(const char *fmt, ...);
static void cli_prompt(void);
static void cli_process_line(char *line);
static void cli_rx_consume(const uint8_t *data, uint16_t len);

static uint8_t crc8_maxim_calc(const uint8_t *data, uint32_t len);
static void m0603_make_set_mode(uint8_t *buf, uint8_t id, uint8_t mode);
static void m0603_make_speed(uint8_t *buf, uint8_t id, int16_t speed_drpm);
static void m0603_make_current(uint8_t *buf, uint8_t id, float current_a);
static void m0603_make_open(uint8_t *buf, uint8_t id, int16_t open_raw);
static void m0603_make_query_pos(uint8_t *buf, uint8_t id);
static void m0603_make_query_mode(uint8_t *buf, uint8_t id);
static void m0603_make_query_ver(uint8_t *buf, uint8_t id);

// static int  motor_queue_push(const uint8_t *frame, uint8_t expected_cmd, uint16_t timeout_ms, uint8_t retry, const char *note);
// static void motor_service_tx(void);
// static void motor_service_timeout(void);
// static void motor_rx_consume(const uint8_t *data, uint16_t len);
// static void motor_parse_stream(void);
// static void motor_handle_frame(const uint8_t *f);
// static void motor_dump_frame(const char *tag, const uint8_t *f, uint16_t len);
// static void motor_print_status(void);
static const char *motor_mode_name(uint8_t mode);
//
// static void auto_test_start(void);
// static void auto_test_stop(const char *reason);
// static void auto_test_service(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_CLITxTask(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of cliEvtQ */
  osMessageQStaticDef(cliEvtQ, 32, void *, cliEvtQBuffer, &cliEvtQControlBlock);
  cliEvtQHandle = osMessageCreate(osMessageQ(cliEvtQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CLITxTask */
  osThreadStaticDef(CLITxTask, Start_CLITxTask, osPriorityLow, 0, 512, CLITxTask_Buffer, &CLITxTask_ControlBlock);
  CLITxTaskHandle = osThreadCreate(osThread(CLITxTask), NULL);

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
  HAL_GPIO_WritePin(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(3000);

  motor_ctx_init(&g_m7, &huart7, "m7", MOTOR_ID_DEFAULT);
  motor_ctx_init(&g_m10, &huart10, "m10", MOTOR_ID_DEFAULT);

  UART1_StartRxDMA();
  Motor7_StartRxDMA();
  Motor10_StartRxDMA();

  cli_printf("\r\nM0603A dual-motor auto-test CLI ready\r\n");
  cli_printf("USART1=CLI 115200, UART7+USART10=Motor half-duplex 38400\r\n");
  cli_printf("Default target = both motors. Prefix m7 / m10 for single motor.\r\n");
  cli_printf("type 'help'\r\n");
  cli_prompt();

  /* Infinite loop */
  for(;;)
  {

    motor_service_timeout(&g_m7);
    motor_service_tx(&g_m7);
    auto_test_service(&g_m7);

    motor_service_timeout(&g_m10);
    motor_service_tx(&g_m10);
    auto_test_service(&g_m10);

    if (cli_line_ready)
    {
      cli_line_ready = 0u;
      cli_printf("\r\n");
      cli_process_line(cli_line);
      memset(cli_line, 0, sizeof(cli_line));
    }

    if ((HAL_GetTick() - g_m7.last_status_tick) >= 1000u)
    {
      g_m7.last_status_tick = HAL_GetTick();
      if (g_m7.auto_state != AUTO_IDLE)
      {
        motor_print_status(&g_m7);
      }
    }

    if ((HAL_GetTick() - g_m10.last_status_tick) >= 1000u)
    {
      g_m10.last_status_tick = HAL_GetTick();
      if (g_m10.auto_state != AUTO_IDLE)
      {
        motor_print_status(&g_m10);
      }
    }

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_CLITxTask */
/**
* @brief Function implementing the CLITxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CLITxTask */
void Start_CLITxTask(void const * argument)
{
  /* USER CODE BEGIN Start_CLITxTask */

  /* Infinite loop */
  for(;;)
  {

    cli_tx_kick_dma();

    osDelay(1);
  }
  /* USER CODE END Start_CLITxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void UART1_StartRxDMA(void)
{
  // HAL_UART_DMAStop(&CLI_UART);
  if (HAL_UARTEx_ReceiveToIdle_DMA(&CLI_UART, cli_rx_dma, sizeof(cli_rx_dma)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(CLI_UART.hdmarx, DMA_IT_HT);
  }
}

static void motor_ctx_init(motor_ctx_t *m, UART_HandleTypeDef *huart, const char *name, uint8_t motor_id)
{
  memset(m, 0, sizeof(*m));
  m->huart = huart;
  m->name = name;
  m->motor_id = motor_id;
  m->auto_state = AUTO_IDLE;
}

static void Motor7_StartRxDMA(void)
{
  HAL_UART_DMAStop(&huart7);
  HAL_HalfDuplex_EnableReceiver(&huart7);
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, g_m7.rx_dma, sizeof(g_m7.rx_dma)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
  }
}

static void Motor10_StartRxDMA(void)
{
  HAL_UART_DMAStop(&huart10);
  HAL_HalfDuplex_EnableReceiver(&huart10);
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart10, g_m10.rx_dma, sizeof(g_m10.rx_dma)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
  }
}

static void Motor_StopRxDMA(motor_ctx_t *m)
{
  HAL_UART_DMAStop(m->huart);
}

static void cli_tx_enqueue(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((data == NULL) || (len == 0u))
  {
    return;
  }

  taskENTER_CRITICAL();
  for (i = 0; i < len; i++)
  {
    uint16_t next_head = (uint16_t)((cli_tx_head + 1u) % CLI_TX_RING_SZ);

    /* ring full: drop newest bytes */
    if (next_head == cli_tx_tail)
    {
      break;
    }

    cli_tx_ring[cli_tx_head] = data[i];
    cli_tx_head = next_head;
  }
  taskEXIT_CRITICAL();
}

static void cli_tx_kick_dma(void)
{
  uint16_t len = 0u;

  if (cli_tx_busy)
  {
    return;
  }

  taskENTER_CRITICAL();
  while ((cli_tx_tail != cli_tx_head) && (len < CLI_TX_DMA_CHUNK_SZ))
  {
    cli_tx_dma_buf[len++] = cli_tx_ring[cli_tx_tail];
    cli_tx_tail = (uint16_t)((cli_tx_tail + 1u) % CLI_TX_RING_SZ);
  }

  if (len > 0u)
  {
    cli_tx_busy = 1u;
    cli_tx_dma_len = len;
  }
  taskEXIT_CRITICAL();

  if (len > 0u)
  {
    if (HAL_UART_Transmit_DMA(&CLI_UART, cli_tx_dma_buf, len) != HAL_OK)
    {
      taskENTER_CRITICAL();
      cli_tx_busy = 0u;
      cli_tx_dma_len = 0u;
      taskEXIT_CRITICAL();
    }
  }
}

static void cli_printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return;
  }

  if (n >= (int)sizeof(buf))
  {
    n = (int)(sizeof(buf) - 1u);
  }

  HAL_UART_Transmit(&CLI_UART, (uint8_t *)buf, (uint16_t)n, 1000);
}

static void cli_prompt(void)
{
  cli_printf("m0603> ");
}

static uint8_t crc8_maxim_calc(const uint8_t *data, uint32_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    crc = crc8_maxim_table[crc ^ *data++];
  }
  return crc;
}

static uint32_t m0603_pos_raw_to_mdeg(uint16_t pos_raw)
{
  return (uint32_t)(((uint64_t)pos_raw * 360000ULL) / 32767ULL);
}

static void m0603_make_set_mode(uint8_t *buf, uint8_t id, uint8_t mode)
{
  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0xA0;
  buf[2] = mode;
  buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_speed(uint8_t *buf, uint8_t id, int16_t speed_drpm)
{
  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0x64;
  buf[2] = (uint8_t)((uint16_t)speed_drpm >> 8);
  buf[3] = (uint8_t)(speed_drpm & 0xFF);
  buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_open(uint8_t *buf, uint8_t id, int16_t open_raw)
{
  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0x64;
  buf[2] = (uint8_t)((uint16_t)open_raw >> 8);
  buf[3] = (uint8_t)(open_raw & 0xFF);
  buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_current(uint8_t *buf, uint8_t id, float current_a)
{
  int16_t raw;
  if (current_a > 3.9f) current_a = 3.9f;
  if (current_a < -3.9f) current_a = -3.9f;
  raw = (int16_t)(current_a * (32767.0f / 4.0f));

  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0x64;
  buf[2] = (uint8_t)((uint16_t)raw >> 8);
  buf[3] = (uint8_t)(raw & 0xFF);
  buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_query_pos(uint8_t *buf, uint8_t id)
{
  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0x74;
  buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_query_mode(uint8_t *buf, uint8_t id)
{
  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0x75;
  buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_query_ver(uint8_t *buf, uint8_t id)
{
  memset(buf, 0, M0603_FRAME_LEN);
  buf[0] = id;
  buf[1] = 0xFD;
  buf[9] = crc8_maxim_calc(buf, 9);
}

static int motor_queue_push(motor_ctx_t *m, const uint8_t *frame, uint8_t expected_cmd, uint16_t timeout_ms, uint8_t retry, const char *note)
{
  motor_tx_item_t *it;
  if (m->q_count >= MOTOR_TX_QUEUE_LEN)
  {
    cli_printf("%s queue full\r\n", m->name);
    return -1;
  }

  it = &m->q[m->q_tail];
  memset(it, 0, sizeof(*it));
  memcpy(it->data, frame, M0603_FRAME_LEN);
  it->expected_cmd = expected_cmd;
  it->timeout_ms = timeout_ms;
  it->retries_left = retry;
  it->used = 1;
  if (note != NULL)
  {
    strncpy(it->note, note, sizeof(it->note) - 1);
  }

  m->q_tail = (uint8_t)((m->q_tail + 1u) % MOTOR_TX_QUEUE_LEN);
  m->q_count++;
  return 0;
}

static void motor_dump_frame(const char *tag, const uint8_t *f, uint16_t len)
{
  uint16_t i;
  cli_printf("%s", tag);
  for (i = 0; i < len; i++)
  {
    cli_printf("%02X%s", f[i], (i + 1u < len) ? " " : "");
  }
  cli_printf("\r\n");
}

static void motor_dump_frame_prefix(motor_ctx_t *m, const char *tag, const uint8_t *f, uint16_t len)
{
  uint16_t i;
  cli_printf("%s %s", m->name, tag);
  for (i = 0; i < len; i++)
  {
    cli_printf("%02X%s", f[i], (i + 1u < len) ? " " : "");
  }
  cli_printf("\r\n");
}

static const char *motor_mode_name(uint8_t mode)
{
  switch (mode)
  {
    case 0x00: return "open";
    case 0x01: return "current";
    case 0x02: return "speed";
    case 0x08: return "enable";
    case 0x09: return "disable";
    default:   return "unknown";
  }
}

static void motor_print_status(motor_ctx_t *m)
{
  int32_t curr_mA = ((int32_t)m->fb.current_raw * 4000) / 32767;
  uint32_t curr_abs = (uint32_t)((curr_mA < 0) ? -curr_mA : curr_mA);
  uint32_t pos_mdeg = m0603_pos_raw_to_mdeg(m->fb.position_raw);
  int16_t rpm10 = m->fb.speed_drpm;
  int16_t rpm_abs = (int16_t)((rpm10 < 0) ? -rpm10 : rpm10);

  cli_printf("[%s] id=%u mode=%s rpm=%s%d.%d I=%s%lu.%03luA T=%uC err=0x%02X turns=%ld pos=%lu.%03ludeg\r\n",
             m->name,
             m->motor_id,
             motor_mode_name(m->fb.mode),
             (rpm10 < 0) ? "-" : "",
             rpm_abs / 10,
             rpm_abs % 10,
             (curr_mA < 0) ? "-" : "",
             curr_abs / 1000u,
             curr_abs % 1000u,
             m->fb.temperature_c,
             m->fb.err_code,
             (long)m->fb.total_turns,
             pos_mdeg / 1000u,
             pos_mdeg % 1000u);

  if (m->fb.last_ver_tick != 0u)
  {
    cli_printf("[%s] ver=20%02u-%02u-%02u model=0x%02X sw=0x%02X hw=0x%02X\r\n",
               m->name,
               m->fb.ver_year, m->fb.ver_month, m->fb.ver_day,
               m->fb.ver_model, m->fb.ver_sw, m->fb.ver_hw);
  }
}

static void motor_handle_frame(motor_ctx_t *m, const uint8_t *f)
{
  uint8_t cmd = f[1];

  if (f[0] != m->motor_id)
  {
    return;
  }

  switch (cmd)
  {
    case 0x65:
      m->fb.speed_drpm = (int16_t)(((uint16_t)f[2] << 8) | f[3]);
      m->fb.current_raw = (int16_t)(((uint16_t)f[4] << 8) | f[5]);
      m->fb.accel_time = f[6];
      m->fb.temperature_c = f[7];
      m->fb.err_code = f[8];
      m->fb.last_update_tick = HAL_GetTick();
      break;

    case 0x75:
      m->fb.total_turns = ((int32_t)f[2] << 24) | ((int32_t)f[3] << 16) | ((int32_t)f[4] << 8) | f[5];
      m->fb.position_raw = (uint16_t)(((uint16_t)f[6] << 8) | f[7]);
      m->fb.err_code = f[8];
      m->fb.last_pos_tick = HAL_GetTick();
      break;

    case 0x76:
      m->fb.mode = f[2];
      m->fb.last_mode_tick = HAL_GetTick();
      break;

    case 0xA1:
      m->fb.mode = f[2];
      m->fb.last_mode_tick = HAL_GetTick();
      break;

    case 0xFE:
      m->fb.ver_year  = f[2];
      m->fb.ver_month = f[3];
      m->fb.ver_day   = f[4];
      m->fb.ver_model = f[5];
      m->fb.ver_sw    = f[6];
      m->fb.ver_hw    = f[7];
      m->fb.last_ver_tick = HAL_GetTick();
      break;

    default:
      break;
  }

  motor_dump_frame_prefix(m, "rx: ", f, M0603_FRAME_LEN);

  if (m->wait_reply && (cmd == m->wait_expected))
  {
    m->wait_reply = 0;

    if (cmd == 0x75)
    {
      uint32_t pos_mdeg = m0603_pos_raw_to_mdeg(m->fb.position_raw);
      cli_printf("%s ack: %s -> turns=%ld pos_raw=%u pos=%lu.%03ludeg err=0x%02X\r\n",
                 m->name,
                 m->wait_note,
                 (long)m->fb.total_turns,
                 (unsigned)m->fb.position_raw,
                 pos_mdeg / 1000u,
                 pos_mdeg % 1000u,
                 m->fb.err_code);
    }
    else if (cmd == 0x65)
    {
      int32_t curr_mA = ((int32_t)m->fb.current_raw * 4000) / 32767;
      uint32_t curr_abs = (uint32_t)((curr_mA < 0) ? -curr_mA : curr_mA);
      int16_t rpm10 = m->fb.speed_drpm;
      int16_t rpm_abs = (int16_t)((rpm10 < 0) ? -rpm10 : rpm10);

      cli_printf("%s ack: %s -> rpm=%s%d.%d I=%s%lu.%03luA T=%uC err=0x%02X\r\n",
                 m->name,
                 m->wait_note,
                 (rpm10 < 0) ? "-" : "",
                 rpm_abs / 10,
                 rpm_abs % 10,
                 (curr_mA < 0) ? "-" : "",
                 curr_abs / 1000u,
                 curr_abs % 1000u,
                 m->fb.temperature_c,
                 m->fb.err_code);
    }
    else if (cmd == 0xA1 || cmd == 0x76)
    {
      cli_printf("%s ack: %s -> mode=%s\r\n",
                 m->name,
                 m->wait_note,
                 motor_mode_name(m->fb.mode));
    }
    else if (cmd == 0xFE)
    {
      cli_printf("%s ack: %s -> ver=20%02u-%02u-%02u model=0x%02X sw=0x%02X hw=0x%02X\r\n",
                 m->name,
                 m->wait_note,
                 m->fb.ver_year, m->fb.ver_month, m->fb.ver_day,
                 m->fb.ver_model, m->fb.ver_sw, m->fb.ver_hw);
    }
    else
    {
      cli_printf("%s ack: %s\r\n", m->name, m->wait_note);
    }
  }
}

static void motor_parse_stream(motor_ctx_t *m)
{
  uint16_t i;
  int found = 0;

  while (m->stream_len >= M0603_FRAME_LEN)
  {
    found = 0;
    for (i = 0; i + M0603_FRAME_LEN <= m->stream_len; i++)
    {
      const uint8_t *f = &m->stream[i];
      uint8_t cmd = f[1];
      if (f[0] != m->motor_id)
      {
        continue;
      }
      if ((cmd != 0x65) && (cmd != 0x75) && (cmd != 0x76) && (cmd != 0xA1) && (cmd != 0xFE))
      {
        continue;
      }
      if (crc8_maxim_calc(f, 9) != f[9])
      {
        continue;
      }

      motor_handle_frame(m, f);

      memmove(m->stream, &m->stream[i + M0603_FRAME_LEN], m->stream_len - (i + M0603_FRAME_LEN));
      m->stream_len = (uint16_t)(m->stream_len - (i + M0603_FRAME_LEN));
      found = 1;
      break;
    }

    if (!found)
    {
      if (m->stream_len > (M0603_FRAME_LEN - 1u))
      {
        memmove(m->stream, &m->stream[m->stream_len - (M0603_FRAME_LEN - 1u)], M0603_FRAME_LEN - 1u);
        m->stream_len = M0603_FRAME_LEN - 1u;
      }
      break;
    }
  }
}

static void motor_rx_consume(motor_ctx_t *m, const uint8_t *data, uint16_t len)
{
  uint16_t copy_len;
  if (len == 0u)
  {
    return;
  }

  copy_len = len;
  if (copy_len > (uint16_t)(MOTOR_STREAM_BUF_SZ - m->stream_len))
  {
    copy_len = (uint16_t)(MOTOR_STREAM_BUF_SZ - m->stream_len);
  }
  if (copy_len == 0u)
  {
    m->stream_len = 0u;
    return;
  }

  memcpy(&m->stream[m->stream_len], data, copy_len);
  m->stream_len = (uint16_t)(m->stream_len + copy_len);
  motor_parse_stream(m);
}

static void motor_service_tx(motor_ctx_t *m)
{
  motor_tx_item_t *it;
  uint32_t now = HAL_GetTick();

  if ((m->q_count == 0u) || m->tx_busy || m->wait_reply)
  {
    return;
  }
  if ((now - m->last_tx_tick) < MOTOR_MIN_TX_INTERVAL_MS)
  {
    return;
  }

  it = &m->q[m->q_head];
  if (!it->used)
  {
    m->q_head = (uint8_t)((m->q_head + 1u) % MOTOR_TX_QUEUE_LEN);
    m->q_count--;
    return;
  }

  Motor_StopRxDMA(m);
  HAL_HalfDuplex_EnableTransmitter(m->huart);

  if (HAL_UART_Transmit_DMA(m->huart, it->data, M0603_FRAME_LEN) == HAL_OK)
  {
    m->tx_busy = 1u;
    m->last_tx_tick = now;
    m->active_item = *it;

    if (it->expected_cmd != 0u)
    {
      m->wait_reply = 1u;
      m->wait_expected = it->expected_cmd;
      m->wait_deadline = now + it->timeout_ms;
      memset(m->wait_note, 0, sizeof(m->wait_note));
      memcpy(m->wait_note, it->note, sizeof(it->note));
    }

    motor_dump_frame_prefix(m, "tx: ", it->data, M0603_FRAME_LEN);

    memset(it, 0, sizeof(*it));
    m->q_head = (uint8_t)((m->q_head + 1u) % MOTOR_TX_QUEUE_LEN);
    m->q_count--;
  }
  else
  {
    HAL_HalfDuplex_EnableReceiver(m->huart);
    if (m == &g_m7) Motor7_StartRxDMA();
    else if (m == &g_m10) Motor10_StartRxDMA();
  }
}

static void motor_service_timeout(motor_ctx_t *m)
{
  uint32_t now = HAL_GetTick();

  if (!m->wait_reply)
  {
    return;
  }
  if ((int32_t)(now - m->wait_deadline) < 0)
  {
    return;
  }

  cli_printf("%s timeout: %s\r\n", m->name, m->wait_note);
  m->wait_reply = 0u;

  if (m->active_item.retries_left > 0u)
  {
    m->active_item.retries_left--;
    motor_queue_push(m,
                     m->active_item.data,
                     m->active_item.expected_cmd,
                     m->active_item.timeout_ms,
                     m->active_item.retries_left,
                     m->active_item.note);
    cli_printf("%s retry left=%u\r\n", m->name, m->active_item.retries_left);
  }
  else if ((m->auto_state != AUTO_IDLE) && (m->auto_state != AUTO_DONE))
  {
    m->auto_state = AUTO_FAIL;
  }
}

static void cli_rx_consume(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len; i++)
  {
    char c = (char)data[i];

    if (c == '\r' || c == '\n')
    {
      if (cli_line_len > 0u)
      {
        cli_line[cli_line_len] = '\0';
        cli_line_ready = 1u;
        cli_line_len = 0u;
      }
      continue;
    }

    if ((c == '\b' || c == 127) && cli_line_len > 0u)
    {
      cli_line_len--;
      continue;
    }

    if ((uint8_t)c >= 32u && cli_line_len < (CLI_LINE_BUF_SZ - 1u))
    {
      cli_line[cli_line_len++] = c;
    }
  }
}

static void print_help(void)
{
  cli_printf("help                    : show help\r\n");
  cli_printf("Default target: both motors. Prefix with 'm7' or 'm10' for single motor.\r\n");
  cli_printf("\r\n");

  cli_printf("en | dis                : enable / disable motor\r\n");
  cli_printf("mode open|cur|spd       : set motor mode\r\n");
  cli_printf("spd <rpm>               : speed command, example: spd 30\r\n");
  cli_printf("cur <amp>               : current command, example: cur 0.5\r\n");
  cli_printf("open <value>            : open-loop command, example: open 1000\r\n");
  cli_printf("qpos                    : query total turns + position\r\n");
  cli_printf("qmode                   : query current mode\r\n");
  cli_printf("qver                    : query version\r\n");
  cli_printf("stat                    : print latest cached status\r\n");
  cli_printf("raw xx xx .. xx         : send raw 10-byte hex frame\r\n");
  cli_printf("map                     : print protocol mapping\r\n");
  cli_printf("auto start | auto stop  : run / stop auto test\r\n");
  cli_printf("\r\n");

  cli_printf("m7 <cmd>                : apply command to UART7 motor only\r\n");
  cli_printf("m10 <cmd>               : apply command to USART10 motor only\r\n");
}

static void print_map(void)
{
  cli_printf("A0 -> set mode, ack A1\r\n");
  cli_printf("64 -> drive cmd, ack 65(speed/current/temp/err)\r\n");
  cli_printf("74 -> query turns/pos, ack 75\r\n");
  cli_printf("75 -> query mode, ack 76\r\n");
  cli_printf("FD -> query version, ack FE\r\n");
  cli_printf("mode: 00=open 01=current 02=speed 08=enable 09=disable\r\n");
}

static void cli_process_line(char *line)
{
  char *argv[16];
  int argc = 0;
  char *tok;
  uint8_t frame[10];
  motor_ctx_t *target = NULL;
  int base = 0;
  bool target_is_both = true;

  tok = strtok(line, " \t");
  while ((tok != NULL) && (argc < (int)ARRAY_SIZE(argv)))
  {
    argv[argc++] = tok;
    tok = strtok(NULL, " \t");
  }
  if (argc == 0)
  {
    cli_prompt();
    return;
  }

  if (strcmp(argv[0], "help") == 0)
  {
    print_help();
    cli_prompt();
    return;
  }

  if (strcmp(argv[0], "m7") == 0)
  {
    target = &g_m7;
    base = 1;
    target_is_both = false;
  }
  else if (strcmp(argv[0], "m10") == 0)
  {
    target = &g_m10;
    base = 1;
    target_is_both = false;
  }
  else
  {
    base = 0;
    target_is_both = true;
  }

  if (argc <= base)
  {
    cli_prompt();
    return;
  }

  if (strcmp(argv[base], "en") == 0)
  {
    if (target_is_both)
    {
      m0603_make_set_mode(frame, g_m7.motor_id, M0603_MODE_ENABLE);
      motor_queue_push(&g_m7, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "enable");

      m0603_make_set_mode(frame, g_m10.motor_id, M0603_MODE_ENABLE);
      motor_queue_push(&g_m10, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "enable");
    }
    else
    {
      m0603_make_set_mode(frame, target->motor_id, M0603_MODE_ENABLE);
      motor_queue_push(target, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "enable");
    }
  }
  else if (strcmp(argv[base], "dis") == 0)
  {
    if (target_is_both)
    {
      m0603_make_set_mode(frame, g_m7.motor_id, M0603_MODE_DISABLE);
      motor_queue_push(&g_m7, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "disable");

      m0603_make_set_mode(frame, g_m10.motor_id, M0603_MODE_DISABLE);
      motor_queue_push(&g_m10, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "disable");
    }
    else
    {
      m0603_make_set_mode(frame, target->motor_id, M0603_MODE_DISABLE);
      motor_queue_push(target, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "disable");
    }
  }
  else if (strcmp(argv[base], "mode") == 0)
  {
    uint8_t mode = 0xFF;
    if (argc < base + 2)
    {
      cli_printf("usage: [m7|m10] mode open|cur|spd\r\n");
    }
    else
    {
      if (strcmp(argv[base + 1], "open") == 0) mode = M0603_MODE_OPEN;
      if (strcmp(argv[base + 1], "cur")  == 0) mode = M0603_MODE_CURRENT;
      if (strcmp(argv[base + 1], "spd")  == 0) mode = M0603_MODE_SPEED;

      if (mode == 0xFF)
      {
        cli_printf("bad mode\r\n");
      }
      else if (target_is_both)
      {
        m0603_make_set_mode(frame, g_m7.motor_id, mode);
        motor_queue_push(&g_m7, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "mode");

        m0603_make_set_mode(frame, g_m10.motor_id, mode);
        motor_queue_push(&g_m10, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "mode");
      }
      else
      {
        m0603_make_set_mode(frame, target->motor_id, mode);
        motor_queue_push(target, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "mode");
      }
    }
  }
  else if (strcmp(argv[base], "spd") == 0)
  {
    long rpm;
    if (argc < base + 2)
    {
      cli_printf("usage: [m7|m10] spd <rpm>\r\n");
    }
    else
    {
      rpm = strtol(argv[base + 1], NULL, 0);
      if (rpm > 380L) rpm = 380L;
      if (rpm < -380L) rpm = -380L;

      if (target_is_both)
      {
        m0603_make_speed(frame, g_m7.motor_id, (int16_t)(rpm * 10L));
        motor_queue_push(&g_m7, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "speed");

        m0603_make_speed(frame, g_m10.motor_id, (int16_t)(rpm * 10L));
        motor_queue_push(&g_m10, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "speed");
      }
      else
      {
        m0603_make_speed(frame, target->motor_id, (int16_t)(rpm * 10L));
        motor_queue_push(target, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "speed");
      }
    }
  }
  else if (strcmp(argv[base], "cur") == 0)
  {
    float current;
    if (argc < base + 2)
    {
      cli_printf("usage: [m7|m10] cur <amp>\r\n");
    }
    else
    {
      current = strtof(argv[base + 1], NULL);

      if (target_is_both)
      {
        m0603_make_current(frame, g_m7.motor_id, current);
        motor_queue_push(&g_m7, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "current");

        m0603_make_current(frame, g_m10.motor_id, current);
        motor_queue_push(&g_m10, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "current");
      }
      else
      {
        m0603_make_current(frame, target->motor_id, current);
        motor_queue_push(target, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "current");
      }
    }
  }
  else if (strcmp(argv[base], "open") == 0)
  {
    long open_val;
    if (argc < base + 2)
    {
      cli_printf("usage: [m7|m10] open <value>\r\n");
    }
    else
    {
      open_val = strtol(argv[base + 1], NULL, 0);
      if (open_val > 32767L) open_val = 32767L;
      if (open_val < -32768L) open_val = -32768L;

      if (target_is_both)
      {
        m0603_make_open(frame, g_m7.motor_id, (int16_t)open_val);
        motor_queue_push(&g_m7, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "open");

        m0603_make_open(frame, g_m10.motor_id, (int16_t)open_val);
        motor_queue_push(&g_m10, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "open");
      }
      else
      {
        m0603_make_open(frame, target->motor_id, (int16_t)open_val);
        motor_queue_push(target, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "open");
      }
    }
  }
  else if (strcmp(argv[base], "qpos") == 0)
  {
    if (target_is_both)
    {
      m0603_make_query_pos(frame, g_m7.motor_id);
      motor_queue_push(&g_m7, frame, 0x75, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "qpos");

      m0603_make_query_pos(frame, g_m10.motor_id);
      motor_queue_push(&g_m10, frame, 0x75, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "qpos");
    }
    else
    {
      m0603_make_query_pos(frame, target->motor_id);
      motor_queue_push(target, frame, 0x75, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "qpos");
    }
  }
  else if (strcmp(argv[base], "qmode") == 0)
  {
    if (target_is_both)
    {
      m0603_make_query_mode(frame, g_m7.motor_id);
      motor_queue_push(&g_m7, frame, 0x76, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "qmode");

      m0603_make_query_mode(frame, g_m10.motor_id);
      motor_queue_push(&g_m10, frame, 0x76, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "qmode");
    }
    else
    {
      m0603_make_query_mode(frame, target->motor_id);
      motor_queue_push(target, frame, 0x76, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "qmode");
    }
  }
  else if (strcmp(argv[base], "qver") == 0)
  {
    if (target_is_both)
    {
      m0603_make_query_ver(frame, g_m7.motor_id);
      motor_queue_push(&g_m7, frame, 0xFE, 120u, MOTOR_DEFAULT_RETRY, "qver");

      m0603_make_query_ver(frame, g_m10.motor_id);
      motor_queue_push(&g_m10, frame, 0xFE, 120u, MOTOR_DEFAULT_RETRY, "qver");
    }
    else
    {
      m0603_make_query_ver(frame, target->motor_id);
      motor_queue_push(target, frame, 0xFE, 120u, MOTOR_DEFAULT_RETRY, "qver");
    }
  }
  else if (strcmp(argv[base], "stat") == 0)
  {
    if (target_is_both)
    {
      motor_print_status(&g_m7);
      motor_print_status(&g_m10);
    }
    else
    {
      motor_print_status(target);
    }
  }
  else if (strcmp(argv[base], "map") == 0)
  {
    print_map();
  }
  else if (strcmp(argv[base], "raw") == 0)
  {
    int i;
    if (argc != base + 11)
    {
      cli_printf("usage: [m7|m10] raw xx xx xx xx xx xx xx xx xx xx\r\n");
    }
    else
    {
      for (i = 0; i < 10; i++)
      {
        frame[i] = (uint8_t)strtoul(argv[base + 1 + i], NULL, 16);
      }

      if (target_is_both)
      {
        motor_queue_push(&g_m7, frame, 0x00, MOTOR_DEFAULT_TIMEOUT_MS, 0u, "raw");
        motor_queue_push(&g_m10, frame, 0x00, MOTOR_DEFAULT_TIMEOUT_MS, 0u, "raw");
      }
      else
      {
        motor_queue_push(target, frame, 0x00, MOTOR_DEFAULT_TIMEOUT_MS, 0u, "raw");
      }
    }
  }
  else if (strcmp(argv[base], "auto") == 0)
  {
    if ((argc >= base + 2) && (strcmp(argv[base + 1], "start") == 0))
    {
      if (target_is_both)
      {
        auto_test_start(&g_m7);
        auto_test_start(&g_m10);
      }
      else
      {
        auto_test_start(target);
      }
    }
    else if ((argc >= base + 2) && (strcmp(argv[base + 1], "stop") == 0))
    {
      if (target_is_both)
      {
        auto_test_stop(&g_m7, "manual stop");
        auto_test_stop(&g_m10, "manual stop");
      }
      else
      {
        auto_test_stop(target, "manual stop");
      }
    }
    else
    {
      cli_printf("usage: [m7|m10] auto start|stop\r\n");
    }
  }
  else
  {
    cli_printf("unknown cmd: %s\r\n", argv[base]);
  }

  cli_prompt();
}

static void auto_test_start(motor_ctx_t *m)
{
  if (m->auto_state != AUTO_IDLE && m->auto_state != AUTO_DONE && m->auto_state != AUTO_FAIL)
  {
    cli_printf("%s auto already running\r\n", m->name);
    return;
  }
  m->auto_state = AUTO_START;
  m->auto_state_tick = HAL_GetTick();
  cli_printf("%s auto start\r\n", m->name);
}

static void auto_test_stop(motor_ctx_t *m, const char *reason)
{
  m->auto_state = AUTO_IDLE;
  cli_printf("%s auto stop: %s\r\n", m->name, reason);
}

static void auto_test_service(motor_ctx_t *m)
{
  uint8_t frame[10];
  uint32_t now = HAL_GetTick();

  switch (m->auto_state)
  {
    case AUTO_IDLE:
    case AUTO_DONE:
    case AUTO_FAIL:
      return;

    case AUTO_START:
      m0603_make_set_mode(frame, m->motor_id, M0603_MODE_ENABLE);
      motor_queue_push(m, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto enable");
      m->auto_state = AUTO_WAIT_ENABLE;
      break;

    case AUTO_WAIT_ENABLE:
      if (!m->wait_reply && m->q_count == 0u)
      {
        m0603_make_set_mode(frame, m->motor_id, M0603_MODE_SPEED);
        motor_queue_push(m, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto speed mode");
        m->auto_state = AUTO_WAIT_SPEED_MODE;
      }
      break;

    case AUTO_WAIT_SPEED_MODE:
      if (!m->wait_reply && m->q_count == 0u)
      {
        m0603_make_speed(frame, m->motor_id, 300);
        motor_queue_push(m, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto +30rpm");
        m->auto_state = AUTO_WAIT_FWD;
      }
      break;

    case AUTO_WAIT_FWD:
      if (!m->wait_reply)
      {
        m->auto_state = AUTO_HOLD_FWD;
        m->auto_state_tick = now;
      }
      break;

    case AUTO_HOLD_FWD:
      if ((now - m->auto_state_tick) >= AUTO_HOLD_MS)
      {
        m0603_make_speed(frame, m->motor_id, -300);
        motor_queue_push(m, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto -30rpm");
        m->auto_state = AUTO_WAIT_REV;
      }
      break;

    case AUTO_WAIT_REV:
      if (!m->wait_reply)
      {
        m->auto_state = AUTO_HOLD_REV;
        m->auto_state_tick = now;
      }
      break;

    case AUTO_HOLD_REV:
      if ((now - m->auto_state_tick) >= AUTO_HOLD_MS)
      {
        m0603_make_speed(frame, m->motor_id, 0);
        motor_queue_push(m, frame, 0x65, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto stop");
        m->auto_state = AUTO_WAIT_STOP;
      }
      break;

    case AUTO_WAIT_STOP:
      if (!m->wait_reply)
      {
        m0603_make_query_pos(frame, m->motor_id);
        motor_queue_push(m, frame, 0x75, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto qpos");
        m->auto_state = AUTO_WAIT_QPOS;
      }
      break;

    case AUTO_WAIT_QPOS:
      if (!m->wait_reply)
      {
        m0603_make_query_ver(frame, m->motor_id);
        motor_queue_push(m, frame, 0xFE, 120u, MOTOR_DEFAULT_RETRY, "auto qver");
        m->auto_state = AUTO_WAIT_QVER;
      }
      break;

    case AUTO_WAIT_QVER:
      if (!m->wait_reply)
      {
        m0603_make_set_mode(frame, m->motor_id, M0603_MODE_DISABLE);
        motor_queue_push(m, frame, 0xA1, MOTOR_DEFAULT_TIMEOUT_MS, MOTOR_DEFAULT_RETRY, "auto disable");
        m->auto_state = AUTO_WAIT_DISABLE;
      }
      break;

    case AUTO_WAIT_DISABLE:
      if (!m->wait_reply)
      {
        m->auto_state = AUTO_DONE;
        cli_printf("%s auto done\r\n", m->name);
        motor_print_status(m);
      }
      break;

    default:
      break;
  }

  if (m->auto_state == AUTO_FAIL)
  {
    cli_printf("%s auto fail\r\n", m->name);
    m->auto_state = AUTO_IDLE;
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    if (huart->Instance == UART7)
    {
      HAL_UART_DMAStop(&huart7);
      HAL_UART_Abort(&huart7);
      __HAL_UART_CLEAR_OREFLAG(&huart7);
      __HAL_UART_CLEAR_FEFLAG(&huart7);
      __HAL_UART_CLEAR_NEFLAG(&huart7);
      __HAL_UART_CLEAR_PEFLAG(&huart7);
      __HAL_UART_CLEAR_IDLEFLAG(&huart7);
      g_m7.tx_busy = 0u;
      g_m7.wait_reply = 0u;
      Motor7_StartRxDMA();
      cli_printf("uart7 error recover\r\n");
    }
    else if (huart->Instance == USART10)
    {
      HAL_UART_DMAStop(&huart10);
      HAL_UART_Abort(&huart10);
      __HAL_UART_CLEAR_OREFLAG(&huart10);
      __HAL_UART_CLEAR_FEFLAG(&huart10);
      __HAL_UART_CLEAR_NEFLAG(&huart10);
      __HAL_UART_CLEAR_PEFLAG(&huart10);
      __HAL_UART_CLEAR_IDLEFLAG(&huart10);
      g_m10.tx_busy = 0u;
      g_m10.wait_reply = 0u;
      Motor10_StartRxDMA();
      cli_printf("uart10 error recover\r\n");
    }
    else if (huart->Instance == USART1)
    {
      HAL_UART_DMAStop(&CLI_UART);
      HAL_UART_Abort(&CLI_UART);
      __HAL_UART_CLEAR_OREFLAG(&CLI_UART);
      __HAL_UART_CLEAR_FEFLAG(&CLI_UART);
      __HAL_UART_CLEAR_NEFLAG(&CLI_UART);
      __HAL_UART_CLEAR_PEFLAG(&CLI_UART);
      __HAL_UART_CLEAR_IDLEFLAG(&CLI_UART);

      cli_tx_busy = 0u;
      cli_tx_dma_len = 0u;

      UART1_StartRxDMA();
    }
  }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    cli_tx_busy = 0u;
    cli_tx_dma_len = 0u;
  }
  else if (huart->Instance == UART7)
  {
    g_m7.tx_busy = 0u;
    Motor7_StartRxDMA();
  }
  else if (huart->Instance == USART10)
  {
    g_m10.tx_busy = 0u;
    Motor10_StartRxDMA();
  }
}

  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  {
    if (huart->Instance == USART1)
    {
      cli_rx_consume(cli_rx_dma, Size);
      UART1_StartRxDMA();
    }
    else if (huart->Instance == UART7)
    {
      motor_rx_consume(&g_m7, g_m7.rx_dma, Size);
      Motor7_StartRxDMA();
    }
    else if (huart->Instance == USART10)
    {
      motor_rx_consume(&g_m10, g_m10.rx_dma, Size);
      Motor10_StartRxDMA();
    }
  }

/* USER CODE END Application */
