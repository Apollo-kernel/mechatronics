/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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

/* USER CODE BEGIN PV */
/* Single-bus relay: USART1 (PC) <-> UART7 (half-duplex motor bus). Copied from L475 logic. */
uint8_t usart1_rx_data[256] = {0};
uint8_t uart7_tx_buf[256] = {0};
uint8_t uart7_rx_data[256] = {0};
uint8_t usart1_tx_buf[256] = {0};

volatile uint16_t uart7_tx_len = 0;
volatile uint16_t usart1_tx_len = 0;

volatile uint8_t usart1_tx_busy = 0;
volatile uint8_t uart7_tx_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void USART1_StartRxDMA(void)
{
  HAL_UART_DMAStop(&huart1);
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_data, sizeof(usart1_rx_data)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

static void UART7_StartRxDMA(void)
{
  HAL_UART_DMAStop(&huart7);
  HAL_HalfDuplex_EnableReceiver(&huart7);
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_data, sizeof(uart7_rx_data)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
  }
}

static void UART7_StopRxDMA(void)
{
  HAL_UART_DMAStop(&huart7);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI6_Init();
  MX_USART1_UART_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin);
  HAL_GPIO_TogglePin(Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin);
  HAL_GPIO_TogglePin(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin);
  HAL_Delay(300);
  USART1_StartRxDMA();
  UART7_StartRxDMA();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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
    uart7_tx_busy = 0;
    UART7_StartRxDMA();
  }
  else if (huart->Instance == USART1)
  {
    HAL_UART_DMAStop(&huart1);
    HAL_UART_Abort(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    usart1_tx_busy = 0;
    USART1_StartRxDMA();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    usart1_tx_busy = 0;
  }
  else if (huart->Instance == UART7)
  {
    uart7_tx_busy = 0;
    UART7_StartRxDMA();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    if ((Size > 0U) && (Size <= sizeof(uart7_tx_buf)) && (!uart7_tx_busy))
    {
      memcpy(uart7_tx_buf, usart1_rx_data, Size);
      uart7_tx_len = Size;
      UART7_StopRxDMA();
      HAL_HalfDuplex_EnableTransmitter(&huart7);
      if (HAL_UART_Transmit_DMA(&huart7, uart7_tx_buf, uart7_tx_len) == HAL_OK)
      {
        uart7_tx_busy = 1;
      }
      else
      {
        uart7_tx_busy = 0;
        UART7_StartRxDMA();
      }
    }
    USART1_StartRxDMA();
  }
  else if (huart->Instance == UART7)
  {
    if ((Size > 0U) && (Size <= sizeof(usart1_tx_buf)) && (!usart1_tx_busy))
    {
      memcpy(usart1_tx_buf, uart7_rx_data, Size);
      usart1_tx_len = Size;
      if (HAL_UART_Transmit_DMA(&huart1, usart1_tx_buf, usart1_tx_len) == HAL_OK)
      {
        usart1_tx_busy = 1;
      }
    }
    UART7_StartRxDMA();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
