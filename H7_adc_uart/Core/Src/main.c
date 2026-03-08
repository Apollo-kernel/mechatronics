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
#include <stdio.h>
#include <string.h>
#include "ws2812.h"
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
/* UART RX buffer for WS2812 RGB commands (R0/R1, G0/G1, B0/B1) and echo */
uint8_t rx_data[256] = {0};
uint8_t ws2812_r = 0, ws2812_g = 0, ws2812_b = 0;
/* Motor command packets (M0603A protocol, 10 bytes) - same as H7_0.1.5 */
static const uint8_t data_enable[10]      = {0x01, 0xA0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F};
static const uint8_t data_speed_loop[10]  = {0x01, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE4};
static const uint8_t data_30RPM[10]       = {0x01, 0x64, 0x01, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA6};
static const uint8_t data_10RPM_CW[10]    = {0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F};
static const uint8_t data_10RPM_CCW[10]   = {0x01, 0x64, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A};
static const uint8_t data_30RPM_CCW[10]   = {0x01, 0x64, 0xFE, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void send_vbus(void);
static void delay_ms_with_vbus(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adc_val[2];
volatile uint8_t uart1_tx_busy = 0;

static void send_vbus(void)
{
  char txbuf[64];
  float v = (adc_val[0] * 3.3f / 65535.0f) * 11.0f;
  int len = snprintf(txbuf, sizeof(txbuf), "vbus=%.3f V\r\n", v);
  HAL_UART_Transmit(&huart1, (uint8_t*)txbuf, (uint32_t)len, 0xFFFF);
}

/* Delay ms while sending Vbus every 100 ms on USART1 */
static void delay_ms_with_vbus(uint32_t ms)
{
  while (ms >= 100u)
  {
    send_vbus();
    HAL_Delay(100);
    ms -= 100u;
  }
  if (ms > 0u)
  {
    HAL_Delay(ms);
  }
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
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 2u);
  /* UART1: start receive for WS2812 commands (R0/R1, G0/G1, B0/B1) and echo */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data, sizeof(rx_data));
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  /* Power enable toggles (same as H7_0.1.5) */
  HAL_GPIO_TogglePin(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin);
  HAL_GPIO_TogglePin(Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin);
  HAL_GPIO_TogglePin(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin);
  HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Parse USART1 RX for WS2812 RGB commands (R0/R1, G0/G1, B0/B1) and update LED */
    if (rx_data[0] == 'R') {
      ws2812_r = (rx_data[1] == '0') ? 0 : 15;
    } else if (rx_data[0] == 'G') {
      ws2812_g = (rx_data[1] == '0') ? 0 : 15;
    } else if (rx_data[0] == 'B') {
      ws2812_b = (rx_data[1] == '0') ? 0 : 15;
    }
    WS2812_Ctrl(0, 255, 0);

    /* --- Motor sequence on USART10 (first motor); USART1 reserved for Vbus --- */
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)data_enable, 10u);
    delay_ms_with_vbus(100u);
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)data_speed_loop, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)data_30RPM, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)data_30RPM_CCW, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)data_10RPM_CW, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart10, (uint8_t*)data_10RPM_CCW, 10u);
    delay_ms_with_vbus(3000u);

    /* --- Motor sequence on UART7 (second motor) --- */
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)data_enable, 10u);
    delay_ms_with_vbus(100u);
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)data_speed_loop, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)data_30RPM, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)data_30RPM_CCW, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)data_10RPM_CW, 10u);
    delay_ms_with_vbus(1000u);
    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)data_10RPM_CCW, 10u);
    delay_ms_with_vbus(3000u);

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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_tx_busy = 0;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_tx_busy = 0;
  }
}

/* Echo received bytes on USART1 and re-arm ReceiveToIdle (WS2812 command link) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Transmit_DMA(&huart1, rx_data, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_data, sizeof(rx_data));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
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

#ifdef  USE_FULL_ASSERT
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
