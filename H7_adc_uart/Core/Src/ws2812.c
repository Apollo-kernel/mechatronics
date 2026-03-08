#include "ws2812.h"
#include "spi.h"
#include <stdint.h>
#include <string.h>

/*
 * STM32H723 + SPI6 uses BDMA (D3 domain). Buffer in RAM_D3 (0x3800_0000).
 * NOTE: Ensure nothing else uses this RAM area (linker does not place .bss there by default).
 */
#define WS_D3_BASE      (0x38000000u + 0x100u)
#define LED_BYTES       (24u)
#define RESET_BYTES     (100u)
#define WS_BUF_LEN      (LED_BYTES + RESET_BYTES)
#define WS2812_SYM_0    (0x60u)
#define WS2812_SYM_1    (0x78u)

static uint8_t * const ws_buf     = (uint8_t *)WS_D3_BASE;
static uint8_t * const data_buf   = (uint8_t *)WS_D3_BASE;
static uint8_t * const reset_buf  = (uint8_t *)(WS_D3_BASE + LED_BYTES);
static volatile uint8_t ws_busy = 0;

static inline void ws2812_encode_one(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < 8; i++)
  {
    data_buf[7  - i] = ((g >> i) & 0x01) ? WS2812_SYM_1 : WS2812_SYM_0;
    data_buf[15 - i] = ((r >> i) & 0x01) ? WS2812_SYM_1 : WS2812_SYM_0;
    data_buf[23 - i] = ((b >> i) & 0x01) ? WS2812_SYM_1 : WS2812_SYM_0;
  }
}

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
  if (ws_busy) return;
  ws_busy = 1;
  ws2812_encode_one(r, g, b);
  memset(reset_buf, 0, RESET_BYTES);
  if (HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, ws_buf, WS_BUF_LEN) != HAL_OK)
    ws_busy = 0;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &WS2812_SPI_UNIT)
    ws_busy = 0;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &WS2812_SPI_UNIT)
    ws_busy = 0;
}
