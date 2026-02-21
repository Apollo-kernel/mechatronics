#include "ws2812.h"
#include <stdint.h>
#include <string.h>

/*
 * STM32H723 + SPI6 often uses BDMA (D3 domain). If you don't want to edit .ld,
 * we "reserve" a small chunk in RAM_D3 (0x3800_0000) manually.
 *
 * NOTE: Make sure nothing else in your project uses this RAM area.
 */
#define WS_D3_BASE      (0x38000000u + 0x100u)   // keep some offset for safety

#define LED_BYTES       (24u)                    // 24 "symbols" (each is 1 SPI byte) for 1 LED
#define RESET_BYTES     (100u)                   // extra zero bytes => low-level latch/reset
#define WS_BUF_LEN      (LED_BYTES + RESET_BYTES)

    // Final symbols after your proven ">>1" shaping:
// 0xC0 >> 1 = 0x60, 0xF0 >> 1 = 0x78
#define WS2812_SYM_0    (0x60u)
#define WS2812_SYM_1    (0x78u)

// Whole DMA buffer in D3 (continuous): [0..23]=data, [24..]=reset zeros
static uint8_t * const ws_buf    = (uint8_t *)WS_D3_BASE;
static uint8_t * const data_buf  = (uint8_t *)WS_D3_BASE;               // alias, clarity
static uint8_t * const reset_buf = (uint8_t *)(WS_D3_BASE + LED_BYTES);

static volatile uint8_t ws_busy = 0;

static inline void ws2812_encode_one(uint8_t r, uint8_t g, uint8_t b)
{
    // WS2812 order: G, R, B; MSB first (your indexing already ensures MSB first)
    for (int i = 0; i < 8; i++)
    {
        data_buf[7  - i] = ((g >> i) & 0x01) ? WS2812_SYM_1 : WS2812_SYM_0;
        data_buf[15 - i] = ((r >> i) & 0x01) ? WS2812_SYM_1 : WS2812_SYM_0;
        data_buf[23 - i] = ((b >> i) & 0x01) ? WS2812_SYM_1 : WS2812_SYM_0;
    }
}

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    // Strategy: drop frame if DMA is still busy (non-blocking).
    // If you prefer blocking, replace with: while(ws_busy) {}
    if (ws_busy) return;
    ws_busy = 1;

    ws2812_encode_one(r, g, b);
    memset(reset_buf, 0, RESET_BYTES);

    HAL_StatusTypeDef st = HAL_SPI_Transmit_DMA(&WS2812_SPI_UNIT, ws_buf, WS_BUF_LEN);
    if (st != HAL_OK)
    {
        // if start failed, don't lock up
        ws_busy = 0;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &WS2812_SPI_UNIT)
    {
        ws_busy = 0;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &WS2812_SPI_UNIT)
    {
        // avoid permanent busy state on DMA/SPI error
        ws_busy = 0;
    }
}
