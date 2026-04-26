#include "uart1_log.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART1_LOG_POOL_SIZE  64U

static uart1_log_msg_t s_uart1_log_pool[UART1_LOG_POOL_SIZE];
static volatile uint8_t s_uart1_dma_busy = 0U;

static uart1_log_msg_t *UART1_LogAllocSlot(void)
{
    uart1_log_msg_t *slot = NULL;
    uint32_t i;

    taskENTER_CRITICAL();
    for (i = 0; i < UART1_LOG_POOL_SIZE; i++)
    {
        if (s_uart1_log_pool[i].in_use == 0U)
        {
            s_uart1_log_pool[i].in_use = 1U;
            s_uart1_log_pool[i].len = 0U;
            slot = &s_uart1_log_pool[i];
            break;
        }
    }
    taskEXIT_CRITICAL();

    return slot;
}

static void UART1_LogFreeSlot(uart1_log_msg_t *slot)
{
    if (slot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    slot->in_use = 0U;
    slot->len = 0U;
    taskEXIT_CRITICAL();
}

void UART1_LogInit(void)
{
    memset(s_uart1_log_pool, 0, sizeof(s_uart1_log_pool));
    s_uart1_dma_busy = 0U;
}

int UART1_LogPrintfDrop(const char *fmt, ...)
{
    uart1_log_msg_t *slot;
    va_list ap;
    int len;

    if (fmt == NULL)
    {
        return 0;
    }

    slot = UART1_LogAllocSlot();
    if (slot == NULL)
    {
        return 0;
    }

    va_start(ap, fmt);
    len = vsnprintf(slot->buf, sizeof(slot->buf), fmt, ap);
    va_end(ap);

    if (len <= 0)
    {
        UART1_LogFreeSlot(slot);
        return 0;
    }

    if (len >= (int)sizeof(slot->buf))
    {
        len = (int)sizeof(slot->buf) - 1;
        slot->buf[len] = '\0';
    }

    slot->len = (uint16_t)len;

    if (osMessagePut(cliEvtQHandle, (uint32_t)slot, 0U) != osOK)
    {
        UART1_LogFreeSlot(slot);
        return 0;
    }

    return len;
}

int UART1_LogSendRawDrop(const uint8_t *data, uint16_t len)
{
    uart1_log_msg_t *slot;

    if ((data == NULL) || (len == 0U) || (len > sizeof(s_uart1_log_pool[0].buf)))
    {
        return 0;
    }

    slot = UART1_LogAllocSlot();
    if (slot == NULL)
    {
        return 0;
    }

    memcpy(slot->buf, data, len);
    slot->len = len;

    if (osMessagePut(cliEvtQHandle, (uint32_t)slot, 0U) != osOK)
    {
        UART1_LogFreeSlot(slot);
        return 0;
    }

    return (int)len;
}

void UART1_LogAbortAndFlush(void)
{
    osEvent evt;
    uart1_log_msg_t *msg;

    (void)HAL_UART_AbortTransmit(&huart1);
    s_uart1_dma_busy = 0U;

    for (;;)
    {
        evt = osMessageGet(cliEvtQHandle, 0U);
        if (evt.status != osEventMessage)
        {
            break;
        }

        msg = (uart1_log_msg_t *)evt.value.p;
        UART1_LogFreeSlot(msg);
    }
}

void UART1_LogTask(void const *argument)
{
    osEvent evt;
    uart1_log_msg_t *msg;

    (void)argument;

    for (;;)
    {
        evt = osMessageGet(cliEvtQHandle, osWaitForever);
        if (evt.status != osEventMessage)
        {
            continue;
        }

        msg = (uart1_log_msg_t *)evt.value.p;
        if ((msg == NULL) || (msg->len == 0U))
        {
            UART1_LogFreeSlot(msg);
            continue;
        }

        while ((s_uart1_dma_busy != 0U) ||
               (huart1.gState != HAL_UART_STATE_READY))
        {
            osDelay(1);
        }

        s_uart1_dma_busy = 1U;

        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)msg->buf, msg->len) != HAL_OK)
        {
            s_uart1_dma_busy = 0U;
            UART1_LogFreeSlot(msg);
            osDelay(1);
            continue;
        }

        while (s_uart1_dma_busy != 0U)
        {
            osDelay(1);
        }

        UART1_LogFreeSlot(msg);
    }
}

void UART1_LogTxCpltISR(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        s_uart1_dma_busy = 0U;
    }
}

void UART1_LogErrorISR(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        s_uart1_dma_busy = 0U;
    }
}