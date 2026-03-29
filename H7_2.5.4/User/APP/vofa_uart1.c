#include "vofa_uart1.h"

#include "usart.h"
#include "uart1_log.h"
#include "balance_task.h"

#include <string.h>
#include <stdlib.h>

#define VOFA_UART1_RX_DMA_LEN   64U
#define VOFA_UART1_CMD_BUF_LEN  64U
#define VOFA_JUSTFLOAT_TAIL_U32 0x7F800000UL

typedef struct __attribute__((packed))
{
    float    ch[VOFA_UART1_CH_NUM];
    uint32_t tail;
} vofa_uart1_frame_t;

static uint8_t s_vofa_rx_dma[VOFA_UART1_RX_DMA_LEN];

static volatile uart1_link_mode_t s_uart1_mode = UART1_LINK_MODE_CLI;

static volatile uint8_t  s_vofa_cmd_ready = 0U;
static volatile uint16_t s_vofa_cmd_len   = 0U;
static char s_vofa_cmd_buf[VOFA_UART1_CMD_BUF_LEN];

static volatile uint8_t  s_cli_line_ready     = 0U;
static volatile uint16_t s_cli_line_build_len = 0U;
static char s_cli_line_buf[VOFA_UART1_CMD_BUF_LEN];

static void VOFA_UART1_RestartRxDMA(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_vofa_rx_dma, sizeof(s_vofa_rx_dma)) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

static void VOFA_UART1_ClearRxState(void)
{
    s_vofa_cmd_ready = 0U;
    s_vofa_cmd_len   = 0U;
    memset(s_vofa_cmd_buf, 0, sizeof(s_vofa_cmd_buf));

    s_cli_line_ready     = 0U;
    s_cli_line_build_len = 0U;
    memset(s_cli_line_buf, 0, sizeof(s_cli_line_buf));
}

void VOFA_UART1_SetMode(uart1_link_mode_t mode)
{
    __disable_irq();
    s_uart1_mode = mode;
    VOFA_UART1_ClearRxState();
    __enable_irq();
}

uart1_link_mode_t VOFA_UART1_GetMode(void)
{
    return s_uart1_mode;
}

void VOFA_UART1_Init(void)
{
    memset(s_vofa_rx_dma, 0, sizeof(s_vofa_rx_dma));

    VOFA_UART1_SetMode(UART1_LINK_MODE_CLI);
    VOFA_UART1_RestartRxDMA();
}

int VOFA_UART1_Send8(const float ch[VOFA_UART1_CH_NUM])
{
    vofa_uart1_frame_t frame;

    if (VOFA_UART1_GetMode() != UART1_LINK_MODE_VOFA)
    {
        return 0;
    }

    if (ch == NULL)
    {
        return 0;
    }

    memcpy(frame.ch, ch, sizeof(frame.ch));
    frame.tail = VOFA_JUSTFLOAT_TAIL_U32;

    return UART1_LogSendRawDrop((const uint8_t *)&frame, (uint16_t)sizeof(frame));
}

void VOFA_UART1_RxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;
    uint8_t c;

    if ((huart != &huart1) || (Size == 0U))
    {
        return;
    }

    if (s_uart1_mode == UART1_LINK_MODE_VOFA)
    {
        for (i = 0; i < Size; i++)
        {
            c = s_vofa_rx_dma[i];

            if (c == '#')
            {
                s_vofa_cmd_ready = 0U;
                s_vofa_cmd_len   = 0U;
                s_vofa_cmd_buf[s_vofa_cmd_len++] = '#';
                continue;
            }

            if (s_vofa_cmd_len == 0U)
            {
                continue;
            }

            if (s_vofa_cmd_ready != 0U)
            {
                continue;
            }

            if (s_vofa_cmd_len < (VOFA_UART1_CMD_BUF_LEN - 1U))
            {
                s_vofa_cmd_buf[s_vofa_cmd_len++] = (char)c;
            }
            else
            {
                s_vofa_cmd_len = 0U;
                continue;
            }

            if (c == '!')
            {
                s_vofa_cmd_buf[s_vofa_cmd_len] = '\0';
                s_vofa_cmd_ready = 1U;
                s_vofa_cmd_len   = 0U;
            }
        }
    }
    else
    {
        for (i = 0; i < Size; i++)
        {
            c = s_vofa_rx_dma[i];

            if (s_cli_line_ready != 0U)
            {
                continue;
            }

            if ((c == '\r') || (c == '\n'))
            {
                if (s_cli_line_build_len > 0U)
                {
                    s_cli_line_buf[s_cli_line_build_len] = '\0';
                    s_cli_line_ready = 1U;
                    s_cli_line_build_len = 0U;
                }
                continue;
            }

            if ((c == 0x08U) || (c == 0x7FU))
            {
                if (s_cli_line_build_len > 0U)
                {
                    s_cli_line_build_len--;
                }
                continue;
            }

            if (s_cli_line_build_len < (VOFA_UART1_CMD_BUF_LEN - 1U))
            {
                s_cli_line_buf[s_cli_line_build_len++] = (char)c;
            }
            else
            {
                s_cli_line_build_len = 0U;
            }
        }
    }

    VOFA_UART1_RestartRxDMA();
}

void VOFA_UART1_Poll(void)
{
    char local_cmd[VOFA_UART1_CMD_BUF_LEN];
    char *eq_ptr;
    char *end_ptr;
    unsigned long id_ul;
    float value;

    if (s_uart1_mode == UART1_LINK_MODE_VOFA)
    {
        if (s_vofa_cmd_ready == 0U)
        {
            return;
        }

        __disable_irq();
        memcpy(local_cmd, s_vofa_cmd_buf, sizeof(local_cmd));
        s_vofa_cmd_ready = 0U;
        __enable_irq();

        if (strcmp(local_cmd, "#CLI!") == 0)
        {
            UART1_LogAbortAndFlush();
            VOFA_UART1_SetMode(UART1_LINK_MODE_CLI);

            (void)UART1_LogPrintfDrop("\r\nCLI mode entered\r\n");
            if (Balance_GetClosedLoopEnable() != 0U)
            {
                (void)UART1_LogPrintfDrop("closed-loop balance: ON\r\n");
            }
            else
            {
                (void)UART1_LogPrintfDrop("closed-loop balance: OFF\r\n");
            }
            (void)UART1_LogPrintfDrop("type 'help'\r\n");
            (void)UART1_LogPrintfDrop("m0603> ");
            return;
        }
        if (strcmp(local_cmd, "#BAL=1!") == 0)
        {
            Balance_SetClosedLoopEnable(1U);
            return;
        }

        if (strcmp(local_cmd, "#BAL=0!") == 0)
        {
            Balance_SetClosedLoopEnable(0U);
            return;
        }
        if ((local_cmd[0] != '#') || (local_cmd[1] != 'P'))
        {
            return;
        }

        id_ul = strtoul(&local_cmd[2], &eq_ptr, 10);
        if ((eq_ptr == NULL) || (*eq_ptr != '='))
        {
            return;
        }

        value = strtof(eq_ptr + 1, &end_ptr);
        if ((end_ptr == NULL) || (*end_ptr != '!'))
        {
            return;
        }

        (void)Balance_ParamSetById((uint8_t)id_ul, value);
        return;
    }
    if (s_cli_line_ready != 0U)
    {
        __disable_irq();
        memcpy(local_cmd, s_cli_line_buf, sizeof(local_cmd));
        s_cli_line_ready = 0U;
        __enable_irq();

        Balance_CLI_ProcessLine(local_cmd);
    }
}