#include "vofa_uart1.h"

#include "usart.h"
#include "uart1_log.h"
#include "balance_task.h"

#include <string.h>
#include <stdlib.h>

#define VOFA_UART1_RX_DMA_LEN   64U
#define VOFA_UART1_CMD_BUF_LEN  64U
#define VOFA_JUSTFLOAT_TAIL_U32 0x7F800000UL
#define UART1_VBUS_TX_PERIOD_MS 200U

typedef struct __attribute__((packed))
{
    float    ch[VOFA_UART1_CH_NUM];
    uint32_t tail;
} vofa_uart1_frame_t;

static uint8_t s_vofa_rx_dma[VOFA_UART1_RX_DMA_LEN];

static volatile uart1_link_mode_t s_uart1_mode = UART1_LINK_MODE_VOFA;

static volatile uint8_t  s_vofa_cmd_ready = 0U;
static volatile uint16_t s_vofa_cmd_len   = 0U;
static char s_vofa_cmd_buf[VOFA_UART1_CMD_BUF_LEN];

static volatile uint8_t  s_cli_line_ready     = 0U;
static volatile uint16_t s_cli_line_build_len = 0U;
static char s_cli_line_buf[VOFA_UART1_CMD_BUF_LEN];
static volatile uint8_t s_uart1_rx_recover_req = 0U;
static volatile uint32_t s_uart1_rx_error_count = 0U;
static uint32_t s_vbus_last_tx_ms = 0U;

static void VOFA_UART1_ClearRxState(void);

static void VOFA_UART1_ClearUartErrorFlags(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    huart1.ErrorCode = HAL_UART_ERROR_NONE;
}

static void VOFA_UART1_StartRxDMA(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_vofa_rx_dma, sizeof(s_vofa_rx_dma)) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        s_uart1_rx_recover_req = 0U;
    }
    else
    {
        s_uart1_rx_recover_req = 1U;
    }
}

static void VOFA_UART1_ServiceVoltageTx(void)
{
    const uint32_t now = HAL_GetTick();

    if ((now - s_vbus_last_tx_ms) < UART1_VBUS_TX_PERIOD_MS)
    {
        return;
    }

    s_vbus_last_tx_ms = now;
    (void)UART1_LogPrintfDrop("vbus=%.3f V\r\n", Agile_GetBusVoltage());
}

static void VOFA_UART1_RecoverRxDMA_Task(void)
{
    uint8_t need_recover;

    need_recover = s_uart1_rx_recover_req;

    /*
     * Normal ReceiveToIdle DMA should keep RxState at BUSY_RX.
     * If unplug/noise/error makes RxState leave BUSY_RX, restart RX.
     */
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX)
    {
        need_recover = 1U;
    }

    if (huart1.ErrorCode != HAL_UART_ERROR_NONE)
    {
        need_recover = 1U;
    }

    if (need_recover == 0U)
    {
        return;
    }

    __disable_irq();
    s_uart1_rx_recover_req = 0U;
    VOFA_UART1_ClearRxState();
    __enable_irq();

    /*
     * Only abort RX side. Do not abort TX here unless necessary,
     * because USART1 TX also carries the voltage telemetry for the ESP32 UI.
     */
    (void)HAL_UART_AbortReceive(&huart1);

    VOFA_UART1_ClearUartErrorFlags();

    memset(s_vofa_rx_dma, 0, sizeof(s_vofa_rx_dma));

    VOFA_UART1_StartRxDMA();
}

void VOFA_UART1_ErrorISR(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }

    s_uart1_rx_error_count++;
    s_uart1_rx_recover_req = 1U;
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
    (void)mode;

    __disable_irq();
    /* UART1 is fixed to voltage-TX + joystick-RX mode. */
    s_uart1_mode = UART1_LINK_MODE_VOFA;
    VOFA_UART1_ClearRxState();
    __enable_irq();
}

uart1_link_mode_t VOFA_UART1_GetMode(void)
{
    return UART1_LINK_MODE_VOFA;
}

void VOFA_UART1_Init(void)
{
    memset(s_vofa_rx_dma, 0, sizeof(s_vofa_rx_dma));
    s_vbus_last_tx_ms = HAL_GetTick();

    VOFA_UART1_SetMode(UART1_LINK_MODE_VOFA);
    VOFA_UART1_ClearUartErrorFlags();
    VOFA_UART1_StartRxDMA();
}

int VOFA_UART1_Send8(const float ch[VOFA_UART1_CH_NUM])
{
#if VOFA_UART1_STREAM_ENABLE
    vofa_uart1_frame_t frame;

    if (ch == NULL)
    {
        return 0;
    }

    memcpy(frame.ch, ch, sizeof(frame.ch));
    frame.tail = VOFA_JUSTFLOAT_TAIL_U32;

    return UART1_LogSendRawDrop((const uint8_t *)&frame, (uint16_t)sizeof(frame));
#else
    (void)ch;
    return 0;
#endif
}

void VOFA_UART1_RxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t i;
    uint8_t c;

    if ((huart != &huart1) || (Size == 0U))
    {
        return;
    }

    /* UART1 RX is now command-only.
     * Only collect hash-bang frames such as:
     *   #P15=<lr>,P8=<fb>!
     * Any other bytes, CR/LF text, or old CLI lines are ignored.
     */
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

    VOFA_UART1_StartRxDMA();
}

void VOFA_UART1_Poll(void)
{
    char local_cmd[VOFA_UART1_CMD_BUF_LEN];

    VOFA_UART1_RecoverRxDMA_Task();
    VOFA_UART1_ServiceVoltageTx();

    if (s_vofa_cmd_ready == 0U)
    {
        return;
    }

    __disable_irq();
    memcpy(local_cmd, s_vofa_cmd_buf, sizeof(local_cmd));
    s_vofa_cmd_ready = 0U;
    __enable_irq();

    /* RX accepts only the ESP32 / website joystick packet:
     *   #P15=<lr>,P8=<fb>!
     * lr/fb are raw joystick values in +/-1000.
     * They are mapped in Balance_HandleJoystickCommand():
     *   P15: +/-1000 -> +/-180 deg/s
     *   P8 : +/-1000 -> +/-60 rpm
     * No CLI echo, no #BAL response, and no generic #P setting is sent on UART1.
     * USART1 TX is reserved for voltage telemetry lines:
     *   vbus=<voltage> V
     */
    (void)Balance_HandleJoystickCommand(local_cmd);
}
