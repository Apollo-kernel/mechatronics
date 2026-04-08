#ifndef __UART1_LOG_H
#define __UART1_LOG_H

#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct
    {
        uint16_t len;
        uint8_t  in_use;
        char     buf[512];
    } uart1_log_msg_t;

    extern osMessageQId cliEvtQHandle;

    void UART1_LogInit(void);
    void UART1_LogTask(void const *argument);

    int  UART1_LogPrintfDrop(const char *fmt, ...);

    int UART1_LogSendRawDrop(const uint8_t *data, uint16_t len);
    void UART1_LogAbortAndFlush(void);

    void UART1_LogTxCpltISR(UART_HandleTypeDef *huart);
    void UART1_LogErrorISR(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif