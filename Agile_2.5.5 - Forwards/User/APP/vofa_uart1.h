#ifndef __VOFA_UART1_H
#define __VOFA_UART1_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VOFA_UART1_STREAM_ENABLE      1
#define VOFA_UART1_ASCII_LOG_ENABLE   0

// #define VOFA_UART1_CH_NUM             8U
#define VOFA_UART1_CH_NUM             17U

    typedef enum
    {
        UART1_LINK_MODE_VOFA = 0,
        UART1_LINK_MODE_CLI  = 1,
    } uart1_link_mode_t;

    void VOFA_UART1_Init(void);
    void VOFA_UART1_Poll(void);
    void VOFA_UART1_RxEvent(UART_HandleTypeDef *huart, uint16_t Size);

    void VOFA_UART1_SetMode(uart1_link_mode_t mode);
    uart1_link_mode_t VOFA_UART1_GetMode(void);

    int VOFA_UART1_Send8(const float ch[VOFA_UART1_CH_NUM]);

#ifdef __cplusplus
}
#endif

#endif