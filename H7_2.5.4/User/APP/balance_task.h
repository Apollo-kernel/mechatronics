#ifndef __BALANCE_TASK_H
#define __BALANCE_TASK_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    float balance_kp;
    float balance_kd;

    float speed_kp;
    float speed_ki;
    float speed_target_d_raw_20ms;

    float turn_kp;
    float turn_kd;
} balance_param_t;

int  Balance_ParamSetById(uint8_t id, float value);
// void Balance_GetVofaChannels(float ch[8]);
void Balance_GetVofaChannels(float *ch);
void Balance_Task(void);
void Balance_MotorUartTxDone(UART_HandleTypeDef *huart);
void Balance_MotorUartTxError(UART_HandleTypeDef *huart);
void Balance_MotorUartRxEvent(UART_HandleTypeDef *huart, uint16_t Size);
void Balance_SetClosedLoopEnable(uint8_t enable);
uint8_t Balance_GetClosedLoopEnable(void);

void Balance_CLI_ProcessLine(char *line);

#endif
