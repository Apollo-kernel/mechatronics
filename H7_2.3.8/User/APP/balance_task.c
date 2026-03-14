#include "balance_task.h"
#include "INS_task.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define M0603_FRAME_LEN            10U
#define M0603_ID                   0x01U

#define BALANCE_TASK_PERIOD_MS     10U      /* 100Hz，更适合 38400 半双工 */
#define DEBUG_PRINT_PERIOD_MS      100U

#define BALANCE_DEADBAND_RAD       0.001f    /* 约 1.15 度内不输出 0.02*/
#define BALANCE_OPEN_MIN           4000             //4000
#define BALANCE_OPEN_MAX           30000            //30000

/* 初始建议值，后续你可现场微调 */
#define BALANCE_KP                 88000.0f     //85000 //70000 //90000
#define BALANCE_KD                 00.0f        //260

typedef enum
{
    M0603_MODE_OPEN    = 0x00,
    M0603_MODE_CURRENT = 0x01,
    M0603_MODE_SPEED   = 0x02,
    M0603_MODE_ENABLE  = 0x08,
    M0603_MODE_DISABLE = 0x09,
} m0603_mode_t;

static const uint8_t crc8_maxim_table[256] = {
    0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
    157,195,33,127,252,162,64,30,95,1,227,189,62,96,130,220,
    35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
    190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
    70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
    219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
    101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
    248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
    140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
    17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
    175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
    50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
    202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
    87,9,235,181,54,104,138,212,149,203,41,119,244,170,72,22,
    233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
    116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53
};

static uint8_t crc8_maxim_calc(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = crc8_maxim_table[crc ^ *data++];
    }
    return crc;
}

static void m0603_make_set_mode(uint8_t *buf, uint8_t id, uint8_t mode)
{
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0xA0;
    buf[2] = mode;
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_open(uint8_t *buf, uint8_t id, int16_t open_raw)
{
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0x64;
    buf[2] = (uint8_t)(((uint16_t)open_raw) >> 8);
    buf[3] = (uint8_t)(open_raw & 0xFF);
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void cli_printf(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }
    if (n > (int)sizeof(buf))
    {
        n = sizeof(buf);
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
}

static HAL_StatusTypeDef motor_send_frame(UART_HandleTypeDef *huart, const uint8_t *frame)
{
    HAL_HalfDuplex_EnableTransmitter(huart);
    return HAL_UART_Transmit(huart, (uint8_t *)frame, M0603_FRAME_LEN, 50);
}

static void motor_send_mode(UART_HandleTypeDef *huart, uint8_t mode)
{
    uint8_t frame[M0603_FRAME_LEN];
    m0603_make_set_mode(frame, M0603_ID, mode);
    (void)motor_send_frame(huart, frame);
}

static void motor_send_open(UART_HandleTypeDef *huart, int16_t open_raw)
{
    uint8_t frame[M0603_FRAME_LEN];
    m0603_make_open(frame, M0603_ID, open_raw);
    (void)motor_send_frame(huart, frame);
}

static int16_t balance_limit_open(float u_abs)
{
    int32_t out = (int32_t)(u_abs);

    if (out < BALANCE_OPEN_MIN)
    {
        out = BALANCE_OPEN_MIN;
    }
    if (out > BALANCE_OPEN_MAX)
    {
        out = BALANCE_OPEN_MAX;
    }
    return (int16_t)out;
}

static void balance_motor_init(void)
{
    HAL_GPIO_WritePin(Power_5V_EN_GPIO_Port, Power_5V_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Power_OUT2_EN_GPIO_Port, Power_OUT2_EN_Pin, GPIO_PIN_SET);

    osDelay(3000);

    motor_send_mode(&huart7, M0603_MODE_ENABLE);
    osDelay(20);
    motor_send_mode(&huart10, M0603_MODE_ENABLE);
    osDelay(20);

    motor_send_mode(&huart7, M0603_MODE_OPEN);
    osDelay(20);
    motor_send_mode(&huart10, M0603_MODE_OPEN);
    osDelay(20);

    motor_send_open(&huart7, 0);
    osDelay(5);
    motor_send_open(&huart10, 0);
    osDelay(5);
}

void Balance_Task(void)
{
    uint32_t last_print = HAL_GetTick();

    balance_motor_init();
    cli_printf("balance task start, BMI088 on SPI2, control=Pitch closed-loop\r\n");

    while (1)
    {
        float pitch;
        float pitch_gyro;
        float u;
        float u_abs;
        int16_t open_mag;
        int16_t m7_open = 0;
        int16_t m10_open = 0;

        if (INS.ins_flag == 0)
        {
            motor_send_open(&huart7, 0);
            osDelay(2);
            motor_send_open(&huart10, 0);
            osDelay(BALANCE_TASK_PERIOD_MS);
            continue;
        }

        pitch = INS.Pitch;
        pitch_gyro = INS.Gyro[0];

        /*
         * 目标符号关系：
         * pitch > 0  -> M10 > 0, M7 < 0
         * pitch < 0  -> M10 < 0, M7 > 0
         *
         * 因此先构造 u 与 pitch 同号
         */
        u = BALANCE_KP * pitch - BALANCE_KD * pitch_gyro;

        if (fabsf(pitch) < BALANCE_DEADBAND_RAD)
        {
            u = 0.0f;
        }

        if (u > 0.0f)
        {
            u_abs = fabsf(u);
            open_mag = balance_limit_open(u_abs);

            m10_open =  open_mag;
            m7_open  = -open_mag;
        }
        else if (u < 0.0f)
        {
            u_abs = fabsf(u);
            open_mag = balance_limit_open(u_abs);

            m10_open = -open_mag;
            m7_open  =  open_mag;
        }
        else
        {
            m10_open = 0;
            m7_open  = 0;
        }

        motor_send_open(&huart7, m7_open);
        osDelay(2);
        motor_send_open(&huart10, m10_open);

        if ((HAL_GetTick() - last_print) >= DEBUG_PRINT_PERIOD_MS)
        {
            last_print = HAL_GetTick();
            cli_printf("pitch=%.4f rad (%.2f deg), gyro_x=%.4f, m7=%d, m10=%d\r\n",
                       pitch,
                       pitch * 57.2957795f,
                       pitch_gyro,
                       m7_open,
                       m10_open);
        }

        osDelay(BALANCE_TASK_PERIOD_MS);
    }
}