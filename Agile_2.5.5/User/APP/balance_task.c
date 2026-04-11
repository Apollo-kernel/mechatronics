#include "balance_task.h"
#include "INS_task.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "uart1_log.h"
#include "vofa_uart1.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define M0603_FRAME_LEN            10U
#define M0603_ID                   0x01U
#define MOTOR_REPLY_TIMEOUT_MS     80U
#define MOTOR_VER_TIMEOUT_MS       120U
#define AUTO_HOLD_MS               2000U


// #define BALANCE_OPEN_MIN           4000             //4000
// #define BALANCE_OPEN_MAX           30000            //30000
//

// #define BALANCE_KP                 88000.0f     //85000 //70000 //90000
// #define BALANCE_KD                 00.0f        //260


#define BALANCE_TASK_PERIOD_MS     10U
#define DEBUG_PRINT_PERIOD_MS      100U

#define BALANCE_DEADBAND_RAD       0.001f    /* pitch 很小时不输出 */
#define BALANCE_STOP_RAD           0.60f     /* 倾角过大直接停机并清积分 */

#define BALANCE_OPEN_MIN           4000
#define BALANCE_OPEN_MAX           32767        //32767
#define OPEN_ZERO_THRESH           30.0f    /* 最终命令小于该值直接置 300.0f */


// #define BALANCE_KP                 90000.0f         //90000.0f
// #define BALANCE_KD                 0.0f


// #define SPEED_TARGET_RPM           0.0f
#define SPEED_TARGET_D_RAW_20MS    0.0f

// #define SPEED_KP                   230.0f
// #define SPEED_KI                   0.0f
// #define SPEED_I_LIMIT             6000.0f
#define SPEED_POS_LIMIT            20000.0f             //32000
#define SPEED_OUT_LIMIT            26000.0f



// #define TURN_KP                    0.0f          //5000.0f
// #define TURN_KD                    0.0f           //180.0f
#define TURN_OUT_LIMIT             8000.0f
#define SPEED_LPF_ALPHA            0.95f         //0.92 //0.87 //0.93
static balance_param_t g_balance_param = {
    .balance_kp = 140000.0f,     //100000
    .balance_kd = 0.0f,
    .speed_kp   = 36.0f,        //35 //37       //23        //28
    .speed_ki   = 0.09f,        //0.08 //0.09    //0.05      //0.05
    .turn_kp    = 0.0f,
    .turn_kd    = 0.0f,
};

typedef enum {
    M0603_MODE_OPEN = 0x00,
    M0603_MODE_CURRENT = 0x01,
    M0603_MODE_SPEED = 0x02,
    M0603_MODE_ENABLE = 0x08,
    M0603_MODE_DISABLE = 0x09,
} m0603_mode_t;

typedef enum {
    AUTO_IDLE = 0,
    AUTO_START,
    AUTO_WAIT_ENABLE,
    AUTO_WAIT_SPEED_MODE,
    AUTO_WAIT_FWD,
    AUTO_HOLD_FWD,
    AUTO_WAIT_REV,
    AUTO_HOLD_REV,
    AUTO_WAIT_STOP,
    AUTO_WAIT_QPOS,
    AUTO_WAIT_QVER,
    AUTO_WAIT_DISABLE,
    AUTO_DONE,
    AUTO_FAIL,
} auto_test_state_t;

typedef struct {
    int16_t speed_drpm;
    int16_t current_raw;
    uint8_t accel_time;
    uint8_t temperature_c;
    uint8_t err_code;

    int32_t total_turns;
    uint16_t position_raw;

    int32_t self_total_turns;
    int32_t pos_raw_delta_20ms;

    uint16_t prev_position_raw;
    uint8_t pos_valid;

    uint8_t mode;
    uint8_t ver_year;
    uint8_t ver_month;
    uint8_t ver_day;
    uint8_t ver_model;
    uint8_t ver_sw;
    uint8_t ver_hw;

    uint32_t last_update_tick;
    uint32_t last_pos_tick;
    uint32_t last_mode_tick;
    uint32_t last_ver_tick;
} motor_feedback_t;

typedef struct {
    UART_HandleTypeDef *huart;
    const char *name;
    volatile uint8_t tx_busy;
    uint8_t tx_buf[M0603_FRAME_LEN];

    uint8_t rx_dma[32];
    uint8_t stream[32];
    uint16_t stream_len;

    volatile uint8_t wait_reply;
    uint8_t wait_expected;
    uint32_t wait_deadline;
    char wait_note[20];

    volatile uint8_t ack_pending;
    uint8_t ack_cmd;
    uint8_t ack_frame[M0603_FRAME_LEN];
    char ack_note[20];

    auto_test_state_t auto_state;
    uint32_t auto_state_tick;

    motor_feedback_t fb;
} motor_uart_dma_t;

typedef enum
{
    BAL_CLI_TARGET_BOTH = 0,
    BAL_CLI_TARGET_M7,
    BAL_CLI_TARGET_M10,
} bal_cli_target_t;

typedef struct
{
    uint8_t pending;
    uint8_t remaining;
} cli_query_prompt_state_t;

static motor_uart_dma_t motor_uart7_dma  = {.huart = &huart7,  .name = "m7",  .tx_busy = 0};
static motor_uart_dma_t motor_uart10_dma = {.huart = &huart10, .name = "m10", .tx_busy = 0};

static void motor_start_rx_dma(motor_uart_dma_t *ctx);

static void motor_handle_rx_frame(motor_uart_dma_t *ctx, const uint8_t *f);

static void motor_parse_stream(motor_uart_dma_t *ctx);

static void motor_rx_consume(motor_uart_dma_t *ctx, const uint8_t *data, uint16_t len);

static uint32_t m0603_pos_raw_to_mdeg(uint16_t pos_raw);
static void m0603_make_query_mode(uint8_t *buf, uint8_t id);
static void m0603_make_query_ver(uint8_t *buf, uint8_t id);
static const char *motor_mode_name(uint8_t mode);
static void motor_dump_frame_prefix(const motor_uart_dma_t *ctx, const char *tag, const uint8_t *f, uint16_t len);
static void motor_service_timeout(motor_uart_dma_t *ctx);
static void motor_service_cli_reply(motor_uart_dma_t *ctx);
static uint8_t cli_query_prompt_begin(void);
static void cli_query_prompt_expect(HAL_StatusTypeDef st);
static void cli_query_prompt_finish_send_phase(void);
static void cli_query_prompt_complete_one(void);

static HAL_StatusTypeDef motor_send_query_pos(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef motor_send_query_pos_cli(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef motor_send_query_mode(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef motor_send_query_ver(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef motor_send_raw(UART_HandleTypeDef *huart,
                                        const uint8_t *frame,
                                        uint8_t expected_cmd,
                                        uint32_t timeout_ms,
                                        const char *note);

// static float chassis_speed_rpm(void);
static float chassis_speed_d_raw_20ms(void);
static float motor_continuous_pos_raw(const motor_uart_dma_t *ctx);
static float chassis_position_raw(void);
static void control_state_reset(void);
static HAL_StatusTypeDef motor_send_mode(UART_HandleTypeDef *huart, uint8_t mode);
static HAL_StatusTypeDef motor_send_open(UART_HandleTypeDef *huart, int16_t open_raw);
static const char *motor_mode_cmd_note(uint8_t mode);
static HAL_StatusTypeDef motor_send_mode_cli(UART_HandleTypeDef *huart, uint8_t mode);
static HAL_StatusTypeDef motor_send_open_cli(UART_HandleTypeDef *huart, int16_t open_raw);
static void m0603_make_speed(uint8_t *buf, uint8_t id, int16_t speed_drpm);
static void m0603_make_current(uint8_t *buf, uint8_t id, float current_a);

static HAL_StatusTypeDef motor_send_speed(UART_HandleTypeDef *huart, int16_t rpm);
static HAL_StatusTypeDef motor_send_current(UART_HandleTypeDef *huart, float current_a);

static void balance_cli_print_map(void);
static uint8_t motor_auto_is_running(const motor_uart_dma_t *ctx);
static uint8_t balance_cli_target_auto_running(bal_cli_target_t target);
static void balance_cli_send_raw(bal_cli_target_t target, const uint8_t *frame);

static void auto_test_start(motor_uart_dma_t *ctx);
static void auto_test_stop(motor_uart_dma_t *ctx, const char *reason);
static void auto_test_service(motor_uart_dma_t *ctx);

static float motor_rpm_from_feedback(const motor_uart_dma_t *ctx) {
    return ((float) ctx->fb.speed_drpm) * 0.1f;
}

// static float speed_lpf_rpm = 0.0f;
static float speed_lpf_d_raw_20ms = 0.0f;
// static float speed_i_term = 0.0f;

// static float chassis_pos_hold_state = 0.0f;
static float chassis_pos_fb_raw = 0.0f;
static float chassis_pos_ref_raw = 0.0f;
static uint8_t chassis_pos_ref_valid = 0u;
static uint32_t speed_pos_tick_m7_used  = 0u;
static uint32_t speed_pos_tick_m10_used = 0u;

static float yaw_ref_total = 0.0f;
static uint8_t yaw_ref_valid = 0u;



static float g_vofa_ch[VOFA_UART1_CH_NUM] = {0.0f};
static volatile uint8_t g_balance_closed_loop_enable = 1u;

int Balance_ParamSetById(uint8_t id, float value)
{
    switch (id)
    {
        case 1:
            if ((value >= 0.0f) && (value <= 200000.0f))
            {
                g_balance_param.balance_kp = value;
                return 1;
            }
            break;

        case 2:
            if ((value >= 0.0f) && (value <= 3000.0f))
            {
                g_balance_param.balance_kd = value;
                return 1;
            }
            break;

        case 3:
            if ((value >= 0.0f) && (value <= 2000.0f))
            {
                g_balance_param.speed_kp = value;
                return 1;
            }
            break;

        case 4:
            if ((value >= 0.0f) && (value <= 200.0f))
            {
                g_balance_param.speed_ki = value;
                return 1;
            }
            break;

        case 5:
            if ((value >= 0.0f) && (value <= 10000.0f))
            {
                g_balance_param.turn_kp = value;
                return 1;
            }
            break;

        case 6:
            if ((value >= 0.0f) && (value <= 2000.0f))
            {
                g_balance_param.turn_kd = value;
                return 1;
            }
            break;

        default:
            break;
    }

    return 0;
}

void Balance_GetVofaChannels(float *ch)
{
    if (ch == NULL)
    {
        return;
    }

    memcpy(ch, g_vofa_ch, sizeof(g_vofa_ch));
}

static void balance_fill_vofa_channels(float balance_out, float speed_out, float turn_out)
{
    g_vofa_ch[0]  = INS.Pitch * 57.2957795f;
    g_vofa_ch[1]  = INS.Roll  * 57.2957795f;
    g_vofa_ch[2]  = INS.Yaw   * 57.2957795f;
    g_vofa_ch[3]  = balance_out;
    g_vofa_ch[4]  = speed_out;
    g_vofa_ch[5]  = turn_out;
    g_vofa_ch[6]  = motor_rpm_from_feedback(&motor_uart7_dma);
    g_vofa_ch[7]  = motor_rpm_from_feedback(&motor_uart10_dma);
    g_vofa_ch[8]  = INS.YawTotalAngle * 57.2957795f;
    g_vofa_ch[9]  = (float)motor_uart7_dma.fb.self_total_turns;
    g_vofa_ch[10] = (float)motor_uart7_dma.fb.position_raw;
    g_vofa_ch[11] = ((float)m0603_pos_raw_to_mdeg(
                        motor_uart7_dma.fb.position_raw)) * 0.001f;
    g_vofa_ch[12] = (float)motor_uart7_dma.fb.pos_raw_delta_20ms;
    g_vofa_ch[13] = (float)motor_uart10_dma.fb.self_total_turns;
    g_vofa_ch[14] = (float)motor_uart10_dma.fb.position_raw;
    g_vofa_ch[15] = ((float)m0603_pos_raw_to_mdeg(
                        motor_uart10_dma.fb.position_raw)) * 0.001f;
    g_vofa_ch[16] = (float)motor_uart10_dma.fb.pos_raw_delta_20ms;
}

static cli_query_prompt_state_t g_cli_query_prompt = {0u, 0u};

void Balance_SetClosedLoopEnable(uint8_t enable)
{
    if (enable != 0U)
    {
        control_state_reset();
        motor_uart7_dma.auto_state = AUTO_IDLE;
        motor_uart7_dma.auto_state_tick = 0u;
        motor_uart10_dma.auto_state = AUTO_IDLE;
        motor_uart10_dma.auto_state_tick = 0u;
        g_balance_closed_loop_enable = 1U;
    }
    else
    {
        g_balance_closed_loop_enable = 0U;
        control_state_reset();

        /* ===== 关闭闭环时，立刻给两边一个 0 open，防止继续输出 ===== */
        (void)motor_send_open(&huart7, 0);
        (void)motor_send_open(&huart10, 0);
    }
}

uint8_t Balance_GetClosedLoopEnable(void)
{
    return g_balance_closed_loop_enable;
}

static void balance_cli_prompt(void)
{
    (void)UART1_LogPrintfDrop("m0603> ");
}

static uint8_t cli_query_prompt_begin(void)
{
    if (g_cli_query_prompt.pending)
    {
        (void)UART1_LogPrintfDrop("query busy: wait previous query replies\r\n");
        balance_cli_prompt();
        return 0u;
    }

    g_cli_query_prompt.pending = 1u;
    g_cli_query_prompt.remaining = 0u;
    return 1u;
}

static void cli_query_prompt_expect(HAL_StatusTypeDef st)
{
    if (!g_cli_query_prompt.pending)
    {
        return;
    }

    if (st == HAL_OK)
    {
        if (g_cli_query_prompt.remaining < 0xFFu)
        {
            g_cli_query_prompt.remaining++;
        }
    }
}

static void cli_query_prompt_finish_send_phase(void)
{
    if (!g_cli_query_prompt.pending)
    {
        return;
    }

    if (g_cli_query_prompt.remaining == 0u)
    {
        g_cli_query_prompt.pending = 0u;
        balance_cli_prompt();
    }
}

static void cli_query_prompt_complete_one(void)
{
    if (!g_cli_query_prompt.pending)
    {
        return;
    }

    if (g_cli_query_prompt.remaining > 0u)
    {
        g_cli_query_prompt.remaining--;
    }

    if (g_cli_query_prompt.remaining == 0u)
    {
        g_cli_query_prompt.pending = 0u;
        balance_cli_prompt();
    }
}

static void balance_cli_print_hal(const char *tag, HAL_StatusTypeDef st)
{
    const char *s;

    if (st == HAL_OK)
    {
        s = "ok";
    }
    else if (st == HAL_BUSY)
    {
        s = "busy";
    }
    else
    {
        s = "err";
    }

    (void)UART1_LogPrintfDrop("%s: %s\r\n", tag, s);
}

static void balance_cli_print_help(void)
{
    (void)UART1_LogPrintfDrop("help                    : show help\r\n");
    (void)UART1_LogPrintfDrop("Default target: both motors. Prefix with 'm7' or 'm10' for single motor.\r\n");
    (void)UART1_LogPrintfDrop("bal on | bal off        : enable / disable closed-loop balance\r\n");
    (void)UART1_LogPrintfDrop("vofa                    : switch back to JustFloat stream mode\r\n");

    (void)UART1_LogPrintfDrop("\r\n");
    (void)UART1_LogPrintfDrop("Manual motor cmds require: bal off\r\n");
    (void)UART1_LogPrintfDrop("en | dis                : enable / disable motor\r\n");
    (void)UART1_LogPrintfDrop("mode open|cur|spd       : set motor mode\r\n");
    (void)UART1_LogPrintfDrop("spd <rpm>               : speed command, example: spd 30\r\n");
    (void)UART1_LogPrintfDrop("cur <amp>               : current command, example: cur 0.5\r\n");
    (void)UART1_LogPrintfDrop("open <value>            : open-loop command, example: open 1000\r\n");
    (void)UART1_LogPrintfDrop("qpos                    : query total turns + position\r\n");
    (void)UART1_LogPrintfDrop("qmode                   : query current mode\r\n");
    (void)UART1_LogPrintfDrop("qver                    : query version\r\n");
    (void)UART1_LogPrintfDrop("stat                    : print latest cached status\r\n");
    (void)UART1_LogPrintfDrop("raw xx xx .. xx         : send raw 10-byte hex frame\r\n");
    (void)UART1_LogPrintfDrop("map                     : print protocol mapping\r\n");
    (void)UART1_LogPrintfDrop("auto start | auto stop  : run / stop auto test\r\n");

    (void)UART1_LogPrintfDrop("\r\n");
    (void)UART1_LogPrintfDrop("switch from VOFA stream to CLI with: #CLI!\r\n");
    (void)UART1_LogPrintfDrop("in VOFA mode: #BAL=1! / #BAL=0! to enable / disable balance\r\n");
}

static void balance_cli_print_map(void)
{
    (void)UART1_LogPrintfDrop("A0 -> set mode, ack A1\r\n");
    (void)UART1_LogPrintfDrop("64 -> drive cmd, ack 65(speed/current/temp/err)\r\n");
    (void)UART1_LogPrintfDrop("74 -> query turns/pos, ack 75\r\n");
    (void)UART1_LogPrintfDrop("75 -> query mode, ack 76\r\n");
    (void)UART1_LogPrintfDrop("FD -> query version, ack FE\r\n");
    (void)UART1_LogPrintfDrop("mode: 00=open 01=current 02=speed 08=enable 09=disable\r\n");
}

static void balance_cli_print_status(void)
{
    float pos_err = 0.0f;

    if (chassis_pos_ref_valid)
    {
        pos_err = chassis_pos_ref_raw - chassis_pos_fb_raw;
    }

    (void)UART1_LogPrintfDrop(
        "uart1=%s bal=%s | pitch=%.2fdeg yaw=%.2fdeg spd_d20=%.1f pos_fb=%.1f pos_ref=%.1f pos_err=%.1f\r\n",
        (VOFA_UART1_GetMode() == UART1_LINK_MODE_CLI) ? "CLI" : "VOFA",
        (Balance_GetClosedLoopEnable() != 0U) ? "ON" : "OFF",
        INS.Pitch * 57.2957795f,
        INS.YawTotalAngle * 57.2957795f,
        chassis_speed_d_raw_20ms(),
        chassis_pos_fb_raw,
        chassis_pos_ref_raw,
        pos_err);

    (void)UART1_LogPrintfDrop(
        "P1 bal_kp=%.3f P2 bal_kd=%.3f P3 spd_kp=%.3f P4 pos_ki=%.3f P5 turn_kp=%.3f P6 turn_kd=%.3f\r\n",
        g_balance_param.balance_kp,
        g_balance_param.balance_kd,
        g_balance_param.speed_kp,
        g_balance_param.speed_ki,
        g_balance_param.turn_kp,
        g_balance_param.turn_kd);

    (void)UART1_LogPrintfDrop(
        "m7 rpm=%.1f self_turns=%ld raw=%u pos=%.3fdeg d_raw_20ms=%ld\r\n",
        motor_rpm_from_feedback(&motor_uart7_dma),
        (long)motor_uart7_dma.fb.self_total_turns,
        (unsigned)motor_uart7_dma.fb.position_raw,
        ((float)m0603_pos_raw_to_mdeg(motor_uart7_dma.fb.position_raw)) * 0.001f,
        (long)motor_uart7_dma.fb.pos_raw_delta_20ms);

    (void)UART1_LogPrintfDrop(
        "m10 rpm=%.1f self_turns=%ld raw=%u pos=%.3fdeg d_raw_20ms=%ld\r\n",
        motor_rpm_from_feedback(&motor_uart10_dma),
        (long)motor_uart10_dma.fb.self_total_turns,
        (unsigned)motor_uart10_dma.fb.position_raw,
        ((float)m0603_pos_raw_to_mdeg(motor_uart10_dma.fb.position_raw)) * 0.001f,
        (long)motor_uart10_dma.fb.pos_raw_delta_20ms);
}

static void balance_cli_send_qpos(bal_cli_target_t target)
{
    HAL_StatusTypeDef st;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_query_pos_cli(&huart7);
        balance_cli_print_hal("m7 qpos", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_query_pos_cli(&huart10);
        balance_cli_print_hal("m10 qpos", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_qmode(bal_cli_target_t target)
{
    HAL_StatusTypeDef st;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_query_mode(&huart7);
        balance_cli_print_hal("m7 qmode", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_query_mode(&huart10);
        balance_cli_print_hal("m10 qmode", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_qver(bal_cli_target_t target)
{
    HAL_StatusTypeDef st;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_query_ver(&huart7);
        balance_cli_print_hal("m7 qver", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_query_ver(&huart10);
        balance_cli_print_hal("m10 qver", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_open(bal_cli_target_t target, int16_t open_raw)
{
    HAL_StatusTypeDef st;

    if (open_raw > BALANCE_OPEN_MAX) open_raw = BALANCE_OPEN_MAX;
    if (open_raw < -BALANCE_OPEN_MAX) open_raw = -BALANCE_OPEN_MAX;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_open_cli(&huart7, open_raw);
        balance_cli_print_hal("m7 open", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_open_cli(&huart10, open_raw);
        balance_cli_print_hal("m10 open", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_spd(bal_cli_target_t target, int16_t rpm)
{
    HAL_StatusTypeDef st;

    if (rpm > 380)  rpm = 380;
    if (rpm < -380) rpm = -380;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_speed(&huart7, rpm);
        balance_cli_print_hal("m7 spd", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_speed(&huart10, rpm);
        balance_cli_print_hal("m10 spd", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_cur(bal_cli_target_t target, float current_a)
{
    HAL_StatusTypeDef st;

    if (current_a > 3.9f)  current_a = 3.9f;
    if (current_a < -3.9f) current_a = -3.9f;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_current(&huart7, current_a);
        balance_cli_print_hal("m7 cur", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_current(&huart10, current_a);
        balance_cli_print_hal("m10 cur", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_mode(bal_cli_target_t target, uint8_t mode)
{
    HAL_StatusTypeDef st;

    if (!cli_query_prompt_begin())
    {
        return;
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_mode_cli(&huart7, mode);
        balance_cli_print_hal("m7 mode", st);
        cli_query_prompt_expect(st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_mode_cli(&huart10, mode);
        balance_cli_print_hal("m10 mode", st);
        cli_query_prompt_expect(st);
    }

    cli_query_prompt_finish_send_phase();
}

static void balance_cli_send_raw(bal_cli_target_t target, const uint8_t *frame)
{
    HAL_StatusTypeDef st;

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
    {
        st = motor_send_raw(&huart7, frame, 0u, 0u, "raw");
        balance_cli_print_hal("m7 raw", st);
    }

    if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
    {
        st = motor_send_raw(&huart10, frame, 0u, 0u, "raw");
        balance_cli_print_hal("m10 raw", st);
    }
}

static uint8_t motor_auto_is_running(const motor_uart_dma_t *ctx)
{
    if (ctx == NULL)
    {
        return 0u;
    }

    return (ctx->auto_state != AUTO_IDLE) &&
           (ctx->auto_state != AUTO_DONE) &&
           (ctx->auto_state != AUTO_FAIL);
}

static uint8_t balance_cli_target_auto_running(bal_cli_target_t target)
{
    if (target == BAL_CLI_TARGET_M7)
    {
        return motor_auto_is_running(&motor_uart7_dma);
    }

    if (target == BAL_CLI_TARGET_M10)
    {
        return motor_auto_is_running(&motor_uart10_dma);
    }

    return motor_auto_is_running(&motor_uart7_dma) ||
           motor_auto_is_running(&motor_uart10_dma);
}

void Balance_CLI_ProcessLine(char *line)
{
    char *argv[16];
    int argc = 0;
    int base = 0;
    char *tok;
    bal_cli_target_t target = BAL_CLI_TARGET_BOTH;

    if (line == NULL)
    {
        return;
    }

    while ((*line == ' ') || (*line == '\t'))
    {
        line++;
    }

    if (*line == '\0')
    {
        balance_cli_prompt();
        return;
    }

    tok = strtok(line, " \t");
    while ((tok != NULL) && (argc < 16))
    {
        argv[argc++] = tok;
        tok = strtok(NULL, " \t");
    }

    if (argc == 0)
    {
        balance_cli_prompt();
        return;
    }

    if (strcmp(argv[0], "help") == 0)
    {
        balance_cli_print_help();
        balance_cli_prompt();
        return;
    }

    if (strcmp(argv[0], "stat") == 0)
    {
        balance_cli_print_status();
        balance_cli_prompt();
        return;
    }

    if ((strcmp(argv[0], "bal") == 0) && (argc >= 2))
    {
        if (strcmp(argv[1], "on") == 0)
        {
            Balance_SetClosedLoopEnable(1U);
            (void)UART1_LogPrintfDrop("closed-loop balance: ON\r\n");
        }
        else if (strcmp(argv[1], "off") == 0)
        {
            Balance_SetClosedLoopEnable(0U);
            (void)UART1_LogPrintfDrop("closed-loop balance: OFF\r\n");
        }
        else
        {
            (void)UART1_LogPrintfDrop("usage: bal on | bal off\r\n");
        }

        balance_cli_prompt();
        return;
    }

    if ((strcmp(argv[0], "vofa") == 0) || (strcmp(argv[0], "stream") == 0))
    {
        VOFA_UART1_SetMode(UART1_LINK_MODE_VOFA);
        return;
    }

    if (strcmp(argv[0], "m7") == 0)
    {
        target = BAL_CLI_TARGET_M7;
        base = 1;
    }
    else if (strcmp(argv[0], "m10") == 0)
    {
        target = BAL_CLI_TARGET_M10;
        base = 1;
    }

    if (base >= argc)
    {
        balance_cli_prompt();
        return;
    }

    if (strcmp(argv[base], "map") == 0)
    {
        balance_cli_print_map();
        balance_cli_prompt();
        return;
    }

    if (strcmp(argv[base], "auto") == 0)
    {
        if ((argc > (base + 1)) && (strcmp(argv[base + 1], "stop") == 0))
        {
            if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
            {
                auto_test_stop(&motor_uart7_dma, "manual stop");
            }
            if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
            {
                auto_test_stop(&motor_uart10_dma, "manual stop");
            }

            balance_cli_prompt();
            return;
        }

        if (Balance_GetClosedLoopEnable() != 0U)
        {
            (void)UART1_LogPrintfDrop("manual motor cmd blocked while bal on, use 'bal off' first\r\n");
            balance_cli_prompt();
            return;
        }

        if ((argc > (base + 1)) && (strcmp(argv[base + 1], "start") == 0))
        {
            if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M7))
            {
                auto_test_start(&motor_uart7_dma);
            }
            if ((target == BAL_CLI_TARGET_BOTH) || (target == BAL_CLI_TARGET_M10))
            {
                auto_test_start(&motor_uart10_dma);
            }

            balance_cli_prompt();
            return;
        }

        (void)UART1_LogPrintfDrop("usage: [m7|m10] auto start|stop\r\n");
        balance_cli_prompt();
        return;
    }

    if (balance_cli_target_auto_running(target))
    {
        (void)UART1_LogPrintfDrop("auto running on target, use 'auto stop' first\r\n");
        balance_cli_prompt();
        return;
    }

    if (Balance_GetClosedLoopEnable() != 0U)
    {
        (void)UART1_LogPrintfDrop("manual motor cmd blocked while bal on, use 'bal off' first\r\n");
        balance_cli_prompt();
        return;
    }

    if (strcmp(argv[base], "qpos") == 0)
    {
        balance_cli_send_qpos(target);
        return;
    }

    if (strcmp(argv[base], "qmode") == 0)
    {
        balance_cli_send_qmode(target);
        return;
    }

    if (strcmp(argv[base], "qver") == 0)
    {
        balance_cli_send_qver(target);
        return;
    }

    if (strcmp(argv[base], "en") == 0)
    {
        balance_cli_send_mode(target, M0603_MODE_ENABLE);
        return;
    }

    if (strcmp(argv[base], "dis") == 0)
    {
        balance_cli_send_mode(target, M0603_MODE_DISABLE);
        return;
    }

    if ((strcmp(argv[base], "mode") == 0) && (argc > (base + 1)))
    {
        if (strcmp(argv[base + 1], "open") == 0)
        {
            balance_cli_send_mode(target, M0603_MODE_OPEN);
        }
        else if (strcmp(argv[base + 1], "cur") == 0)
        {
            balance_cli_send_mode(target, M0603_MODE_CURRENT);
        }
        else if (strcmp(argv[base + 1], "spd") == 0)
        {
            balance_cli_send_mode(target, M0603_MODE_SPEED);
        }
        else
        {
            (void)UART1_LogPrintfDrop("usage: [m7|m10] mode open|cur|spd\r\n");
            balance_cli_prompt();
        }

        return;
    }

    if ((strcmp(argv[base], "spd") == 0) && (argc > (base + 1)))
    {
        long rpm = strtol(argv[base + 1], NULL, 0);
        balance_cli_send_spd(target, (int16_t)rpm);
        return;
    }

    if ((strcmp(argv[base], "cur") == 0) && (argc > (base + 1)))
    {
        float current_a = strtof(argv[base + 1], NULL);
        balance_cli_send_cur(target, current_a);
        return;
    }

    if ((strcmp(argv[base], "open") == 0) && (argc > (base + 1)))
    {
        int16_t open_raw = (int16_t)atoi(argv[base + 1]);
        balance_cli_send_open(target, open_raw);
        return;
    }

    if (strcmp(argv[base], "raw") == 0)
    {
        uint8_t frame[M0603_FRAME_LEN];
        int i;

        if (argc != (base + 11))
        {
            (void)UART1_LogPrintfDrop("usage: [m7|m10] raw xx xx xx xx xx xx xx xx xx xx\r\n");
            balance_cli_prompt();
            return;
        }

        for (i = 0; i < M0603_FRAME_LEN; i++)
        {
            frame[i] = (uint8_t)strtoul(argv[base + 1 + i], NULL, 16);
        }

        balance_cli_send_raw(target, frame);
        balance_cli_prompt();
        return;
    }

    (void)UART1_LogPrintfDrop("unknown cmd\r\n");
    balance_cli_prompt();
}

static float clampf_local(float x, float min_v, float max_v) {
    if (x < min_v) {
        return min_v;
    }
    if (x > max_v) {
        return max_v;
    }
    return x;
}

static float chassis_speed_d_raw_20ms(void)
{
    float m7_d = (float)motor_uart7_dma.fb.pos_raw_delta_20ms;
    float m10_d = (float)motor_uart10_dma.fb.pos_raw_delta_20ms;

    return 0.5f * (m7_d - m10_d);
}

static float motor_continuous_pos_raw(const motor_uart_dma_t *ctx)
{
    if ((ctx == NULL) || (ctx->fb.pos_valid == 0u))
    {
        return 0.0f;
    }

    return ((float)ctx->fb.self_total_turns * 32768.0f)
         + (float)ctx->fb.position_raw;
}

static float chassis_position_raw(void)
{
    float m7_pos = motor_continuous_pos_raw(&motor_uart7_dma);
    float m10_pos = motor_continuous_pos_raw(&motor_uart10_dma);

    return 0.5f * (m7_pos - m10_pos);
}

static uint8_t chassis_pos_sample_ready(void)
{
    if ((motor_uart7_dma.fb.last_pos_tick == 0u) ||
        (motor_uart10_dma.fb.last_pos_tick == 0u))
    {
        return 0u;
    }

    if ((motor_uart7_dma.fb.last_pos_tick != speed_pos_tick_m7_used) &&
        (motor_uart10_dma.fb.last_pos_tick != speed_pos_tick_m10_used))
    {
        return 1u;
    }

    return 0u;
}

static void control_state_reset(void)
{
    speed_lpf_d_raw_20ms = 0.0f;

    chassis_pos_fb_raw = 0.0f;
    chassis_pos_ref_raw = 0.0f;
    chassis_pos_ref_valid = 0u;
    speed_pos_tick_m7_used  = motor_uart7_dma.fb.last_pos_tick;
    speed_pos_tick_m10_used = motor_uart10_dma.fb.last_pos_tick;

    yaw_ref_total = 0.0f;
    yaw_ref_valid = 0u;
}

static float speed_loop_calc(float speed_d_raw_20ms, uint8_t reset)
{
    float speed_err;
    float pos_err;
    float out;

    if (reset)
    {
        speed_lpf_d_raw_20ms = 0.0f;
        // speed_i_term = 0.0f;

        chassis_pos_fb_raw = 0.0f;
        chassis_pos_ref_raw = 0.0f;
        chassis_pos_ref_valid = 0u;
        speed_pos_tick_m7_used  = motor_uart7_dma.fb.last_pos_tick;
        speed_pos_tick_m10_used = motor_uart10_dma.fb.last_pos_tick;
        return 0.0f;
    }

    speed_lpf_d_raw_20ms = SPEED_LPF_ALPHA * speed_lpf_d_raw_20ms
                         + (1.0f - SPEED_LPF_ALPHA) * speed_d_raw_20ms;

    if (chassis_pos_sample_ready())
    {
        chassis_pos_fb_raw = chassis_position_raw();

        if (!chassis_pos_ref_valid)
        {
            chassis_pos_ref_raw = chassis_pos_fb_raw;
            chassis_pos_ref_valid = 1u;
        }

        speed_pos_tick_m7_used  = motor_uart7_dma.fb.last_pos_tick;
        speed_pos_tick_m10_used = motor_uart10_dma.fb.last_pos_tick;
    }

    speed_err = SPEED_TARGET_D_RAW_20MS - speed_lpf_d_raw_20ms;

    if (chassis_pos_ref_valid)
    {
        pos_err = clampf_local(chassis_pos_ref_raw - chassis_pos_fb_raw,
                               -SPEED_POS_LIMIT,
                               SPEED_POS_LIMIT);
    }
    else
    {
        pos_err = 0.0f;
    }

    out = g_balance_param.speed_kp * speed_err
        + g_balance_param.speed_ki * pos_err;

    return clampf_local(out, -SPEED_OUT_LIMIT, SPEED_OUT_LIMIT);
}

static float turn_loop_calc(float yaw_total, float gyro_z, uint8_t reset) {
    float err;
    float out;

    if (reset) {
        yaw_ref_valid = 0u;
        return 0.0f;
    }

    if (!yaw_ref_valid) {
        yaw_ref_total = yaw_total;
        yaw_ref_valid = 1u;
        return 0.0f;
    }

    err = yaw_ref_total - yaw_total;
    out = g_balance_param.turn_kp * err - g_balance_param.turn_kd * gyro_z;

    return clampf_local(out, -TURN_OUT_LIMIT, TURN_OUT_LIMIT);
}

static int16_t open_from_signed(float u) {
    float mag = fabsf(u);

    if (mag < OPEN_ZERO_THRESH) {
        return 0;
    }

    if (mag < BALANCE_OPEN_MIN) {
        mag = BALANCE_OPEN_MIN;
    }

    if (mag > BALANCE_OPEN_MAX) {
        mag = BALANCE_OPEN_MAX;
    }

    return (int16_t) ((u > 0.0f) ? mag : -mag);
}

static uint32_t m0603_pos_raw_to_mdeg(uint16_t pos_raw) {
    return (uint32_t)(((uint64_t)pos_raw * 360000ULL) / 32768ULL);
}

static int32_t m0603_update_self_turns_and_delta(int32_t *self_turns,
                                                 uint16_t raw_now,
                                                 uint16_t raw_prev)
{
    int32_t d_raw = (int32_t)raw_now - (int32_t)raw_prev;

    if (d_raw < -16384)
    {
        (*self_turns)++;
        d_raw += 32768;
    }
    else if (d_raw > 16384)
    {
        (*self_turns)--;
        d_raw -= 32768;
    }

    return d_raw;
}

static int32_t m0603_pos_raw_delta_shortest(uint16_t raw_now, uint16_t raw_prev)
{
    int32_t d = (int32_t)raw_now - (int32_t)raw_prev;

    if (d > 16384)
    {
        d -= 32768;
    }
    else if (d < -16384)
    {
        d += 32768;
    }

    return d;
}

static const uint8_t crc8_maxim_table[256] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

static uint8_t crc8_maxim_calc(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc = crc8_maxim_table[crc ^ *data++];
    }
    return crc;
}

static void m0603_make_set_mode(uint8_t *buf, uint8_t id, uint8_t mode) {
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0xA0;
    buf[2] = mode;
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_speed(uint8_t *buf, uint8_t id, int16_t speed_drpm) {
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0x64;
    buf[2] = (uint8_t)(((uint16_t)speed_drpm) >> 8);
    buf[3] = (uint8_t)(speed_drpm & 0xFF);
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_current(uint8_t *buf, uint8_t id, float current_a) {
    int16_t raw;

    if (current_a > 3.9f)  current_a = 3.9f;
    if (current_a < -3.9f) current_a = -3.9f;
    raw = (int16_t)(current_a * (32767.0f / 4.0f));

    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0x64;
    buf[2] = (uint8_t)(((uint16_t)raw) >> 8);
    buf[3] = (uint8_t)(raw & 0xFF);
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_open(uint8_t *buf, uint8_t id, int16_t open_raw) {
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0x64;
    buf[2] = (uint8_t) (((uint16_t) open_raw) >> 8);
    buf[3] = (uint8_t) (open_raw & 0xFF);
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_query_pos(uint8_t *buf, uint8_t id) {
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0x74;
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_query_mode(uint8_t *buf, uint8_t id) {
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0x75;
    buf[9] = crc8_maxim_calc(buf, 9);
}

static void m0603_make_query_ver(uint8_t *buf, uint8_t id) {
    memset(buf, 0, M0603_FRAME_LEN);
    buf[0] = id;
    buf[1] = 0xFD;
    buf[9] = crc8_maxim_calc(buf, 9);
}

static const char *motor_mode_name(uint8_t mode)
{
    switch (mode)
    {
        case 0x00: return "open";
        case 0x01: return "current";
        case 0x02: return "speed";
        case 0x08: return "enable";
        case 0x09: return "disable";
        default:   return "unknown";
    }
}

static const char *motor_mode_cmd_note(uint8_t mode)
{
    switch (mode)
    {
        case M0603_MODE_ENABLE:  return "enable";
        case M0603_MODE_DISABLE: return "disable";
        default:                 return "mode";
    }
}

static void motor_dump_frame_prefix(const motor_uart_dma_t *ctx, const char *tag, const uint8_t *f, uint16_t len)
{
    char line[128];
    int n = 0;
    uint16_t i;

    if ((ctx == NULL) || (tag == NULL) || (f == NULL)) {
        return;
    }

    n += snprintf(&line[n], sizeof(line) - (size_t)n, "%s %s", ctx->name, tag);

    for (i = 0; (i < len) && (n < (int)sizeof(line)); i++) {
        n += snprintf(&line[n], sizeof(line) - (size_t)n,
                      "%02X%s", f[i], (i + 1u < len) ? " " : "");
    }

    if (n < (int)(sizeof(line) - 2u)) {
        line[n++] = '\r';
        line[n++] = '\n';
        line[n] = '\0';
    } else {
        line[sizeof(line) - 3u] = '\r';
        line[sizeof(line) - 2u] = '\n';
        line[sizeof(line) - 1u] = '\0';
    }

    (void)UART1_LogPrintfDrop("%s", line);
}

// static void cli_printf(const char *fmt, ...)
// {
//     char buf[200];
//     va_list ap;
//     int n;
//
//     va_start(ap, fmt);
//     n = vsnprintf(buf, sizeof(buf), fmt, ap);
//     va_end(ap);
//
//     if (n <= 0)
//     {
//         return;
//     }
//     if (n > (int)sizeof(buf))
//     {
//         n = sizeof(buf);
//     }
//
//     HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
// }

// static HAL_StatusTypeDef motor_send_frame(UART_HandleTypeDef *huart, const uint8_t *frame)
// {
//     HAL_HalfDuplex_EnableTransmitter(huart);
//     return HAL_UART_Transmit(huart, (uint8_t *)frame, M0603_FRAME_LEN, 50);
// }
//
// static void motor_send_mode(UART_HandleTypeDef *huart, uint8_t mode)
// {
//     uint8_t frame[M0603_FRAME_LEN];
//     m0603_make_set_mode(frame, M0603_ID, mode);
//     (void)motor_send_frame(huart, frame);
// }
//
// static void motor_send_open(UART_HandleTypeDef *huart, int16_t open_raw)
// {
//     uint8_t frame[M0603_FRAME_LEN];
//     m0603_make_open(frame, M0603_ID, open_raw);
//     (void)motor_send_frame(huart, frame);
// }
static motor_uart_dma_t *motor_dma_ctx_from_huart(UART_HandleTypeDef *huart) {
    if (huart == &huart7) {
        return &motor_uart7_dma;
    }
    if (huart == &huart10) {
        return &motor_uart10_dma;
    }
    return NULL;
}

static void motor_start_rx_dma(motor_uart_dma_t *ctx) {
    if (ctx == NULL) {
        return;
    }

    HAL_UART_DMAStop(ctx->huart);
    HAL_HalfDuplex_EnableReceiver(ctx->huart);

    if (HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart, ctx->rx_dma, sizeof(ctx->rx_dma)) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
    }
}

static void motor_handle_rx_frame(motor_uart_dma_t *ctx, const uint8_t *f) {
    uint8_t cmd;

    if ((ctx == NULL) || (f == NULL)) {
        return;
    }

    if ((f[0] != M0603_ID) || (crc8_maxim_calc(f, 9) != f[9])) {
        return;
    }

    cmd = f[1];

    switch (cmd)
    {
        case 0x65:
            ctx->fb.speed_drpm = (int16_t)(((uint16_t)f[2] << 8) | f[3]);
            ctx->fb.current_raw = (int16_t)(((uint16_t)f[4] << 8) | f[5]);
            ctx->fb.accel_time = f[6];
            ctx->fb.temperature_c = f[7];
            ctx->fb.err_code = f[8];
            ctx->fb.last_update_tick = HAL_GetTick();
            break;

        case 0x75:
        {
            int32_t turns_now;
            uint16_t raw_now;

            turns_now = ((int32_t)f[2] << 24)
                      | ((int32_t)f[3] << 16)
                      | ((int32_t)f[4] << 8)
                      | (int32_t)f[5];

            raw_now = (uint16_t)(((uint16_t)f[6] << 8) | f[7]);

            if (ctx->fb.pos_valid) {
                ctx->fb.pos_raw_delta_20ms =
                    m0603_update_self_turns_and_delta(&ctx->fb.self_total_turns,
                                                      raw_now,
                                                      ctx->fb.prev_position_raw);
            } else {
                ctx->fb.pos_raw_delta_20ms = 0;
                ctx->fb.self_total_turns = 0;
                ctx->fb.pos_valid = 1u;
            }

            ctx->fb.total_turns = turns_now;
            ctx->fb.position_raw = raw_now;
            ctx->fb.prev_position_raw = raw_now;
            ctx->fb.err_code = f[8];
            ctx->fb.last_pos_tick = HAL_GetTick();
            break;
        }

        case 0x76:
            ctx->fb.mode = f[2];
            ctx->fb.last_mode_tick = HAL_GetTick();
            break;

        case 0xA1:
            ctx->fb.mode = f[2];
            ctx->fb.last_mode_tick = HAL_GetTick();
            break;

        case 0xFE:
            ctx->fb.ver_year  = f[2];
            ctx->fb.ver_month = f[3];
            ctx->fb.ver_day   = f[4];
            ctx->fb.ver_model = f[5];
            ctx->fb.ver_sw    = f[6];
            ctx->fb.ver_hw    = f[7];
            ctx->fb.last_ver_tick = HAL_GetTick();
            break;

        default:
            break;
    }

    if (ctx->wait_reply && (cmd == ctx->wait_expected))
    {
        ctx->wait_reply = 0u;
        ctx->wait_expected = 0u;
        ctx->wait_deadline = 0u;

        memcpy(ctx->ack_frame, f, M0603_FRAME_LEN);
        ctx->ack_cmd = cmd;

        memset(ctx->ack_note, 0, sizeof(ctx->ack_note));
        strncpy(ctx->ack_note, ctx->wait_note, sizeof(ctx->ack_note) - 1u);

        ctx->ack_pending = 1u;
    }
}

static void motor_parse_stream(motor_uart_dma_t *ctx) {
    uint16_t i;
    int found;

    if (ctx == NULL) {
        return;
    }

    while (ctx->stream_len >= M0603_FRAME_LEN) {
        found = 0;

        for (i = 0; i + M0603_FRAME_LEN <= ctx->stream_len; i++) {
            const uint8_t *f = &ctx->stream[i];

            if ((f[0] != M0603_ID) || (crc8_maxim_calc(f, 9) != f[9])) {
                continue;
            }

            if ((f[1] != 0x65) &&
                (f[1] != 0x75) &&
                (f[1] != 0x76) &&
                (f[1] != 0xA1) &&
                (f[1] != 0xFE)) {
                continue;
                }

            motor_handle_rx_frame(ctx, f);

            memmove(ctx->stream,
                    &ctx->stream[i + M0603_FRAME_LEN],
                    ctx->stream_len - (i + M0603_FRAME_LEN));
            ctx->stream_len = (uint16_t)(ctx->stream_len - (i + M0603_FRAME_LEN));
            found = 1;
            break;
        }

        if (!found) {
            if (ctx->stream_len > (M0603_FRAME_LEN - 1u)) {
                memmove(ctx->stream,
                        &ctx->stream[ctx->stream_len - (M0603_FRAME_LEN - 1u)],
                        M0603_FRAME_LEN - 1u);
                ctx->stream_len = M0603_FRAME_LEN - 1u;
            }
            break;
        }
    }
}

static void motor_rx_consume(motor_uart_dma_t *ctx, const uint8_t *data, uint16_t len) {
    uint16_t copy_len;

    if ((ctx == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }

    copy_len = len;
    if (copy_len > (uint16_t) (sizeof(ctx->stream) - ctx->stream_len)) {
        copy_len = (uint16_t) (sizeof(ctx->stream) - ctx->stream_len);
    }

    if (copy_len == 0u) {
        ctx->stream_len = 0u;
        return;
    }

    memcpy(&ctx->stream[ctx->stream_len], data, copy_len);
    ctx->stream_len = (uint16_t) (ctx->stream_len + copy_len);

    motor_parse_stream(ctx);
}

static HAL_StatusTypeDef motor_send_frame_dma(motor_uart_dma_t *ctx, const uint8_t *frame) {
    if ((ctx == NULL) || (frame == NULL)) {
        return HAL_ERROR;
    }

    if (ctx->tx_busy) {
        return HAL_BUSY;
    }

    HAL_UART_DMAStop(ctx->huart);

    memcpy(ctx->tx_buf, frame, M0603_FRAME_LEN);

    HAL_HalfDuplex_EnableTransmitter(ctx->huart);

    if (HAL_UART_Transmit_DMA(ctx->huart, ctx->tx_buf, M0603_FRAME_LEN) != HAL_OK) {
        HAL_HalfDuplex_EnableReceiver(ctx->huart);
        motor_start_rx_dma(ctx);
        ctx->tx_busy = 0u;
        return HAL_ERROR;
    }

    ctx->tx_busy = 1u;
    return HAL_OK;
}

static HAL_StatusTypeDef motor_send_raw(UART_HandleTypeDef *huart,
                                        const uint8_t *frame,
                                        uint8_t expected_cmd,
                                        uint32_t timeout_ms,
                                        const char *note)
{
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);
    HAL_StatusTypeDef st;

    if ((ctx == NULL) || (frame == NULL))
    {
        return HAL_ERROR;
    }

    if (ctx->wait_reply || ctx->ack_pending)
    {
        return HAL_BUSY;
    }

    st = motor_send_frame_dma(ctx, frame);
    if (st != HAL_OK)
    {
        return st;
    }

    motor_dump_frame_prefix(ctx, "tx:", frame, M0603_FRAME_LEN);

    if ((expected_cmd != 0u) && (timeout_ms != 0u))
    {
        ctx->wait_reply = 1u;
        ctx->wait_expected = expected_cmd;
        ctx->wait_deadline = HAL_GetTick() + timeout_ms;

        memset(ctx->wait_note, 0, sizeof(ctx->wait_note));
        if (note != NULL)
        {
            strncpy(ctx->wait_note, note, sizeof(ctx->wait_note) - 1u);
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef motor_send_mode(UART_HandleTypeDef *huart, uint8_t mode) {
    uint8_t frame[M0603_FRAME_LEN];
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);

    m0603_make_set_mode(frame, M0603_ID, mode);
    return motor_send_frame_dma(ctx, frame);
}

static HAL_StatusTypeDef motor_send_open(UART_HandleTypeDef *huart, int16_t open_raw) {
    uint8_t frame[M0603_FRAME_LEN];
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);

    m0603_make_open(frame, M0603_ID, open_raw);
    return motor_send_frame_dma(ctx, frame);
}

static HAL_StatusTypeDef motor_send_mode_cli(UART_HandleTypeDef *huart, uint8_t mode)
{
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_set_mode(frame, M0603_ID, mode);
    return motor_send_raw(huart,
                          frame,
                          0xA1,
                          MOTOR_REPLY_TIMEOUT_MS,
                          motor_mode_cmd_note(mode));
}

static HAL_StatusTypeDef motor_send_open_cli(UART_HandleTypeDef *huart, int16_t open_raw)
{
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_open(frame, M0603_ID, open_raw);
    return motor_send_raw(huart,
                          frame,
                          0x65,
                          MOTOR_REPLY_TIMEOUT_MS,
                          "open");
}

static HAL_StatusTypeDef motor_send_speed(UART_HandleTypeDef *huart, int16_t rpm) {
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_speed(frame, M0603_ID, (int16_t)(rpm * 10));
    return motor_send_raw(huart, frame, 0x65, MOTOR_REPLY_TIMEOUT_MS, "speed");
}

static HAL_StatusTypeDef motor_send_current(UART_HandleTypeDef *huart, float current_a) {
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_current(frame, M0603_ID, current_a);
    return motor_send_raw(huart, frame, 0x65, MOTOR_REPLY_TIMEOUT_MS, "current");
}

static HAL_StatusTypeDef motor_send_query_pos(UART_HandleTypeDef *huart) {
    uint8_t frame[M0603_FRAME_LEN];
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);

    m0603_make_query_pos(frame, M0603_ID);
    return motor_send_frame_dma(ctx, frame);
}

static HAL_StatusTypeDef motor_send_query_pos_cli(UART_HandleTypeDef *huart) {
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_query_pos(frame, M0603_ID);
    return motor_send_raw(huart, frame, 0x75, MOTOR_REPLY_TIMEOUT_MS, "qpos");
}

static HAL_StatusTypeDef motor_send_query_mode(UART_HandleTypeDef *huart) {
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_query_mode(frame, M0603_ID);
    return motor_send_raw(huart, frame, 0x76, MOTOR_REPLY_TIMEOUT_MS, "qmode");
}

static HAL_StatusTypeDef motor_send_query_ver(UART_HandleTypeDef *huart) {
    uint8_t frame[M0603_FRAME_LEN];

    m0603_make_query_ver(frame, M0603_ID);
    return motor_send_raw(huart, frame, 0xFE, MOTOR_VER_TIMEOUT_MS, "qver");
}

static void auto_test_start(motor_uart_dma_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    if (motor_auto_is_running(ctx))
    {
        (void)UART1_LogPrintfDrop("%s auto already running\r\n", ctx->name);
        return;
    }

    ctx->auto_state = AUTO_START;
    ctx->auto_state_tick = HAL_GetTick();
    (void)UART1_LogPrintfDrop("%s auto start\r\n", ctx->name);
}

static void auto_test_stop(motor_uart_dma_t *ctx, const char *reason)
{
    uint8_t frame[M0603_FRAME_LEN];
    HAL_StatusTypeDef st;

    if (ctx == NULL)
    {
        return;
    }

    ctx->auto_state = AUTO_IDLE;
    ctx->auto_state_tick = 0u;

    ctx->wait_reply = 0u;
    ctx->wait_expected = 0u;
    ctx->wait_deadline = 0u;
    ctx->ack_pending = 0u;
    ctx->ack_cmd = 0u;
    ctx->wait_note[0] = '\0';
    ctx->ack_note[0] = '\0';

    /* 立即发 disable */
    m0603_make_set_mode(frame, M0603_ID, M0603_MODE_DISABLE);
    st = HAL_UART_Transmit_DMA(ctx->huart, frame, M0603_FRAME_LEN);

    (void)UART1_LogPrintfDrop("%s auto stop: %s, disable=%s\r\n",
                              ctx->name,
                              (reason != NULL) ? reason : "",
                              (st == HAL_OK) ? "ok" :
                              (st == HAL_BUSY) ? "busy" :
                              (st == HAL_ERROR) ? "error" : "timeout");
}

static void auto_test_service(motor_uart_dma_t *ctx)
{
    uint8_t frame[M0603_FRAME_LEN];
    uint32_t now;

    if (ctx == NULL)
    {
        return;
    }

    now = HAL_GetTick();

    switch (ctx->auto_state)
    {
        case AUTO_IDLE:
        case AUTO_DONE:
        case AUTO_FAIL:
            break;

        case AUTO_START:
            m0603_make_set_mode(frame, M0603_ID, M0603_MODE_ENABLE);
            if (motor_send_raw(ctx->huart, frame, 0xA1, MOTOR_REPLY_TIMEOUT_MS, "auto enable") == HAL_OK)
            {
                ctx->auto_state = AUTO_WAIT_ENABLE;
            }
            break;

        case AUTO_WAIT_ENABLE:
            if ((!ctx->wait_reply) && (!ctx->ack_pending) && (!ctx->tx_busy))
            {
                m0603_make_set_mode(frame, M0603_ID, M0603_MODE_SPEED);
                if (motor_send_raw(ctx->huart, frame, 0xA1, MOTOR_REPLY_TIMEOUT_MS, "auto speed mode") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_SPEED_MODE;
                }
            }
            break;

        case AUTO_WAIT_SPEED_MODE:
            if ((!ctx->wait_reply) && (!ctx->ack_pending) && (!ctx->tx_busy))
            {
                m0603_make_speed(frame, M0603_ID, 300);   /* +30.0 rpm */
                if (motor_send_raw(ctx->huart, frame, 0x65, MOTOR_REPLY_TIMEOUT_MS, "auto +30rpm") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_FWD;
                }
            }
            break;

        case AUTO_WAIT_FWD:
            if ((!ctx->wait_reply) && (!ctx->ack_pending))
            {
                ctx->auto_state = AUTO_HOLD_FWD;
                ctx->auto_state_tick = now;
            }
            break;

        case AUTO_HOLD_FWD:
            if ((now - ctx->auto_state_tick) >= AUTO_HOLD_MS)
            {
                m0603_make_speed(frame, M0603_ID, -300);  /* -30.0 rpm */
                if (motor_send_raw(ctx->huart, frame, 0x65, MOTOR_REPLY_TIMEOUT_MS, "auto -30rpm") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_REV;
                }
            }
            break;

        case AUTO_WAIT_REV:
            if ((!ctx->wait_reply) && (!ctx->ack_pending))
            {
                ctx->auto_state = AUTO_HOLD_REV;
                ctx->auto_state_tick = now;
            }
            break;

        case AUTO_HOLD_REV:
            if ((now - ctx->auto_state_tick) >= AUTO_HOLD_MS)
            {
                m0603_make_speed(frame, M0603_ID, 0);
                if (motor_send_raw(ctx->huart, frame, 0x65, MOTOR_REPLY_TIMEOUT_MS, "auto stop") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_STOP;
                }
            }
            break;

        case AUTO_WAIT_STOP:
            if ((!ctx->wait_reply) && (!ctx->ack_pending) && (!ctx->tx_busy))
            {
                m0603_make_query_pos(frame, M0603_ID);
                if (motor_send_raw(ctx->huart, frame, 0x75, MOTOR_REPLY_TIMEOUT_MS, "auto qpos") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_QPOS;
                }
            }
            break;

        case AUTO_WAIT_QPOS:
            if ((!ctx->wait_reply) && (!ctx->ack_pending) && (!ctx->tx_busy))
            {
                m0603_make_query_ver(frame, M0603_ID);
                if (motor_send_raw(ctx->huart, frame, 0xFE, MOTOR_VER_TIMEOUT_MS, "auto qver") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_QVER;
                }
            }
            break;

        case AUTO_WAIT_QVER:
            if ((!ctx->wait_reply) && (!ctx->ack_pending) && (!ctx->tx_busy))
            {
                m0603_make_set_mode(frame, M0603_ID, M0603_MODE_DISABLE);
                if (motor_send_raw(ctx->huart, frame, 0xA1, MOTOR_REPLY_TIMEOUT_MS, "auto disable") == HAL_OK)
                {
                    ctx->auto_state = AUTO_WAIT_DISABLE;
                }
            }
            break;

        case AUTO_WAIT_DISABLE:
            if ((!ctx->wait_reply) && (!ctx->ack_pending))
            {
                ctx->auto_state = AUTO_DONE;
                (void)UART1_LogPrintfDrop("%s auto done\r\n", ctx->name);
            }
            break;

        default:
            break;
    }

    if (ctx->auto_state == AUTO_FAIL)
    {
        (void)UART1_LogPrintfDrop("%s auto fail\r\n", ctx->name);
        ctx->auto_state = AUTO_IDLE;
    }
}

static void motor_service_timeout(motor_uart_dma_t *ctx)
{
    uint32_t now;

    if ((ctx == NULL) || !ctx->wait_reply)
    {
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - ctx->wait_deadline) < 0)
    {
        return;
    }

    ctx->wait_reply = 0u;
    ctx->wait_expected = 0u;
    ctx->wait_deadline = 0u;

    (void)UART1_LogPrintfDrop("%s timeout: %s\r\n", ctx->name, ctx->wait_note);

    if ((ctx->auto_state != AUTO_IDLE) &&
        (ctx->auto_state != AUTO_DONE) &&
        (ctx->auto_state != AUTO_FAIL))
    {
        ctx->auto_state = AUTO_FAIL;
    }

    cli_query_prompt_complete_one();
}

static void motor_service_cli_reply(motor_uart_dma_t *ctx)
{
    int32_t curr_mA;
    uint32_t curr_abs;
    int16_t rpm10;
    int16_t rpm_abs;
    uint32_t pos_mdeg;

    if ((ctx == NULL) || !ctx->ack_pending)
    {
        return;
    }

    ctx->ack_pending = 0u;

    motor_dump_frame_prefix(ctx, "rx:", ctx->ack_frame, M0603_FRAME_LEN);

    if (ctx->ack_cmd == 0x75)
    {
        pos_mdeg = m0603_pos_raw_to_mdeg(ctx->fb.position_raw);
        (void)UART1_LogPrintfDrop(
            "%s ack: %s -> turns=%ld self_turns=%ld pos_raw=%u pos=%lu.%03ludeg d_raw_20ms=%ld err=0x%02X\r\n",
            ctx->name,
            ctx->ack_note,
            (long)ctx->fb.total_turns,
            (long)ctx->fb.self_total_turns,
            (unsigned)ctx->fb.position_raw,
            pos_mdeg / 1000u,
            pos_mdeg % 1000u,
            (long)ctx->fb.pos_raw_delta_20ms,
            ctx->fb.err_code);
    }
    else if ((ctx->ack_cmd == 0x76) || (ctx->ack_cmd == 0xA1))
    {
        (void)UART1_LogPrintfDrop(
            "%s ack: %s -> mode=%s\r\n",
            ctx->name,
            ctx->ack_note,
            motor_mode_name(ctx->fb.mode));
    }
    else if (ctx->ack_cmd == 0xFE)
    {
        (void)UART1_LogPrintfDrop(
            "%s ack: %s -> ver=20%02u-%02u-%02u model=0x%02X sw=0x%02X hw=0x%02X\r\n",
            ctx->name,
            ctx->ack_note,
            ctx->fb.ver_year,
            ctx->fb.ver_month,
            ctx->fb.ver_day,
            ctx->fb.ver_model,
            ctx->fb.ver_sw,
            ctx->fb.ver_hw);
    }
    else if (ctx->ack_cmd == 0x65)
    {
        curr_mA = ((int32_t)ctx->fb.current_raw * 4000) / 32767;
        curr_abs = (uint32_t)((curr_mA < 0) ? -curr_mA : curr_mA);
        rpm10 = ctx->fb.speed_drpm;
        rpm_abs = (int16_t)((rpm10 < 0) ? -rpm10 : rpm10);

        (void)UART1_LogPrintfDrop(
            "%s ack: %s -> rpm=%s%d.%d I=%s%lu.%03luA T=%uC err=0x%02X\r\n",
            ctx->name,
            ctx->ack_note,
            (rpm10 < 0) ? "-" : "",
            rpm_abs / 10,
            rpm_abs % 10,
            (curr_mA < 0) ? "-" : "",
            curr_abs / 1000u,
            curr_abs % 1000u,
            ctx->fb.temperature_c,
            ctx->fb.err_code);
    }
    else
    {
        (void)UART1_LogPrintfDrop("%s ack: %s\r\n", ctx->name, ctx->ack_note);
    }

    cli_query_prompt_complete_one();
}

void Balance_MotorUartTxDone(UART_HandleTypeDef *huart) {
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);
    if (ctx != NULL) {
        ctx->tx_busy = 0;
        motor_start_rx_dma(ctx);
    }
}

void Balance_MotorUartTxError(UART_HandleTypeDef *huart) {
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);
    if (ctx != NULL) {
        HAL_UART_DMAStop(ctx->huart);
        HAL_UART_Abort(ctx->huart);

        __HAL_UART_CLEAR_OREFLAG(ctx->huart);
        __HAL_UART_CLEAR_FEFLAG(ctx->huart);
        __HAL_UART_CLEAR_NEFLAG(ctx->huart);
        __HAL_UART_CLEAR_PEFLAG(ctx->huart);
        __HAL_UART_CLEAR_IDLEFLAG(ctx->huart);

        ctx->tx_busy = 0;
        ctx->stream_len = 0;

        motor_start_rx_dma(ctx);
    }
}

void Balance_MotorUartRxEvent(UART_HandleTypeDef *huart, uint16_t Size) {
    motor_uart_dma_t *ctx = motor_dma_ctx_from_huart(huart);
    if (ctx != NULL) {
        motor_rx_consume(ctx, ctx->rx_dma, Size);
        motor_start_rx_dma(ctx);
    }
}


// static int16_t balance_limit_open(float u_abs)
// {
//     int32_t out = (int32_t)(u_abs);
//
//     if (out < BALANCE_OPEN_MIN)
//     {
//         out = BALANCE_OPEN_MIN;
//     }
//     if (out > BALANCE_OPEN_MAX)
//     {
//         out = BALANCE_OPEN_MAX;
//     }
//     return (int16_t)out;
// }

static void balance_motor_init(void) {
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

    motor_start_rx_dma(&motor_uart7_dma);
    motor_start_rx_dma(&motor_uart10_dma);
}

// void Balance_Task(void)
// {
//     uint32_t last_print = HAL_GetTick();
//
//     balance_motor_init();
//     // cli_printf("balance task start, BMI088 on SPI2, control=Pitch closed-loop\r\n");
//     (void)UART1_LogPrintfDrop("balance task start, BMI088 on SPI2, control=Pitch closed-loop\r\n");
//     while (1)
//     {
//         float pitch;
//         float pitch_gyro;
//         float u;
//         float u_abs;
//         int16_t open_mag;
//         int16_t m7_open = 0;
//         int16_t m10_open = 0;
//
//         if (INS.ins_flag == 0)
//         {
//             motor_send_open(&huart7, 0);
//             // osDelay(2);
//             motor_send_open(&huart10, 0);
//             osDelay(BALANCE_TASK_PERIOD_MS);
//             continue;
//         }
//
//         pitch = INS.Pitch;
//         pitch_gyro = INS.Gyro[0];
//
//         /*
//          * 目标符号关系：
//          * pitch > 0  -> M10 > 0, M7 < 0
//          * pitch < 0  -> M10 < 0, M7 > 0
//          *
//          * 因此先构造 u 与 pitch 同号
//          */
//         u = BALANCE_KP * pitch - BALANCE_KD * pitch_gyro;
//
//         if (fabsf(pitch) < BALANCE_DEADBAND_RAD)
//         {
//             u = 0.0f;
//         }
//
//         if (u > 0.0f)
//         {
//             u_abs = fabsf(u);
//             open_mag = balance_limit_open(u_abs);
//
//             m10_open =  open_mag;
//             m7_open  = -open_mag;
//         }
//         else if (u < 0.0f)
//         {
//             u_abs = fabsf(u);
//             open_mag = balance_limit_open(u_abs);
//
//             m10_open = -open_mag;
//             m7_open  =  open_mag;
//         }
//         else
//         {
//             m10_open = 0;
//             m7_open  = 0;
//         }
//
//         motor_send_open(&huart7, m7_open);
//         // osDelay(2);
//         motor_send_open(&huart10, m10_open);
//
//         if ((HAL_GetTick() - last_print) >= DEBUG_PRINT_PERIOD_MS)
//         {
//             last_print = HAL_GetTick();
//             // cli_printf("pitch=%.4f rad (%.2f deg), gyro_x=%.4f, m7=%d, m10=%d\r\n",
//             //            pitch,
//             //            pitch * 57.2957795f,
//             //            pitch_gyro,
//             //            m7_open,
//             //            m10_open);
//             (void)UART1_LogPrintfDrop("pitch=%.4f rad (%.2f deg), gyro_x=%.4f, m7=%d rpm=%.1f, m10=%d rpm=%.1f\r\n",
//                           pitch,
//                           pitch * 57.2957795f,
//                           pitch_gyro,
//                           m7_open,
//                           motor_rpm_from_feedback(&motor_uart7_dma),
//                           m10_open,
//                           motor_rpm_from_feedback(&motor_uart10_dma));
//         }
//
//         osDelay(BALANCE_TASK_PERIOD_MS);
//     }
// }

void Balance_Task(void) {
    uint32_t last_print = HAL_GetTick();
    uint32_t last_vofa  = HAL_GetTick();
    uint8_t control_phase = 0u;

    balance_motor_init();
    control_state_reset();

#if VOFA_UART1_ASCII_LOG_ENABLE
    (void) UART1_LogPrintfDrop(
        "balance task start, control=parallel 3-loop (balance+speed+turn), open=50Hz qpos=50Hz\r\n");
#endif

    while (1) {
        VOFA_UART1_Poll();
        motor_service_timeout(&motor_uart7_dma);
        motor_service_timeout(&motor_uart10_dma);
        motor_service_cli_reply(&motor_uart7_dma);
        motor_service_cli_reply(&motor_uart10_dma);

        auto_test_service(&motor_uart7_dma);
        auto_test_service(&motor_uart10_dma);

        if (Balance_GetClosedLoopEnable() == 0U)
{
            balance_fill_vofa_channels(0.0f, 0.0f, 0.0f);


#if VOFA_UART1_STREAM_ENABLE
    if ((VOFA_UART1_GetMode() == UART1_LINK_MODE_VOFA) &&
        ((HAL_GetTick() - last_vofa) >= 20U))
    {
        float ch[VOFA_UART1_CH_NUM];

        last_vofa = HAL_GetTick();
        Balance_GetVofaChannels(ch);
        (void)VOFA_UART1_Send8(ch);

        (void)motor_send_query_pos(&huart7);
        (void)motor_send_query_pos(&huart10);
    }
#endif

    osDelay(BALANCE_TASK_PERIOD_MS);
    continue;
}
        float pitch;
        float pitch_gyro;
        float yaw_total;
        float yaw_gyro;
        float speed_d_raw_20ms;

        float balance_out;
        float speed_out;
        float turn_out;
        float common_out;

        int16_t m7_open = 0;
        int16_t m10_open = 0;

        if (INS.ins_flag == 0) {
            control_state_reset();

            motor_send_open(&huart7, 0);
            motor_send_open(&huart10, 0);

            osDelay(BALANCE_TASK_PERIOD_MS);
            continue;
        }

        pitch = INS.Pitch;
        pitch_gyro = INS.Gyro[0];

        yaw_total = INS.YawTotalAngle;
        yaw_gyro = INS.Gyro[2];

        speed_d_raw_20ms = chassis_speed_d_raw_20ms();

        if (fabsf(pitch) > BALANCE_STOP_RAD) {
            control_state_reset();

            motor_send_open(&huart7, 0);
            motor_send_open(&huart10, 0);

            osDelay(BALANCE_TASK_PERIOD_MS);
            continue;
        }

        balance_out = g_balance_param.balance_kp * pitch
                    - g_balance_param.balance_kd * pitch_gyro;

        if (fabsf(pitch) < BALANCE_DEADBAND_RAD) {
            balance_out = 0.0f;
        }

        speed_out = speed_loop_calc(speed_d_raw_20ms, 0u);

        /* ===================== 3) 转向环：PD ===================== */
        turn_out = turn_loop_calc(yaw_total, yaw_gyro, 0u);

        /*
         * 并联三环：
         * common_out = 直立环 + 速度环
         * turn_out   = 转向差动分量
         *
         * 对当前这套机械安装方向：
         * - 车体前后公共驱动分量，在 M10/M7 上符号相反
         * - 原地转向分量，在 M10/M7 上符号相同
         */
        common_out = balance_out + speed_out;

        m10_open = open_from_signed(common_out + turn_out);
        m7_open = open_from_signed(-common_out + turn_out);

        balance_fill_vofa_channels(balance_out, speed_out, turn_out);

        if (control_phase == 0u) {
            motor_send_open(&huart7, m7_open);
            motor_send_open(&huart10, m10_open);
        } else {
            motor_send_query_pos(&huart7);
            motor_send_query_pos(&huart10);
        }

        control_phase ^= 1u;

#if VOFA_UART1_STREAM_ENABLE
        if ((VOFA_UART1_GetMode() == UART1_LINK_MODE_VOFA) &&
            ((HAL_GetTick() - last_vofa) >= 20U))
        {
            float ch[VOFA_UART1_CH_NUM];

            last_vofa = HAL_GetTick();
            Balance_GetVofaChannels(ch);
            (void)VOFA_UART1_Send8(ch);
        }
#endif

#if VOFA_UART1_ASCII_LOG_ENABLE
        if ((HAL_GetTick() - last_print) >= DEBUG_PRINT_PERIOD_MS) {
            last_print = HAL_GetTick();

            (void) UART1_LogPrintfDrop(
    "pitch=%.4f rad(%.2f deg) gx=%.4f yaw=%.2fdeg gz=%.4f spd_d20=%.1f | "
    "bal=%.1f spd=%.1f turn=%.1f | "
    "m7=%d rpm=%.1f turns=%ld raw=%u pos=%.3fdeg d_raw_20ms=%ld | "
    "m10=%d rpm=%.1f turns=%ld raw=%u pos=%.3fdeg d_raw_20ms=%ld\r\n",
    pitch,
    pitch * 57.2957795f,
    pitch_gyro,
    yaw_total * 57.2957795f,
    yaw_gyro,
    speed_d_raw_20ms,
    balance_out,
    speed_out,
    turn_out,

                m7_open,
                motor_rpm_from_feedback(&motor_uart7_dma),
                (long) motor_uart7_dma.fb.total_turns,
                (unsigned) motor_uart7_dma.fb.position_raw,
                ((float) m0603_pos_raw_to_mdeg(motor_uart7_dma.fb.position_raw)) * 0.001f,
                (long) motor_uart7_dma.fb.pos_raw_delta_20ms,

                m10_open,
                motor_rpm_from_feedback(&motor_uart10_dma),
                (long) motor_uart10_dma.fb.total_turns,
                (unsigned) motor_uart10_dma.fb.position_raw,
                ((float) m0603_pos_raw_to_mdeg(motor_uart10_dma.fb.position_raw)) * 0.001f,
                (long) motor_uart10_dma.fb.pos_raw_delta_20ms);
        }
#endif

        osDelay(BALANCE_TASK_PERIOD_MS);
    }
}
