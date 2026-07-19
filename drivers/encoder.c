/**
 * @file    encoder.c
 * @brief   四路 AB 相编码器驱动 —— 混合模式（硬件 QEI + 软件轮询）
 *
 *   C1 → TIMG8 硬件正交编码器模式（QEI），自动计数
 *   C2/C3/C4 → SysTick 1kHz 中断 + 软件轮询
 *
 * == 引脚映射 ==
 *   C1 (右前): PB21/PA30 → TIMG8
 *   C2 (左前): PB22/PB16
 *   C3 (右后): PB20/PA22（接线反相，代码已修正）
 *   C4 (左后): PA15/PA14
 */

#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "delay.h"
#include <string.h>

volatile uint32_t sys_tick_ms = 0;

#define SW_FIRST_IDX    1
#define SW_COUNT        3

#ifdef GPIO_ENCODER_C2_A_PORT
  #define ENC_C2_A_PORT   GPIO_ENCODER_C2_A_PORT
  #define ENC_C2_A_PIN    GPIO_ENCODER_C2_A_PIN
  #define ENC_C2_B_PORT   GPIO_ENCODER_C2_B_PORT
  #define ENC_C2_B_PIN    GPIO_ENCODER_C2_B_PIN
  #define ENC_C3_A_PORT   GPIO_ENCODER_C3_A_PORT
  #define ENC_C3_A_PIN    GPIO_ENCODER_C3_A_PIN
  #define ENC_C3_B_PORT   GPIO_ENCODER_C3_B_PORT
  #define ENC_C3_B_PIN    GPIO_ENCODER_C3_B_PIN
  #define ENC_C4_A_PORT   GPIO_ENCODER_C4_A_PORT
  #define ENC_C4_A_PIN    GPIO_ENCODER_C4_A_PIN
  #define ENC_C4_B_PORT   GPIO_ENCODER_C4_B_PORT
  #define ENC_C4_B_PIN    GPIO_ENCODER_C4_B_PIN
#else
  #define ENC_C2_A_PORT   GPIOB
  #define ENC_C2_A_PIN    DL_GPIO_PIN_22
  #define ENC_C2_B_PORT   GPIOB
  #define ENC_C2_B_PIN    DL_GPIO_PIN_16
  #define ENC_C3_A_PORT   GPIOB
  #define ENC_C3_A_PIN    DL_GPIO_PIN_20
  #define ENC_C3_B_PORT   GPIOA
  #define ENC_C3_B_PIN    DL_GPIO_PIN_22
  #define ENC_C4_A_PORT   GPIOA
  #define ENC_C4_A_PIN    DL_GPIO_PIN_15
  #define ENC_C4_B_PORT   GPIOA
  #define ENC_C4_B_PIN    DL_GPIO_PIN_14
#endif

typedef struct {
    GPIO_Regs *port_a; uint32_t pin_a;
    GPIO_Regs *port_b; uint32_t pin_b;
    uint8_t last_a; int32_t count;
    int16_t delta; int32_t prev_count;
} EncoderSw_t;

static EncoderSw_t s_enc_sw[SW_COUNT];
static uint32_t s_hw_prev_count = 0;

/* ---- 硬件编码器初始化 ---- */
static void Encoder_Init_HW(void)
{
#ifdef TIMG8_INST
    DL_TimerG_setTimerCount(TIMG8_INST, 0);
    DL_TimerG_startCounter(TIMG8_INST);
#endif
}

/* ---- 软件编码器扫描（ISR 内调用）---- */
void Encoder_Scan(void)
{
    for (int sw = 0; sw < SW_COUNT; sw++) {
        EncoderSw_t *e = &s_enc_sw[sw];
        int32_t prev = e->count;
        uint32_t raw = DL_GPIO_readPins(e->port_a, e->pin_a);
        uint8_t  cur = (raw != 0) ? 1 : 0;

        if (e->last_a != cur) {
            uint32_t b_raw = DL_GPIO_readPins(e->port_b, e->pin_b);
            int8_t   dir;
            if (sw == 1)  /* C3 接线反相 */
                dir = (b_raw != 0) ? -1 : 1;
            else
                dir = (b_raw != 0) ? 1 : -1;

            if (cur == 0) e->count += dir;
            else          e->count -= dir;
        }
        e->last_a = cur;
        e->delta = (int16_t)(e->count - prev);
    }
}

/* ---- SysTick 1kHz 初始化 ---- */
static void Encoder_ScanTimer_Init(void)
{
    SysTick->LOAD = 31999U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn,     192);
    NVIC_SetPriority(UART0_INT_IRQn,    64);
}

/* ---- SysTick ISR ---- */
void SysTick_Handler(void)
{
    sys_tick_ms++;
    Encoder_Scan();
}

/* ---- 初始化 ---- */
void Encoder_Init(void)
{
    Encoder_Init_HW();
    s_hw_prev_count = 0;
    memset(s_enc_sw, 0, sizeof(s_enc_sw));

    s_enc_sw[0].port_a = ENC_C2_A_PORT; s_enc_sw[0].pin_a = ENC_C2_A_PIN;
    s_enc_sw[0].port_b = ENC_C2_B_PORT; s_enc_sw[0].pin_b = ENC_C2_B_PIN;
    s_enc_sw[1].port_a = ENC_C3_A_PORT; s_enc_sw[1].pin_a = ENC_C3_A_PIN;
    s_enc_sw[1].port_b = ENC_C3_B_PORT; s_enc_sw[1].pin_b = ENC_C3_B_PIN;
    s_enc_sw[2].port_a = ENC_C4_A_PORT; s_enc_sw[2].pin_a = ENC_C4_A_PIN;
    s_enc_sw[2].port_b = ENC_C4_B_PORT; s_enc_sw[2].pin_b = ENC_C4_B_PIN;

    for (int i = 0; i < SW_COUNT; i++) {
        uint32_t v = DL_GPIO_readPins(s_enc_sw[i].port_a, s_enc_sw[i].pin_a);
        s_enc_sw[i].last_a = (v != 0) ? 1 : 0;
        s_enc_sw[i].prev_count = s_enc_sw[i].count;
    }

    Encoder_ScanTimer_Init();
}

/* ---- 读取计数 ---- */
int32_t Encoder_GetCount(EncoderID_t id)
{
    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        return (int32_t)DL_TimerG_getTimerCount(TIMG8_INST);
#else
        return 0;
#endif
    }
    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return 0;
    return s_enc_sw[sw].count;
}

/* ---- 复位 ---- */
void Encoder_Reset(EncoderID_t id)
{
    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        DL_TimerG_setTimerCount(TIMG8_INST, 0);
#endif
        s_hw_prev_count = 0;
        return;
    }
    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return;
    s_enc_sw[sw].count = 0;
    s_enc_sw[sw].delta = 0;
    s_enc_sw[sw].prev_count = 0;
}

void Encoder_ResetAll(void)
{
#ifdef TIMG8_INST
    DL_TimerG_setTimerCount(TIMG8_INST, 0);
#endif
    s_hw_prev_count = 0;
    for (int i = 0; i < SW_COUNT; i++) {
        s_enc_sw[i].count = 0;
        s_enc_sw[i].delta = 0;
        s_enc_sw[i].prev_count = 0;
    }
}

/* ---- 测速 ---- */
bool Encoder_GetSpeed(EncoderID_t id, float dt, float *rpm)
{
    if (rpm == NULL || dt <= 0.0f) return false;

    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        int32_t cur  = (int32_t)DL_TimerG_getTimerCount(TIMG8_INST);
        int32_t diff = cur - (int32_t)s_hw_prev_count;
        s_hw_prev_count = (uint32_t)cur;
        float revs = (float)diff / (float)ENCODER_CPR_HW;
        *rpm = revs / (dt / 60.0f);
        return true;
#else
        return false;
#endif
    }

    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return false;

    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    int32_t cur  = s_enc_sw[sw].count;
    int32_t diff = cur - s_enc_sw[sw].prev_count;
    s_enc_sw[sw].prev_count = cur;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    float revs = (float)diff / (float)ENCODER_CPR;
    *rpm = revs / (dt / 60.0f);
    return true;
}

/* ---- 增量 ---- */
int16_t Encoder_GetDelta(EncoderID_t id)
{
    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        int32_t cur  = (int32_t)DL_TimerG_getTimerCount(TIMG8_INST);
        return (int16_t)(cur - (int32_t)s_hw_prev_count);
#else
        return 0;
#endif
    }
    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return 0;
    return s_enc_sw[sw].delta;
}
