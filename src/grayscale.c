/**
 * @file    grayscale.c
 * @brief   五路灰度传感器模块（TCRT5000 ×5 + LM393）
 *
 * 引脚映射：
 *   S1(最左)=PB27, S2=PA29, S3(中)=PB23, S4=PB12, S5(最右)=PA12
 */

#include "grayscale.h"
#include "ti_msp_dl_config.h"

static const int16_t GS_WEIGHT[5] = { -2000, -1000, 0, 1000, 2000 };

void Grayscale_Init(void) { /* SysConfig 已完成 */ }

static inline uint8_t GS_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0) ? 1 : 0;
}

void Grayscale_Read(Grayscale_t *gs)
{
    gs->raw.s1 = GS_ReadPin(GS_S1_PORT, GS_S1_PIN);
    gs->raw.s2 = GS_ReadPin(GS_S2_PORT, GS_S2_PIN);
    gs->raw.s3 = GS_ReadPin(GS_S3_PORT, GS_S3_PIN);
    gs->raw.s4 = GS_ReadPin(GS_S4_PORT, GS_S4_PIN);
    gs->raw.s5 = GS_ReadPin(GS_S5_PORT, GS_S5_PIN);

    gs->line_count = (!gs->raw.s1) + (!gs->raw.s2) + (!gs->raw.s3)
                   + (!gs->raw.s4) + (!gs->raw.s5);
    gs->line_detected = (gs->line_count > 0) ? 1 : 0;
}

static void GS_ReadToArray(const GrayscaleData_t *raw, uint8_t *arr)
{
    arr[0] = raw->s1; arr[1] = raw->s2; arr[2] = raw->s3;
    arr[3] = raw->s4; arr[4] = raw->s5;
}

int16_t Grayscale_GetPosition(Grayscale_t *gs)
{
    if (!gs->line_detected) return gs->position;

    uint8_t arr[5];
    GS_ReadToArray(&gs->raw, arr);

    int32_t sum_weighted = 0;
    uint8_t sum_count = 0;
    for (uint8_t i = 0; i < 5; i++) {
        if (arr[i] == 0) { sum_weighted += GS_WEIGHT[i]; sum_count++; }
    }

    if (sum_count > 0) gs->position = (int16_t)(sum_weighted / sum_count);
    return gs->position;
}

int16_t Grayscale_GetPositionFiltered(Grayscale_t *gs, uint8_t samples)
{
    if (samples == 0) return gs->position;
    if (samples > 9) samples = 9;

    int16_t buf[9];
    for (uint8_t i = 0; i < samples; i++) {
        Grayscale_Read(gs);
        Grayscale_GetPosition(gs);
        buf[i] = gs->position;
    }

    for (uint8_t i = 0; i < samples - 1; i++)
        for (uint8_t j = 0; j < samples - 1 - i; j++)
            if (buf[j] > buf[j + 1]) {
                int16_t tmp = buf[j]; buf[j] = buf[j + 1]; buf[j + 1] = tmp;
            }

    if (samples & 1)
        return buf[samples / 2];
    else
        return (int16_t)((buf[samples / 2 - 1] + buf[samples / 2]) / 2);
}
