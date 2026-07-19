#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define GS_S1_PORT  GPIO_GRAYSCALE_GS_S1_PORT
#define GS_S1_PIN   GPIO_GRAYSCALE_GS_S1_PIN
#define GS_S2_PORT  GPIO_GRAYSCALE_GS_S2_PORT
#define GS_S2_PIN   GPIO_GRAYSCALE_GS_S2_PIN
#define GS_S3_PORT  GPIO_GRAYSCALE_GS_S3_PORT
#define GS_S3_PIN   GPIO_GRAYSCALE_GS_S3_PIN
#define GS_S4_PORT  GPIO_GRAYSCALE_GS_S4_PORT
#define GS_S4_PIN   GPIO_GRAYSCALE_GS_S4_PIN
#define GS_S5_PORT  GPIO_GRAYSCALE_GS_S5_PORT
#define GS_S5_PIN   GPIO_GRAYSCALE_GS_S5_PIN

typedef struct {
    uint8_t s1, s2, s3, s4, s5;
} GrayscaleData_t;

typedef struct {
    GrayscaleData_t raw;
    int16_t position;
    uint8_t line_detected;
    uint8_t line_count;
} Grayscale_t;

void Grayscale_Init(void);
void Grayscale_Read(Grayscale_t *gs);
int16_t Grayscale_GetPosition(Grayscale_t *gs);
int16_t Grayscale_GetPositionFiltered(Grayscale_t *gs, uint8_t samples);

#endif
