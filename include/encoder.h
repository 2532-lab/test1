/**
 * @file    encoder.h
 * @brief   JGB37-520 四路增量式编码器驱动 —— API 定义
 *
 * == 编码器型号 ==
 *   JGB37-520 减速电机配套 AB 相正交编码器
 *   减速比 1:30，编码器 11 PPR（每圈脉冲数）
 *   输出轴每转脉冲数 CPR = 11 × 30 = 330
 *
 * == 架构（混合模式）==
 *   C1 → TIMG8 硬件编码器模式（QEI）
 *   C2/C3/C4 → 软件轮询 + SysTick 1kHz 中断扫描
 *
 * == 引脚映射 ==
 *   C1 (右前轮): A=PB21, B=PA30  → TIMG8 CCP0/CCP1（硬件 QEI）
 *   C2 (左前轮): A=PB22, B=PB16
 *   C3 (右后轮): A=PB20, B=PA22（AB 相接线反相，代码已处理）
 *   C4 (左后轮): A=PA15, B=PA14
 *
 * == 方向约定 ==
 *   轮子正转（车前进方向）→ count 正向累加 → rpm 为正
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ENCODER_C1 = 0,         /**< C1: 右前轮, TIMG8 硬件 QEI */
    ENCODER_C2 = 1,         /**< C2: 左前轮, 软件轮询 */
    ENCODER_C3 = 2,         /**< C3: 右后轮, 软件轮询（接线反相） */
    ENCODER_C4 = 3,         /**< C4: 左后轮, 软件轮询 */
    ENCODER_COUNT = 4
} EncoderID_t;

#define ENCODER_GEAR_RATIO      30
#define ENCODER_PPR             11
#define ENCODER_CPR             (ENCODER_PPR * 2 * ENCODER_GEAR_RATIO)
#define ENCODER_CPR_HW          ENCODER_CPR
#define ENC_SCAN_FREQ_HZ        1000U

void Encoder_Init(void);
void Encoder_Scan(void);
int32_t Encoder_GetCount(EncoderID_t id);
void Encoder_Reset(EncoderID_t id);
void Encoder_ResetAll(void);
bool Encoder_GetSpeed(EncoderID_t id, float dt, float *rpm);
int16_t Encoder_GetDelta(EncoderID_t id);

#endif
