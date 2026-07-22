/**
 * @file    grayscale.h
 * @brief   五路灰度传感器模块 —— API
 *
 * ======================== 硬件原理 ========================
 *
 *   TCRT5000 ×5 + LM393 电压比较器方案。
 *
 *   每个 TCRT5000 包含一个红外发射管和一个接收管。
 *   黑色表面吸收红外光 → 接收管电流小 → LM393 输出低电平 (0)
 *   白色表面反射红外光 → 接收管电流大 → LM393 输出高电平 (1)
 *
 *   LM393 将模拟的反射光强度转换为数字的 0/1。
 *   调节 LM393 上的电位器可以改变触发阈值。
 *
 * ======================== 引脚映射（经实车验证）========================
 *
 *   模块 7-pin 排针 → MSPM0G3507：
 *     Pin1: +5V  （外部供电，不经过 MCU）
 *     Pin2: GND  （与 MCU 共地）
 *     Pin3: PB27 → 传感器 1（车体最右侧）
 *     Pin4: PA29 → 传感器 2
 *     Pin5: PB23 → 传感器 3（车体中央）
 *     Pin6: PB12 → 传感器 4
 *     Pin7: PA12 → 传感器 5（车体最左侧）
 *
 * ======================== 位置计算方法 ========================
 *
 *   加权平均法：
 *     对每个检测到黑线（低电平）的传感器，用其权重累加到加权和，
 *     除以检测到黑线的传感器数量。
 *
 *   权重表：
 *     S1 = -2000   (最左)
 *     S2 = -1000
 *     S3 = 0       (中央)
 *     S4 = +1000
 *     S5 = +2000   (最右)
 *
 *   举例：
 *     仅 S3 黑 → pos = 0（线在中央）
 *     仅 S1 黑 → pos = -2000（线在最左）
 *     S2+S3 黑 → pos = (-1000+0)/2 = -500（线偏左）
 *     5路全黑 → pos = 0（异常：线太宽或模块离地太近）
 */

#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ===================================================================
 *  引脚宏（引用 SysConfig 自动生成的宏）
 *
 *  SysConfig 中 GPIO2 实例名为 "GPIO_GRAYSCALE"，
 *  引脚名为 GS_S1 ~ GS_S5，SysConfig 自动生成如下宏：
 *    GPIO_GRAYSCALE_GS_S1_PORT / GPIO_GRAYSCALE_GS_S1_PIN
 * =================================================================== */

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

/* ===================================================================
 *  数据结构
 * =================================================================== */

/** 五路灰度传感器原始读数（1=白, 0=黑） */
typedef struct {
    uint8_t s1;  /**< 最左 */
    uint8_t s2;
    uint8_t s3;  /**< 中央 */
    uint8_t s4;
    uint8_t s5;  /**< 最右 */
} GrayscaleData_t;

/** 五路灰度传感器运行时句柄 */
typedef struct {
    GrayscaleData_t raw;         /**< 最新原始读数 */
    int16_t position;            /**< 加权位置 [-2000, +2000] */
    uint8_t line_detected;       /**< 是否至少有一路检测到黑线 */
    uint8_t line_count;          /**< 检测到黑线的传感器数量 */
} Grayscale_t;

/* ===================================================================
 *  API
 * =================================================================== */

void Grayscale_Init(void);
void Grayscale_Read(Grayscale_t *gs);
int16_t Grayscale_GetPosition(Grayscale_t *gs);
int16_t Grayscale_GetPositionFiltered(Grayscale_t *gs, uint8_t samples);

#endif /* GRAYSCALE_H */
