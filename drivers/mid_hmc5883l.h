/**
 * @file    mid_hmc5883l.h
 * @brief   中间层接口 — HMC5883L 寄存器协议、配置、原始数据
 *
 * 参考 HMC5883L 数据手册（Honeywell 3-Axis Digital Compass IC）。
 * 寄存器顺序注意：X → Z → Y（Z 在 Y 之前）。
 */

#ifndef MID_HMC5883L_H
#define MID_HMC5883L_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *  寄存器地址
 * ============================================================================ */

#define HMC5883L_REG_CRA            0x00  /**< Config Register A：平均 + 数据率 + 偏置 */
#define HMC5883L_REG_CRB            0x01  /**< Config Register B：增益 */
#define HMC5883L_REG_MR             0x02  /**< Mode Register：模式选择 */
#define HMC5883L_REG_DXRA           0x03  /**< Data Output X MSB */
#define HMC5883L_REG_DXRB           0x04  /**< Data Output X LSB */
#define HMC5883L_REG_DZRA           0x05  /**< Data Output Z MSB（注意：Z 在 Y 之前） */
#define HMC5883L_REG_DZRB           0x06  /**< Data Output Z LSB */
#define HMC5883L_REG_DYRA           0x07  /**< Data Output Y MSB */
#define HMC5883L_REG_DYRB           0x08  /**< Data Output Y LSB */
#define HMC5883L_REG_SR             0x09  /**< Status Register */
#define HMC5883L_REG_IRA            0x0A  /**< Identification Register A（'H'=0x48） */
#define HMC5883L_REG_IRB            0x0B  /**< Identification Register B（'4'=0x34） */
#define HMC5883L_REG_IRC            0x0C  /**< Identification Register C（'3'=0x33） */

/* ============================================================================
 *  增益枚举（CRB bits 5-7 → LSB/Gauss）
 * ============================================================================ */

typedef enum {
    HMC5883L_GAIN_1370 = 0x00,  /**< ±0.88 Ga, 1370 LSB/Gauss */
    HMC5883L_GAIN_1090 = 0x01,  /**< ±1.3  Ga, 1090 LSB/Gauss（默认） */
    HMC5883L_GAIN_820  = 0x02,  /**< ±1.9  Ga, 820  LSB/Gauss */
    HMC5883L_GAIN_660  = 0x03,  /**< ±2.5  Ga, 660  LSB/Gauss */
    HMC5883L_GAIN_440  = 0x04,  /**< ±4.0  Ga, 440  LSB/Gauss */
    HMC5883L_GAIN_390  = 0x05,  /**< ±4.7  Ga, 390  LSB/Gauss */
    HMC5883L_GAIN_330  = 0x06,  /**< ±5.6  Ga, 330  LSB/Gauss */
    HMC5883L_GAIN_230  = 0x07,  /**< ±8.1  Ga, 230  LSB/Gauss */
} HMC5883L_Gain_t;

/* ============================================================================
 *  数据率枚举（CRA bits 2-4）
 * ============================================================================ */

typedef enum {
    HMC5883L_RATE_0_75HZ = 0x00,
    HMC5883L_RATE_1_5HZ  = 0x01,
    HMC5883L_RATE_3HZ    = 0x02,
    HMC5883L_RATE_7_5HZ  = 0x03,
    HMC5883L_RATE_15HZ   = 0x04,
    HMC5883L_RATE_30HZ   = 0x05,
    HMC5883L_RATE_75HZ   = 0x06,  /**< 最高 75 Hz（默认） */
} HMC5883L_Rate_t;

/* ============================================================================
 *  采样平均枚举（CRA bits 5-6）
 * ============================================================================ */

typedef enum {
    HMC5883L_AVG_1 = 0x00,
    HMC5883L_AVG_2 = 0x01,
    HMC5883L_AVG_4 = 0x02,
    HMC5883L_AVG_8 = 0x03,  /**< 默认 8 次平均 */
} HMC5883L_Average_t;

/* ============================================================================
 *  测量模式枚举（MR bits 0-1）
 * ============================================================================ */

typedef enum {
    HMC5883L_MODE_CONTINUOUS = 0x00,  /**< 连续测量（默认） */
    HMC5883L_MODE_SINGLE     = 0x01,  /**< 单次测量（测量后自动进入空闲） */
    HMC5883L_MODE_IDLE       = 0x02,  /**< 空闲/待机 */
} HMC5883L_Mode_t;

/* ============================================================================
 *  增益 → 量纲换算表（raw → μT）
 *
 *  1 Gauss = 100 μT
 *  mag_uT = raw * (100.0 / gain_lsb_per_gauss)
 * ============================================================================ */

/** 根据增益枚举值返回 LSB/Gauss 数值 */
static inline uint16_t HMC5883L_GainLSB(HMC5883L_Gain_t g)
{
    const uint16_t tbl[] = { 1370, 1090, 820, 660, 440, 390, 330, 230 };
    return tbl[g];
}

/* ============================================================================
 *  数据结构
 * ============================================================================ */

/** 原始磁力计数据（寄存器顺序：X→Z→Y） */
typedef struct {
    int16_t x;  /**< X 轴原始值 */
    int16_t z;  /**< Z 轴原始值（在 Y 之前） */
    int16_t y;  /**< Y 轴原始值 */
} HMC5883L_RawData_t;

/** 配置参数 */
typedef struct {
    HMC5883L_Average_t avg;    /**< 采样平均次数（默认 8） */
    HMC5883L_Rate_t    rate;   /**< 数据输出率（默认 75 Hz） */
    HMC5883L_Gain_t    gain;   /**< 增益档位（默认 ±1.3 Ga） */
    HMC5883L_Mode_t    mode;   /**< 测量模式（默认连续） */
} HMC5883L_Config_t;

/** 硬铁 + 软铁校准参数 */
typedef struct {
    int16_t offset[3];  /**< 硬铁偏移（raw LSB），顺序 X/Y/Z */
    float   scale[3];   /**< 软铁缩放（默认 {1,1,1}），顺序 X/Y/Z */
    bool    valid;      /**< 校准参数是否有效 */
} HMC5883L_Calib_t;

/* ============================================================================
 *  API
 * ============================================================================ */

/**
 * @brief  初始化 HMC5883L
 * @param  cfg  配置参数（NULL 使用默认：8x平均, 75Hz, ±1.3Ga, 连续模式）
 * @return true 初始化成功（probe 通过 + 全部寄存器配置成功）
 *
 * 内部流程：
 *   1. 调用 HAL_HMC5883L_Init()（DRDY 引脚）
 *   2. 使能 MPU6050 I2C Bypass（读→改 INT_PIN_CFG bit1→写回）
 *   3. delay_ms(10) 等待 bypass 稳定
 *   4. Probe：读 ID 寄存器，预期 'H' '4' '3'
 *   5. 配置 CRA（平均 + 数据率）、CRB（增益）、MR（连续模式）
 */
bool MID_HMC5883L_Init(const HMC5883L_Config_t *cfg);

/**
 * @brief  探测 HMC5883L 是否在线
 * @return true ID 寄存器一致（0x48, 0x34, 0x33）
 */
bool MID_HMC5883L_Probe(void);

/**
 * @brief  读取原始磁力计数据（6 字节：XH/XL/ZH/ZL/YH/YL）
 * @param  raw  输出缓冲区
 * @return true 读取成功
 */
bool MID_HMC5883L_ReadRaw(HMC5883L_RawData_t *raw);

/**
 * @brief  检查磁力计数据是否就绪（轮询 DRDY 引脚 PA13）
 * @return true 数据已就绪
 */
bool MID_HMC5883L_IsDataReady(void);

/**
 * @brief  获取当前配置
 * @return 配置指针（内部静态变量）
 */
const HMC5883L_Config_t *MID_HMC5883L_GetConfig(void);

/**
 * @brief  设定增益（需在初始化之后调用）
 * @param  gain  增益档位
 * @return true 写入成功
 */
bool MID_HMC5883L_SetGain(HMC5883L_Gain_t gain);

/**
 * @brief  设定数据率（需在初始化之后调用）
 * @param  rate  数据率
 * @return true 写入成功
 */
bool MID_HMC5883L_SetRate(HMC5883L_Rate_t rate);

/**
 * @brief  将原始值转换为 μT（已校准）
 * @param  raw    原始数据
 * @param  calib  校准参数（可为 NULL 表示未校准，offset=0, scale=1）
 * @param  mag    输出 μT 数组 [3]（顺序 X/Y/Z）
 */
void MID_HMC5883L_RawToMicroTesla(const HMC5883L_RawData_t *raw,
                                  const HMC5883L_Calib_t *calib,
                                  float mag[3]);

#endif /* MID_HMC5883L_H */
