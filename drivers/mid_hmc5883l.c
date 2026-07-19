/**
 * @file    mid_hmc5883l.c
 * @brief   中间层实现 — HMC5883L 寄存器协议
 *
 * 初始化时需先使能 MPU6050 I2C Bypass（INT_PIN_CFG bit 1），
 * 才能通过共享的 I2C1 总线访问 HMC5883L（地址 0x1E）。
 */

#include "mid_hmc5883l.h"
#include "hal_hmc5883l.h"
#include "delay.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 *  内部常量
 * ============================================================================ */

/** MPU6050 7-bit I2C 地址（用于 bypass 使能，不改动 mid_mpu6050 代码） */
#define MPU6050_ADDR            0x68

/** INT_PIN_CFG 寄存器地址 */
#define MPU6050_REG_INT_PIN_CFG 0x37
/** USER_CTRL 寄存器地址 */
#define MPU6050_REG_USER_CTRL   0x6A

/** I2C BYPASS 使能位 */
#define MPU6050_BIT_BYPASS_EN   (1 << 1)
/** AUX I2C Master 使能位（必须先清零才能开 bypass） */
#define MPU6050_BIT_I2C_MST_EN  (1 << 5)
/** AUX I2C 状态机复位位 */
#define MPU6050_BIT_I2C_MST_RST (1 << 1)

/* ============================================================================
 *  默认配置
 * ============================================================================ */

static HMC5883L_Config_t s_config = {
    .avg  = HMC5883L_AVG_8,
    .rate = HMC5883L_RATE_75HZ,
    .gain = HMC5883L_GAIN_1090,
    .mode = HMC5883L_MODE_CONTINUOUS,
};

/* ============================================================================
 *  内部辅助 — Bypass 使能
 * ============================================================================ */

/**
 * @brief 使能 MPU6050 的 I2C Bypass 模式（完整三步序列）
 *
 * MPU6050 的 AUX_DA/AUX_CL 引脚连接到 HMC5883L 的 SDA/SCL。
 * 当 BYPASS_EN=1 时，MPU6050 将 AUX I2C 直接连通到主 I2C 总线，
 * 使 MSPM0 可以通过同一总线访问 HMC5883L（地址 0x1E）。
 *
 * 时序要求（GY-87 标准流程）：
 *   1. 关闭 AUX I2C Master 模式（USER_CTRL bit5=0），释放总线控制权
 *      → 若 I2C_MST_EN 已置位，bypass 位被硬件忽略！
 *   2. 使能 Bypass（INT_PIN_CFG bit1=1）
 *   3. 等待 10ms 让 bypass 电路稳定
 *
 * 每步读→改→写，不破坏其他位。
 */
static bool enable_mpu6050_bypass(void)
{
    uint8_t val;

    /* 步骤 1：关闭 AUX I2C Master 模式（释放 AUX 总线控制权） */
    if (!HAL_HMC5883L_ReadReg(MPU6050_ADDR, MPU6050_REG_USER_CTRL, &val)) {
        return false;
    }
    val &= ~MPU6050_BIT_I2C_MST_EN;  /* 清零 bit 5 (I2C_MST_EN) */
    val |=  MPU6050_BIT_I2C_MST_RST; /* 置位 bit 1 (复位 AUX I2C 状态机) */
    if (!HAL_HMC5883L_WriteReg(MPU6050_ADDR, MPU6050_REG_USER_CTRL, val)) {
        return false;
    }

    /* 步骤 2：使能 Bypass 模式 */
    if (!HAL_HMC5883L_ReadReg(MPU6050_ADDR, MPU6050_REG_INT_PIN_CFG, &val)) {
        return false;
    }
    val |= MPU6050_BIT_BYPASS_EN;
    if (!HAL_HMC5883L_WriteReg(MPU6050_ADDR, MPU6050_REG_INT_PIN_CFG, val)) {
        return false;
    }

    delay_ms(10);  /* 等待 bypass 电路稳定 */
    return true;
}

/* ============================================================================
 *  初始化
 * ============================================================================ */

bool MID_HMC5883L_Init(const HMC5883L_Config_t *cfg)
{
    if (cfg) {
        s_config = *cfg;
    }

    /* 1. HAL 初始化（仅 PA13 DRDY 引脚，不重初始化 I2C1） */
    HAL_HMC5883L_Init();

    /* 2. 使能 MPU6050 I2C Bypass（关键——HMC5883L 在 MPU6050 AUX I2C 后面） */
    if (!enable_mpu6050_bypass()) {
        return false;
    }

    /* 3. 探测 HMC5883L */
    if (!MID_HMC5883L_Probe()) {
        return false;
    }

    /* 4. 配置 CRA：采样平均 + 数据率
     *    bit 7-6：0（MS1/MS0 偏置，不使用）
     *    bit 5-6：avg[5:6] → avg << 5
     *    bit 2-4：rate[2:4] → rate << 2
     *    bit 0-1：0（DO2/DO1 偏置，不使用）
     */
    uint8_t cra = ((uint8_t)s_config.avg << 5) | ((uint8_t)s_config.rate << 2);
    if (!HAL_HMC5883L_WriteReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_CRA, cra)) {
        return false;
    }

    /* 5. 配置 CRB：增益
     *    bit 5-7：gain → gain << 5
     */
    uint8_t crb = (uint8_t)s_config.gain << 5;
    if (!HAL_HMC5883L_WriteReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_CRB, crb)) {
        return false;
    }

    /* 6. 配置 MR：连续测量模式
     *    bit 0-1：HS0/HS1 模式
     */
    if (!HAL_HMC5883L_WriteReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_MR,
                               (uint8_t)s_config.mode)) {
        return false;
    }

    return true;
}

/* ============================================================================
 *  探测
 * ============================================================================ */

bool MID_HMC5883L_Probe(void)
{
    uint8_t id_a, id_b, id_c;

    if (!HAL_HMC5883L_ReadReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_IRA, &id_a)) return false;
    if (!HAL_HMC5883L_ReadReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_IRB, &id_b)) return false;
    if (!HAL_HMC5883L_ReadReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_IRC, &id_c)) return false;

    /* HMC5883L ID: 'H'=0x48, '4'=0x34, '3'=0x33 */
    return (id_a == 0x48 && id_b == 0x34 && id_c == 0x33);
}

/* ============================================================================
 *  原始数据读取
 * ============================================================================ */

bool MID_HMC5883L_ReadRaw(HMC5883L_RawData_t *raw)
{
    if (!raw) return false;

    uint8_t buf[6];

    /* 从 DXRA (0x03) 开始连续读 6 字节 */
    if (!HAL_HMC5883L_ReadMulti(HMC5883L_ADDR_7BIT,
                                HMC5883L_REG_DXRA, buf, 6)) {
        return false;
    }

    /* 寄存器顺序：X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB（注意 Z 在 Y 前） */
    raw->x = (int16_t)((buf[0] << 8) | buf[1]);
    raw->z = (int16_t)((buf[2] << 8) | buf[3]);
    raw->y = (int16_t)((buf[4] << 8) | buf[5]);

    return true;
}

/* ============================================================================
 *  数据就绪检查
 * ============================================================================ */

bool MID_HMC5883L_IsDataReady(void)
{
    return HAL_HMC5883L_IsDataReady();
}

/* ============================================================================
 *  量纲转换（raw → μT）
 * ============================================================================ */

void MID_HMC5883L_RawToMicroTesla(const HMC5883L_RawData_t *raw,
                                  const HMC5883L_Calib_t *calib,
                                  float mag[3])
{
    if (!raw || !mag) return;

    uint16_t lsb_per_g = HMC5883L_GainLSB(s_config.gain);

    /* raw → Gauss → μT（×100），同时减去硬铁偏移、乘软铁缩放 */
    float raw_f[3] = {
        (float)raw->x,
        (float)raw->y,   /* HMC5883L_RawData_t 存储逻辑顺序 X/Y/Z */
        (float)raw->z,
    };

    for (int i = 0; i < 3; i++) {
        float adjusted = raw_f[i];
        if (calib && calib->valid) {
            adjusted -= (float)calib->offset[i];
            adjusted *= calib->scale[i];
        }
        mag[i] = adjusted * (100.0f / (float)lsb_per_g);
    }
}

/* ============================================================================
 *  配置查询与修改
 * ============================================================================ */

const HMC5883L_Config_t *MID_HMC5883L_GetConfig(void)
{
    return &s_config;
}

bool MID_HMC5883L_SetGain(HMC5883L_Gain_t gain)
{
    s_config.gain = gain;
    uint8_t crb = (uint8_t)gain << 5;
    return HAL_HMC5883L_WriteReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_CRB, crb);
}

bool MID_HMC5883L_SetRate(HMC5883L_Rate_t rate)
{
    s_config.rate = rate;
    /* 读→改→写 CRA，保留其他位 */
    uint8_t cra;
    if (!HAL_HMC5883L_ReadReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_CRA, &cra)) {
        return false;
    }
    cra &= ~0x1C;                          /* 清 bit 2-4 */
    cra |= (uint8_t)rate << 2;             /* 设新数据率 */
    return HAL_HMC5883L_WriteReg(HMC5883L_ADDR_7BIT, HMC5883L_REG_CRA, cra);
}
