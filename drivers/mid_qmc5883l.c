/**
 * @file    mid_qmc5883l.c
 * @brief   中间层实现 — QMC5883L 寄存器协议
 *
 * 与 HMC5883L 共享 I2C1 总线（PA16=SDA, PA17=SCL, 400kHz），
 * 通过 MPU6050 AUX I2C Bypass 访问。
 *
 * I2C 读写函数复用 hal_hmc5883l（地址作为参数传入，通用）。
 * 初始化时需确保 MPU6050 bypass 已使能。
 */

#include "mid_qmc5883l.h"
#include "hal_hmc5883l.h"
#include "delay.h"
#include <string.h>

/* ============================================================================
 *  内部常量
 * ============================================================================ */

/** MPU6050 7-bit I2C 地址 */
#define MPU6050_ADDR             0x68

/** MPU6050 寄存器 */
#define MPU6050_REG_INT_PIN_CFG  0x37
#define MPU6050_REG_USER_CTRL    0x6A

/** 位定义 */
#define MPU6050_BIT_BYPASS_EN    (1 << 1)   /**< INT_PIN_CFG bit 1 */
#define MPU6050_BIT_I2C_MST_EN   (1 << 5)   /**< USER_CTRL bit 5 */
#define MPU6050_BIT_I2C_MST_RST  (1 << 1)   /**< USER_CTRL bit 1 */

/* ============================================================================
 *  默认配置
 * ============================================================================ */

static QMC5883L_Config_t s_config = {
    .osr   = QMC5883L_OSR_512,
    .range = QMC5883L_RANGE_8G,
    .odr   = QMC5883L_ODR_200HZ,
    .mode  = QMC5883L_MODE_CONTINUOUS,
};

/* ============================================================================
 *  内部辅助 — MPU6050 Bypass 使能
 * ============================================================================ */

/**
 * @brief 使能 MPU6050 的 I2C Bypass 模式（完整三步序列）
 *
 * 步骤：
 *   1. 确保 MPU6050 已唤醒（PWR_MGMT_1 = 0x00）
 *   2. 关闭 AUX I2C Master 模式（USER_CTRL bit5=0）
 *   3. 使能 I2C Bypass（INT_PIN_CFG bit1=1）
 *
 * 安全做法：每步读→改→写，不破坏其他位。
 */
static bool enable_mpu6050_bypass(void)
{
    uint8_t val;

    /* 步骤 1：确保 AUX I2C Master 关闭（释放 AUX 总线控制权） */
    if (!HAL_HMC5883L_ReadReg(MPU6050_ADDR, MPU6050_REG_USER_CTRL, &val)) {
        return false;
    }
    val &= ~MPU6050_BIT_I2C_MST_EN;   /* 清零 bit 5 (I2C_MST_EN) */
    val |=  MPU6050_BIT_I2C_MST_RST;  /* 置位 bit 1 (复位 AUX I2C 状态机) */
    if (!HAL_HMC5883L_WriteReg(MPU6050_ADDR, MPU6050_REG_USER_CTRL, val)) {
        return false;
    }

    /* 步骤 2：使能 Bypass 模式（AUX 直通主总线） */
    if (!HAL_HMC5883L_ReadReg(MPU6050_ADDR, MPU6050_REG_INT_PIN_CFG, &val)) {
        return false;
    }
    val |= MPU6050_BIT_BYPASS_EN;     /* 置位 bit 1 */
    if (!HAL_HMC5883L_WriteReg(MPU6050_ADDR, MPU6050_REG_INT_PIN_CFG, val)) {
        return false;
    }

    delay_ms(10);  /* 等待 bypass 电路稳定 */
    return true;
}

/* ============================================================================
 *  初始化
 * ============================================================================ */

bool MID_QMC5883L_Init(const QMC5883L_Config_t *cfg)
{
    if (cfg) {
        s_config = *cfg;
    }

    /* 1. 使能 MPU6050 I2C Bypass（QMC5883L 在 AUX I2C 后面） */
    if (!enable_mpu6050_bypass()) {
        return false;
    }

    /* 2. 探测 QMC5883L */
    if (!MID_QMC5883L_Probe()) {
        return false;
    }

    /* 3. 软复位（CTRL2 bit7=1）——确保寄存器回到默认值 */
    HAL_HMC5883L_WriteReg(QMC5883L_ADDR_7BIT, QMC5883L_REG_CTRL2,
                          QMC5883L_CTRL2_SOFT_RST);
    delay_ms(10);

    /* 4. 配置 SET/RESET 周期寄存器（手册强制要求！漏写数据异常） */
    if (!HAL_HMC5883L_WriteReg(QMC5883L_ADDR_7BIT,
                                QMC5883L_REG_SET_RESET, 0x01)) {
        return false;
    }

    /* 5. 配置 CTRL1：OSR + RNG + ODR + MODE 组合 */
    uint8_t ctrl1 = ((uint8_t)s_config.osr   << QMC5883L_CTRL1_OSR_SHIFT)
                  | ((uint8_t)s_config.range << QMC5883L_CTRL1_RNG_SHIFT)
                  | ((uint8_t)s_config.odr   << QMC5883L_CTRL1_ODR_SHIFT)
                  | ((uint8_t)s_config.mode  << QMC5883L_CTRL1_MODE_SHIFT);

    if (!HAL_HMC5883L_WriteReg(QMC5883L_ADDR_7BIT,
                                QMC5883L_REG_CTRL1, ctrl1)) {
        return false;
    }

    return true;
}

/* ============================================================================
 *  探测
 * ============================================================================ */

bool MID_QMC5883L_Probe(void)
{
    uint8_t id;

    if (!HAL_HMC5883L_ReadReg(QMC5883L_ADDR_7BIT,
                               QMC5883L_REG_CHIP_ID, &id)) {
        return false;
    }

    /* QMC5883L 芯片 ID = 0xFF */
    return (id == 0xFF);
}

/* ============================================================================
 *  原始数据读取
 * ============================================================================ */

bool MID_QMC5883L_ReadRaw(QMC5883L_RawData_t *raw)
{
    if (!raw) return false;

    uint8_t buf[6];

    /* 从 DATA_X_L (0x00) 开始连续读 6 字节 */
    if (!HAL_HMC5883L_ReadMulti(QMC5883L_ADDR_7BIT,
                                QMC5883L_REG_DATA_X_L, buf, 6)) {
        return false;
    }

    /* 寄存器顺序：X_L, X_H, Y_L, Y_H, Z_L, Z_H（LSB 在前，与 HMC 全反！） */
    raw->x = (int16_t)((uint16_t)(buf[1]) << 8 | buf[0]);
    raw->y = (int16_t)((uint16_t)(buf[3]) << 8 | buf[2]);
    raw->z = (int16_t)((uint16_t)(buf[5]) << 8 | buf[4]);

    return true;
}

/* ============================================================================
 *  数据就绪检查
 * ============================================================================ */

bool MID_QMC5883L_IsDataReady(void)
{
    uint8_t status;

    if (!HAL_HMC5883L_ReadReg(QMC5883L_ADDR_7BIT,
                               QMC5883L_REG_STATUS, &status)) {
        return false;
    }

    /* bit 0 = DRDY, bit 2 = DOR (data overflow/overrun) */
    return (status & 0x01) && !(status & 0x04);
}

/* ============================================================================
 *  量纲转换（raw → μT）
 * ============================================================================ */

void MID_QMC5883L_RawToMicroTesla(const QMC5883L_RawData_t *raw, float mag[3])
{
    if (!raw || !mag) return;

    float scale;
    switch (s_config.range) {
        case QMC5883L_RANGE_2G: scale = QMC5883L_SCALE_2G; break;
        case QMC5883L_RANGE_8G: scale = QMC5883L_SCALE_8G; break;
        default:                 scale = QMC5883L_SCALE_8G; break;
    }

    /* raw → Gauss → μT（×100） */
    mag[0] = (float)raw->x * (100.0f / scale);
    mag[1] = (float)raw->y * (100.0f / scale);
    mag[2] = (float)raw->z * (100.0f / scale);
}

/* ============================================================================
 *  配置查询
 * ============================================================================ */

const QMC5883L_Config_t *MID_QMC5883L_GetConfig(void)
{
    return &s_config;
}
