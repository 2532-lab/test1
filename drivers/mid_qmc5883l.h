/**
 * @file    mid_qmc5883l.h
 * @brief   中间层接口 — QMC5883L 寄存器协议、配置、原始数据
 *
 * QMC5883L 是 HMC5883L 的国产替代，I2C 地址 0x0D（非 0x1E）。
 * 数据手册关键差异：
 *   - 寄存器映射完全不同（CTRL1/2 代替 CRA/CRB/MR）
 *   - 数据顺序 X→Y→Z（非 X→Z→Y），LSB 在前（非 MSB 在前）
 *   - 必须初始化 SET/RESET 周期寄存器（0x0B=0x01），漏写数据异常
 *   - 量程/灵敏度参数不同
 *
 * I2C 通信复用 hal_hmc5883l 的 I2C1 函数（地址作为参数传入，可通用）。
 */

#ifndef MID_QMC5883L_H
#define MID_QMC5883L_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *  I2C 地址
 * ============================================================================ */

/** QMC5883L 7-bit I2C 地址 */
#define QMC5883L_ADDR_7BIT       0x0D

/* ============================================================================
 *  寄存器地址
 * ============================================================================ */

#define QMC5883L_REG_DATA_X_L    0x00  /**< X 轴数据低字节 */
#define QMC5883L_REG_DATA_X_H    0x01  /**< X 轴数据高字节 */
#define QMC5883L_REG_DATA_Y_L    0x02  /**< Y 轴数据低字节 */
#define QMC5883L_REG_DATA_Y_H    0x03  /**< Y 轴数据高字节 */
#define QMC5883L_REG_DATA_Z_L    0x04  /**< Z 轴数据低字节 */
#define QMC5883L_REG_DATA_Z_H    0x05  /**< Z 轴数据高字节 */
#define QMC5883L_REG_STATUS      0x06  /**< 状态寄存器（bit0=DRDY, bit1=OVL, bit2=DOR） */
#define QMC5883L_REG_TEMP_L      0x07  /**< 温度低字节 */
#define QMC5883L_REG_TEMP_H      0x08  /**< 温度高字节 */
#define QMC5883L_REG_CTRL1       0x09  /**< 控制寄存器1：OSR/RNG/ODR/MODE */
#define QMC5883L_REG_CTRL2       0x0A  /**< 控制寄存器2：软复位/INT */
#define QMC5883L_REG_SET_RESET   0x0B  /**< SET/RESET 周期（必须写 0x01！） */
#define QMC5883L_REG_CHIP_ID     0x0D  /**< 芯片 ID（应返回 0xFF） */

/* ============================================================================
 *  CTRL1 位域
 * ============================================================================ */

#define QMC5883L_CTRL1_OSR_SHIFT   6    /**< 过采样率位偏移 */
#define QMC5883L_CTRL1_RNG_SHIFT   4    /**< 量程位偏移 */
#define QMC5883L_CTRL1_ODR_SHIFT   2    /**< 数据率位偏移 */
#define QMC5883L_CTRL1_MODE_SHIFT  0    /**< 模式位偏移 */

/* ============================================================================
 *  CTRL2 位域
 * ============================================================================ */

#define QMC5883L_CTRL2_SOFT_RST   (1 << 7)  /**< 软复位 */
#define QMC5883L_CTRL2_ROL_PTR    (1 << 6)  /**< 指针自动回卷 */
#define QMC5883L_CTRL2_INT_ENB    (1 << 0)  /**< 中断使能 */

/* ============================================================================
 *  枚举类型
 * ============================================================================ */

/** 过采样率 */
typedef enum {
    QMC5883L_OSR_512 = 0x00,    /**< 512 次过采样（最高精度，最慢） */
    QMC5883L_OSR_256 = 0x01,    /**< 256 次 */
    QMC5883L_OSR_128 = 0x02,    /**< 128 次 */
    QMC5883L_OSR_64  = 0x03,    /**< 64 次（最快） */
} QMC5883L_Osr_t;

/** 满量程范围 */
typedef enum {
    QMC5883L_RANGE_2G = 0x00,   /**< ±2 Gauss, 12000 LSB/Gauss */
    QMC5883L_RANGE_8G = 0x01,   /**< ±8 Gauss, 3000 LSB/Gauss */
} QMC5883L_Range_t;

/** 数据输出率 */
typedef enum {
    QMC5883L_ODR_10HZ  = 0x00,  /**< 10 Hz */
    QMC5883L_ODR_50HZ  = 0x01,  /**< 50 Hz */
    QMC5883L_ODR_100HZ = 0x02,  /**< 100 Hz */
    QMC5883L_ODR_200HZ = 0x03,  /**< 200 Hz */
} QMC5883L_Odr_t;

/** 工作模式 */
typedef enum {
    QMC5883L_MODE_STANDBY    = 0x00,  /**< 待机 */
    QMC5883L_MODE_CONTINUOUS = 0x01,  /**< 连续测量 */
} QMC5883L_Mode_t;

/* ============================================================================
 *  灵敏度常量（LSB/Gauss）
 * ============================================================================ */

/** ±2G 量程灵敏度 */
#define QMC5883L_SCALE_2G          (12000.0f)
/** ±8G 量程灵敏度 */
#define QMC5883L_SCALE_8G          (3000.0f)

/* ============================================================================
 *  数据结构
 * ============================================================================ */

/** 原始磁力计数据（顺序 X→Y→Z） */
typedef struct {
    int16_t x;  /**< X 轴原始值 */
    int16_t y;  /**< Y 轴原始值 */
    int16_t z;  /**< Z 轴原始值 */
} QMC5883L_RawData_t;

/** 配置参数 */
typedef struct {
    QMC5883L_Osr_t   osr;     /**< 过采样率（默认 512） */
    QMC5883L_Range_t range;   /**< 量程（默认 ±8G） */
    QMC5883L_Odr_t   odr;     /**< 数据率（默认 200 Hz） */
    QMC5883L_Mode_t  mode;    /**< 工作模式（默认连续） */
} QMC5883L_Config_t;

/* ============================================================================
 *  API
 * ============================================================================ */

/**
 * @brief  初始化 QMC5883L
 * @param  cfg  配置参数（NULL 使用默认：OSR512, ±8G, 200Hz, 连续模式）
 * @return true 初始化成功
 *
 * 内部流程：
 *   1. 使能 MPU6050 I2C Bypass（读写 INT_PIN_CFG + USER_CTRL）
 *   2. delay_ms(10) 等待 bypass 稳定
 *   3. Probe：读 CHIP_ID (0x0D)，预期 0xFF
 *   4. 写 SET/RESET 周期寄存器（0x0B=0x01，手册强制要求）
 *   5. 配置 CTRL1（OSR + RNG + ODR + MODE）
 *
 * @note  需在 MID_MPU6050_Init() 之后调用（I2C1 已配置为 400kHz）
 */
bool MID_QMC5883L_Init(const QMC5883L_Config_t *cfg);

/**
 * @brief  探测 QMC5883L 是否在线
 * @return true CHIP_ID = 0xFF
 */
bool MID_QMC5883L_Probe(void);

/**
 * @brief  读取原始磁力计数据（6 字节：XL/XH/YL/YH/ZL/ZH）
 * @param  raw  输出缓冲区
 * @return true 读取成功
 *
 * 数据字节序：LSB 在前（与 HMC5883L 相反！）
 */
bool MID_QMC5883L_ReadRaw(QMC5883L_RawData_t *raw);

/**
 * @brief  将原始值转换为 μT（未校准）
 * @param  raw  原始数据
 * @param  mag  输出 μT 数组 [3]（顺序 X/Y/Z）
 *
 * 换算公式：mag_uT = raw * (100.0 / scale_lsb_per_gauss)
 *   1 Gauss = 100 μT
 */
void MID_QMC5883L_RawToMicroTesla(const QMC5883L_RawData_t *raw, float mag[3]);

/**
 * @brief  获取当前配置
 * @return 配置指针（内部静态变量）
 */
const QMC5883L_Config_t *MID_QMC5883L_GetConfig(void);

/**
 * @brief  检查数据是否就绪（读 STATUS 寄存器 bit0）
 * @return true DRDY=1，数据已就绪
 */
bool MID_QMC5883L_IsDataReady(void);

#endif /* MID_QMC5883L_H */
