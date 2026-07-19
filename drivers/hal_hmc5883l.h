/**
 * @file    hal_hmc5883l.h
 * @brief   硬件抽象层接口 — I2C1 通信 + DRDY 引脚
 *
 * HMC5883L 与 MPU6050 共享 I2C1 总线（PA16=SDA, PA17=SCL, 400kHz Fast Mode）。
 * 本层不重新初始化 I2C1——时钟速率、时钟延展、毛刺滤波均由 HAL_MPU6050_Init 设置。
 * 本层仅配置 PA13（DRDY 引脚）并提供参数化的 I2C 读写接口。
 */

#ifndef HAL_HMC5883L_H
#define HAL_HMC5883L_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* ============================================================================
 *  常量
 * ============================================================================ */

/** HMC5883L 7-bit I2C 地址 */
#define HMC5883L_ADDR_7BIT       0x1E

/** I2C 外设实例（与 MPU6050 共享 I2C1） */
#define HMC_I2C_INST             I2C1

/** I2C 等待超时（循环次数） */
#define HMC_I2C_TIMEOUT          200000UL

/** DRDY 引脚（PA13）：磁力计数据就绪，高电平有效 */
#define HMC_DRDY_PORT            GPIOA
#define HMC_DRDY_PIN             DL_GPIO_PIN_13

/* ============================================================================
 *  API
 * ============================================================================ */

/**
 * @brief  初始化 HMC5883L 硬件接口
 * @note   仅配置 PA13 DRDY 引脚（输入 + 上拉），不重新初始化 I2C1。
 *         需在 HAL_MPU6050_Init() 之后调用。
 */
void HAL_HMC5883L_Init(void);

/**
 * @brief  向 HMC5883L 寄存器写入 1 字节
 * @param  addr  7-bit I2C 地址（通常 HMC5883L_ADDR_7BIT）
 * @param  reg   寄存器地址
 * @param  data  待写入数据
 * @return true 写入成功，false 超时或 I2C 错误
 */
bool HAL_HMC5883L_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief  从 HMC5883L 寄存器读取 1 字节
 * @param  addr  7-bit I2C 地址
 * @param  reg   寄存器地址
 * @param  data  输出缓冲区
 * @return true 读取成功
 */
bool HAL_HMC5883L_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data);

/**
 * @brief  从 HMC5883L 连续读取多个字节（自动递增）
 * @param  addr  7-bit I2C 地址
 * @param  reg   起始寄存器地址
 * @param  buf   输出缓冲区
 * @param  len   读取字节数
 * @return true 读取成功
 */
bool HAL_HMC5883L_ReadMulti(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  检查磁力计数据是否就绪（轮询 DRDY 引脚）
 * @return true 数据就绪（DRDY=高）
 */
bool HAL_HMC5883L_IsDataReady(void);

#endif /* HAL_HMC5883L_H */
