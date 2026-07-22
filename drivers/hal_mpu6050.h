/**
 * @file    hal_mpu6050.h
 * @brief   硬件抽象层 — MPU6050 I2C 通信与中断引脚
 *
 * 本层是三层架构的最底层，仅负责 MSPM0G3507 与 MPU6050 之间的
 * 物理层通信抽象，包含：
 *   - I2C1 控制器初始化与收发
 *   - INT 引脚 (PB24) 中断配置
 *   - 不包含任何 MPU6050 协议相关的逻辑
 *
 * 引脚映射（MPU6050 8-pin 排针 → MSPM0G3507）：
 *   引脚 1：+3.3V  → 3.3V 电源
 *   引脚 2：GND    → 电源地
 *   引脚 3：A17    → I2C1_SCL
 *   引脚 4：A16    → I2C1_SDA
 *   引脚 5-7：NC   → 悬空
 *   引脚 8：B24    → GPIO 输入（INT 中断引脚）
 *
 * ==================== SysConfig 配置指南 ====================
 * 使用前，请在 .syscfg 文件中添加以下外设实例：
 *
 *   == I2C1 ==
 *   添加 I2C 实例，命名 "I2C_1"：
 *     - SCL pin = PA17 (IOMUX_PINCM37, 功能 I2C1_SCL)
 *     - SDA pin = PA16 (IOMUX_PINCM38, 功能 I2C1_SDA)
 *
 * SysConfig 将自动生成 IOMUX 初始化代码及宏定义：
 *   I2C_1_INST, GPIO_I2C_1_SDA_PORT / GPIO_I2C_1_SDA_PIN
 *   GPIO_I2C_1_SCL_PORT / GPIO_I2C_1_SCL_PIN
 *
 * 注意：
 *   INT 引脚 (PB24) 由 SysConfig (GPIO_MPU_INT) 管理，
 *   中断极性和 NVIC 使能仍由 HAL 层手动控制。
 * ==================================================================
 */

#ifndef HAL_MPU6050_H
#define HAL_MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* ============================================================================
 *  I2C 接口定义
 * ============================================================================ */

/** I2C 外设实例（需在 SysConfig 中添加 I2C1） */
#define MPU_I2C_INST            I2C_1_INST

/** MPU6050 I2C 器件地址（7 位，AD0=0 时默认地址 0x68） */
#define MPU6050_ADDR_7BIT       0x68

/* ============================================================================
 *  INT 中断引脚定义
 * ============================================================================ */

/** INT 引脚 GPIO 端口 */
#define MPU_INT_PORT            GPIO_MPU_INT_PIN_INT_PORT
/** INT 引脚号 */
#define MPU_INT_PIN             GPIO_MPU_INT_PIN_INT_PIN
/** INT 引脚 IOMUX 编号（MSPM0G3507 60-pin：PB24 = IOMUX_PINCM52） */
#define MPU_INT_IOMUX           GPIO_MPU_INT_PIN_INT_IOMUX

/* ============================================================================
 *  I2C 超时参数
 * ============================================================================ */

/** 单次 I2C 操作最大等待循环次数 */
#define MPU_I2C_TIMEOUT         200000UL

/* ============================================================================
 *  中断回调类型
 * ============================================================================ */

/** MPU6050 数据就绪中断回调函数指针 */
typedef void (*HAL_MPU_IntCallback_t)(void);

/* ============================================================================
 *  硬件层函数声明
 * ============================================================================ */

/**
 * @brief   初始化硬件层
 *
 * 完成以下工作：
 *   1. 使能 I2C1 控制器，设定通信速率（约 400kHz）
 *   2. 使能 I2C 时钟延展（clock stretching）
 *   3. 使能数字毛刺滤波器（125 ns @ 32 MHz BUSCLK）
 *   4. 配置 PB24 为 GPIO 数字输入（INT 引脚）
 *
 * SysConfig 已处理的 I2C 配置（本函数不重复）：
 *   - I2C 时钟源选择（BUSCLK）
 *   - 模拟毛刺滤波器（50 ns 脉宽）
 *   - SDA/SCL 引脚 IOMUX 复用
 *
 * @note  必须在 SYSCFG_DL_init() 之后调用
 */
void HAL_MPU6050_Init(void);

/**
 * @brief   写单个寄存器
 * @param   addr  7 位 I2C 器件地址
 * @param   reg   寄存器地址
 * @param   data  要写入的数据
 * @return  true = 成功, false = 超时或通信错误
 */
bool HAL_MPU6050_WriteReg(uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief   读单个寄存器
 * @param   addr  7 位 I2C 器件地址
 * @param   reg   寄存器地址
 * @param   data  存放读出数据的指针
 * @return  true = 成功, false = 超时或通信错误
 */
bool HAL_MPU6050_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data);

/**
 * @brief   连续读取多个寄存器
 * @param   addr  7 位 I2C 器件地址
 * @param   reg   起始寄存器地址
 * @param   buf   读出数据缓冲区指针
 * @param   len   读取字节数
 * @return  true = 成功, false = 超时或通信错误
 */
bool HAL_MPU6050_ReadMulti(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief   注册 INT 引脚上升沿中断回调
 * @param   cb  回调函数指针（传 NULL 取消注册）
 *
 * 注册后，每次 MPU6050 数据就绪（INT 引脚上升沿）时自动调用 cb。
 * 需要全局提供一个 GPIOB_IRQHandler，本层使用弱符号提供默认实现。
 */
void HAL_MPU6050_SetIntCallback(HAL_MPU_IntCallback_t cb);

#endif /* HAL_MPU6050_H */
