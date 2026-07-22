/**
 * @file    mid_mpu6050.h
 * @brief   中间层 — MPU6050 寄存器协议与原始数据读取
 *
 * 本层是三层架构的中间层，封装了 MPU6050 的寄存器映射和通信协议：
 *   - 器件初始化（唤醒、时钟源、量程、DLPF）
 *   - 原始加速度/角速度/温度数据读取
 *   - 自检、量程/滤波器配置
 *
 * 本层不包含姿态解算或滤波算法（由应用层 app_imu 提供）。
 * 本层不直接操作硬件（由硬件层 hal_mpu6050 提供 I2C 通信）。
 */

#ifndef MID_MPU6050_H
#define MID_MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_mpu6050.h"

/* ============================================================================
 *  MPU6050 寄存器映射
 * ============================================================================ */

/* ---- 配置寄存器 ---- */
#define MPU6050_REG_SMPLRT_DIV     0x19    /**< 采样率分频器 */
#define MPU6050_REG_CONFIG         0x1A    /**< 配置（DLPF 带宽） */
#define MPU6050_REG_GYRO_CONFIG    0x1B    /**< 陀螺仪量程配置 */
#define MPU6050_REG_ACCEL_CONFIG   0x1C    /**< 加速度计量程配置 */

/* ---- 中断寄存器 ---- */
#define MPU6050_REG_INT_PIN_CFG    0x37    /**< 中断引脚配置 */
#define MPU6050_REG_INT_ENABLE     0x38    /**< 中断使能 */
#define MPU6050_REG_INT_STATUS     0x3A    /**< 中断状态 */

/* ---- 数据输出寄存器 ---- */
#define MPU6050_REG_ACCEL_XOUT_H   0x3B    /**< 加速度 X 轴高字节 */
#define MPU6050_REG_ACCEL_XOUT_L   0x3C    /**< 加速度 X 轴低字节 */
#define MPU6050_REG_ACCEL_YOUT_H   0x3D    /**< 加速度 Y 轴高字节 */
#define MPU6050_REG_ACCEL_YOUT_L   0x3E    /**< 加速度 Y 轴低字节 */
#define MPU6050_REG_ACCEL_ZOUT_H   0x3F    /**< 加速度 Z 轴高字节 */
#define MPU6050_REG_ACCEL_ZOUT_L   0x40    /**< 加速度 Z 轴低字节 */
#define MPU6050_REG_TEMP_OUT_H     0x41    /**< 温度高字节 */
#define MPU6050_REG_TEMP_OUT_L     0x42    /**< 温度低字节 */
#define MPU6050_REG_GYRO_XOUT_H    0x43    /**< 角速度 X 轴高字节 */
#define MPU6050_REG_GYRO_XOUT_L    0x44    /**< 角速度 X 轴低字节 */
#define MPU6050_REG_GYRO_YOUT_H    0x45    /**< 角速度 Y 轴高字节 */
#define MPU6050_REG_GYRO_YOUT_L    0x46    /**< 角速度 Y 轴低字节 */
#define MPU6050_REG_GYRO_ZOUT_H    0x47    /**< 角速度 Z 轴高字节 */
#define MPU6050_REG_GYRO_ZOUT_L    0x48    /**< 角速度 Z 轴低字节 */

/* ---- 电源管理寄存器 ---- */
#define MPU6050_REG_PWR_MGMT_1     0x6B    /**< 电源管理 1 */
#define MPU6050_REG_PWR_MGMT_2     0x6C    /**< 电源管理 2 */

/* ---- 识别寄存器 ---- */
#define MPU6050_REG_WHO_AM_I       0x75    /**< 器件 ID（应返回 0x68 或 0x70） */

/* ---- 自检寄存器 ---- */
#define MPU6050_REG_SELF_TEST_X    0x0D    /**< X 轴自检 */
#define MPU6050_REG_SELF_TEST_Y    0x0E    /**< Y 轴自检 */
#define MPU6050_REG_SELF_TEST_Z    0x0F    /**< Z 轴自检 */
#define MPU6050_REG_SELF_TEST_A    0x10    /**< 加速度自检 */

/* ---- 信号通路 ---- */
#define MPU6050_REG_SIGNAL_PATH_RESET  0x68 /**< 信号通路复位 */
#define MPU6050_REG_USER_CTRL          0x6A /**< 用户控制 */


/* ============================================================================
 *  位域定义
 * ============================================================================ */

/* PWR_MGMT_1 */
#define MPU6050_PWR1_DEVICE_RESET   (1 << 7)    /**< 器件复位 */
#define MPU6050_PWR1_SLEEP          (1 << 6)    /**< 睡眠模式 */
#define MPU6050_PWR1_CYCLE          (1 << 5)    /**< 循环模式 */
#define MPU6050_PWR1_TEMP_DIS       (1 << 3)    /**< 温度传感器禁用 */
#define MPU6050_PWR1_CLKSEL_MASK    0x07        /**< 时钟源掩码 */
#define MPU6050_PWR1_CLKSEL_INTERNAL   0x00     /**< 内部 8MHz 振荡器 */
#define MPU6050_PWR1_CLKSEL_XGYRO      0x01     /**< X 轴陀螺仪 PLL */
#define MPU6050_PWR1_CLKSEL_YGYRO      0x02     /**< Y 轴陀螺仪 PLL */
#define MPU6050_PWR1_CLKSEL_ZGYRO      0x03     /**< Z 轴陀螺仪 PLL */

/* GYRO_CONFIG */
#define MPU6050_GYRO_CFG_FS_SEL_MASK  0x18      /**< 满量程选择掩码 */
#define MPU6050_GYRO_CFG_FS_SEL_250   0x00      /**< ±250 °/s */
#define MPU6050_GYRO_CFG_FS_SEL_500   0x08      /**< ±500 °/s */
#define MPU6050_GYRO_CFG_FS_SEL_1000  0x10      /**< ±1000 °/s */
#define MPU6050_GYRO_CFG_FS_SEL_2000  0x18      /**< ±2000 °/s */
#define MPU6050_GYRO_CFG_ZGYRO_SELF_TEST (1 << 3)
#define MPU6050_GYRO_CFG_YGYRO_SELF_TEST (1 << 2)
#define MPU6050_GYRO_CFG_XGYRO_SELF_TEST (1 << 1)

/* ACCEL_CONFIG */
#define MPU6050_ACCEL_CFG_FS_SEL_MASK  0x18     /**< 满量程选择掩码 */
#define MPU6050_ACCEL_CFG_FS_SEL_2G   0x00      /**< ±2g */
#define MPU6050_ACCEL_CFG_FS_SEL_4G   0x08      /**< ±4g */
#define MPU6050_ACCEL_CFG_FS_SEL_8G   0x10      /**< ±8g */
#define MPU6050_ACCEL_CFG_FS_SEL_16G  0x18      /**< ±16g */
#define MPU6050_ACCEL_CFG_AZ_SELF_TEST (1 << 3)
#define MPU6050_ACCEL_CFG_AY_SELF_TEST (1 << 2)
#define MPU6050_ACCEL_CFG_AX_SELF_TEST (1 << 1)

/* CONFIG (DLPF) */
#define MPU6050_CONFIG_DLPF_CFG_MASK  0x07      /**< DLPF 配置掩码 */
#define MPU6050_CONFIG_EXT_SYNC_MASK  0x38      /**< 外部同步掩码 */

/* INT_PIN_CFG */
#define MPU6050_INT_CFG_INT_LEVEL     (1 << 7)  /**< 中断电平：1=高有效 */
#define MPU6050_INT_CFG_INT_OPEN      (1 << 6)  /**< 中断输出类型：1=开漏 */
#define MPU6050_INT_CFG_LATCH_INT_EN  (1 << 5)  /**< 中断锁存使能 */
#define MPU6050_INT_CFG_INT_RD_CLEAR  (1 << 4)  /**< 读数据清除中断 */
#define MPU6050_INT_CFG_FSYNC_INT_LEVEL (1 << 3)
#define MPU6050_INT_CFG_FSYNC_INT_MODE  (1 << 2)

/* INT_ENABLE */
#define MPU6050_INT_EN_DATA_RDY      (1 << 0)   /**< 数据就绪中断 */

/* INT_STATUS */
#define MPU6050_INT_STS_DATA_RDY     (1 << 0)   /**< 数据就绪标志 */


/* ============================================================================
 *  枚举类型
 * ============================================================================ */

/** 陀螺仪满量程范围 */
typedef enum {
    MPU6050_GYRO_RANGE_250   = 0x00,   /**< ±250 °/s  */
    MPU6050_GYRO_RANGE_500   = 0x08,   /**< ±500 °/s  */
    MPU6050_GYRO_RANGE_1000  = 0x10,   /**< ±1000 °/s */
    MPU6050_GYRO_RANGE_2000  = 0x18    /**< ±2000 °/s */
} MPU6050_GyroRange_t;

/** 加速度计满量程范围 */
typedef enum {
    MPU6050_ACCEL_RANGE_2G   = 0x00,   /**< ±2g  */
    MPU6050_ACCEL_RANGE_4G   = 0x08,   /**< ±4g  */
    MPU6050_ACCEL_RANGE_8G   = 0x10,   /**< ±8g  */
    MPU6050_ACCEL_RANGE_16G  = 0x18    /**< ±16g */
} MPU6050_AccelRange_t;

/** DLPF 带宽配置 */
typedef enum {
    MPU6050_DLPF_260HZ  = 0x00,    /**< 260 Hz 加速度 / 256 Hz 陀螺仪 */
    MPU6050_DLPF_184HZ  = 0x01,    /**< 184 Hz / 188 Hz */
    MPU6050_DLPF_94HZ   = 0x02,    /**< 94 Hz / 98 Hz */
    MPU6050_DLPF_44HZ   = 0x03,    /**< 44 Hz / 42 Hz */
    MPU6050_DLPF_21HZ   = 0x04,    /**< 21 Hz / 20 Hz */
    MPU6050_DLPF_10HZ   = 0x05,    /**< 10 Hz / 10 Hz */
    MPU6050_DLPF_5HZ    = 0x06     /**< 5 Hz / 5 Hz */
} MPU6050_DLPFBandwidth_t;

/** 时钟源 */
typedef enum {
    MPU6050_CLK_INTERNAL = 0x00,   /**< 内部 8MHz 振荡器 */
    MPU6050_CLK_PLL_XGYRO = 0x01,  /**< PLL 参考 X 轴陀螺仪 */
    MPU6050_CLK_PLL_YGYRO = 0x02,  /**< PLL 参考 Y 轴陀螺仪 */
    MPU6050_CLK_PLL_ZGYRO = 0x03   /**< PLL 参考 Z 轴陀螺仪（推荐） */
} MPU6050_ClockSource_t;


/* ============================================================================
 *  灵敏度常量（LSB/物理单位）
 *
 *  加速度：2^15 / 满量程(g)
 *  陀螺仪：2^15 / 满量程(°/s)
 *
 *  accel_scale = FS_SEL 对应的 LSB/g 系数
 *  gyro_scale  = FS_SEL 对应的 LSB/(°/s) 系数
 * ============================================================================ */

/** 陀螺仪灵敏度：±250 °/s */
#define MPU6050_GYRO_SCALE_250      (131.0f)
/** 陀螺仪灵敏度：±500 °/s */
#define MPU6050_GYRO_SCALE_500      (65.5f)
/** 陀螺仪灵敏度：±1000 °/s */
#define MPU6050_GYRO_SCALE_1000     (32.8f)
/** 陀螺仪灵敏度：±2000 °/s */
#define MPU6050_GYRO_SCALE_2000     (16.4f)

/** 加速度灵敏度：±2g */
#define MPU6050_ACCEL_SCALE_2G      (16384.0f)
/** 加速度灵敏度：±4g */
#define MPU6050_ACCEL_SCALE_4G      (8192.0f)
/** 加速度灵敏度：±8g */
#define MPU6050_ACCEL_SCALE_8G      (4096.0f)
/** 加速度灵敏度：±16g */
#define MPU6050_ACCEL_SCALE_16G     (2048.0f)

/** 温度传感器：T(°C) = (TEMP_OUT / 340.0f) + 36.53f */
#define MPU6050_TEMP_SCALE          (340.0f)
#define MPU6050_TEMP_OFFSET         (36.53f)


/* ============================================================================
 *  数据结构
 * ============================================================================ */

/**
 * @brief MPU6050 原始数据（16 位有符号整型，直接从寄存器读出）
 */
typedef struct {
    int16_t accel_x;    /**< 加速度 X 轴原始值 */
    int16_t accel_y;    /**< 加速度 Y 轴原始值 */
    int16_t accel_z;    /**< 加速度 Z 轴原始值 */
    int16_t temp;       /**< 温度传感器原始值 */
    int16_t gyro_x;     /**< 角速度 X 轴原始值 */
    int16_t gyro_y;     /**< 角速度 Y 轴原始值 */
    int16_t gyro_z;     /**< 角速度 Z 轴原始值 */
} MPU6050_RawData_t;

/**
 * @brief MPU6050 配置句柄（记录当前量程设置，用于后续换算）
 */
typedef struct {
    MPU6050_GyroRange_t     gyro_range;     /**< 当前陀螺仪量程 */
    MPU6050_AccelRange_t    accel_range;    /**< 当前加速度量程 */
    MPU6050_DLPFBandwidth_t dlpf_bw;        /**< 当前 DLPF 带宽 */
    uint8_t                 sample_div;     /**< 当前采样分频 */
    float                   gyro_scale;     /**< 当前陀螺仪灵敏度 LSB/(°/s) */
    float                   accel_scale;    /**< 当前加速度灵敏度 LSB/g */
} MPU6050_Config_t;


/* ============================================================================
 *  函数声明
 * ============================================================================ */

/**
 * @brief   检测 MPU6050 是否在线
 * @return  true = 检测到器件 (WHO_AM_I = 0x68 或 0x70)
 * @note    在 Init 之前调用可验证硬件连接
 */
bool MID_MPU6050_Probe(void);

/**
 * @brief   初始化 MPU6050
 *
 * 默认配置：
 *   - 时钟源：PLL 参考 Z 轴陀螺仪
 *   - 陀螺仪量程：±1000 °/s
 *   - 加速度量程：±4g
 *   - DLPF：44 Hz
 *   - 采样率：200 Hz（SMPLRT_DIV = 4, 1kHz/(1+4)）
 *   - 数据就绪中断：关闭（由 app_imu 按需开启）
 *
 * @param   cfg 初始化参数（传 NULL 使用默认配置）
 * @return  true = 初始化成功, false = 失败
 */
bool MID_MPU6050_Init(const MPU6050_Config_t *cfg);

/**
 * @brief   读取全部原始数据
 * @param   raw  存放原始数据的结构体指针
 * @return  true = 读取成功, false = 失败
 *
 * 通过一次 I2C 多字节读（从 ACCEL_XOUT_H 起 14 字节）
 * 同时获取加速度、温度、角速度全部数据。
 */
bool MID_MPU6050_ReadRaw(MPU6050_RawData_t *raw);

/**
 * @brief   获取当前配置句柄
 * @return  const pointer to MPU6050_Config_t
 */
const MPU6050_Config_t *MID_MPU6050_GetConfig(void);

/**
 * @brief   获取 WHO_AM_I 寄存器值
 * @return  寄存器值（正常应为 0x68 或 0x70）
 */
uint8_t MID_MPU6050_GetDeviceID(void);

/**
 * @brief   执行自检 (Self-Test)
 *
 * 读取 6 个自检寄存器，计算陀螺仪和加速度计的自检响应。
 * 用于验证传感器是否正常工作。
 *
 * @param   gyro_st  [out] 陀螺仪自检结果 [X, Y, Z]（百分比，0 = 失败）
 * @param   accel_st [out] 加速度计自检结果 [X, Y, Z]（百分比，0 = 失败）
 * @return  true = 自检完成, false = I2C 通信失败
 */
bool MID_MPU6050_SelfTest(float gyro_st[3], float accel_st[3]);

/**
 * @brief   设置陀螺仪满量程
 * @param   range   量程枚举值
 * @return  true = 成功
 */
bool MID_MPU6050_SetGyroRange(MPU6050_GyroRange_t range);

/**
 * @brief   设置加速度计满量程
 * @param   range   量程枚举值
 * @return  true = 成功
 */
bool MID_MPU6050_SetAccelRange(MPU6050_AccelRange_t range);

/**
 * @brief   设置 DLPF 带宽
 * @param   bw  DLPF 配置枚举值
 * @return  true = 成功
 */
bool MID_MPU6050_SetDLPF(MPU6050_DLPFBandwidth_t bw);

/**
 * @brief   设置采样分频系数
 * @param   div  分频值（0~255）
 *               SampleRate = GyroOutputRate / (1 + div)
 *               GyroOutputRate = 1kHz (DLPF 开启) 或 8kHz (DLPF 关闭)
 * @return  true = 成功
 */
bool MID_MPU6050_SetSampleDiv(uint8_t div);

/**
 * @brief   使能或禁用数据就绪中断
 * @param   enable  true = 使能, false = 禁用
 * @return  true = 成功
 */
bool MID_MPU6050_EnableDataReadyInt(bool enable);

/**
 * @brief   读取中断状态
 * @return  中断状态寄存器值（位 0 = DATA_RDY）
 */
uint8_t MID_MPU6050_GetIntStatus(void);

/**
 * @brief   读取温度原始值并转换为摄氏度
 * @return  温度值（°C）
 */
float MID_MPU6050_ReadTemperature(void);

#endif /* MID_MPU6050_H */
