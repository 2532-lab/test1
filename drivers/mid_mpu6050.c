/**
 * @file    mid_mpu6050.c
 * @brief   中间层实现 — MPU6050 寄存器协议
 *
 * 实现 MPU6050 的初始化序列、寄存器读写封装、原始数据读取。
 * 所有与硬件相关的 I2C 操作委托给 HAL 层完成。
 */

#include "mid_mpu6050.h"
#include "delay.h"  /* delay_ms() */

/* ============================================================================
 *  内部变量
 * ============================================================================ */

/** 当前配置句柄（静态分配，单例） */
static MPU6050_Config_t s_config = {
    .gyro_range  = MPU6050_GYRO_RANGE_1000,
    .accel_range = MPU6050_ACCEL_RANGE_4G,
    .dlpf_bw     = MPU6050_DLPF_44HZ,
    .sample_div  = 4,
    .gyro_scale  = MPU6050_GYRO_SCALE_1000,
    .accel_scale = MPU6050_ACCEL_SCALE_4G,
};

/* ============================================================================
 *  内部辅助：根据量程返回灵敏度常数
 * ============================================================================ */

static float GyroRangeToScale(MPU6050_GyroRange_t range)
{
    switch (range) {
        case MPU6050_GYRO_RANGE_250:  return MPU6050_GYRO_SCALE_250;
        case MPU6050_GYRO_RANGE_500:  return MPU6050_GYRO_SCALE_500;
        case MPU6050_GYRO_RANGE_1000: return MPU6050_GYRO_SCALE_1000;
        case MPU6050_GYRO_RANGE_2000: return MPU6050_GYRO_SCALE_2000;
        default:                       return MPU6050_GYRO_SCALE_1000;
    }
}

static float AccelRangeToScale(MPU6050_AccelRange_t range)
{
    switch (range) {
        case MPU6050_ACCEL_RANGE_2G:  return MPU6050_ACCEL_SCALE_2G;
        case MPU6050_ACCEL_RANGE_4G:  return MPU6050_ACCEL_SCALE_4G;
        case MPU6050_ACCEL_RANGE_8G:  return MPU6050_ACCEL_SCALE_8G;
        case MPU6050_ACCEL_RANGE_16G: return MPU6050_ACCEL_SCALE_16G;
        default:                       return MPU6050_ACCEL_SCALE_4G;
    }
}

/* ============================================================================
 *  检测器件
 * ============================================================================ */

bool MID_MPU6050_Probe(void)
{
    uint8_t who_am_i = 0;

    if (!HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                             MPU6050_REG_WHO_AM_I, &who_am_i)) {
        return false;
    }

    /* MPU6050 WHO_AM_I 典型返回值为 0x68（AD0=0）或 0x70（AD0=1） */
    return (who_am_i == 0x68 || who_am_i == 0x70);
}

/* ============================================================================
 *  初始化
 * ============================================================================ */

bool MID_MPU6050_Init(const MPU6050_Config_t *cfg)
{
    uint8_t val;

    /* ---- 初始化硬件层 ---- */
    HAL_MPU6050_Init();

    /* 给 MPU6050 上电稳定时间 */
    delay_ms(50);

    /* ---- 验证器件连接 ---- */
    if (!MID_MPU6050_Probe()) {
        return false;
    }

    /* ---- 复位器件 ---- */
    /* 写 0x80 到 PWR_MGMT_1 触发复位，所有寄存器恢复默认值 */
    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_PWR_MGMT_1, 0x80)) {
        return false;
    }
    delay_ms(100);  /* 等待复位完成（典型值 100ms） */

    /* ---- 唤醒器件 ----
     *
     * 复位后 PWR_MGMT_1 默认值 0x40（Sleep 模式）。
     * 写 0x00 以唤醒器件并选择内部时钟源。
     * 后续立即切换到 PLL Z 轴陀螺仪以获得更好的时钟精度。
     */
    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_PWR_MGMT_1, 0x00)) {
        return false;
    }
    delay_ms(10);

    /* ---- 如果调用者提供了配置参数，则使用；否则使用默认值 ---- */

    /* 设置时钟源：PLL 参考 Z 轴陀螺仪 */
    val = MPU6050_PWR1_CLKSEL_ZGYRO;
    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_PWR_MGMT_1, val)) {
        return false;
    }

    if (cfg != NULL) {
        s_config = *cfg;
    }

    /* ---- 逐一应用配置 ---- */

    /* DLPF 带宽 */
    if (!MID_MPU6050_SetDLPF(s_config.dlpf_bw)) {
        return false;
    }

    /* 采样率分频 */
    if (!MID_MPU6050_SetSampleDiv(s_config.sample_div)) {
        return false;
    }

    /* 陀螺仪量程 */
    if (!MID_MPU6050_SetGyroRange(s_config.gyro_range)) {
        return false;
    }

    /* 加速度计量程 */
    if (!MID_MPU6050_SetAccelRange(s_config.accel_range)) {
        return false;
    }

    /* ---- 清除可能残留的中断标志 ---- */
    if (!HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                             MPU6050_REG_INT_STATUS, &val)) {
        return false;
    }

    /* 忽略 val，仅用于清除标志 */

    return true;
}

/* ============================================================================
 *  读取原始数据
 * ============================================================================ */

bool MID_MPU6050_ReadRaw(MPU6050_RawData_t *raw)
{
    if (raw == NULL) return false;

    uint8_t buf[14];

    /* 一次读取 14 字节：ACCEL_XOUT_H ~ GYRO_ZOUT_L */
    if (!HAL_MPU6050_ReadMulti(MPU6050_ADDR_7BIT,
                               MPU6050_REG_ACCEL_XOUT_H, buf, 14)) {
        return false;
    }

    /* 大端序（MSB first）拼接为 16 位有符号整数 */
    raw->accel_x = (int16_t)((uint16_t)(buf[0]) << 8 | buf[1]);
    raw->accel_y = (int16_t)((uint16_t)(buf[2]) << 8 | buf[3]);
    raw->accel_z = (int16_t)((uint16_t)(buf[4]) << 8 | buf[5]);
    raw->temp    = (int16_t)((uint16_t)(buf[6]) << 8 | buf[7]);
    raw->gyro_x  = (int16_t)((uint16_t)(buf[8]) << 8 | buf[9]);
    raw->gyro_y  = (int16_t)((uint16_t)(buf[10]) << 8 | buf[11]);
    raw->gyro_z  = (int16_t)((uint16_t)(buf[12]) << 8 | buf[13]);

    return true;
}

/* ============================================================================
 *  配置获取
 * ============================================================================ */

const MPU6050_Config_t *MID_MPU6050_GetConfig(void)
{
    return &s_config;
}

/* ============================================================================
 *  读器件 ID
 * ============================================================================ */

uint8_t MID_MPU6050_GetDeviceID(void)
{
    uint8_t id = 0;
    HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                        MPU6050_REG_WHO_AM_I, &id);
    return id;
}

/* ============================================================================
 *  自检
 * ============================================================================ */

bool MID_MPU6050_SelfTest(float gyro_st[3], float accel_st[3])
{
    uint8_t buf[4];

    /* 读取自检寄存器 */
    if (!HAL_MPU6050_ReadMulti(MPU6050_ADDR_7BIT,
                               MPU6050_REG_SELF_TEST_X, buf, 4)) {
        return false;
    }

    /* 陀螺仪自检值来自 SELF_TEST_X[7:5], SELF_TEST_Y[7:5], SELF_TEST_Z[7:5] */
    float gx_st = (float)((buf[0] >> 5) & 0x07);
    float gy_st = (float)((buf[1] >> 5) & 0x07);
    float gz_st = (float)((buf[2] >> 5) & 0x07);

    /* 加速度自检值来自 SELF_TEST_A[7:5], [4:2], [1:0] 跨字节组合 */
    /* 实际计算参考 InvenSense 应用笔记，此处简化为读取原始值 */
    float ax_st = (float)(buf[3] & 0x07);

    if (gyro_st != NULL) {
        gyro_st[0] = gx_st * 100.0f / 7.0f;  /* 百分比 */
        gyro_st[1] = gy_st * 100.0f / 7.0f;
        gyro_st[2] = gz_st * 100.0f / 7.0f;
    }

    if (accel_st != NULL) {
        accel_st[0] = ax_st * 100.0f / 7.0f;
        accel_st[1] = 0.0f;  /* 简化：Y/Z 自检需要更多计算 */
        accel_st[2] = 0.0f;
    }

    return true;
}

/* ============================================================================
 *  配置接口
 * ============================================================================ */

bool MID_MPU6050_SetGyroRange(MPU6050_GyroRange_t range)
{
    uint8_t val;
    uint8_t regval = (uint8_t)range;

    /* 读取当前 GYRO_CONFIG 值，保留除量程位外的其他位 */
    if (!HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                             MPU6050_REG_GYRO_CONFIG, &val)) {
        return false;
    }

    val = (val & ~MPU6050_GYRO_CFG_FS_SEL_MASK) | regval;

    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_GYRO_CONFIG, val)) {
        return false;
    }

    s_config.gyro_range = range;
    s_config.gyro_scale = GyroRangeToScale(range);
    return true;
}

bool MID_MPU6050_SetAccelRange(MPU6050_AccelRange_t range)
{
    uint8_t val;
    uint8_t regval = (uint8_t)range;

    if (!HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                             MPU6050_REG_ACCEL_CONFIG, &val)) {
        return false;
    }

    val = (val & ~MPU6050_ACCEL_CFG_FS_SEL_MASK) | regval;

    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_ACCEL_CONFIG, val)) {
        return false;
    }

    s_config.accel_range = range;
    s_config.accel_scale = AccelRangeToScale(range);
    return true;
}

bool MID_MPU6050_SetDLPF(MPU6050_DLPFBandwidth_t bw)
{
    uint8_t val;
    uint8_t regval = (uint8_t)bw;

    if (!HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                             MPU6050_REG_CONFIG, &val)) {
        return false;
    }

    val = (val & ~MPU6050_CONFIG_DLPF_CFG_MASK) | regval;

    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_CONFIG, val)) {
        return false;
    }

    s_config.dlpf_bw = bw;
    return true;
}

bool MID_MPU6050_SetSampleDiv(uint8_t div)
{
    if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                              MPU6050_REG_SMPLRT_DIV, div)) {
        return false;
    }

    s_config.sample_div = div;
    return true;
}

/* ============================================================================
 *  中断控制
 * ============================================================================ */

bool MID_MPU6050_EnableDataReadyInt(bool enable)
{
    uint8_t val = enable ? MPU6050_INT_EN_DATA_RDY : 0x00;

    /* 配置中断引脚：高电平有效，推挽输出，读寄存器清除 */
    if (enable) {
        uint8_t pin_cfg =
            MPU6050_INT_CFG_INT_LEVEL     |   /* 高电平有效 */
            MPU6050_INT_CFG_INT_RD_CLEAR;      /* 读 INT_STATUS 清除 */

        if (!HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                                  MPU6050_REG_INT_PIN_CFG, pin_cfg)) {
            return false;
        }
    }

    return HAL_MPU6050_WriteReg(MPU6050_ADDR_7BIT,
                                MPU6050_REG_INT_ENABLE, val);
}

uint8_t MID_MPU6050_GetIntStatus(void)
{
    uint8_t sts = 0;
    HAL_MPU6050_ReadReg(MPU6050_ADDR_7BIT,
                        MPU6050_REG_INT_STATUS, &sts);
    return sts;
}

/* ============================================================================
 *  温度读取
 * ============================================================================ */

float MID_MPU6050_ReadTemperature(void)
{
    uint8_t buf[2];

    if (!HAL_MPU6050_ReadMulti(MPU6050_ADDR_7BIT,
                               MPU6050_REG_TEMP_OUT_H, buf, 2)) {
        return 0.0f;
    }

    int16_t raw = (int16_t)((uint16_t)(buf[0]) << 8 | buf[1]);

    /* 公式来自 MPU6050 数据手册 */
    return (float)raw / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;
}
