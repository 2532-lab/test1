/**
 * @file    app_imu_9dof.c
 * @brief   9-DOF AHRS 实现 — 互补滤波 Roll/Pitch + 磁力计修正 Yaw
 *
 * 传感器：
 *   - MPU6050：3 轴陀螺仪 + 3 轴加速度计（I2C1, 地址 0x68）
 *   - 磁力计：自动鉴别 HMC5883L（0x1E）或 QMC5883L（0x0D）
 *     经 MPU6050 AUX I2C Bypass 访问
 *
 * 融合算法：
 *   Roll/Pitch：互补滤波器（陀螺积分 + 加速度计绝对参考）
 *   Yaw：      互补滤波器（陀螺积分 + 磁力计倾角补偿航向）
 *               磁力计读取失败时 yaw 仅靠陀螺积分过渡，不被动拉向 0°
 *
 * 不改动任何现有的 MPU6050 三层文件。
 *
 * @note 编译本文件前需确认以下依赖已就位：
 *       - mid_hmc5883l.h / mid_hmc5883l.c
 *       - mid_qmc5883l.h / mid_qmc5883l.c
 *       如不需要磁力计融合，请使用 app_imu.c（纯 6-DOF）替代。
 */

#include "app_imu_9dof.h"
#include "mid_mpu6050.h"
#include "mid_hmc5883l.h"
#include "mid_qmc5883l.h"
#include "delay.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 *  内部常量
 * ============================================================================ */

#define IMU_9DOF_DEFAULT_ALPHA       (0.98f)   /**< Roll/Pitch 互补系数 */
#define IMU_9DOF_DEFAULT_ALPHA_YAW   (0.98f)   /**< Yaw 互补系数（与 Roll/Pitch 一致，磁力计噪声由 DRDY 预检查抑制） */
#define IMU_9DOF_DEFAULT_DT          (0.005f)  /**< 默认滤波周期（200Hz） */
#define RAD_TO_DEG                   (57.2957795131f)
#define DEG_TO_RAD                   (0.01745329252f)

/* ============================================================================
 *  内部辅助函数
 * ============================================================================ */

/** 将角度限制到 (-180, +180] */
static float wrap_180(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

/** 将角度限制到 [0, 360) */
static float wrap_360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle <  0.0f)   angle += 360.0f;
    return angle;
}

/* ============================================================================
 *  内部变量
 * ============================================================================ */

static bool            s_initialized = false;
static IMU_MagChip_t   s_mag_chip    = IMU_MAG_CHIP_NONE;
static IMU_9DOF_Data_t s_data        = { 0 };

static IMU_9DOF_Filter_t s_filter = {
    .alpha       = IMU_9DOF_DEFAULT_ALPHA,
    .alpha_yaw   = IMU_9DOF_DEFAULT_ALPHA_YAW,
    .dt          = IMU_9DOF_DEFAULT_DT,
    .declination = -5.0f,     /**< 默认磁偏角 -5°（中国中东部近似值，需根据比赛地调整） */
};

static IMU_9DOF_Calib_t s_calib = { 0 };

/* I2C 故障计数——连续失败超过阈值自动重初始化传感器 */
static uint8_t s_i2c_fail_count = 0;
#define I2C_FAIL_RESET_THRESHOLD  20   /* 20 帧连续失败 ≈ 100ms @ 200Hz */

/* ============================================================================
 *  初始化（自动鉴别磁力计芯片）
 * ============================================================================ */

bool IMU_9DOF_Init(void)
{
    /* 1. 初始化 MPU6050（内部自动调用 HAL_MPU6050_Init 配置 I2C1 为 400kHz） */
    if (!MID_MPU6050_Init(NULL)) {
        s_initialized = false;
        s_mag_chip = IMU_MAG_CHIP_NONE;
        return false;
    }

    /* 2. 自动鉴别磁力计芯片：先试 HMC5883L（正品），再试 QMC5883L（替代品） */

    /* —— 尝试 HMC5883L（地址 0x1E）—— */
    if (MID_HMC5883L_Init(NULL)) {
        s_mag_chip = IMU_MAG_CHIP_HMC5883L;
    }
    /* —— 尝试 QMC5883L（地址 0x0D）—— */
    else if (MID_QMC5883L_Init(NULL)) {
        s_mag_chip = IMU_MAG_CHIP_QMC5883L;
    }
    else {
        /* 两种磁力计都不在线——降级为纯 6-DOF 模式
         * yaw 仅靠陀螺积分，会缓慢漂移（~1-5°/min），但系统不宕机 */
        s_mag_chip = IMU_MAG_CHIP_NONE;
        /* 继续执行——6-DOF 模式仍可使用 MPU6050 */
    }

    /* 3. 清零姿态缓存 */
    memset(&s_data, 0, sizeof(s_data));

    /* 4. 滤波器使用默认参数 */
    s_filter.alpha       = IMU_9DOF_DEFAULT_ALPHA;
    s_filter.alpha_yaw   = IMU_9DOF_DEFAULT_ALPHA_YAW;
    s_filter.dt          = IMU_9DOF_DEFAULT_DT;
    s_filter.declination = 0.0f;

    /* 5. 校准参数复位 */
    memset(&s_calib, 0, sizeof(s_calib));
    s_calib.mag_scale[0] = 1.0f;
    s_calib.mag_scale[1] = 1.0f;
    s_calib.mag_scale[2] = 1.0f;

    s_initialized = true;
    return true;
}

IMU_MagChip_t IMU_9DOF_GetMagChipType(void)
{
    return s_mag_chip;
}

/* ============================================================================
 *  陀螺仪校准（与芯片无关，直接读 MPU6050）
 * ============================================================================ */

void IMU_9DOF_CalibrateGyro(uint16_t samples)
{
    if (!s_initialized || samples == 0) return;

    float sum[3] = { 0 };
    uint16_t valid = 0;
    const MPU6050_Config_t *cfg = MID_MPU6050_GetConfig();

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_RawData_t raw;
        if (MID_MPU6050_ReadRaw(&raw)) {
            sum[0] += (float)raw.gyro_x / cfg->gyro_scale;
            sum[1] += (float)raw.gyro_y / cfg->gyro_scale;
            sum[2] += (float)raw.gyro_z / cfg->gyro_scale;
            valid++;
        }
    }

    if (valid > 0) {
        s_calib.gyro_offset[0] = sum[0] / (float)valid;
        s_calib.gyro_offset[1] = sum[1] / (float)valid;
        s_calib.gyro_offset[2] = sum[2] / (float)valid;
        s_calib.gyro_calibrated = true;
    }
}

/* ============================================================================
 *  磁力计校准（硬铁偏移：min/max 法，自动适配芯片类型）
 * ============================================================================ */

void IMU_9DOF_CalibrateMag(uint16_t samples, uint32_t delay_each_ms)
{
    if (!s_initialized || samples == 0) return;
    if (s_mag_chip == IMU_MAG_CHIP_NONE) return;

    float mag[3] = { 0 };

    /* ---- 采集第一帧初始化 min/max ---- */
    bool ok = false;
    if (s_mag_chip == IMU_MAG_CHIP_HMC5883L) {
        HMC5883L_RawData_t raw;
        if (MID_HMC5883L_ReadRaw(&raw)) {
            MID_HMC5883L_RawToMicroTesla(&raw, NULL, mag);
            ok = true;
        }
    } else { /* QMC5883L */
        QMC5883L_RawData_t raw;
        if (MID_QMC5883L_ReadRaw(&raw)) {
            MID_QMC5883L_RawToMicroTesla(&raw, mag);
            ok = true;
        }
    }
    if (!ok) return;

    float min_val[3] = { mag[0], mag[1], mag[2] };
    float max_val[3] = { mag[0], mag[1], mag[2] };

    /* ---- 采集剩余样本 ---- */
    for (uint16_t i = 1; i < samples; i++) {
        delay_ms(delay_each_ms);

        ok = false;
        if (s_mag_chip == IMU_MAG_CHIP_HMC5883L) {
            HMC5883L_RawData_t raw;
            if (MID_HMC5883L_ReadRaw(&raw)) {
                MID_HMC5883L_RawToMicroTesla(&raw, NULL, mag);
                ok = true;
            }
        } else {
            QMC5883L_RawData_t raw;
            if (MID_QMC5883L_ReadRaw(&raw)) {
                MID_QMC5883L_RawToMicroTesla(&raw, mag);
                ok = true;
            }
        }
        if (!ok) continue;

        for (int a = 0; a < 3; a++) {
            if (mag[a] < min_val[a]) min_val[a] = mag[a];
            if (mag[a] > max_val[a]) max_val[a] = mag[a];
        }
    }

    /* ---- 计算硬铁偏移（球心）---- */
    s_calib.mag_offset[0] = (max_val[0] + min_val[0]) * 0.5f;
    s_calib.mag_offset[1] = (max_val[1] + min_val[1]) * 0.5f;
    s_calib.mag_offset[2] = (max_val[2] + min_val[2]) * 0.5f;

    /* ---- 计算软铁缩放（各轴半径归一化）---- */
    float radius[3];
    for (int a = 0; a < 3; a++) {
        radius[a] = (max_val[a] - min_val[a]) * 0.5f;
        if (radius[a] < 1.0f) radius[a] = 1.0f;   /* 防除零 */
    }
    float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
    for (int a = 0; a < 3; a++) {
        s_calib.mag_scale[a] = avg_radius / radius[a];
    }

    s_calib.mag_calibrated = true;
}

/* ============================================================================
 *  滤波器设置
 * ============================================================================ */

void IMU_9DOF_SetFilter(float alpha, float alpha_yaw, float dt)
{
    if (alpha     < 0.90f)  alpha     = 0.90f;
    if (alpha     > 0.995f) alpha     = 0.995f;
    if (alpha_yaw < 0.90f)  alpha_yaw = 0.90f;
    if (alpha_yaw > 0.995f) alpha_yaw = 0.995f;
    if (dt        < 0.001f) dt        = 0.001f;
    if (dt        > 0.1f)   dt        = 0.1f;

    s_filter.alpha     = alpha;
    s_filter.alpha_yaw = alpha_yaw;
    s_filter.dt        = dt;
}

void IMU_9DOF_SetDeclination(float declination_deg)
{
    s_filter.declination = declination_deg;
}

void IMU_9DOF_SetMagCalib(const float offset[3], const float scale[3])
{
    if (offset) {
        s_calib.mag_offset[0] = offset[0];
        s_calib.mag_offset[1] = offset[1];
        s_calib.mag_offset[2] = offset[2];
    }
    if (scale) {
        s_calib.mag_scale[0] = scale[0];
        s_calib.mag_scale[1] = scale[1];
        s_calib.mag_scale[2] = scale[2];
    }
    s_calib.mag_calibrated = true;
}

/* ============================================================================
 *  9-DOF 数据更新（核心姿态解算）
 * ============================================================================ */

bool IMU_9DOF_Update(IMU_9DOF_Data_t *data)
{
    if (!s_initialized) return false;

    const float dt = s_filter.dt;
    const float alpha     = s_filter.alpha;
    const float alpha_yaw = s_filter.alpha_yaw;

    /* ---- 1. 读取 MPU6050 ---- */
    MPU6050_RawData_t raw_mpu;
    if (!MID_MPU6050_ReadRaw(&raw_mpu)) {
        /* I2C 故障计数——连续失败超阈值自动重初始化传感器 */
        s_i2c_fail_count++;
        if (s_i2c_fail_count >= I2C_FAIL_RESET_THRESHOLD) {
            s_initialized = false;
            IMU_9DOF_Init();         /* 重新探测 + 初始化 I2C 总线 */
            s_i2c_fail_count = 0;
        }
        return false;
    }
    s_i2c_fail_count = 0;  /* 成功则复位计数器 */

    const MPU6050_Config_t *cfg = MID_MPU6050_GetConfig();

    /* 加速度：LSB → g */
    float accel[3] = {
        (float)raw_mpu.accel_x / cfg->accel_scale,
        (float)raw_mpu.accel_y / cfg->accel_scale,
        (float)raw_mpu.accel_z / cfg->accel_scale,
    };

    /* 陀螺仪：LSB → °/s，减去零偏 */
    float gyro[3] = {
        (float)raw_mpu.gyro_x / cfg->gyro_scale - s_calib.gyro_offset[0],
        (float)raw_mpu.gyro_y / cfg->gyro_scale - s_calib.gyro_offset[1],
        (float)raw_mpu.gyro_z / cfg->gyro_scale - s_calib.gyro_offset[2],
    };

    float temp = (float)raw_mpu.temp / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;

    /* ---- 2. 读取磁力计（自动适配芯片类型）---- */
    float mag[3] = { 0 };
    bool  mag_valid = false;

    if (s_mag_chip == IMU_MAG_CHIP_HMC5883L) {
        /* DRDY 预检查：HMC5883L 输出率 75Hz，主循环可能更快。
         * 未就绪时跳读，避免读到上一帧的重复数据 */
        if (MID_HMC5883L_IsDataReady()) {
            HMC5883L_RawData_t raw_mag;
            if (MID_HMC5883L_ReadRaw(&raw_mag)) {
                /* 先转 μT（未校准），再在 μT 域手动应用校准 */
                MID_HMC5883L_RawToMicroTesla(&raw_mag, NULL, mag);
                if (s_calib.mag_calibrated) {
                    mag[0] = (mag[0] - s_calib.mag_offset[0]) * s_calib.mag_scale[0];
                    mag[1] = (mag[1] - s_calib.mag_offset[1]) * s_calib.mag_scale[1];
                    mag[2] = (mag[2] - s_calib.mag_offset[2]) * s_calib.mag_scale[2];
                }
                mag_valid = true;
            }
        }
    } else if (s_mag_chip == IMU_MAG_CHIP_QMC5883L) {
        /* QMC5883L 输出率 200Hz，同样做 DRDY 预检查防止重复数据 */
        if (MID_QMC5883L_IsDataReady()) {
            QMC5883L_RawData_t raw_mag;
            if (MID_QMC5883L_ReadRaw(&raw_mag)) {
                MID_QMC5883L_RawToMicroTesla(&raw_mag, mag);
                if (s_calib.mag_calibrated) {
                    mag[0] = (mag[0] - s_calib.mag_offset[0]) * s_calib.mag_scale[0];
                    mag[1] = (mag[1] - s_calib.mag_offset[1]) * s_calib.mag_scale[1];
                    mag[2] = (mag[2] - s_calib.mag_offset[2]) * s_calib.mag_scale[2];
                }
                mag_valid = true;
            }
        }
    }
    /* 磁力计读取失败 → mag_valid=false，mag 保持零。
     * yaw 仅靠陀螺积分过渡一帧，不被拉向 0°。 */

    /* ---- 3. 加速度计 → Roll / Pitch ---- */

    float accel_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (accel_norm < 0.001f) accel_norm = 0.001f;

    /* 非重力加速度拒收：碰撞/急加速时 accel_norm 偏离 1g，跳过加速度计修正
     * ——仅靠陀螺积分过渡一帧，避免姿态被惯性加速度拉偏 */
    bool accel_valid = (fabsf(accel_norm - 1.0f) < 0.3f);

    float roll_accel  = atan2f(accel[1], accel[2]) * RAD_TO_DEG;
    float pitch_accel = atan2f(-accel[0],
                                sqrtf(accel[1]*accel[1] + accel[2]*accel[2]))
                        * RAD_TO_DEG;

    /* ---- 4. 互补滤波器 —— Roll / Pitch ---- */

    /* 陀螺积分增量 */
    float gyro_roll_inc  = gyro[0] * dt;
    float gyro_pitch_inc = gyro[1] * dt;
    float gyro_yaw_inc   = gyro[2] * dt;

    /* 互补滤波——非重力加速度时仅用陀螺积分（等效 α=1.0） */
    if (accel_valid) {
        s_data.roll  = alpha * (s_data.roll  + gyro_roll_inc)
                       + (1.0f - alpha) * roll_accel;
        s_data.pitch = alpha * (s_data.pitch + gyro_pitch_inc)
                       + (1.0f - alpha) * pitch_accel;
    } else {
        s_data.roll  += gyro_roll_inc;
        s_data.pitch += gyro_pitch_inc;
    }

    /* ---- 5. 磁力计倾角补偿 → 航向角（仅磁力计有效时计算）---- */

    if (mag_valid) {
        /* 使用互补滤波后的 roll/pitch 做倾角补偿（比用 accel 瞬时值更稳定） */
        float roll_rad  = s_data.roll  * DEG_TO_RAD;
        float pitch_rad = s_data.pitch * DEG_TO_RAD;
        float cos_r = cosf(roll_rad),  sin_r = sinf(roll_rad);
        float cos_p = cosf(pitch_rad), sin_p = sinf(pitch_rad);

        /* 将磁力计矢量从机体坐标系旋转到水平面（tilt compensation） */
        float mx_h = mag[0] * cos_p
                   + mag[1] * sin_r * sin_p
                   + mag[2] * cos_r * sin_p;
        float my_h = mag[1] * cos_r
                   - mag[2] * sin_r;

        /* 磁航向：水平面内 atan2(-Y, X)，0=磁北，顺时针增大 */
        float mag_heading = atan2f(-my_h, mx_h) * RAD_TO_DEG;
        /* 加磁偏角（东偏为正）得到真北航向 */
        mag_heading += s_filter.declination;
        mag_heading = wrap_360(mag_heading);

        s_data.heading = mag_heading;

        /* ---- 6. 互补滤波器 —— Yaw（磁力计修正陀螺漂移）---- */

        /* 误差 = 磁航向 - 当前 yaw，包络到 ±180°（处理 0/360 环绕） */
        float yaw_err = wrap_180(mag_heading - s_data.yaw);

        /* Yaw 互补滤波：陀螺积分 + 磁力计缓慢拉回 */
        s_data.yaw += gyro_yaw_inc + yaw_err * (1.0f - alpha_yaw);
        s_data.yaw  = wrap_360(s_data.yaw);
    } else {
        /* 磁力计无效 —— yaw 仅靠陀螺仪积分（会缓慢漂移，但不会跳变） */
        s_data.yaw += gyro_yaw_inc;
        s_data.yaw  = wrap_360(s_data.yaw);
        /* heading 保持上一帧值，不清零 */
    }

    /* ---- 7. Roll / Pitch 归一化 ---- */

    s_data.roll  = wrap_180(s_data.roll);
    if      (s_data.pitch >  90.0f) s_data.pitch =  90.0f;
    else if (s_data.pitch < -90.0f) s_data.pitch = -90.0f;

    /* ---- 8. 填充输出结构 ---- */

    s_data.accel[0] = accel[0];
    s_data.accel[1] = accel[1];
    s_data.accel[2] = accel[2];
    s_data.gyro[0]  = gyro[0];
    s_data.gyro[1]  = gyro[1];
    s_data.gyro[2]  = gyro[2];
    s_data.mag[0]   = mag[0];
    s_data.mag[1]   = mag[1];
    s_data.mag[2]   = mag[2];
    s_data.temperature = temp;

    if (data != NULL) {
        *data = s_data;
    }

    return true;
}

/* ============================================================================
 *  偏航角复位
 * ============================================================================ */

void IMU_9DOF_ResetYaw(void)
{
    s_data.yaw = 0.0f;
}

/* ============================================================================
 *  状态查询
 * ============================================================================ */

const IMU_9DOF_Calib_t *IMU_9DOF_GetCalibration(void)
{
    return &s_calib;
}
