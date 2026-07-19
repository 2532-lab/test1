/**
 * @file    app_imu.c
 * @brief   应用层实现 — IMU 姿态解算与数据融合
 *
 * 实现互补滤波器，融合加速度计（低频可靠）和陀螺仪（高频可靠）
 * 的测量值，估计 Roll/Pitch 姿态角。
 *
 * 算法参考：
 *   complementary_filter(angle, gyro, accel, dt):
 *     angle = alpha * (angle + gyro*dt) + (1-alpha) * accel_angle;
 */

#include "app_imu.h"
#include "mid_mpu6050.h"
#include <math.h>

/* ============================================================================
 *  内部常量
 * ============================================================================ */

/** 互补滤波器默认 alpha（时间常数 ~0.25s @ 200Hz） */
#define IMU_DEFAULT_ALPHA       (0.98f)
/** 默认滤波周期（秒），对应 200Hz 更新率 */
#define IMU_DEFAULT_DT          (0.005f)

/** 弧度转角度系数 */
#define RAD_TO_DEG              (57.2957795131f)
/** 角度转弧度系数 */
#define DEG_TO_RAD              (0.01745329252f)

/* ============================================================================
 *  内部变量
 * ============================================================================ */

/** 初始化状态 */
static bool   s_initialized = false;

/** 最新缓存的 IMU 数据 */
static IMU_Data_t s_data = { 0 };

/** 滤波器参数 */
static IMU_Filter_t s_filter = {
    .alpha = IMU_DEFAULT_ALPHA,
    .dt    = IMU_DEFAULT_DT,
};

/** 校准数据 */
static IMU_Calib_t s_calib = {
    .gyro_offset_x  = 0.0f,
    .gyro_offset_y  = 0.0f,
    .gyro_offset_z  = 0.0f,
    .accel_offset_x = 0.0f,
    .accel_offset_y = 0.0f,
    .accel_offset_z = 0.0f,
    .calibrated     = false,
};

/* I2C 故障计数——连续失败超阈值自动重初始化 */
static uint8_t s_i2c_fail_count = 0;
#define IMU_I2C_FAIL_RESET  20   /* 20 帧连续失败 ≈ 100ms @ 200Hz */

/* ============================================================================
 *  初始化
 * ============================================================================ */

bool IMU_Init(void)
{
    /* 初始化中间层（内部自动调用 HAL_MPU6050_Init） */
    if (!MID_MPU6050_Init(NULL)) {
        s_initialized = false;
        return false;
    }

    /* 清除缓存数据 */
    s_data.roll  = 0.0f;
    s_data.pitch = 0.0f;
    s_data.yaw   = 0.0f;

    /* 滤波器使用默认参数 */
    s_filter.alpha = IMU_DEFAULT_ALPHA;
    s_filter.dt    = IMU_DEFAULT_DT;

    /* 校准标记为未校准 */
    s_calib.calibrated = false;
    s_calib.gyro_offset_x  = 0.0f;
    s_calib.gyro_offset_y  = 0.0f;
    s_calib.gyro_offset_z  = 0.0f;
    s_calib.accel_offset_x = 0.0f;
    s_calib.accel_offset_y = 0.0f;
    s_calib.accel_offset_z = 0.0f;

    s_initialized = true;
    return true;
}

/* ============================================================================
 *  校准
 * ============================================================================ */

void IMU_CalibrateGyro(uint16_t samples)
{
    if (!s_initialized || samples == 0) return;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t valid = 0;

    const MPU6050_Config_t *cfg = MID_MPU6050_GetConfig();

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_RawData_t raw;

        if (MID_MPU6050_ReadRaw(&raw)) {
            /* 原始值 → 物理单位 (LSB → °/s) */
            sum_x += (float)raw.gyro_x / cfg->gyro_scale;
            sum_y += (float)raw.gyro_y / cfg->gyro_scale;
            sum_z += (float)raw.gyro_z / cfg->gyro_scale;
            valid++;
        }
    }

    if (valid > 0) {
        s_calib.gyro_offset_x = sum_x / (float)valid;
        s_calib.gyro_offset_y = sum_y / (float)valid;
        s_calib.gyro_offset_z = sum_z / (float)valid;
        s_calib.calibrated = true;
    }
}

void IMU_CalibrateAccel(uint16_t samples)
{
    if (!s_initialized || samples == 0) return;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t valid = 0;

    const MPU6050_Config_t *cfg = MID_MPU6050_GetConfig();

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_RawData_t raw;

        if (MID_MPU6050_ReadRaw(&raw)) {
            /* 原始值 → 物理单位 (LSB → g) */
            sum_x += (float)raw.accel_x / cfg->accel_scale;
            sum_y += (float)raw.accel_y / cfg->accel_scale;
            sum_z += (float)raw.accel_z / cfg->accel_scale;
            valid++;
        }
    }

    if (valid > 0) {
        /* 传感器水平静止时 X/Y 期望为 0，Z 期望为 +1g
         * accel_offset = measured - expected
         * 使用 Z 轴减去 1g 以得到真实的零偏 */
        s_calib.accel_offset_x = sum_x / (float)valid;
        s_calib.accel_offset_y = sum_y / (float)valid;
        s_calib.accel_offset_z = (sum_z / (float)valid) - 1.0f;
    }

    s_calib.calibrated = true;
}

/* ============================================================================
 *  滤波器设置
 * ============================================================================ */

void IMU_SetFilter(float alpha, float dt)
{
    if (alpha < 0.90f)  alpha = 0.90f;
    if (alpha > 0.995f) alpha = 0.995f;
    if (dt    < 0.001f) dt    = 0.001f;
    if (dt    > 0.1f)   dt    = 0.1f;

    s_filter.alpha = alpha;
    s_filter.dt    = dt;
}

/* ============================================================================
 *  IMU 数据更新（核心姿态解算）
 * ============================================================================ */

bool IMU_Update(IMU_Data_t *data)
{
    if (!s_initialized) return false;

    MPU6050_RawData_t raw;
    if (!MID_MPU6050_ReadRaw(&raw)) {
        /* I2C 故障计数——连续失败超阈值自动重初始化 */
        s_i2c_fail_count++;
        if (s_i2c_fail_count >= IMU_I2C_FAIL_RESET) {
            s_initialized = false;
            IMU_Init();
            s_i2c_fail_count = 0;
        }
        return false;   /* 读取失败，data 不变 */
    }
    s_i2c_fail_count = 0;  /* 成功则复位计数器 */

    const MPU6050_Config_t *cfg = MID_MPU6050_GetConfig();
    const float dt = s_filter.dt;

    /* ---- 1. 换算为物理单位 ---- */

    /* 加速度：LSB → g */
    float accel_x = (float)raw.accel_x / cfg->accel_scale - s_calib.accel_offset_x;
    float accel_y = (float)raw.accel_y / cfg->accel_scale - s_calib.accel_offset_y;
    float accel_z = (float)raw.accel_z / cfg->accel_scale - s_calib.accel_offset_z;

    /* 陀螺仪：LSB → °/s */
    float gyro_x = (float)raw.gyro_x / cfg->gyro_scale - s_calib.gyro_offset_x;
    float gyro_y = (float)raw.gyro_y / cfg->gyro_scale - s_calib.gyro_offset_y;
    float gyro_z = (float)raw.gyro_z / cfg->gyro_scale - s_calib.gyro_offset_z;

    /* 温度——burst 读已包含 temp 字段，无需额外 I2C 事务 */
    float temp = (float)raw.temp / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;

    /* ---- 2. 加速度计计算 Roll/Pitch ---- */

    float accel_norm = sqrtf(accel_x * accel_x +
                             accel_y * accel_y +
                             accel_z * accel_z);

    if (accel_norm < 0.001f) {
        accel_norm = 0.001f;
    }

    /* 非重力加速度拒收——碰撞/急加速时跳过 accel 修正 */
    bool accel_valid = (fabsf(accel_norm - 1.0f) < 0.3f);

    /* Roll = atan2(Ay, Az) —— 绕 X 轴旋转 */
    float roll_accel = atan2f(accel_y, accel_z) * RAD_TO_DEG;

    /* Pitch = atan2(-Ax, sqrt(Ay² + Az²)) —— 绕 Y 轴旋转 */
    float pitch_accel = atan2f(-accel_x,
                                sqrtf(accel_y * accel_y + accel_z * accel_z))
                        * RAD_TO_DEG;

    /* ---- 3. 互补滤波器 ---- */

    /* 陀螺仪积分（角度增量 = 角速度 × dt） */
    float gyro_roll_increment  = gyro_x * dt;
    float gyro_pitch_increment = gyro_y * dt;
    float gyro_yaw_increment   = gyro_z * dt;

    /* 互补滤波：高频信任陀螺仪，低频信任加速度计
     * 非重力加速度时仅用陀螺积分（碰撞/急加速保护） */
    float alpha = s_filter.alpha;
    if (accel_valid) {
        s_data.roll  = alpha * (s_data.roll  + gyro_roll_increment)
                       + (1.0f - alpha) * roll_accel;
        s_data.pitch = alpha * (s_data.pitch + gyro_pitch_increment)
                       + (1.0f - alpha) * pitch_accel;
    } else {
        s_data.roll  += gyro_roll_increment;
        s_data.pitch += gyro_pitch_increment;
    }

    /* 偏航角无绝对参考，仅靠陀螺仪积分（必然漂移） */
    s_data.yaw += gyro_yaw_increment;

    /* ---- 4. 角度归一化 ---- */

    /* Roll: 限制在 -180 ~ +180 */
    if (s_data.roll > 180.0f)       s_data.roll -= 360.0f;
    else if (s_data.roll < -180.0f) s_data.roll += 360.0f;

    /* Pitch: 限制在 -90 ~ +90 */
    if (s_data.pitch > 90.0f)       s_data.pitch = 90.0f;
    else if (s_data.pitch < -90.0f) s_data.pitch = -90.0f;

    /* Yaw: 限制在 0 ~ 360 */
    if (s_data.yaw > 360.0f)        s_data.yaw -= 360.0f;
    else if (s_data.yaw < 0.0f)     s_data.yaw += 360.0f;

    /* ---- 5. 填充输出结构 ---- */

    s_data.accel_x = accel_x;
    s_data.accel_y = accel_y;
    s_data.accel_z = accel_z;
    s_data.gyro_x  = gyro_x;
    s_data.gyro_y  = gyro_y;
    s_data.gyro_z  = gyro_z;
    s_data.temperature = temp;

    if (data != NULL) {
        *data = s_data;
    }

    return true;
}

void IMU_GetData(IMU_Data_t *data)
{
    if (data != NULL) {
        *data = s_data;
    }
}

/* ============================================================================
 *  偏航角复位
 * ============================================================================ */

void IMU_ResetYaw(void)
{
    s_data.yaw = 0.0f;
}

/* ============================================================================
 *  状态查询
 * ============================================================================ */

bool IMU_IsReady(void)
{
    return s_initialized;
}

const IMU_Calib_t *IMU_GetCalibration(void)
{
    return &s_calib;
}

/* ============================================================================
 *  中断支持
 * ============================================================================ */

/**
 * 数据就绪中断回调转发。
 * 用户设置的回调通过此函数注册到 HAL 层。
 */
static void (*s_data_ready_cb)(void) = NULL;

static void IntCallbackBridge(void)
{
    if (s_data_ready_cb != NULL) {
        s_data_ready_cb();
    }
}

void IMU_SetDataReadyCallback(void (*cb)(void))
{
    s_data_ready_cb = cb;
    HAL_MPU6050_SetIntCallback(cb ? IntCallbackBridge : NULL);
}

bool IMU_EnableInterrupt(bool enable)
{
    if (!s_initialized) return false;

    /* 使能 MPU6050 内部中断 */
    if (!MID_MPU6050_EnableDataReadyInt(enable)) {
        return false;
    }

    /* 使能/禁用 HAL 层 GPIO 中断 */
    HAL_MPU6050_SetIntCallback(enable ? IntCallbackBridge : NULL);

    return true;
}
