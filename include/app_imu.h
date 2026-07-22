/**
 * @file    app_imu.h
 * @brief   应用层 — IMU 姿态解算与数据融合
 *
 * 本层是三层架构的最顶层，基于中间层 (mid_mpu6050) 的原始数据
 * 完成以下功能：
 *   - 陀螺仪零偏校准
 *   - 加速度计互补滤波 + 陀螺仪积分 → Roll/Pitch 姿态角
 *   - 可选的数据就绪中断驱动更新模式
 *   - 物理单位换算（g, °/s, °C）
 *
 * 姿态解算算法：互补滤波器
 *   roll  = α × (roll  + gyro_x × dt) + (1-α) × roll_accel
 *   pitch = α × (pitch + gyro_y × dt) + (1-α) × pitch_accel
 *
 * 典型 α 值 0.98（时间常数 τ = 0.98 × dt / (1-0.98)）
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>
#include <stdbool.h>


/* ============================================================================
 *  数据结构
 * ============================================================================ */

/**
 * @brief IMU 物理量数据（浮点，SI 单位制）
 */
typedef struct {
    /* 加速度（单位：g） */
    float accel_x;
    float accel_y;
    float accel_z;

    /* 角速度（单位：°/s） */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* 温度（单位：°C） */
    float temperature;

    /* 姿态角（单位：°） */
    float roll;     /**< 横滚角（绕 X 轴），范围 -180 ~ +180 */
    float pitch;    /**< 俯仰角（绕 Y 轴），范围 -90 ~ +90 */
    float yaw;      /**< 偏航角（绕 Z 轴），范围 -180 ~ +180，无磁力计会漂移 */
} IMU_Data_t;

/**
 * @brief 互补滤波器参数
 */
typedef struct {
    float alpha;        /**< 互补滤波系数（0.90 ~ 0.995，推荐 0.98） */
    float dt;           /**< 滤波周期（秒），需与应用更新周期一致 */
} IMU_Filter_t;

/**
 * @brief IMU 校准偏移量
 */
typedef struct {
    float gyro_offset_x;    /**< 陀螺仪 X 轴零偏 (°/s) */
    float gyro_offset_y;    /**< 陀螺仪 Y 轴零偏 (°/s) */
    float gyro_offset_z;    /**< 陀螺仪 Z 轴零偏 (°/s) */
    float accel_offset_x;   /**< 加速度 X 轴零偏 (g) */
    float accel_offset_y;   /**< 加速度 Y 轴零偏 (g) */
    float accel_offset_z;   /**< 加速度 Z 轴零偏 (g) - 通常减去 1g 得到静态偏移 */
    bool  calibrated;       /**< 是否已完成校准 */
} IMU_Calib_t;


/* ============================================================================
 *  函数声明
 * ============================================================================ */

/**
 * @brief   初始化 IMU 应用层
 *
 * 内部依次调用：
 *   1. HAL_MPU6050_Init() — 硬件层初始化
 *   2. MID_MPU6050_Init() — 中间层初始化（使用默认配置）
 *   3. 内部状态清零
 *
 * @return  true = 初始化成功, false = 失败
 */
bool IMU_Init(void);

/**
 * @brief   执行陀螺仪零偏校准
 *
 * 传感器静止时采集 samples 次数据，取均值作为零偏。
 * 该校准值会被 IMU_Update() 自动减去。
 *
 * @param   samples  采样次数（建议 200 ~ 1000）
 *                   200 次 @ 200Hz = 1 秒采样窗口
 */
void IMU_CalibrateGyro(uint16_t samples);

/**
 * @brief   执行加速度计零偏校准
 *
 * 传感器水平静止时采集 samples 次数据。
 * X/Y 零偏应接近 0，Z 零偏为 measured_Z - 1g。
 *
 * @param   samples  采样次数（建议 200 ~ 1000）
 */
void IMU_CalibrateAccel(uint16_t samples);

/**
 * @brief   设置互补滤波器参数
 * @param   alpha  滤波系数（0.90 ~ 0.995）
 * @param   dt     滤波周期（秒）
 *
 * 时间常数 τ = alpha × dt / (1 - alpha)
 *   例：alpha=0.98, dt=0.005s (200Hz) → τ ≈ 0.245s
 *       即相信加速度计的时间尺度约为 0.25 秒
 */
void IMU_SetFilter(float alpha, float dt);

/**
 * @brief   更新 IMU 数据（单次采样 + 姿态解算）
 *
 * 执行流程：
 *   1. 读取 MPU6050 原始数据
 *   2. 减去校准零偏
 *   3. 换算为物理单位（g 和 °/s）
 *   4. 互补滤波器解算 Roll/Pitch
 *   5. 读取温度
 *
 * @param   data  [out] 输出本次更新后的 IMU 数据
 * @return  true = 数据有效, false = 读取失败（data 不变）
 *
 * @note 应在固定周期的定时器中断或主循环中调用，
 *       确保 dt 与调用周期一致。
 */
bool IMU_Update(IMU_Data_t *data);

/**
 * @brief   获取当前 IMU 数据（不发起新的读取）
 * @param   data  [out] 输出缓存的 IMU 数据
 */
void IMU_GetData(IMU_Data_t *data);

/**
 * @brief   重置偏航角（将当前 yaw 设为零点）
 *
 * 调用后 yaw 输出以当前朝向为 0° 基准重新累积。
 */
void IMU_ResetYaw(void);

/**
 * @brief   检查 IMU 是否已完成初始化
 * @return  true = 已初始化, false = 未初始化
 */
bool IMU_IsReady(void);

/**
 * @brief   获取校准状态指针
 * @return  const pointer to IMU_Calib_t
 */
const IMU_Calib_t *IMU_GetCalibration(void);

/**
 * @brief   设置数据就绪中断回调（可选）
 *
 * 启用后，每次 MPU6050 数据就绪会自动触发此回调。
 * 可在回调中置一个标志，主循环检测到标志后调用 IMU_Update()。
 *
 * @param   cb  回调函数（传 NULL 禁用中断模式）
 */
void IMU_SetDataReadyCallback(void (*cb)(void));

/**
 * @brief   启用或禁用 MPU6050 数据就绪中断
 * @param   enable  true = 使能, false = 禁用
 * @return  true = 操作成功
 */
bool IMU_EnableInterrupt(bool enable);

#endif /* APP_IMU_H */
