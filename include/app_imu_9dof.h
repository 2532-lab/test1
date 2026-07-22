/**
 * @file    app_imu_9dof.h
 * @brief   9-DOF AHRS 接口 — 互补滤波 Roll/Pitch + 磁力计修正 Yaw
 *
 * 组合 MPU6050（陀螺仪 + 加速度计，地址 0x68）与
 * 磁力计（自动鉴别 HMC5883L 地址 0x1E 或 QMC5883L 地址 0x0D，
 * 通过 MPU6050 AUX I2C Bypass 访问），实现无漂移的 3 轴姿态角输出。
 *
 * 芯片鉴别：Init 时先探 HMC5883L（地址 0x1E），失败则探 QMC5883L（地址 0x0D）。
 *
 * @warning 本文件依赖磁力计驱动（mid_hmc5883l / mid_qmc5883l）。
 *          如果不需要 9 轴融合，请使用 app_imu.h（纯 6-DOF 互补滤波）。
 */

#ifndef APP_IMU_9DOF_H
#define APP_IMU_9DOF_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *  枚举类型
 * ============================================================================ */

/** 磁力计芯片类型 */
typedef enum {
    IMU_MAG_CHIP_NONE     = 0,   /**< 未探测到磁力计 */
    IMU_MAG_CHIP_HMC5883L = 1,   /**< HMC5883L（地址 0x1E） */
    IMU_MAG_CHIP_QMC5883L = 2,   /**< QMC5883L（地址 0x0D） */
} IMU_MagChip_t;

/* ============================================================================
 *  数据结构
 * ============================================================================ */

/** 9-DOF 传感器数据与姿态角 */
typedef struct {
    float accel[3];       /**< 加速度（g），顺序 X/Y/Z */
    float gyro[3];        /**< 角速度（°/s），顺序 X/Y/Z */
    float mag[3];         /**< 磁力计（μT），已校准，顺序 X/Y/Z */
    float temperature;    /**< 温度（°C） */
    float roll;           /**< 横滚角（°），-180 ~ +180 */
    float pitch;          /**< 俯仰角（°），-90 ~ +90 */
    float yaw;            /**< 偏航角（°），0 ~ 360，磁力计修正无漂移 */
    float heading;        /**< 磁航向（°），0=磁北，纯磁力计倾角补偿后的原始值 */
} IMU_9DOF_Data_t;

/** 校准参数 */
typedef struct {
    float   gyro_offset[3];  /**< 陀螺零偏（°/s），顺序 X/Y/Z */
    float   mag_offset[3];   /**< 磁力计硬铁偏移（μT），顺序 X/Y/Z */
    float   mag_scale[3];    /**< 磁力计软铁缩放，顺序 X/Y/Z，默认 {1,1,1} */
    bool    gyro_calibrated; /**< 陀螺校准完成 */
    bool    mag_calibrated;  /**< 磁力计校准完成 */
} IMU_9DOF_Calib_t;

/** 滤波器参数 */
typedef struct {
    float alpha;       /**< Roll/Pitch 互补系数（默认 0.98，~0.25s @ 200Hz） */
    float alpha_yaw;   /**< Yaw 互补系数（默认 0.95，磁力计噪声较大需略低） */
    float dt;          /**< 滤波周期（秒，默认 0.005 = 200Hz） */
    float declination; /**< 磁偏角（°，正=东偏，中国大部分地区 -2~-11°） */
} IMU_9DOF_Filter_t;

/* ============================================================================
 *  API
 * ============================================================================ */

/**
 * @brief  初始化 9-DOF 系统（自动鉴别磁力计芯片型号）
 * @return true 所有传感器初始化成功
 *
 * 内部流程：
 *   1. MID_MPU6050_Init(NULL)  → MPU6050 硬件 + I2C1 配置
 *   2. 先探 HMC5883L（0x1E），成功则用 HMC 驱动
 *   3. 失败则探 QMC5883L（0x0D），成功则用 QMC 驱动
 *   4. 都失败 → 返回 false（可降级为纯 6-DOF 模式）
 */
bool IMU_9DOF_Init(void);

/**
 * @brief  获取当前磁力计芯片类型
 * @return 芯片枚举值（Init 之后有效）
 */
IMU_MagChip_t IMU_9DOF_GetMagChipType(void);

/**
 * @brief  校准陀螺仪零偏
 * @param  samples  采样点数（建议 ≥500）
 * @note   需保持传感器静止。内部调用 MID_MPU6050_ReadRaw。
 */
void IMU_9DOF_CalibrateGyro(uint16_t samples);

/**
 * @brief  校准磁力计硬铁偏移
 * @param  samples   采样点数（建议 ≥1000）
 * @param  delay_ms  每次采样间隔（ms）
 * @note   用户需在校准期间缓慢旋转设备，覆盖全方向。
 *         算法：采集每轴 min/max → offset = (max+min)/2，
 *         scale 归一化使三维球面半径一致。
 */
void IMU_9DOF_CalibrateMag(uint16_t samples, uint32_t delay_each_ms);

/**
 * @brief  9-DOF 数据更新（每帧调用一次）
 * @param  data  输出结构体（可为 NULL 则仅更新内部缓存）
 * @return true 读取并融合成功
 *
 * 融合流程：
 *   1. 读 MPU6050 → 转换为物理单位 → 减去陀螺零偏
 *   2. 读磁力计 → 转换为 μT → 减去硬铁偏移、乘软铁缩放
 *      若读取失败，yaw 仅靠陀螺积分过渡，不被动拉向 0°
 *   3. 加速度计算 roll_accel / pitch_accel
 *   4. 互补滤波：roll = α*(roll + gyro_x*dt) + (1-α)*roll_accel
 *   5. 磁力计倾角补偿 → 航向角 heading
 *   6. 互补滤波 yaw：yaw += gyro_z*dt + 误差修正*(1-α_yaw)
 *      （仅磁力计有效时做修正，失败则纯陀螺积分）
 */
bool IMU_9DOF_Update(IMU_9DOF_Data_t *data);

/**
 * @brief  设置滤波器参数
 * @param  alpha       Roll/Pitch 互补系数（0.90 ~ 0.995）
 * @param  alpha_yaw   Yaw 互补系数（0.90 ~ 0.995）
 * @param  dt          滤波周期（秒，须与实际调用频率一致）
 */
void IMU_9DOF_SetFilter(float alpha, float alpha_yaw, float dt);

/**
 * @brief  设置磁偏角（用于 true north 修正）
 * @param  declination_deg  磁偏角（°），正=东偏
 *
 * 中国主要城市磁偏角（2025 年近似值）：
 *   北京 -7°, 上海 -5°, 广州 -2°, 成都 -2°, 哈尔滨 -11°, 乌鲁木齐 +3°
 */
void IMU_9DOF_SetDeclination(float declination_deg);

/**
 * @brief  手动设置磁力计校准参数（跳过自动校准）
 * @param  offset  硬铁偏移 [3]（μT）
 * @param  scale   软铁缩放 [3]（默认 {1,1,1}）
 */
void IMU_9DOF_SetMagCalib(const float offset[3], const float scale[3]);

/** @brief 偏航角归零（当前朝向设为 0°） */
void IMU_9DOF_ResetYaw(void);

/** @brief 获取内部校准数据（可保存到 Flash 供下次启动使用） */
const IMU_9DOF_Calib_t *IMU_9DOF_GetCalibration(void);

#endif /* APP_IMU_9DOF_H */
