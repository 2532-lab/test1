/**
 * @file    app_imu.h
 * @brief   应用层 — IMU 姿态解算与数据融合 (头文件)
 *
 * 本层是三层架构的最顶层，负责 MPU6050 数据的姿态解算与融合。
 * 核心算法：互补滤波器 (Complementary Filter)
 *   融合加速度计（低频可靠）和陀螺仪（高频可靠）的测量值，
 *   估计 Roll/Pitch 姿态角。
 *
 * 调用链:
 *   main() → IMU_Init() → MID_MPU6050_Init() → HAL_MPU6050_Init()
 *          → IMU_Update() → MID_MPU6050_ReadRaw() → HAL_MPU6050_ReadMulti()
 *
 * 典型用法:
 *   IMU_Init();
 *   IMU_CalibrateGyro(500);   // 静止采样 500 次校准陀螺仪零偏
 *   IMU_CalibrateAccel(200);  // 静止采样 200 次校准加速度计零偏
 *   while(1) {
 *       IMU_Data_t data;
 *       IMU_Update(&data);    // 更新姿态 (200Hz)
 *       // data.roll, data.pitch 即当前姿态角
 *   }
 *
 * 依赖:
 *   - mid_mpu6050.h → hal_mpu6050.h → ti_msp_dl_config.h
 *   - math.h (atan2f, sqrtf)
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  数据结构
 * ============================================================================ */

/**
 * @brief IMU 姿态与传感器数据（更新后输出）
 *
 * 角度定义 (右手定则):
 *   Roll  (绕 X 轴): 右倾为正, 范围 [-180°, +180°]
 *   Pitch (绕 Y 轴): 抬头为正, 范围 [-90°,  +90°]
 *   Yaw   (绕 Z 轴): 右转为正, 范围 [0°,    360°] (仅陀螺仪积分，必然漂移)
 */
typedef struct {
    float roll;         /**< 横滚角 (度) */
    float pitch;        /**< 俯仰角 (度) */
    float yaw;          /**< 偏航角 (度), 仅陀螺仪积分，无磁力计修正 */
    float accel_x;      /**< 加速度 X (g), 扣除校准零偏 */
    float accel_y;      /**< 加速度 Y (g) */
    float accel_z;      /**< 加速度 Z (g) */
    float gyro_x;       /**< 角速度 X (°/s), 扣除校准零偏 */
    float gyro_y;       /**< 角速度 Y (°/s) */
    float gyro_z;       /**< 角速度 Z (°/s) */
    float temperature;  /**< 芯片温度 (°C) */
} IMU_Data_t;

/**
 * @brief 互补滤波器参数
 *
 * 互补滤波公式:
 *   angle[n] = alpha * (angle[n-1] + gyro * dt) + (1 - alpha) * accel_angle
 *
 * alpha 越大 → 越信任陀螺仪（响应快但容易漂移）
 * alpha 越小 → 越信任加速度计（稳定但噪声大）
 * dt   为滤波周期 (秒), 应与实际 IMU_Update() 调用间隔匹配
 */
typedef struct {
    float alpha;    /**< 互补滤波系数, 典型值 0.96~0.995, 默认 0.98 */
    float dt;       /**< 滤波周期 (秒), 默认 0.005 (200Hz) */
} IMU_Filter_t;

/**
 * @brief IMU 校准数据
 *
 * 校准应在传感器静止、水平放置时进行。
 * 陀螺仪零偏: 静止时角速度输出的均值
 * 加速度计零偏: 静止时加速度输出与 [0, 0, 1g] 的偏差
 */
typedef struct {
    float gyro_offset_x;    /**< 陀螺仪 X 零偏 (°/s) */
    float gyro_offset_y;    /**< 陀螺仪 Y 零偏 (°/s) */
    float gyro_offset_z;    /**< 陀螺仪 Z 零偏 (°/s) */
    float accel_offset_x;   /**< 加速度计 X 零偏 (g) */
    float accel_offset_y;   /**< 加速度计 Y 零偏 (g) */
    float accel_offset_z;   /**< 加速度计 Z 零偏 (g), 静止时读数为 1g */
    bool  calibrated;       /**< 加速度计是否已校准 */
} IMU_Calib_t;


/* ============================================================================
 *  初始化与校准
 * ============================================================================ */

/**
 * @brief   初始化 IMU 模块
 *
 * 内部调用 MID_MPU6050_Init() 完成:
 *   1. HAL_MPU6050_Init() — I2C1 控制器 + INT 引脚配置
 *   2. MPU6050 复位 + 唤醒 + 时钟源 (PLL Z-gyro)
 *   3. 量程配置: 陀螺仪 ±1000°/s, 加速度 ±4g
 *   4. DLPF 44Hz, 采样率 200Hz (SMPLRT_DIV=4)
 *
 * @return  true = 初始化成功, false = 器件未检测到或配置失败
 *
 * @note   必须在 SYSCFG_DL_init() 之后调用
 */
bool IMU_Init(void);

/**
 * @brief   校准陀螺仪零偏
 *
 * 在传感器**完全静止**时调用, 采样 `samples` 次角速度数据,
 * 计算均值作为零偏, 存入 s_calib.gyro_offset_*。
 *
 * @param   samples   采样次数 (建议 200~1000, 越多越准确)
 *
 * @note    校准期间阻塞约 samples × 5ms (200Hz 采样)
 * @note    校准完成后 IMU_Update() 自动扣除零偏
 */
void IMU_CalibrateGyro(uint16_t samples);

/**
 * @brief   校准加速度计零偏
 *
 * 在传感器**水平静止**时调用, 采样 `samples` 次加速度数据,
 * 计算 X/Y 均值作为零偏, Z 轴均值 - 1g 作为 Z 零偏。
 *
 * @param   samples   采样次数 (建议 100~500)
 *
 * @note    校准期间阻塞约 samples × 5ms
 * @note    校准后 s_calib.calibrated = true
 */
void IMU_CalibrateAccel(uint16_t samples);


/* ============================================================================
 *  核心 API
 * ============================================================================ */

/**
 * @brief   更新 IMU 姿态数据 (★ 核心函数, 每次主循环调用)
 *
 * 执行流程:
 *   1. MID_MPU6050_ReadRaw() — 一次 I2C 读取 14 字节原始数据
 *   2. LSB → 物理单位, 扣除校准零偏
 *   3. 加速度计计算 Roll_accel / Pitch_accel
 *   4. 互补滤波融合: angle = alpha*(angle + gyro*dt) + (1-alpha)*accel
 *   5. 角度归一化 (Roll: ±180°, Pitch: ±90°, Yaw: 0~360°)
 *   6. 填充 IMU_Data_t 输出
 *
 * @param   data    [out] 存放姿态数据, 传 NULL 仅更新内部缓存
 * @return  true = 更新成功, false = I2C 读取失败或模块未初始化
 *
 * @note    调用频率应与 s_filter.dt 匹配: 默认 200Hz = 每 5ms
 * @note    I2C 通信耗时约 200μs @ 400kHz, 不影响主循环实时性
 * @note    此函数不阻塞, I2C 超时时返回 false
 */
bool IMU_Update(IMU_Data_t *data);

/**
 * @brief   获取最近一次 IMU_Update() 的缓存数据
 *
 * 不触发新的 I2C 读取, 直接返回 s_data 的副本。
 * 适合多个模块需要读取姿态数据的场景 (如显示 + 控制)。
 *
 * @param   data    [out] 存放姿态数据
 */
void IMU_GetData(IMU_Data_t *data);


/* ============================================================================
 *  滤波器配置
 * ============================================================================ */

/**
 * @brief   设置互补滤波器参数
 *
 * @param   alpha   互补滤波系数 [0.90, 0.995], 越小越平滑但响应慢
 * @param   dt      滤波周期 (秒) [0.001, 0.1]
 *
 * 推荐组合:
 *   - 高速响应: alpha=0.96, dt=0.005  (适合快速追踪)
 *   - 平衡模式: alpha=0.98, dt=0.005  (默认, 适合一般场景)
 *   - 高稳定性: alpha=0.995, dt=0.005 (适合静态保持)
 */
void IMU_SetFilter(float alpha, float dt);


/* ============================================================================
 *  辅助函数
 * ============================================================================ */

/**
 * @brief   复位偏航角为零
 *
 * 将 s_data.yaw 清零, 后续仅靠陀螺仪积分。
 * 无磁力计修正时 Yaw 会在数分钟内漂移数十度。
 */
void IMU_ResetYaw(void);

/**
 * @brief   查询 IMU 模块是否已成功初始化
 * @return  true = IMU_Init() 已成功执行
 */
bool IMU_IsReady(void);

/**
 * @brief   获取校准数据指针 (只读)
 * @return  指向内部 s_calib 的 const 指针
 */
const IMU_Calib_t *IMU_GetCalibration(void);


/* ============================================================================
 *  中断支持 (可选, 低功耗场景)
 * ============================================================================ */

/**
 * @brief   注册 MPU6050 数据就绪中断回调
 *
 * 注册后, 每次 MPU6050 数据就绪 (INT 引脚上升沿) 时自动调用 cb。
 * 适合低功耗场景: 主循环休眠, 中断唤醒后读取数据。
 *
 * @param   cb   回调函数指针 (传 NULL 取消注册)
 *
 * @note    需先调用 IMU_EnableInterrupt(true) 使能 MPU6050 内部中断
 */
void IMU_SetDataReadyCallback(void (*cb)(void));

/**
 * @brief   使能或禁用 MPU6050 数据就绪中断
 *
 * 内部调用 MID_MPU6050_EnableDataReadyInt() 配置 MPU6050 寄存器,
 * 并注册/注销 HAL 层 GPIO 中断回调。
 *
 * @param   enable   true = 使能, false = 禁用
 * @return  true = 成功
 */
bool IMU_EnableInterrupt(bool enable);


#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H */
