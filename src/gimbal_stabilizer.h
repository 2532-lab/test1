/**
 * @file    gimbal_stabilizer.h
 * @brief   云台增稳控制器 — IMU 姿态反馈 → 步进电机协同闭环控制 (头文件)
 *
 * ============================================================================
 * === ★ 模块定位 ===
 * ============================================================================
 *
 * 本模块是 MPU6050 姿态获取与云台步进电机的**协同控制桥梁**，
 * 实现 IMU 姿态反馈 → PID 闭环控制 → gimbal_control_angle() 的完整链路。
 *
 *    ┌──────────┐      ┌─────────────────┐      ┌──────────────┐
 *    │ app_imu  │ ───▶ │ gimbal_stabilizer│ ───▶ │gimbal_control│
 *    │ Roll/Pitch│     │  PID + 死区 +    │      │  _angle()    │
 *    │ Yaw      │      │  限幅 + 前馈     │      │  (非阻塞)     │
 *    └──────────┘      └─────────────────┘      └──────────────┘
 *
 * === ★ 三种工作模式 ===
 *
 *   [STABILIZE_HOLD]  增稳保持
 *     锁定当前姿态角, 持续抵抗外部扰动 (风吹/底盘倾斜)。
 *     类比: 相机三脚架模式 —— 云台主动抵消一切偏转。
 *
 *   [STABILIZE_TRACK] 目标追踪
 *     平滑跟踪外部给定的移动目标角度 (可来自上位机/CanMV/手动摇杆)。
 *     内置速度前馈 + EMA 平滑, 适合追踪缓慢移动的目标。
 *
 *   [STABILIZE_IDLE]  空闲
 *     关闭所有控制输出, 电机自由。用于初始化和紧急停止。
 *
 * === ★ 关键设计决策 ===
 *
 *   1. PID 输出的是**角度增量**, 叠加到目标角度后通过
 *      gimbal_control_angle() 下发。这样做的好处是:
 *      - 复用现有的非阻塞位置控制架构
 *      - ISR 自动完成步数计数和停止, 无需主循环干预
 *      - 步进电机的位置保持天然无累积误差
 *
 *   2. 死区 (deadband) 设在 0.15° (略大于单步 0.1125°),
 *      防止电机在目标位置附近来回微振 (hunting)。
 *
 *   3. 俯仰轴 (TILT) 映射 IMU Pitch, 水平轴 (PAN) 映射 IMU Yaw。
 *
 *   4. Roll 轴在 2 轴云台中无法直接补偿, 但 Roll 会影响 Pitch 测量:
 *      当 Roll 较大时, Pitch_accel 计算会出现耦合误差。
 *
 * === ★ 快速上手 (main.c 中的最小示例) ===
 *
 *   #include "app_imu.h"
 *   #include "gimbal_stabilizer.h"
 *
 *   int main(void) {
 *       SYSCFG_DL_init();
 *       gimbal_init();
 *       IMU_Init();
 *
 *       Stabilizer_Init();
 *       Stabilizer_SetMode(STABILIZE_HOLD);
 *
 *       while (1) {
 *           IMU_Data_t imu;
 *           if (IMU_Update(&imu)) {
 *               Stabilizer_Update(&imu);   // ★ 每 5ms 调用一次
 *           }
 *       }
 *   }
 *
 * ============================================================================
 */

#ifndef GIMBAL_STABILIZER_H
#define GIMBAL_STABILIZER_H

#include <stdint.h>
#include <stdbool.h>
#include "app_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  工作模式枚举
 * ============================================================================ */

/** 增稳控制器工作模式 */
typedef enum {
    STABILIZE_IDLE   = 0,   /**< 空闲: 无控制输出, 电机自由 */
    STABILIZE_HOLD   = 1,   /**< 增稳保持: 锁定目标角度, 抵抗扰动 */
    STABILIZE_TRACK  = 2,   /**< 目标追踪: 平滑跟踪移动目标 */
} StabilizeMode_t;

/* ============================================================================
 *  轴选择器
 * ============================================================================ */

/** 增稳轴索引 */
typedef enum {
    STAB_AXIS_PAN  = 0,     /**< 水平轴 (PAN) — 映射 IMU Yaw */
    STAB_AXIS_TILT = 1,     /**< 俯仰轴 (TILT) — 映射 IMU Pitch */
} StabilizeAxis_t;


/* ============================================================================
 *  PID 控制器结构体
 * ============================================================================ */

/**
 * @brief 单轴 PID 控制器
 *
 * 采用位置式 PID 算法 (带积分分离 + 抗饱和):
 *
 *   output = Kp * error
 *          + Ki * integral(error) * dt   (|error| < I_ZONE 时使能)
 *          + Kd * (error - last_error) / dt
 *
 * 积分项有硬限幅 (i_max), 防止 windup。
 * 输出有硬限幅 (out_max), 防止步进电机丢步。
 */
typedef struct {
    /* ── 增益 (需根据实际负载调优) ── */
    float Kp;               /**< 比例增益 (度/度 = 无量纲) */
    float Ki;               /**< 积分增益 (1/秒) */
    float Kd;               /**< 微分增益 (秒) */

    /* ── 限幅 ── */
    float out_max;          /**< 输出限幅 (±度), 单次 PID 输出的最大角度补偿 */
    float i_max;            /**< 积分项限幅 (±度), 防止 windup */

    /* ── 高级参数 ── */
    float i_zone;           /**< 积分分离区 (度), |error| > i_zone 时 Ki=0 */
    float deadband;         /**< 死区 (度), |error| < deadband 时输出 0 */

    /* ── 内部状态 (调用者无需关心) ── */
    float error;            /**< 当前误差 */
    float last_error;       /**< 上一次误差 (调试用) */
    float last_measured;    /**< 上一次测量值 (微分项 = -Kd * Δmeasured / dt) */
    float integral;         /**< 积分累加值 */
    float output;           /**< 当前 PID 输出 */
    float dt;               /**< 控制周期 (秒) */
} StabilizerPID_t;


/* ============================================================================
 *  增稳控制器状态结构体
 * ============================================================================ */

/**
 * @brief 单轴增稳控制状态
 *
 * 包含 PID 控制器 + 目标追踪 + EMA 滤波的全部状态。
 */
typedef struct {
    StabilizerPID_t pid;            /**< PID 控制器 */

    float target_angle;             /**< 目标绝对角度 (度), 用户设置 */
    float current_angle;            /**< 当前 IMU 测量角度 (度) */
    float output_angle;             /**< 下发到 gimbal_control 的角度 (度) */

    /* ── 追踪模式 EMA 平滑 ── */
    float ema_target;               /**< EMA 平滑后的目标角度 */
    float ema_alpha;                /**< 追踪 EMA 系数 (0~1) */

    /* ── 状态记录 ── */
    uint32_t last_update_ms;        /**< 上次更新时间戳 */
    bool     active;                /**< 轴是否处于活跃控制状态 */
} StabilizerAxis_t;


/* ============================================================================
 *  默认 PID 参数表 (实测调优起点)
 * ============================================================================
 *
 * 这些默认值已经过滤波和限幅, 可直接使用。
 * 调优顺序: Kp → Kd → Ki (按此顺序, 每次只改一个参数)。
 *
 *   现象                         | 原因     | 调整
 *   ────────────────────────────|─────────|──────────
 *   云台缓慢到达目标, 不振荡      | Kp 太小  | ↑ Kp × 1.5
 *   云台在目标附近快速来回摆动     | Kp 太大  | ↓ Kp × 0.7
 *   云台到达目标后过冲再回来       | Kd 太小  | ↑ Kd × 1.5
 *   云台到达目标前就明显减速       | Kd 太大  | ↓ Kd × 0.7
 *   云台长期有小角度静差           | Ki 太小  | ↑ Ki × 2.0
 *   云台低频振荡 (周期 > 1s)      | Ki 太大  | ↓ Ki × 0.5
 */

/* ── PAN 轴 (水平) 默认 PID 参数 ── */
#define STAB_PAN_KP_DEFAULT         1.2f    /**< P: 中等响应, 无振荡 */
#define STAB_PAN_KI_DEFAULT         0.15f   /**< I: 消除静差, 不引起低频振荡 */
#define STAB_PAN_KD_DEFAULT         0.35f   /**< D: 适度阻尼, 抑制过冲 */
#define STAB_PAN_OUT_MAX_DEFAULT    3.0f    /**< 单次最大补偿 3° */
#define STAB_PAN_I_MAX_DEFAULT      2.0f    /**< 积分上限 2° */
#define STAB_PAN_I_ZONE_DEFAULT     5.0f    /**< 误差 < 5° 时积分才累加 */
#define STAB_PAN_DEADBAND_DEFAULT   0.15f   /**< 死区 0.15° (略大于步距 0.1125°) */

/* ── TILT 轴 (俯仰) 默认 PID 参数 ── */
#define STAB_TILT_KP_DEFAULT        1.2f
#define STAB_TILT_KI_DEFAULT        0.15f
#define STAB_TILT_KD_DEFAULT        0.35f
#define STAB_TILT_OUT_MAX_DEFAULT   3.0f
#define STAB_TILT_I_MAX_DEFAULT     2.0f
#define STAB_TILT_I_ZONE_DEFAULT    5.0f
#define STAB_TILT_DEADBAND_DEFAULT  0.15f


/* ============================================================================
 *  追踪模式默认参数
 * ============================================================================ */

/** 追踪 EMA 系数 (0~1), 越大响应越快, 默认 0.3 = 约 150ms 平滑 */
#define STAB_TRACK_EMA_DEFAULT      0.30f

/** 追踪模式下 PID 更新间隔 (ms), 更快更新以跟上移动目标 */
#define STAB_TRACK_UPDATE_MS        10

/** 增稳保持模式 PID 更新间隔 (ms), 稍慢以减少 I2C 开销 */
#define STAB_HOLD_UPDATE_MS          5


/* ============================================================================
 *  安全限幅参数
 * ============================================================================ */

/** 绝对角度硬限幅 (±度), 防止软件 bug 导致机械碰撞 */
#define STAB_PAN_ANGLE_HARD_LIMIT   170.0f
#define STAB_TILT_ANGLE_HARD_LIMIT  90.0f

/** 单次角度增量硬限幅 (±度), 防止数据异常突变 */
#define STAB_DELTA_HARD_LIMIT        10.0f


/* ============================================================================
 *  初始化
 * ============================================================================ */

/**
 * @brief   初始化增稳控制器
 *
 * 将两轴 PID 设为默认参数, 清零所有状态, 模式设为 STABILIZE_IDLE。
 * 必须在 gimbal_init() 和 IMU_Init() 之后调用。
 *
 * @note    初始化后需调用 Stabilizer_SetMode(STABILIZE_HOLD) 激活增稳
 */
void Stabilizer_Init(void);


/* ============================================================================
 *  核心 API — 控制循环 (★ 主循环中周期性调用)
 * ============================================================================ */

/**
 * @brief   增稳控制核心 — 将 IMU 数据转换为云台角度指令 (★ 核心函数)
 *
 * 这是整个增稳模块的核心, 应在主循环中以固定频率调用 (5~10ms 间隔)。
 *
 * 内部流程:
 *   [HOLD 模式]
 *     1. 读取 IMU 姿态 (pitch → TILT, yaw → PAN)
 *     2. 计算误差: error = target_angle - current_angle
 *     3. PID 计算角度修正量
 *     4. 死区检查: |error| < deadband → 跳过
 *     5. 叠加修正: output = target + pid_output
 *     6. 角度限幅后调用 gimbal_control_angle()
 *
 *   [TRACK 模式]
 *     1. 外部 set_target 的目标角度经 EMA 平滑
 *     2. 平滑后的目标作为 PID 的 setpoint
 *     3. 其余流程同 HOLD 模式
 *
 *   [IDLE 模式]
 *     不执行任何控制, 直接返回。
 *
 * @param   imu    IMU_Update() 输出的姿态数据指针 (不可为 NULL)
 *
 * @retval  true  = 控制循环正常执行
 * @retval  false = IMU 数据无效 (s_initialized == false) 或 IDLE 模式
 *
 * @note    调用频率: 100~200Hz (5~10ms), 需与 PID.dt 一致
 * @note    内部调用 gimbal_control_angle(), 非阻塞立即返回
 * @note    IDLE 模式下本函数几乎零开销 (仅一次 if 判断)
 */
bool Stabilizer_Update(const IMU_Data_t *imu);


/* ============================================================================
 *  模式切换
 * ============================================================================ */

/**
 * @brief   切换增稳工作模式
 *
 * 模式切换时的行为:
 *   → HOLD:  将当前 IMU 姿态角锁定为目标角度 (保持当前位置)
 *   → TRACK: 保持当前目标角度不变, 等待 Stabilizer_SetTarget() 下发新目标
 *   → IDLE:  调用 gimbal_stop() 停止所有电机
 *
 * @param   mode   目标模式 (STABILIZE_HOLD / STABILIZE_TRACK / STABILIZE_IDLE)
 */
void Stabilizer_SetMode(StabilizeMode_t mode);

/**
 * @brief   获取当前工作模式
 * @return  StabilizeMode_t 枚举值
 */
StabilizeMode_t Stabilizer_GetMode(void);


/* ============================================================================
 *  目标角度设置 (TRACK 模式)
 * ============================================================================ */

/**
 * @brief   设置追踪目标角度 (TRACK 模式下使用)
 *
 * 目标角度经 EMA 平滑后作为 PID 的 setpoint,
 * 云台会平滑过渡到目标角度。
 *
 * @param   pan_deg    水平目标角度 (度), [-85°, +85°] 推荐
 * @param   tilt_deg   俯仰目标角度 (度), [-45°, +45°] 推荐
 *
 * @note    仅在 TRACK 模式下有效; HOLD/IDLE 模式下忽略
 * @note    多次调用会逐帧更新 EMA 平滑目标
 */
void Stabilizer_SetTarget(float pan_deg, float tilt_deg);

/**
 * @brief   立即跳转到目标角度 (绕过 EMA 平滑, HOLD 模式下使用)
 *
 * HOLD 模式下调用此函数可改变增稳锁定位置。
 * 云台将使用 PID 控制平滑到达新位置。
 *
 * @param   pan_deg    水平目标角度 (度)
 * @param   tilt_deg   俯仰目标角度 (度)
 */
void Stabilizer_SetTargetInstant(float pan_deg, float tilt_deg);


/* ============================================================================
 *  PID 参数调整 (在线调参)
 * ============================================================================ */

/**
 * @brief   设置指定轴的 PID 增益
 *
 * @param   axis   轴选择 (STAB_AXIS_PAN / STAB_AXIS_TILT)
 * @param   Kp     比例增益
 * @param   Ki     积分增益
 * @param   Kd     微分增益
 *
 * @note    可在运行时动态调整, 无需重新初始化
 */
void Stabilizer_SetPID(StabilizeAxis_t axis, float Kp, float Ki, float Kd);

/**
 * @brief   设置指定轴的 PID 限幅与死区参数
 *
 * @param   axis       轴选择
 * @param   out_max    输出限幅 (±度)
 * @param   i_max      积分限幅 (±度)
 * @param   i_zone     积分分离区 (度)
 * @param   deadband   死区 (度)
 */
void Stabilizer_SetPIDLimits(StabilizeAxis_t axis,
                             float out_max, float i_max,
                             float i_zone, float deadband);

/**
 * @brief   获取指定轴的 PID 参数 (用于调试/显示)
 * @param   axis   轴选择
 * @return  指向只读 PID 结构体的指针
 */
const StabilizerPID_t *Stabilizer_GetPID(StabilizeAxis_t axis);


/* ============================================================================
 *  紧急停止与状态查询
 * ============================================================================ */

/**
 * @brief   紧急停止 — 立即终止所有电机运动, 切换到 IDLE 模式
 *
 * 调用 gimbal_stop() 后切换到 IDLE, 重置 PID 积分项。
 * 恢复增稳需重新调用 Stabilizer_SetMode(HOLD 或 TRACK)。
 */
void Stabilizer_EmergencyStop(void);

/**
 * @brief   查询增稳控制器是否处于活跃状态
 * @retval  true = HOLD 或 TRACK 模式
 */
bool Stabilizer_IsActive(void);

/**
 * @brief   查询是否有任何轴在运动中
 * @retval  true = stepper_pulse 层报告至少一轴 busy
 */
bool Stabilizer_IsMoving(void);

/**
 * @brief   查询指定轴的 PID 误差 (调试用)
 * @param   axis   轴选择
 * @return  当前误差值 (度)
 */
float Stabilizer_GetError(StabilizeAxis_t axis);

/**
 * @brief   查询指定轴的 PID 输出 (调试用)
 * @param   axis   轴选择
 * @return  PID 输出值 (度)
 */
float Stabilizer_GetOutput(StabilizeAxis_t axis);


#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_STABILIZER_H */
