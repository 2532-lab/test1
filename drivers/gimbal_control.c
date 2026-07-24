/*
 * gimbal_control.c — 二轴步进电机云台控制 (实现)
 *
 * 目标芯片:    MSPM0G3507 (仅使用 TI DriverLib，严禁 STM32 HAL / 标准库)
 * 依赖模块:    stepper_pulse.h → 脉冲+方向+步数计数 底层控制
 *
 * === ★ 双模式架构 ===
 *
 *   [角度/位置模式] gimbal_control_angle()
 *        │
 *        ▼
 *   ┌──────────────────────────────────────────────┐
 *   │ gimbal_control_angle(pan_deg, tilt_deg)       │
 *   │                                              │
 *   │ 1. 读取当前角度 (stepper_pulse_get_pos_deg)   │
 *   │ 2. 计算 delta = target - current              │
 *   │ 3. 角度 → 步数: steps = |delta| * PPR / 360   │
 *   │ 4. stepper_pulse_move(axis, dir, rpm, steps)  │
 *   │ 5. 更新 target_angle, 立即返回                 │
 *   │                                              │
 *   │ ISR 在后台计数, 到达目标步数后自动停止          │
 *   │ 主循环非阻塞轮询 gimbal_is_moving()            │
 *   └──────────────────────────────────────────────┘
 *
 *   [速度追踪模式] gimbal_track_velocity()
 *        │
 *   外部偏差输入 (50Hz)
 *        │
 *        ▼
 *   ┌──────────────────────────────────────────────┐
 *   │ gimbal_track_velocity                         │
 *   │                                              │
 *   │ 0. 超时检测        │ >200ms 无数据 → error=0  │
 *   │ 1. 死区过滤        │ error < deadband → 0     │
 *   │ 2. 非对称 EMA 滤波 │ 加速 α=0.15 / 减速 α=0.30│
 *   │ 3. P 控制器        │ target_rpm = Kp × |ema|  │
 *   │ 4. RPM 变化率限制  │ 防突变丢步 (200RPM/帧)   │
 *   │ 5. 原始误差定方向  │ ★ 无 EMA 滞后!           │
 *   │ 6. stepper_pulse_  │ 同方向不停止 → 丝滑调频! │
 *   │    set_speed()     │                          │
 *   └──────────────────────────────────────────────┘
 *
 * === 与旧版的区别 ===
 *   v1: 阻塞式 velocity 模式, delay_ms() 等待 → main() 卡死
 *   v2: gimbal_track_velocity 速度追踪 → 丝滑但需定时调用
 *   v3: v2 + 非对称 EMA + 原始误差定方向 + 内置超时
 *   ★ v4: gimbal_control_angle 位置模式 → 非阻塞, ISR 自动停止,
 *         适合小车系统集成。gimbal_control_velocity → @deprecated。
 */

#include "gimbal_control.h"
#include <math.h>
#include <stdlib.h>

#define ROUND(x)  ((int32_t)((x) + 0.5f))

/* ==========================================================================
 * 全局轴实例
 * ========================================================================== */
gimbal_axis_t gimbal_pan = {
    .angle         = 0.0f,//当前软件估算角度
    .target_angle  = 0.0f,//目标绝对角度
    .motor_addr    = STEPPER_AXIS_PAN,
    .uart          = NULL,
    .rpm           = STEPPER_PULSE_DEFAULT_RPM,
    .accel         = 0,
    .angle_limit   = GIMBAL_PAN_ANGLE_MAX,
    .target_point  = 0,
    .current_point  = 0,
    .ema_error     = 0.0f,
    .current_rpm   = 0,
    .last_data_ms  = 0,
};

gimbal_axis_t gimbal_tilt = {
    .angle         = 0.0f,
    .target_angle  = 0.0f,
    .motor_addr    = STEPPER_AXIS_TILT,
    .uart          = NULL,
    .rpm           = STEPPER_PULSE_DEFAULT_RPM,
    .accel         = 0,
    .angle_limit   = GIMBAL_TILT_ANGLE_MAX,
    .target_point  = 0,
    .current_point  = 0,
    .ema_error     = 0.0f,
    .current_rpm   = 0,
    .last_data_ms  = 0,
};

/* ==========================================================================
 * 工具: clamp 函数
 * ========================================================================== */

/**
 * @brief   将浮点值钳位到 [lo, hi] 区间, NaN 安全。
 * @param   val   输入值。
 * @param   lo    下限。
 * @param   hi    上限。
 * @retval  钳位后的值; NaN → lo (安全回退)。
 */
static inline float clampf(float val, float lo, float hi)
{
    /* ★ NaN 守卫: NaN 与任何值比较均为 false, 会穿透 if-else 链原样返回,
     *   污染下游计算。在此强制返回 lo。 */
    if (val != val) return lo;
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief   将 int16_t 值钳位到 [lo, hi] 区间。
 * @param   val   输入值。
 * @param   lo    下限。
 * @param   hi    上限。
 * @retval  钳位后的值。
 */
static inline int16_t clampi16(int16_t val, int16_t lo, int16_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ==========================================================================
 * gimbal_init — 初始化双轴
 * ========================================================================== */

/**
 * @brief   初始化双轴云台控制模块。
 *
 * 必须在 SYSCFG_DL_init() 之后调用一次。
 * 内部调用 stepper_pulse_init() 配置 DIR 引脚 + 使能步数计数 ISR,
 * 并重置 PAN/TILT 两轴的追踪状态 (EMA、RPM、时间戳、角度)。
 */
void gimbal_init(void)
{
    stepper_pulse_init();

    /* 重置追踪状态 */
    gimbal_pan.ema_error    = 0.0f;
    gimbal_pan.current_rpm  = 0;
    gimbal_pan.last_data_ms = 0;
    gimbal_pan.angle        = 0.0f;
    gimbal_pan.target_angle = 0.0f;
    gimbal_tilt.ema_error    = 0.0f;
    gimbal_tilt.current_rpm  = 0;
    gimbal_tilt.last_data_ms = 0;
    gimbal_tilt.angle        = 0.0f;
    gimbal_tilt.target_angle = 0.0f;
}

/**
 * @brief   标记当前双轴位置为逻辑零点。
 *
 * 清零硬件步数计数器 + 软件角度值。
 * 通常在初始标定或需要重新建立位置基准时调用。刚上电的那一刻即为0点
 */
void gimbal_reset_zero(void)
{
    stepper_pulse_set_position_zero(STEPPER_AXIS_PAN);
    stepper_pulse_set_position_zero(STEPPER_AXIS_TILT);

    gimbal_pan.angle   = 0.0f;
    gimbal_tilt.angle  = 0.0f;
}

/**
 * @brief   兼容接口 — 标记当前位置为零点。
 * @param   store   保留参数 (当前未使用, 始终清零)。
 */
void gimbal_set_zero(bool store)
{
    (void)store;
    gimbal_reset_zero();
}

/* ==========================================================================
 * ★ gimbal_control_angle — 绝对角度位置控制 (非阻塞)
 * ==========================================================================
 *
 * 这是替代 gimbal_control_velocity() 的新位置控制 API。
 *
 * 处理流程:
 *   1. 读取当前位置 (从硬件步数计数器)
 *   2. 计算角度差: delta = target - current
 *   3. 死区: |delta| <= GIMBAL_ANGLE_DEADBAND → 跳过
 *   4. 角度 → 步数: steps = |delta| * PPR / 360
 *   5. 判断方向: delta > 0 → CCW, delta < 0 → CW
 *   6. 调用 stepper_pulse_move() 下发运动 (立即返回!)
 *   7. 更新 target_angle → angle (软件跟踪)
 *
 * @param   pan_deg    水平目标角度 (度), 范围 [-GIMBAL_PAN_ANGLE_MAX, +GIMBAL_PAN_ANGLE_MAX]。
 * @param   tilt_deg   俯仰目标角度 (度), 范围 [-GIMBAL_TILT_ANGLE_MAX, +GIMBAL_TILT_ANGLE_MAX]。
 *
 * @note    如果轴正在运动中, stepper_pulse_move() 会先停止再重启。
 * @note    本函数立即返回, 不等待运动完成; 主循环用 gimbal_is_moving() 轮询。
 * @note    死区检查: |delta| <= GIMBAL_ANGLE_DEADBAND 时跳过, 防止亚步抖动。
 */
void gimbal_control_angle(float pan_deg, float tilt_deg)
{
    float current_pan, current_tilt;
    float delta_pan, delta_tilt;
    uint32_t steps;
    uint8_t dir;
    uint16_t rpm;

    /* ★ v7.1+v9 NaN 纵深防御 (双层):
     *   Layer1 (此处): 入口显式拦截 NaN/Inf — 防御性编程, 即使 clampf 失效也安全
     *   Layer2 (clampf L105): val!=val→lo, 同样拦截 NaN
     *   两层独立防护, 任一有效即可保证安全。 */
    if (pan_deg != pan_deg || tilt_deg != tilt_deg) return;
    if (pan_deg > 1e8f || pan_deg < -1e8f) return;
    if (tilt_deg > 1e8f || tilt_deg < -1e8f) return;

    /* ═══════════════════════════════════════════════════════════════════
     * PAN 轴 (水平)
     * ═══════════════════════════════════════════════════════════════════ */

    /* ── 钳位目标角度 ── */
    pan_deg = clampf(pan_deg, -gimbal_pan.angle_limit, gimbal_pan.angle_limit);

    /* ── 1. 读取当前位置 ── */
    current_pan = stepper_pulse_get_position_deg(STEPPER_AXIS_PAN);

    /* ── 2. 计算角度差 ── */
    delta_pan = pan_deg - current_pan;

    /* ── 3. 死区: |delta| <= 0.12° (略大于单步 360/3200=0.1125°) ── */
    if (fabsf(delta_pan) > GIMBAL_ANGLE_DEADBAND && !gimbal_pan.fault) {
        /* ── 4. 角度 → 步数 (四舍五入, 消除截断累积误差) ── */
        steps = (uint32_t)ROUND(fabsf(delta_pan) * (float)STEPPER_PULSES_PER_REV
                                / GIMBAL_DEG_PER_REV);
        if (steps > 0) {
            /* ── 5. 方向 ── */
            dir = (delta_pan > 0.0f) ? STEPPER_DIR_CCW : STEPPER_DIR_CW;
            rpm = gimbal_pan.rpm;
            if (rpm == 0) rpm = GIMBAL_ANGLE_DEFAULT_RPM;

            /*
             * ★ 5.5 提前读取当前位置 (必须在 stepper_pulse_move 之前!)
             *
             * 原因: stepper_pulse_move() 启动定时器后 ISR 可能在微秒内触发并更新
             * g_position_steps。若读在 move 之后, ISR 已会计入若干步, 再加上
             * 预测步数 → 位置被双重计数, 产生 ~1-2 步的累积偏移。
             * 先读后发: pos 是下发时刻的确切位置, +预测步数 = 正确终止位置。
             */
            int32_t pos = stepper_pulse_get_position_steps(STEPPER_AXIS_PAN);

            /* ── 6. 下发运动 (非阻塞) ── */
            stepper_pulse_move(STEPPER_AXIS_PAN, dir, rpm, steps);

            /* ── 7. 更新软件状态 (基于下发时刻位置 + 预测步数) ── */
            gimbal_pan.target_angle = pan_deg;
            {
                pos += (dir == STEPPER_DIR_CCW) ? (int32_t)steps : -(int32_t)steps;
                gimbal_pan.angle = (float)pos * GIMBAL_DEG_PER_REV
                                   / (float)STEPPER_PULSES_PER_REV;
            }
        }
    }

    /* ═══════════════════════════════════════════════════════════════════
     * TILT 轴 (垂直)
     * ═══════════════════════════════════════════════════════════════════ */

    /* ── 钳位目标角度 ── */
    tilt_deg = clampf(tilt_deg, -gimbal_tilt.angle_limit, gimbal_tilt.angle_limit);

    /* ── 1. 读取当前位置 ── */
    current_tilt = stepper_pulse_get_position_deg(STEPPER_AXIS_TILT);

    /* ── 2. 计算角度差 ── */
    delta_tilt = tilt_deg - current_tilt;

    /* ── 3. 死区: |delta| <= GIMBAL_ANGLE_DEADBAND (略大于单步 360/3200=0.1125°) ── */
    if (fabsf(delta_tilt) > GIMBAL_ANGLE_DEADBAND && !gimbal_tilt.fault) {
        /* ── 4. 角度 → 步数 (四舍五入, 消除截断累积误差) ── */
        steps = (uint32_t)ROUND(fabsf(delta_tilt) * (float)STEPPER_PULSES_PER_REV
                                / GIMBAL_DEG_PER_REV);
        if (steps > 0) {
            /* ── 5. 方向 ── */
            dir = (delta_tilt > 0.0f) ? STEPPER_DIR_CCW : STEPPER_DIR_CW;
            rpm = gimbal_tilt.rpm;
            if (rpm == 0) rpm = GIMBAL_ANGLE_DEFAULT_RPM;

            /*
             * ★ 5.5 提前读取当前位置 (必须在 stepper_pulse_move 之前!)
             *    理由同 PAN 轴注释 — 防 ISR 竞态导致位置双重计数。
             */
            int32_t pos = stepper_pulse_get_position_steps(STEPPER_AXIS_TILT);

            /* ── 6. 下发运动 (非阻塞) ── */
            stepper_pulse_move(STEPPER_AXIS_TILT, dir, rpm, steps);

            /* ── 7. 更新软件状态 (基于下发时刻位置 + 预测步数) ── */
            gimbal_tilt.target_angle = tilt_deg;
            {
                pos += (dir == STEPPER_DIR_CCW) ? (int32_t)steps : -(int32_t)steps;
                gimbal_tilt.angle = (float)pos * GIMBAL_DEG_PER_REV
                                    / (float)STEPPER_PULSES_PER_REV;
            }
        }
    }
}

/**
 * @brief   单轴水平角度控制 (保持 TILT 当前位置不变)。
 * @param   pan_deg   水平目标角度 (度)。
 */
void gimbal_control_pan_only(float pan_deg)
{
    /* 保持 TILT 当前位置不变 */
    float current_tilt = stepper_pulse_get_position_deg(STEPPER_AXIS_TILT);
    gimbal_control_angle(pan_deg, current_tilt);
}

/**
 * @brief   单轴俯仰角度控制 (保持 PAN 当前位置不变)。
 * @param   tilt_deg  俯仰目标角度 (度)。
 */
void gimbal_control_tilt_only(float tilt_deg)
{
    /* 保持 PAN 当前位置不变 */
    float current_pan = stepper_pulse_get_position_deg(STEPPER_AXIS_PAN);
    gimbal_control_angle(current_pan, tilt_deg);
}

/* ==========================================================================
 * gimbal_is_moving — 查询是否有轴在运动中
 * ========================================================================== */
/**
 * @brief   查询是否有任何轴正在运动中。
 * @retval  true 表示至少一轴正在执行 stepper_pulse_move() 或速度模式未停止。
 */
bool gimbal_is_moving(void)
{
    return stepper_pulse_is_busy(STEPPER_AXIS_PAN)
        || stepper_pulse_is_busy(STEPPER_AXIS_TILT);
}

/* ==========================================================================
 * 角度查询函数
 * ========================================================================== */

/**
 * @brief   查询 PAN 轴当前角度 (从硬件步数计数器读取)。
 * @retval  角度值 (度), 分辨率 0.1125°。
 */
float gimbal_get_current_angle_pan(void)
{
    return stepper_pulse_get_position_deg(STEPPER_AXIS_PAN);
}

/**
 * @brief   查询 TILT 轴当前角度 (从硬件步数计数器读取)。
 * @retval  角度值 (度), 分辨率 0.1125°。
 */
float gimbal_get_current_angle_tilt(void)
{
    return stepper_pulse_get_position_deg(STEPPER_AXIS_TILT);
}

/* ==========================================================================
 * ★ gimbal_track_velocity — 速度追踪核心
 * ==========================================================================
 *
 * 这是用于 CanMV 视觉追踪的连续速度追踪引擎。
 * 与 gimbal_control_angle() 的区别:
 *   - 本函数需要每 ~20ms 调用一次
 *   - 输出连续 PWM 调速, 永不自动停止
 *   - 适合响应式追踪 (目标移动 → 实时跟随)
 *
 * @param   error_x   水平像素偏差 (目标.cx - 160), 范围约 ±160 像素。
 * @param   error_y   垂直像素偏差 (目标.cy - 120), 范围约 ±120 像素。
 * @param   now_ms    当前系统毫秒时间戳 (millis()), 用于超时检测。
 *
 * @note    内部处理: 超时检测 → 死区过滤 → 非对称 EMA 滤波 → P 控制器
 *           → RPM 变化率限制 → stepper_pulse_set_speed()。
 * @note    方向由原始误差决定, 不受 EMA 滞后影响。
 */
void gimbal_track_velocity(int16_t error_x, int16_t error_y, uint32_t now_ms)
{
    /* ====================================================================
     * 超时检测: 超过 GIMBAL_LOST_TIMEOUT_MS 无新数据 → 强制 error=0
     * ==================================================================== */
    uint32_t pan_age  = now_ms - gimbal_pan.last_data_ms;
    uint32_t tilt_age = now_ms - gimbal_tilt.last_data_ms;

    /* 首次调用: 初始化时间戳 (避免 now_ms=0 时溢出) */
    if (gimbal_pan.last_data_ms == 0) {
        gimbal_pan.last_data_ms  = now_ms;
        gimbal_tilt.last_data_ms = now_ms;
        pan_age  = 0;
        tilt_age = 0;
    }

    if (pan_age >= GIMBAL_LOST_TIMEOUT_MS) {
        error_x = 0;
    }
    if (tilt_age >= GIMBAL_LOST_TIMEOUT_MS) {
        error_y = 0;
    }

    /* 更新数据时间戳 (有非零误差或首次数据时) */
    if (error_x != 0 || error_y != 0) {
        gimbal_pan.last_data_ms  = now_ms;
        gimbal_tilt.last_data_ms = now_ms;
    }

    /* ====================================================================
     * PAN 轴 (水平)
     * ==================================================================== */

    /* ── 1. 死区 ── */
    if (abs(error_x) < GIMBAL_DEADBAND_PAN) {
        error_x = 0;
    }

    /* ── 2. 非对称 EMA 低通滤波 ── */
    float effective_alpha;
    if (fabsf((float)error_x) < fabsf(gimbal_pan.ema_error)) {
        effective_alpha = GIMBAL_EMA_ALPHA_DECAY;   /* 0.30: 更快的衰减 */
    } else {
        effective_alpha = GIMBAL_EMA_ALPHA;         /* 0.15: 正常的平滑 */
    }
    gimbal_pan.ema_error = effective_alpha * (float)error_x
                         + (1.0f - effective_alpha) * gimbal_pan.ema_error;

    float ema_pan = gimbal_pan.ema_error;

    /* ── 3. P 控制器: 误差 → 目标 RPM ── */
    float target_rpm_pan = GIMBAL_TRACK_KP_PAN * fabsf(ema_pan);
    target_rpm_pan = clampf(target_rpm_pan, 0.0f, (float)GIMBAL_TRACK_MAX_RPM);

    /* ── 4. RPM 变化率限制 (带符号, 允许减速) ── */
    float rpm_delta_pan = target_rpm_pan - (float)gimbal_pan.current_rpm;
    rpm_delta_pan = clampf(rpm_delta_pan,
                           -(float)GIMBAL_RPM_SLEW_RATE,
                            (float)GIMBAL_RPM_SLEW_RATE);
    int16_t new_rpm_pan = (int16_t)gimbal_pan.current_rpm + (int16_t)rpm_delta_pan;

    /* ── 5. RPM 钳位: [0, MAX_RPM], 低于 MIN_RPM → 0 ── */
    new_rpm_pan = clampi16(new_rpm_pan, 0, (int16_t)GIMBAL_TRACK_MAX_RPM);
    if (new_rpm_pan > 0 && new_rpm_pan < (int16_t)GIMBAL_TRACK_MIN_RPM) {
        new_rpm_pan = 0;
    }

    /* ── 6. 方向: ★ 从原始误差决定 (立即响应, 无 EMA 滞后) ── */
    uint8_t dir_pan;
    if (error_x > 0) {
        dir_pan = STEPPER_DIR_CCW;
    } else if (error_x < 0) {
        dir_pan = STEPPER_DIR_CW;
    } else {
        dir_pan = stepper_pulse_get_current_dir(STEPPER_AXIS_PAN);
    }

    /* ── 7. 应用 ── */
    if (!gimbal_pan.fault) {
        stepper_pulse_set_speed(STEPPER_AXIS_PAN, dir_pan, (uint16_t)new_rpm_pan);
    }
    gimbal_pan.current_rpm = (uint16_t)new_rpm_pan;

    /* ====================================================================
     * TILT 轴 (垂直)
     * ==================================================================== */

    /* ── 1. 死区 ── */
    if (abs(error_y) < GIMBAL_DEADBAND_TILT) {
        error_y = 0;
    }

    /* ── 2. 非对称 EMA 滤波 ── */
    if (fabsf((float)error_y) < fabsf(gimbal_tilt.ema_error)) {
        effective_alpha = GIMBAL_EMA_ALPHA_DECAY;
    } else {
        effective_alpha = GIMBAL_EMA_ALPHA;
    }
    gimbal_tilt.ema_error = effective_alpha * (float)error_y
                          + (1.0f - effective_alpha) * gimbal_tilt.ema_error;

    float ema_tilt = gimbal_tilt.ema_error;

    /* ── 3. P 控制器 ── */
    float target_rpm_tilt = GIMBAL_TRACK_KP_TILT * fabsf(ema_tilt);
    target_rpm_tilt = clampf(target_rpm_tilt, 0.0f, (float)GIMBAL_TRACK_MAX_RPM);

    /* ── 4. RPM 变化率限制 ── */
    float rpm_delta_tilt = target_rpm_tilt - (float)gimbal_tilt.current_rpm;
    rpm_delta_tilt = clampf(rpm_delta_tilt,
                            -(float)GIMBAL_RPM_SLEW_RATE,
                             (float)GIMBAL_RPM_SLEW_RATE);
    int16_t new_rpm_tilt = (int16_t)gimbal_tilt.current_rpm + (int16_t)rpm_delta_tilt;

    /* ── 5. RPM 钳位 ── */
    new_rpm_tilt = clampi16(new_rpm_tilt, 0, (int16_t)GIMBAL_TRACK_MAX_RPM);
    if (new_rpm_tilt > 0 && new_rpm_tilt < (int16_t)GIMBAL_TRACK_MIN_RPM) {
        new_rpm_tilt = 0;
    }

    /* ── 6. 方向: ★ 从原始误差决定 ── */
    uint8_t dir_tilt;
    if (error_y > 0) {
        dir_tilt = STEPPER_DIR_CCW;
    } else if (error_y < 0) {
        dir_tilt = STEPPER_DIR_CW;
    } else {
        dir_tilt = stepper_pulse_get_current_dir(STEPPER_AXIS_TILT);
    }

    /* ── 7. 应用 ── */
    if (!gimbal_tilt.fault) {
        stepper_pulse_set_speed(STEPPER_AXIS_TILT, dir_tilt, (uint16_t)new_rpm_tilt);
    }
    gimbal_tilt.current_rpm = (uint16_t)new_rpm_tilt;
}

/**
 * @brief   平滑停止双轴速度追踪。
 *
 * 清零 EMA 误差 + current_rpm → stepper_pulse_stop() 停止双轴 PWM。
 * 调用后 gimbal_track_velocity() 可重新启动追踪。
 */
void gimbal_track_stop(void)
{
    /* 先清零 EMA (防止残留误差导致重启) */
    gimbal_pan.ema_error   = 0.0f;
    gimbal_tilt.ema_error  = 0.0f;
    gimbal_pan.current_rpm  = 0;
    gimbal_tilt.current_rpm = 0;

    /* 发送零速 → stepper_pulse_set_speed(rpm=0) 平滑停止 */
    stepper_pulse_stop(STEPPER_AXIS_PAN);
    stepper_pulse_stop(STEPPER_AXIS_TILT);
}

/* ==========================================================================
 * gimbal_track_is_active — 追踪活跃查询
 * ========================================================================== */
/**
 * @brief   查询速度追踪是否活跃 (有轴正在运转)。
 * @retval  true 表示追踪中。
 */
bool gimbal_track_is_active(void)
{
    return stepper_pulse_is_busy(STEPPER_AXIS_PAN)
        || stepper_pulse_is_busy(STEPPER_AXIS_TILT);
}

/* ==========================================================================
 * 调试查询 — 获取追踪引擎内部状态 (用于 OLED/TFT 显示)
 * ========================================================================== */

/**
 * @brief   获取 PAN 轴 EMA 滤波后的像素误差值 (调试用)。
 * @retval  EMA 滤波后的误差值 (像素)。
 */
float gimbal_get_filtered_error_x(void)
{
    return gimbal_pan.ema_error;
}

/**
 * @brief   获取 TILT 轴 EMA 滤波后的像素误差值 (调试用)。
 * @retval  EMA 滤波后的误差值 (像素)。
 */
float gimbal_get_filtered_error_y(void)
{
    return gimbal_tilt.ema_error;
}

/**
 * @brief   获取 PAN 轴当前输出的 RPM 值 (调试用)。
 * @retval  当前 RPM。
 */
uint16_t gimbal_get_output_rpm_pan(void)
{
    return gimbal_pan.current_rpm;
}

/**
 * @brief   获取 TILT 轴当前输出的 RPM 值 (调试用)。
 * @retval  当前 RPM。
 */
uint16_t gimbal_get_output_rpm_tilt(void)
{
    return gimbal_tilt.current_rpm;
}

/* ==========================================================================
 * gimbal_control_velocity — 速度模式 (@deprecated)
 * ========================================================================== */

/**
 * @brief   速度模式 — 直接发送 RPM 持续运转 (已废弃)。
 *
 * 保留用于编译兼容和简单测试。
 * 新代码请使用:
 *   - gimbal_control_angle()    用于绝对角度定位 (非阻塞)
 *   - gimbal_track_velocity()   用于视觉追踪 (平滑连续调速)
 *
 * @param   v1   PAN  轴 RPM (正=CCW, 负=CW, |v1|<=0.01 则停止)。
 * @param   v2   TILT 轴 RPM (正=CCW, 负=CW, |v2|<=0.01 则停止)。
 *
 * @deprecated  本函数阻塞主循环, 且无 EMA 滤波和死区处理。
 */
void gimbal_control_velocity(float v1, float v2)
{
    if (!gimbal_pan.fault) {
        if (fabsf(v1) > 0.01f) {
            uint16_t rpm = (uint16_t)ROUND(fabsf(v1));
            if (rpm < STEPPER_PULSE_MIN_RPM) rpm = STEPPER_PULSE_MIN_RPM;
            stepper_pulse_set_speed(STEPPER_AXIS_PAN,
                (v1 > 0.0f) ? STEPPER_DIR_CCW : STEPPER_DIR_CW, rpm);
        } else {
            stepper_pulse_set_speed(STEPPER_AXIS_PAN,
                STEPPER_DIR_CW, STEPPER_RPM_STOP);
        }
    }

    if (!gimbal_tilt.fault) {
        if (fabsf(v2) > 0.01f) {
            uint16_t rpm = (uint16_t)ROUND(fabsf(v2));
            if (rpm < STEPPER_PULSE_MIN_RPM) rpm = STEPPER_PULSE_MIN_RPM;
            stepper_pulse_set_speed(STEPPER_AXIS_TILT,
                (v2 > 0.0f) ? STEPPER_DIR_CCW : STEPPER_DIR_CW, rpm);
        } else {
            stepper_pulse_set_speed(STEPPER_AXIS_TILT,
                STEPPER_DIR_CW, STEPPER_RPM_STOP);
        }
    }
}

/* ==========================================================================
 * gimbal_stop — 双轴急停
 * ========================================================================== */
/**
 * @brief   双轴紧急停止 — 清零所有状态并停止 PWM 输出。
 *
 * 同时清零 EMA 误差、RPM, 调用 stepper_pulse_emergency_stop()。
 */
void gimbal_stop(void)
{
    stepper_pulse_emergency_stop();
    gimbal_pan.current_rpm   = 0;
    gimbal_tilt.current_rpm  = 0;
    gimbal_pan.ema_error     = 0.0f;
    gimbal_tilt.ema_error    = 0.0f;
}

/* ==========================================================================
 * 堵转检测 (软件备用, 当前为占位实现)
 * ========================================================================== */

/**
 * @brief   初始化堵转检测状态, 清零故障标志和确认计数。
 */
void gimbal_stall_monitor_init(void)
{
    gimbal_pan.stall_confirm_count  = 0;
    gimbal_pan.fault               = STALL_FAULT_NONE;
    gimbal_tilt.stall_confirm_count = 0;
    gimbal_tilt.fault              = STALL_FAULT_NONE;
}

/**
 * @brief   堵转检测轮询 (占位函数, 后续可扩展编码器反馈或电流检测)。
 *
 * 当前逻辑: 速度模式下 RPM>0 但脉冲计数长时间不变 → 可能堵转。
 */
void gimbal_stall_monitor_tick(void)
{
    /* 速度追踪模式下, 软件堵转检测较为简单。
     * 如果某轴 RPM > 0 但脉冲计数长时间不变 → 可能堵转。
     * 当前保留为占位, 后续可扩展。 */
}

/**
 * @brief   查询是否有任何轴处于堵转故障状态。
 * @retval  true 表示至少一轴故障。
 */
bool gimbal_is_any_stalled(void)
{
    return (gimbal_pan.fault != STALL_FAULT_NONE)
        || (gimbal_tilt.fault != STALL_FAULT_NONE);
}

/**
 * @brief   查询 PAN 轴是否堵转。
 * @retval  true 表示 PAN 轴故障。
 */
bool gimbal_is_pan_stalled(void)
{
    return (gimbal_pan.fault != STALL_FAULT_NONE);
}

/**
 * @brief   查询 TILT 轴是否堵转。
 * @retval  true 表示 TILT 轴故障。
 */
bool gimbal_is_tilt_stalled(void)
{
    return (gimbal_tilt.fault != STALL_FAULT_NONE);
}

/**
 * @brief   获取 PAN 轴当前故障类型。
 * @retval  stall_fault_t 枚举值。
 */
stall_fault_t gimbal_get_pan_fault(void)
{
    return gimbal_pan.fault;
}

/**
 * @brief   获取 TILT 轴当前故障类型。
 * @retval  stall_fault_t 枚举值。
 */
stall_fault_t gimbal_get_tilt_fault(void)
{
    return gimbal_tilt.fault;
}

/**
 * @brief   清除双轴堵转故障标志 (复位为 STALL_FAULT_NONE)。
 */
void gimbal_clear_stall_fault(void)
{
    gimbal_stall_monitor_init();
}

/**
 * @brief   堵转回调 (弱符号, 用户可覆盖实现自定义处理)。
 * @param   motor_addr   轴索引 (STEPPER_AXIS_PAN / STEPPER_AXIS_TILT)。
 * @param   fault        故障类型。
 *
 * @note    默认实现为空; 覆盖此函数即可实现自定义堵转响应。
 */
__attribute__((weak))
void gimbal_stall_callback(uint8_t motor_addr, stall_fault_t fault)
{
    (void)motor_addr;
    (void)fault;
}
