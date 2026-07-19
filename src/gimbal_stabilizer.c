/**
 * @file    gimbal_stabilizer.c
 * @brief   云台增稳控制器实现 — IMU 姿态反馈 PID 闭环控制
 *
 * ============================================================================
 * === ★ 架构概览 ===
 * ============================================================================
 *
 * 本模块是 MPU6050 姿态获取与云台步进电机的协同控制桥梁:
 *
 *   IMU_Update() 姿态数据
 *        │
 *        ▼
 *   Stabilizer_Update(&imu)
 *        │
 *        ├─ [HOLD]  error = locked_angle - imu_angle
 *        │          PID → correction → gimbal_control_angle()
 *        │
 *        ├─ [TRACK] ema_target = EMA(external_target)
 *        │          error = ema_target - imu_angle
 *        │          PID → correction → gimbal_control_angle()
 *        │
 *        └─ [IDLE]  return (no control)
 *
 *   轴映射 (2 轴云台):
 *     IMU Pitch → TILT 电机 (俯仰补偿)
 *     IMU Yaw   → PAN  电机 (水平补偿)
 *     IMU Roll  → 无法直接补偿 (二轴云台物理限制)
 *
 * ============================================================================
 * === ★ PID 控制算法 ===
 * ============================================================================
 *
 * 采用位置式 PID + 积分分离 + 抗饱和:
 *
 *   error = target - current
 *   P = Kp * error
 *   I = I_prev + Ki * error * dt     (仅当 |error| < i_zone)
 *   I = clamp(I, -i_max, +i_max)     (抗饱和)
 *   D = Kd * (error - last_error) / dt
 *   output = P + I + D
 *   output = clamp(output, -out_max, +out_max)
 *   if |error| < deadband → output = 0, I = 0   (防微振)
 *
 * ============================================================================
 * === ★ 安全保护机制 ===
 * ============================================================================
 *
 *   1. 机械限位: 硬编码角度上限, 防止机械碰撞
 *   2. 增量限幅: 单次角度变化 > STAB_DELTA_HARD_LIMIT → 拉回, 防数据突变
 *   3. PID 输出限幅: out_max 限制单次补偿量, 防过冲丢步
 *   4. 积分抗饱和: i_max + i_zone 双保险防 windup
 *   5. 死区保护: deadband 防止目标附近的微振 (hunting)
 *   6. 故障检测: 读取异常值时跳过本次更新, 维持上次输出
 *
 * ============================================================================
 */

#include "gimbal_stabilizer.h"
#include "drivers/gimbal_control.h"
#include "drivers/delay.h"
#include <math.h>

/* ============================================================================
 *  内部工具宏
 * ============================================================================ */

/** 浮点钳位 (编译为无分支 CMOV, 高效) */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/** 取绝对值 */
static inline float fabsf_local(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ============================================================================
 *  全局状态
 * ============================================================================ */

/** 当前工作模式 */
static StabilizeMode_t s_mode = STABILIZE_IDLE;

/** 双轴控制状态 */
static StabilizerAxis_t s_pan;
static StabilizerAxis_t s_tilt;

/** 是否已初始化 */
static bool s_initialized = false;

/** 是否已收到至少一次有效 IMU 数据 (防止 HOLD 模式在 IMU 就绪前锁定到 0°) */
static bool s_has_imu_data = false;


/* ============================================================================
 *  PID 核心算法
 * ============================================================================ */

/**
 * @brief   执行一次 PID 计算 (位置式, 带积分分离 + 抗饱和)
 *
 * @param   pid       PID 控制器指针
 * @param   setpoint  目标值 (度)
 * @param   measured  测量值 (度)
 * @param   dt        控制周期 (秒), 若为 0 则使用 pid->dt
 * @return  PID 输出 (度)
 *
 * 算法:
 *   1. error = setpoint - measured
 *   2. deadband: |error| < deadband → 清零积分, 返回 0
 *   3. P = Kp * error
 *   4. I += Ki * error * dt (仅当 |error| < i_zone, 积分分离)
 *   5. I = clamp(I, ±i_max)
 *   6. D = Kd * (error - last_error) / dt
 *   7. output = clamp(P + I + D, ±out_max)
 */
static float PID_Compute(StabilizerPID_t *pid, float setpoint, float measured, float dt)
{
    float error, p_term, i_term, d_term, output;

    /* ── 1. 误差 ── */
    error = setpoint - measured;

    /* ── ★ 角度误差归一化: 强制取最短路径 [-180°, +180°] ── */
    /*      防止 PAN (yaw) 在 0°/360° 边界穿越时产生正向 340° 误差
     *      而非正确的 -20° 误差 (会导致电机反向旋转)            */
    if (error > 180.0f)       error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    pid->error = error;

    /* ── 2. 死区: 关闭所有输出, 清零积分 ── */
    if (fabsf_local(error) < pid->deadband) {
        pid->integral      = 0.0f;
        pid->output        = 0.0f;
        pid->last_error    = error;
        pid->last_measured = measured;
        return 0.0f;
    }

    /* ── 使用传入的 dt 或 PID 内部 dt ── */
    if (dt <= 0.0f) dt = pid->dt;
    if (dt < 0.0001f) dt = 0.0001f;   /* 防止除零 */
    if (dt > 0.1f)    dt = 0.1f;      /* 防止超长间隔 */

    /* ── 3. 比例项 ── */
    p_term = pid->Kp * error;

    /* ── 4. 积分项 (积分分离 + 抗饱和) ── */
    if (fabsf_local(error) < pid->i_zone) {
        /* 仅在误差较小时累加积分, 防止大误差下的积分饱和 */
        pid->integral += pid->Ki * error * dt;
        pid->integral  = clampf(pid->integral, -pid->i_max, pid->i_max);
    } else {
        /* 误差大时清零积分, 防止 windup */
        pid->integral = 0.0f;
    }
    i_term = pid->integral;

    /* ── 5. 微分项 (对测量值微分, 非对误差微分)
     *
     * ★ derivative-on-measurement: 避免 setpoint 突变时的 derivative kick,
     *   仅对物理扰动 (测量值变化) 产生阻尼, 适合增稳/扰动抑制应用    */
    d_term = -pid->Kd * (measured - pid->last_measured) / dt;
    pid->last_measured = measured;
    pid->last_error    = error;

    /* ── 6. 合成 + 限幅 ── */
    output = p_term + i_term + d_term;
    output = clampf(output, -pid->out_max, pid->out_max);

    pid->output = output;
    return output;
}

/**
 * @brief   重置 PID 控制器内部状态 (切换模式/紧急停止时调用)
 * @param   pid   PID 控制器指针
 */
static void PID_Reset(StabilizerPID_t *pid)
{
    pid->error         = 0.0f;
    pid->last_error    = 0.0f;
    pid->last_measured = 0.0f;
    pid->integral      = 0.0f;
    pid->output        = 0.0f;
}


/* ============================================================================
 *  初始化与配置
 * ============================================================================ */

/**
 * @brief   初始化单轴 PID 为默认参数
 * @param   pid       PID 结构体指针
 * @param   Kp, Ki, Kd   默认增益
 * @param   out_max, i_max, i_zone, deadband   限幅参数
 * @param   dt        控制周期 (秒)
 */
static void PID_InitDefault(StabilizerPID_t *pid,
                            float Kp, float Ki, float Kd,
                            float out_max, float i_max,
                            float i_zone, float deadband, float dt)
{
    pid->Kp        = Kp;
    pid->Ki        = Ki;
    pid->Kd        = Kd;
    pid->out_max   = out_max;
    pid->i_max     = i_max;
    pid->i_zone    = i_zone;
    pid->deadband  = deadband;
    pid->dt        = dt;
    pid->error     = 0.0f;
    pid->last_error = 0.0f;
    pid->last_measured = 0.0f;
    pid->integral  = 0.0f;
    pid->output    = 0.0f;
}

/**
 * @brief   初始化单轴增稳状态
 * @param   ax        轴状态指针
 * @param   Kp, Ki, Kd   默认增益
 * @param   out_max, i_max, i_zone, deadband   限幅参数
 */
static void StabilizerAxis_Init(StabilizerAxis_t *ax,
                                float Kp, float Ki, float Kd,
                                float out_max, float i_max,
                                float i_zone, float deadband)
{
    PID_InitDefault(&ax->pid, Kp, Ki, Kd, out_max, i_max, i_zone, deadband,
                    (float)STAB_HOLD_UPDATE_MS / 1000.0f);

    ax->target_angle   = 0.0f;
    ax->current_angle  = 0.0f;
    ax->output_angle   = 0.0f;
    ax->ema_target     = 0.0f;
    ax->ema_alpha      = STAB_TRACK_EMA_DEFAULT;
    ax->last_update_ms = 0;
    ax->active         = false;
}

void Stabilizer_Init(void)
{
    StabilizerAxis_Init(&s_pan,
        STAB_PAN_KP_DEFAULT, STAB_PAN_KI_DEFAULT, STAB_PAN_KD_DEFAULT,
        STAB_PAN_OUT_MAX_DEFAULT, STAB_PAN_I_MAX_DEFAULT,
        STAB_PAN_I_ZONE_DEFAULT, STAB_PAN_DEADBAND_DEFAULT);

    StabilizerAxis_Init(&s_tilt,
        STAB_TILT_KP_DEFAULT, STAB_TILT_KI_DEFAULT, STAB_TILT_KD_DEFAULT,
        STAB_TILT_OUT_MAX_DEFAULT, STAB_TILT_I_MAX_DEFAULT,
        STAB_TILT_I_ZONE_DEFAULT, STAB_TILT_DEADBAND_DEFAULT);

    s_mode        = STABILIZE_IDLE;
    s_initialized = true;
}


/* ============================================================================
 *  模式切换
 * ============================================================================ */

void Stabilizer_SetMode(StabilizeMode_t mode)
{
    if (!s_initialized) return;

    switch (mode) {

    case STABILIZE_HOLD:
        /*
         * 切换到 HOLD 模式: 将当前 IMU 姿态锁定为目标角度。
         * 这样云台会保持在切换到 HOLD 瞬间的姿态。
         *
         * ★ s_has_imu_data 确保至少收到过一次 IMU 数据后才锁定,
         *   避免在 IMU 未就绪时锁定到初始值 0°。
         */
        if (s_has_imu_data) {
            s_pan.target_angle  = s_pan.current_angle;
            s_tilt.target_angle = s_tilt.current_angle;
        }
        /* 重置 EMA (HOLD 模式不使用 EMA) */
        s_pan.ema_target  = s_pan.target_angle;
        s_tilt.ema_target = s_tilt.target_angle;
        /* 重置 PID 状态, 从零开始 */
        PID_Reset(&s_pan.pid);
        PID_Reset(&s_tilt.pid);
        s_pan.active  = true;
        s_tilt.active = true;
        s_mode = mode;
        break;

    case STABILIZE_TRACK:
        /*
         * 切换到 TRACK 模式: 保持当前目标角度不变,
         * 等待 Stabilizer_SetTarget() 下发新目标。
         */
        s_pan.ema_target  = s_pan.target_angle;
        s_tilt.ema_target = s_tilt.target_angle;
        PID_Reset(&s_pan.pid);
        PID_Reset(&s_tilt.pid);
        s_pan.active  = true;
        s_tilt.active = true;
        s_mode = mode;
        break;

    case STABILIZE_IDLE:
    default:
        /* 停止所有电机, 重置状态 */
        gimbal_stop();
        PID_Reset(&s_pan.pid);
        PID_Reset(&s_tilt.pid);
        s_pan.active  = false;
        s_tilt.active = false;
        s_mode = STABILIZE_IDLE;
        break;
    }
}

StabilizeMode_t Stabilizer_GetMode(void)
{
    return s_mode;
}


/* ============================================================================
 *  目标角度设置
 * ============================================================================ */

void Stabilizer_SetTarget(float pan_deg, float tilt_deg)
{
    /* 钳位到机械限位 */
    pan_deg  = clampf(pan_deg,  -STAB_PAN_ANGLE_HARD_LIMIT,  STAB_PAN_ANGLE_HARD_LIMIT);
    tilt_deg = clampf(tilt_deg, -STAB_TILT_ANGLE_HARD_LIMIT, STAB_TILT_ANGLE_HARD_LIMIT);

    /* 仅在 TRACK 模式下更新原始目标 */
    if (s_mode == STABILIZE_TRACK) {
        s_pan.target_angle  = pan_deg;
        s_tilt.target_angle = tilt_deg;
    }
    /* HOLD/IDLE 模式下忽略, 用户应使用 Stabilizer_SetTargetInstant() */
}

void Stabilizer_SetTargetInstant(float pan_deg, float tilt_deg)
{
    /* 钳位 */
    pan_deg  = clampf(pan_deg,  -STAB_PAN_ANGLE_HARD_LIMIT,  STAB_PAN_ANGLE_HARD_LIMIT);
    tilt_deg = clampf(tilt_deg, -STAB_TILT_ANGLE_HARD_LIMIT, STAB_TILT_ANGLE_HARD_LIMIT);

    /* 直接设置目标 + 同步 EMA */
    s_pan.target_angle  = pan_deg;
    s_tilt.target_angle = tilt_deg;
    s_pan.ema_target    = pan_deg;
    s_tilt.ema_target   = tilt_deg;

    /* 重置 PID (清除上次误差的残余积分/微分) */
    PID_Reset(&s_pan.pid);
    PID_Reset(&s_tilt.pid);
}


/* ============================================================================
 *  PID 参数设置
 * ============================================================================ */

void Stabilizer_SetPID(StabilizeAxis_t axis, float Kp, float Ki, float Kd)
{
    StabilizerPID_t *pid;

    /* ★ NaN 守卫: NaN != NaN 是 IEEE 754 规定的 NaN 检测方法 */
    if (Kp != Kp || Ki != Ki || Kd != Kd) return;

    /* 安全性: 拒绝极端参数 (同时捕获 ±Inf: Inf > 100.0f = true) */
    if (Kp < 0.0f || Kp > 100.0f)  return;
    if (Ki < 0.0f || Ki > 50.0f)   return;
    if (Kd < 0.0f || Kd > 20.0f)   return;

    if (axis == STAB_AXIS_PAN) {
        pid = &s_pan.pid;
    } else {
        pid = &s_tilt.pid;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    /* 改增益后重置积分, 防止旧积分在新增益下产生突变 */
    pid->integral = 0.0f;
}

void Stabilizer_SetPIDLimits(StabilizeAxis_t axis,
                             float out_max, float i_max,
                             float i_zone, float deadband)
{
    StabilizerPID_t *pid;

    /* ★ NaN 守卫: NaN != NaN */
    if (out_max != out_max || i_max != i_max
        || i_zone != i_zone || deadband != deadband) return;

    if (axis == STAB_AXIS_PAN) {
        pid = &s_pan.pid;
    } else {
        pid = &s_tilt.pid;
    }

    if (out_max   > 0.0f && out_max   < 100.0f) pid->out_max  = out_max;
    if (i_max     > 0.0f && i_max     < 100.0f) pid->i_max    = i_max;
    if (i_zone    > 0.0f && i_zone    < 180.0f) pid->i_zone   = i_zone;
    if (deadband  > 0.0f && deadband  < 10.0f)  pid->deadband = deadband;

    pid->integral = 0.0f;
}

const StabilizerPID_t *Stabilizer_GetPID(StabilizeAxis_t axis)
{
    return (axis == STAB_AXIS_PAN) ? &s_pan.pid : &s_tilt.pid;
}


/* ============================================================================
 *  ★ 核心控制循环
 * ============================================================================ */

/**
 * @brief   处理单轴增稳控制 (PAN 或 TILT)
 *
 * 本函数封装了单轴控制的完整流程:
 *   [TRACK] EMA 平滑目标 → PID 计算 → 限幅 → 下发角度
 *   [HOLD]  直接使用锁定目标 → PID 计算 → 限幅 → 下发角度
 *
 * @param   ax        轴状态指针
 * @param   imu_angle 当前 IMU 测量的角度 (度)
 * @param   hard_limit 机械硬限幅 (度)
 * @param   now_ms    当前系统时间戳 (millis())
 * @param   fn_get_angle 获取当前云台电机角度的函数指针
 * @param   fn_control   设置云台电机角度的函数指针
 */
static void StabilizerAxis_Update(StabilizerAxis_t *ax,
                                  float imu_angle,
                                  float hard_limit,
                                  uint32_t now_ms,
                                  float (*fn_get_angle)(void),
                                  void (*fn_control)(float))
{
    float setpoint, pid_output, final_angle, delta;
    float dt_sec;

    /* ── 更新当前 IMU 角度 ── */
    ax->current_angle = imu_angle;

    /* ── TRACK 模式: EMA 平滑目标角度 ── */
    if (s_mode == STABILIZE_TRACK) {
        ax->ema_target = ax->ema_alpha * ax->target_angle
                       + (1.0f - ax->ema_alpha) * ax->ema_target;
        setpoint = ax->ema_target;
    } else {
        /* HOLD 模式: 使用锁定的目标角度 */
        setpoint = ax->target_angle;
    }

    /* ── 计算控制周期 ── */
    if (ax->last_update_ms != 0) {
        uint32_t elapsed = now_ms - ax->last_update_ms;
        dt_sec = (float)elapsed / 1000.0f;
        if (dt_sec > 0.1f) dt_sec = 0.1f;   /* 长时间未更新, 限制 dt */
    } else {
        dt_sec = ax->pid.dt;                 /* 首次调用使用默认 dt */
    }
    ax->last_update_ms = now_ms;

    /* ── PID 计算 ── */
    pid_output = PID_Compute(&ax->pid, setpoint, imu_angle, dt_sec);

    /* ── 计算最终输出角度 ── */
    /* pid_output 是相对于 setpoint 的修正量。
     *
     * 对于增稳: 我们希望 IMU 的角度保持在 setpoint。
     * 如果 IMU 偏离了 setpoint (error ≠ 0), PID 输出非零修正。
     * 将修正叠加到当前电机位置即可补偿:
     *
     *   final_angle = current_motor_angle + pid_output
     *
     * 其中 pid_output 的符号:
     *   若 IMU 偏右 (yaw > setpoint_yaw), error_yaw < 0
     *     → pid 输出负值 → motor 向左 → 补偿偏右
     *
     * 注意: 此方案要求云台电机角度与 IMU 角度方向一致。
     * 若物理安装方向相反, 需在调用处对 imu_angle 取反。
     */
    float current_motor_angle = fn_get_angle();
    final_angle = current_motor_angle + pid_output;

    /* ── 增量限幅 (防止数据异常/传感器故障导致电机飞车) ── */
    delta = final_angle - current_motor_angle;
    if (fabsf_local(delta) > STAB_DELTA_HARD_LIMIT) {
        /* 异常大增量 → 钳位到安全范围 */
        final_angle = current_motor_angle
                    + (delta > 0.0f ? STAB_DELTA_HARD_LIMIT : -STAB_DELTA_HARD_LIMIT);
        /* 同时重置 PID 积分, 防止累计错误 */
        ax->pid.integral = 0.0f;
    }

    /* ── 机械硬限幅 ── */
    final_angle = clampf(final_angle, -hard_limit, hard_limit);

    /* ── 死区: 若最终角度与当前电机角度相差无几, 跳过 ── */
    if (fabsf_local(final_angle - current_motor_angle) < ax->pid.deadband) {
        ax->output_angle = current_motor_angle;
        return;  /* 不调用 gimbal_control_angle, 避免不必要的 PWM 重启 */
    }

    /* ── 存储输出 ── */
    ax->output_angle = final_angle;

    /* ── 下发到云台控制层 (非阻塞) ── */
    fn_control(final_angle);
}


/* ============================================================================
 *  云台角度回调 (桥接 gimbal_control 层)
 * ============================================================================ */

/** PAN 轴当前角度读取回调 */
static float GetPanAngle(void)
{
    return gimbal_get_current_angle_pan();
}

/** TILT 轴当前角度读取回调 */
static float GetTiltAngle(void)
{
    return gimbal_get_current_angle_tilt();
}

/** PAN 轴角度控制回调 (仅控制 PAN, 不干扰 TILT 轴正在进行的运动) */
static void ControlPan(float angle_deg)
{
    gimbal_control_pan_only(angle_deg);
}

/** TILT 轴角度控制回调 (仅控制 TILT, 不干扰 PAN 轴正在进行的运动) */
static void ControlTilt(float angle_deg)
{
    gimbal_control_tilt_only(angle_deg);
}


/* ============================================================================
 *  Stabilizer_Update — 主入口 (每帧调用)
 * ============================================================================ */

bool Stabilizer_Update(const IMU_Data_t *imu)
{
    if (!s_initialized || imu == NULL) return false;

    /* ── IDLE 模式: 无操作 ── */
    if (s_mode == STABILIZE_IDLE) {
        return false;
    }

    /* ── ★ 标记已收到 IMU 数据 (供 HOLD 模式判断) ── */
    s_has_imu_data = true;

    /* ── ★ 堵转故障保护: 任一轴故障时跳过 PID, 防止积分累积 ── */
    if (gimbal_is_any_stalled()) {
        /* 清零积分, 防止故障恢复后积分突变 */
        s_pan.pid.integral  = 0.0f;
        s_tilt.pid.integral = 0.0f;
        return false;
    }

    /* ── IMU 数据鲜度检查: 超过 500ms 未更新视为数据过期 ── */
    if (s_pan.last_update_ms != 0) {
        uint32_t age = millis() - s_pan.last_update_ms;
        if (age > 500) {
            /* 数据过期: 清零积分, 但不改变电机位置 (fail-safe) */
            s_pan.pid.integral  = 0.0f;
            s_tilt.pid.integral = 0.0f;
            return false;
        }
    }

    uint32_t now_ms = millis();

    /*
     * ── 轴映射 ──
     *
     * 物理安装: MPU6050 固定在相机平台上
     *   X 轴 = 相机前方 (Roll)
     *   Y 轴 = 相机左侧 (Pitch)
     *   Z 轴 = 相机上方 (Yaw)
     *
     * 映射到云台:
     *   IMU Pitch → TILT 电机 (俯仰补偿)
     *   IMU Yaw   → PAN  电机 (水平补偿)
     *
     * 若你的安装方向不同, 修改下面的映射即可。
     * 常见变体:
     *   - 若 PAN 方向相反: 把 imu->yaw 改为 -imu->yaw
     *   - 若 TILT 方向相反: 把 imu->pitch 改为 -imu->pitch
     *   - 若需要 Roll 补偿 PAN: 把 imu->yaw 改为 imu->roll
     */

    /* ── PAN 轴: IMU Yaw → PAN motor ── */
    StabilizerAxis_Update(&s_pan, imu->yaw, STAB_PAN_ANGLE_HARD_LIMIT,
                          now_ms, GetPanAngle, ControlPan);

    /* ── TILT 轴: IMU Pitch → TILT motor ── */
    StabilizerAxis_Update(&s_tilt, imu->pitch, STAB_TILT_ANGLE_HARD_LIMIT,
                          now_ms, GetTiltAngle, ControlTilt);

    return true;
}


/* ============================================================================
 *  紧急停止
 * ============================================================================ */

void Stabilizer_EmergencyStop(void)
{
    gimbal_stop();

    s_mode = STABILIZE_IDLE;

    PID_Reset(&s_pan.pid);
    PID_Reset(&s_tilt.pid);

    s_pan.active  = false;
    s_tilt.active = false;
}


/* ============================================================================
 *  状态查询
 * ============================================================================ */

bool Stabilizer_IsActive(void)
{
    return (s_mode == STABILIZE_HOLD || s_mode == STABILIZE_TRACK);
}

bool Stabilizer_IsMoving(void)
{
    return gimbal_is_moving();
}

float Stabilizer_GetError(StabilizeAxis_t axis)
{
    return (axis == STAB_AXIS_PAN) ? s_pan.pid.error : s_tilt.pid.error;
}

float Stabilizer_GetOutput(StabilizeAxis_t axis)
{
    return (axis == STAB_AXIS_PAN) ? s_pan.pid.output : s_tilt.pid.output;
}
