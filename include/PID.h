/**
 * @file    PID.h
 * @brief   增量式 PID 控制器 —— 声明
 *
 * ======================== 为什么用增量式而非位置式？ ========================
 *
 *   位置式 PID 的输出是控制量的绝对值：
 *     u[k] = Kp×e[k] + Ki×Σe + Kd×(e[k]-e[k-1])
 *   问题：积分项 Σe 会无限累积，MCU 复位或模式切换时需要额外逻辑清零。
 *
 *   增量式 PID 的输出是控制量的变化量 Δu：
 *     Δu[k] = Kp×(e[k]-e[k-1]) + Ki×e[k]×dt + Kd×(e[k]-2e[k-1]+e[k-2])/dt
 *     u[k] = u[k-1] + Δu[k]
 *   优势：
 *     - 积分项天然有界（只乘当前误差 e[k]），不会饱和
 *     - 模式切换时只需重置 prev_output 即可，无需清零积分累加器
 *     - 适合"增量叠加"型执行器（如 PWM 占空比的逐帧调整）
 *
 * ======================== 参数含义 ========================
 *
 *   Kp: 比例增益 — 误差 × Kp → 直接修正量
 *       例：Kp=0.2, error=500 → Δu_P = 0.2×(500-480) = 4
 *
 *   Ki: 积分增益 — 消除稳态误差
 *       例：Ki=0.05, error=10, dt=0.01 → Δu_I = 0.05×10×0.01 = 0.005
 *       Ki 越大，稳态误差消除越快，但过大会引起振荡
 *
 *   Kd: 微分增益 — 预测误差变化趋势，抑制振荡
 *       例：Kd=0.1, e上升快 → Δu_D 为负（提前刹车）
 *       对噪声敏感，通常需要配合低通滤波
 *
 *   deadband: 死区 — |error| < deadband 时输出不变
 *       防止微小误差导致的"抖动追逐"（jitter hunting）
 *
 *   out_min/out_max: 输出限幅 — 防止执行器超出物理范围
 *       例：PWM 占空比限幅 [0, 80]
 */

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;                /**< 比例增益 */
    float Ki;                /**< 积分增益 */
    float Kd;                /**< 微分增益 */
    float setpoint;          /**< 期望值（目标位置/速度） */
    float dt;                /**< 控制周期（秒） */

    float prev_error;        /**< e[k-1]：上一次误差 */
    float prev_prev_error;   /**< e[k-2]：上上次误差 */
    float prev_output;       /**< u[k-1]：上一次输出（用于增量叠加） */

    float out_min;           /**< 输出下限 */
    float out_max;           /**< 输出上限 */
    float deadband;          /**< 死区：|error| < deadband → 不调整 */
} IncPID_t;

/**
 * @brief   初始化增量式 PID 控制器
 * @param   p         PID 实例指针
 * @param   kp,ki,kd  PID 增益
 * @param   dt        控制周期（秒），用于积分/微分项的时间归一化
 * @param   out_min   输出下限
 * @param   out_max   输出上限
 */
void IncPID_Init(IncPID_t *p, float kp, float ki, float kd,
                 float dt, float out_min, float out_max);

/**
 * @brief   输入当前测量值，计算并返回控制输出
 * @param   p            PID 实例指针
 * @param   measurement  当前测量值（如当前转速 rpm）
 * @return  控制输出（如目标 PWM 占空比）
 *
 * 调用方式（每控制周期一次）：
 * @code
 *   float duty = IncPID_Compute(&pid, actual_rpm);
 *   Set_Motor_Speed(duty);
 * @endcode
 */
float IncPID_Compute(IncPID_t *p, float measurement);

/**
 * @brief   手动设置当前输出值（用于模式切换时的初始化）
 * @param   p       PID 实例指针
 * @param   output 新的输出值
 *
 * 例：从"手动控制"切换到"PID 控制"时，先把 prev_output 设为
 * 当前手动输出的值，避免 PID 从 0 开始产生突变。
 */
void IncPID_SetOutput(IncPID_t *p, float output);

#endif /* PID_H */
