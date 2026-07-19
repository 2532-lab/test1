/**
 * @file    PID.c
 * @brief   增量式 PID 控制器 —— 实现
 *
 * ======================== 核心公式 ========================
 *
 *   计算误差：
 *     error[k] = setpoint - measurement
 *
 *   死区判断：
 *     if |error[k]| < deadband → 不计算 Δu，直接返回上次输出
 *     （更新误差历史以便死区退出后能正确计算微分项）
 *
 *   增量计算：
 *     Δu = Kp × (e[k] - e[k-1])                              ← 比例增量
 *        + Ki × e[k] × dt                                     ← 积分增量
 *        + Kd × (e[k] - 2×e[k-1] + e[k-2]) / dt              ← 微分增量
 *
 *   叠加输出：
 *     u[k] = u[k-1] + Δu
 *
 *   输出限幅：
 *     if u[k] > out_max → u[k] = out_max
 *     if u[k] < out_min → u[k] = out_min
 *
 * ======================== 与位置式 PID 的区别 ========================
 *
 *   位置式（Positional）：
 *     u[k] = Kp×e[k] + Ki×Σe[i]×dt + Kd×(e[k]-e[k-1])/dt
 *     特点：输出是"当前应该是什么"，积分项需要无限累加。
 *
 *   增量式（Incremental）：
 *     Δu = 上述增量公式
 *     u[k] = u[k-1] + Δu
 *     特点：输出是"应该改变多少"，天然抗积分饱和。
 *
 * ======================== 使用示例 ========================
 *
 * @code
 *   IncPID_t pid;
 *   IncPID_Init(&pid, 0.15f, 0.02f, 0.0f, 0.01f, 0, 80);
 *   pid.setpoint = 100.0f;  // 目标转速 100rpm
 *
 *   while (1) {
 *       float actual = Encoder_GetRPM();       // 实测转速
 *       float duty   = IncPID_Compute(&pid, actual);
 *       Set_Motor_Speed((uint8_t)duty);
 *       delay_ms(10);  // dt=0.01 对应 10ms 周期
 *   }
 * @endcode
 */

#include "PID.h"
#include <math.h>    /* fabsf */

void IncPID_Init(IncPID_t *p, float kp, float ki, float kd,
                 float dt, float out_min, float out_max)
{
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->dt = dt;
    p->setpoint        = 0.0f;
    p->prev_error      = 0.0f;
    p->prev_prev_error = 0.0f;
    p->prev_output     = 0.0f;
    p->out_min         = out_min;
    p->out_max         = out_max;
    p->deadband        = 0.0f;   /* 默认无死区，上层可后续设置 */
}

float IncPID_Compute(IncPID_t *p, float measurement)
{
    /*
     * 计算当前误差
     * setpoint(期望) - measurement(实际) → 正误差=偏低, 负误差=偏高
     */
    float error = p->setpoint - measurement;

    /*
     * 死区判断：误差太小 → 不调整，避免"抖动追逐"
     *
     * 为什么仍要更新误差历史？
     *   如果死区内不更新历史，退出死区时 e[k-1] 还是进入死区前的旧值，
     *   导致退出瞬间产生一个虚假的比例增量（e[k]-e[k-1] 突然变大）。
     *   更新历史让退出死区时 e[k]-e[k-1] 接近 0，平滑过渡。
     */
    if (fabsf(error) < p->deadband) {
        p->prev_prev_error = p->prev_error;
        p->prev_error      = error;
        return p->prev_output;
    }

    /*
     * ====== 增量式 PID 核心公式 ======
     *
     * P 增量：Kp × (e[k] - e[k-1])
     *   误差变化率 ——"误差在变大还是变小？"
     *   误差增大 → P 增量同号，进一步加大修正
     *   误差减小 → P 增量异号，开始回撤修正
     *
     * I 增量：Ki × e[k] × dt
     *   当前误差积分 ——"还有多少偏差没消除？"
     *   误差持续存在 → I 增量逐步积累，消除稳态误差
     *   dt 归一化：Ki 的物理含义是"每秒消除多少比例"
     *
     * D 增量：Kd × (e[k] - 2e[k-1] + e[k-2]) / dt
     *   误差加速度 ——"误差变化的速度在加速还是减速？"
     *   两阶差分：二阶导数近似，预测误差未来趋势
     *   / dt 归一化：Kd 的物理含义是"误差变化率对应的修正量"
     */
    float delta = p->Kp * (error - p->prev_error)
                + p->Ki * error * p->dt
                + p->Kd * (error - 2.0f * p->prev_error
                                  + p->prev_prev_error) / p->dt;

    /* 更新误差历史 —— 必须在计算输出之前 */
    p->prev_prev_error = p->prev_error;
    p->prev_error      = error;

    /* 增量叠加：u[k] = u[k-1] + Δu */
    float output = p->prev_output + delta;

    /* 输出限幅 —— 保护执行器 */
    if (output > p->out_max) output = p->out_max;
    if (output < p->out_min) output = p->out_min;

    p->prev_output = output;
    return output;
}

/**
 * @brief   外部设置当前输出值
 *
 * 典型用法：从"手动模式"切换到"自动模式"时调用。
 * 将 prev_output 设为当前手动输出值，避免 PID 从 0 开始产生阶跃。
 *
 * 同时清除误差历史，防止旧的误差数据影响微分项计算。
 */
void IncPID_SetOutput(IncPID_t *p, float output)
{
    p->prev_output     = output;
    p->prev_error      = 0.0f;
    p->prev_prev_error = 0.0f;
}
