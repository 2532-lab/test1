/**
 * @file    PID.c
 * @brief   增量式 PID 控制器 —— 实现（含抗积分饱和 + 输出限幅）
 *
 * ======================== 抗积分饱和原理 ========================
 *
 *   增量式 PID 天然比位置式更抗饱和（没有 Σe 累加器），
 *   但仍有一个隐蔽风险：当输出持续饱和时，prev_output 钉在极值，
 *   Ki×error×dt 的微小增量被 P 的负增量抵消，误差历史依然在更新。
 *   当条件恢复正常时，微分项可能出现阶跃。
 *
 *   本实现采用"条件积分"（Conditional Integration）：
 *     当 output ≥ out_max 且 error > 0（需要更大输出但已到顶）
 *     或 output ≤ out_min 且 error < 0（需要更小输出但已到底），
 *     跳过积分项的累加。
 *
 *   效果：车轮卡住时积分不堆积，松开瞬间正常起步，
 *   不会出现"积分弹射"。
 */

#include "PID.h"
#include <math.h>
#include <stdbool.h>

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
    p->deadband        = 0.0f;
}

float IncPID_Compute(IncPID_t *p, float measurement)
{
    float error = p->setpoint - measurement;

    /* 死区：误差太小→不调整，但更新误差历史（平滑退出死区） */
    if (fabsf(error) < p->deadband) {
        p->prev_prev_error = p->prev_error;
        p->prev_error      = error;
        return p->prev_output;
    }

    /* ====== P 增量 ====== */
    float p_term = p->Kp * (error - p->prev_error);

    /* ====== I 增量（含抗积分饱和） ====== */
    float i_term = 0.0f;
    {
        /*
         * 条件积分判断：
         *   输出已饱和 且 误差方向与饱和方向一致 → 不累加积分
         *
         * 例：
         *   prev_output >= out_max(80) 且 error > 0 → 电机已在最大输出，
         *   轮子仍然不够快，但再加大输出也没用了 → 跳过积分
         *
         *   prev_output <= out_min(0) 且 error < 0 → 已在最小输出，
         *   再减小也没用 → 跳过积分
         */
        bool saturated_high = (p->prev_output >= p->out_max) && (error > 0.0f);
        bool saturated_low  = (p->prev_output <= p->out_min) && (error < 0.0f);

        if (!saturated_high && !saturated_low) {
            i_term = p->Ki * error * p->dt;
        }
        /* 饱和时 i_term=0，积分不累加 */
    }

    /* ====== D 增量 ====== */
    float d_term = 0.0f;
    if (p->dt > 0.0f) {
        d_term = p->Kd * (error - 2.0f * p->prev_error
                                 + p->prev_prev_error) / p->dt;
    }

    /* ====== 叠加 → 限幅 → 输出 ====== */
    float delta  = p_term + i_term + d_term;
    float output = p->prev_output + delta;

    if (output > p->out_max) output = p->out_max;
    if (output < p->out_min) output = p->out_min;

    /* 更新历史 */
    p->prev_prev_error = p->prev_error;
    p->prev_error      = error;
    p->prev_output     = output;

    return output;
}

/**
 * @brief   紧急清零：输出归零 + 历史全部擦除
 *
 * 适用场景：
 *   1. 停车（LineFollower_Stop）
 *   2. PIVOT 旋转完成退出（防止积分弹射）
 *   3. 轮子离地后再落地
 *
 * 为什么不能用 SetOutput(0)？
 *   SetOutput 只改 prev_output，不清误差历史。
 *   下次 Compute 时，prev_error 和 prev_prev_error 仍是停车前的旧值，
 *   D 项可能产生一个尖锐的阶跃。
 */
void IncPID_Reset(IncPID_t *p)
{
    p->prev_output     = 0.0f;
    p->prev_error      = 0.0f;
    p->prev_prev_error = 0.0f;
}

void IncPID_SetOutput(IncPID_t *p, float output)
{
    p->prev_output     = output;
    p->prev_error      = 0.0f;
    p->prev_prev_error = 0.0f;
}
