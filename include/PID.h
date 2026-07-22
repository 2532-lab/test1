/**
 * @file    PID.h
 * @brief   增量式 PID 控制器 —— 声明（含抗积分饱和）
 */

#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float dt;
    float prev_error;
    float prev_prev_error;
    float prev_output;
    float out_min, out_max;
    float deadband;
} IncPID_t;

void IncPID_Init(IncPID_t *p, float kp, float ki, float kd,
                 float dt, float out_min, float out_max);

/**
 * @brief   增量式 PID 计算（含抗积分饱和）
 *
 * 抗积分饱和（Conditional Integration）：
 *   当输出已饱和（output==out_max 且 error>0，或 output==out_min 且 error<0），
 *   跳过积分项的累加——防止"积分弹射"。
 *
 *   例：车轮卡住 rpm=0，target=20，error=20 持续存在。
 *   没有抗饱和：每次 Δu += Ki×20×dt，积分不断累加。
 *   松开车轮瞬间输出 80% + 历史积分 → 车飞出去。
 *   有抗饱和：输出到 80% 后，error 仍 >0 → 停止累加 Ki 项。
 *   松开车轮瞬间输出 = 80% + 仅 P 增量 → 正常起步。
 */
float IncPID_Compute(IncPID_t *p, float measurement);

/** 手动清零输出和误差历史（模式切换、停车时调用） */
void IncPID_Reset(IncPID_t *p);

/** 设置当前输出值（从手动模式切换到 PID 时初始化） */
void IncPID_SetOutput(IncPID_t *p, float output);

#endif /* PID_H */
