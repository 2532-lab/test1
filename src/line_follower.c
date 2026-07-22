/**
 * @file    line_follower.c
 * @brief   灰度循迹 —— P控制 + 传感器引导直角弯（b1b6fa6 移植版）
 *
 * == 整体架构 ==
 *   每帧调用一次 LineFollower_Update()，内部分三个阶段：
 *     阶段1. 直角弯平滑转向 — 低速 + 大差速，四轮前进拐弯
 *     阶段2. 触发检测 — 内侧有黑+对侧全白 → 立即进入转弯
 *     阶段3. 正常循迹 — PID控制，自适应Kp平滑调速
 *
 * == 直角弯触发条件 ==
 *   右直角：S1或S2黑(右侧见线) 且 S4+S5白 → turn_dir=+1
 *   左直角：S4或S5黑(左侧见线) 且 S1+S2白 → turn_dir=-1
 *   启动保护：上电后30帧内不检测
 *
 * == 直角弯执行 ==
 *   1. 进入转弯 → 四轮前进，低速(12%) + 大差速(±30%)
 *   2. 前60帧强制转 → 防误退出
 *   3. 60帧后：线居中(|pos|<600)连续3帧 → 退出
 *   4. 超时1500帧强制退出
 *
 * == 循迹 PID ==
 *   Kp=0.05→0.18(线性), Ki=0.02, Kd=0.12
 *   bp=25-|pos|×0.02, 最低12%
 */

#include "line_follower.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ===================================================================
 *  速度闭环桩
 * =================================================================== */

void SpeedCtrl_Init(SpeedCtrl_t *s, const SpeedCtrlCfg_t *c) { (void)s; (void)c; }
int16_t SpeedCtrl_Update(SpeedCtrl_t *s, float t, float a) { (void)s; (void)t; (void)a; return 0; }

/* ===================================================================
 *  默认配置
 * =================================================================== */

LineFollowerCfg_t LineFollower_GetDefaultCfg(void) {
    LineFollowerCfg_t c;
    c.Kp_pos       = 0.20f;
    c.Ki_pos       = 0;
    c.Kd_pos       = 0;
    c.deadband_pos = 2.0f;
    c.max_rpm_diff = 55.0f;
    c.base_rpm     = 10.0f;
    c.dt           = 0.01f;
    return c;
}

void LineFollower_Init(LineFollower_t *lf, const LineFollowerCfg_t *cfg) {
    if (!lf) return;
    memset(lf, 0, sizeof(*lf));
    lf->cfg = cfg ? *cfg : LineFollower_GetDefaultCfg();
}

/* ===================================================================
 *  主更新函数
 * =================================================================== */
void LineFollower_Update(LineFollower_t *lf, float rl, float rr) {
    (void)rl; (void)rr;
    if (!lf) return;

    Grayscale_Read(&lf->gs);
    lf->position = Grayscale_GetPosition(&lf->gs);
    float pos = (float)lf->position;

    /* ================================================================
     *  阶段1：直角弯平滑转向（降速 + 大差速转弯，不原地旋转）
     * ================================================================ */
    if (lf->turn_state == LF_TURN_PIVOT) {

        /* 退出条件：线回到中心附近，连续2帧（前60帧强制转，防误退出） */
        if (lf->turn_timeout > 60
            && lf->gs.line_detected && fabsf(pos) < 600.0f) {
            if (++lf->turn_found_cnt > 2) {
                lf->turn_state     = LF_TURN_IDLE;
                lf->run_frames     = 0;
                lf->rpm_diff       = 0;
                lf->lost_count     = 0;
                lf->line_lost      = false;
                lf->turn_found_cnt = 0;
                lf->pos_integral   = 0;
                lf->pos_prev       = pos;
                return;
            }
        } else {
            lf->turn_found_cnt = 0;
        }

        /* 超时退出 */
        if (++lf->turn_timeout > 1500) {
            Motor_StopAll();
            Motor_SetAllForward();
            lf->turn_state   = LF_TURN_IDLE;
            lf->run_frames   = 0;
            lf->pos_integral = 0;
            lf->pos_prev     = 0;
            return;
        }

        /* 平滑转弯：低速 + 大差速 */
        float bp = 12.0f;
        float dp = (lf->turn_dir > 0) ? 30.0f : -30.0f;

        lf->left_duty  = (int16_t)(bp + dp);
        lf->right_duty = (int16_t)(bp - dp);

        if (lf->left_duty  < 3)  lf->left_duty  = 3;
        if (lf->left_duty  > 80) lf->left_duty  = 80;
        if (lf->right_duty < 3)  lf->right_duty = 3;
        if (lf->right_duty > 80) lf->right_duty = 80;

        Motor_SetAllForward();
        Set_Motor_FrontRight_Speed((uint8_t)lf->right_duty);
        Set_Motor_BackRight_Speed((uint8_t)lf->right_duty);
        Set_Motor_FrontLeft_Speed((uint8_t)lf->left_duty);
        Set_Motor_BackLeft_Speed((uint8_t)lf->left_duty);
        return;
    }

    /* ================================================================
     *  阶段2：直角弯检测（即时触发，无去抖）
     *
     *  S1/S2=右侧, S4/S5=左侧（权重已修正，与旧版相反）
     *  右直角：S1/S2有黑(右侧见线) + S4/S5全白 → 线在右边
     *  左直角：S4/S5有黑(左侧见线) + S1/S2全白 → 线在左边
     * ================================================================ */
    if (lf->gs.line_detected) lf->run_frames++;

    if (lf->turn_state == LF_TURN_IDLE && lf->run_frames > 30) {

        bool right_turn = (!lf->gs.raw.s1 || !lf->gs.raw.s2) &&
                          (lf->gs.raw.s4 && lf->gs.raw.s5);
        bool left_turn  = (!lf->gs.raw.s4 || !lf->gs.raw.s5) &&
                          (lf->gs.raw.s1 && lf->gs.raw.s2);

        if (right_turn || left_turn) {
            lf->turn_state     = LF_TURN_PIVOT;
            lf->turn_dir       = right_turn ? 1 : -1;
            lf->turn_timeout   = 0;
            lf->turn_found_cnt = 0;
            return;
        }
    }

    /* ================================================================
     *  阶段3：正常循迹 —— 纯 P 控制
     * ================================================================ */

    if (!lf->gs.line_detected) {
        lf->lost_count++;
        lf->line_lost = true;
        if (lf->lost_count > 300) {
            LineFollower_Stop(lf);
            return;
        }
        lf->rpm_diff = lf->last_rpm_diff * 0.5f;
        /* 丢线时冻结积分，防止积压 */
        goto out;
    }

    lf->lost_count = 0;
    lf->line_lost  = false;
    {
        /* Kp 线性平滑：直线柔和，直角发力 */
        float pos_abs = fabsf(pos);
        float kp = 0.05f + (pos_abs / 2000.0f) * 0.13f;

        /* P 项 */
        float p_term = kp * pos;

        /* I 项 */
        float ki = 0.02f;
        lf->pos_integral += pos * 0.002f;
        if (lf->pos_integral >  200.0f) lf->pos_integral =  200.0f;
        if (lf->pos_integral < -200.0f) lf->pos_integral = -200.0f;
        float i_term = ki * lf->pos_integral;

        /* D 项：抑制摆头 */
        float kd = 0.12f;
        float d_term = kd * (pos - lf->pos_prev);
        lf->pos_prev = pos;

        lf->rpm_diff = p_term + i_term + d_term;
        if (lf->rpm_diff >  lf->cfg.max_rpm_diff) lf->rpm_diff =  lf->cfg.max_rpm_diff;
        if (lf->rpm_diff < -lf->cfg.max_rpm_diff) lf->rpm_diff = -lf->cfg.max_rpm_diff;
        if (fabsf(lf->rpm_diff) < lf->cfg.deadband_pos) lf->rpm_diff = 0;
        lf->last_rpm_diff = lf->rpm_diff;
    }

out:
    {
        float dp = lf->rpm_diff * (50.0f / lf->cfg.max_rpm_diff);
        if (dp >  42) dp =  42;
        if (dp < -42) dp = -42;

        /* 动态基础速度：偏差越大越慢，给差速腾空间 */
        float bp = 25.0f - fabsf(pos) * 0.020f;
        if (bp < 12.0f) bp = 12.0f;

        lf->left_duty  = (int16_t)(bp + dp);
        lf->right_duty = (int16_t)(bp - dp);

        if (lf->left_duty  < 3)  lf->left_duty  = 3;
        if (lf->left_duty  > 80) lf->left_duty  = 80;
        if (lf->right_duty < 3)  lf->right_duty = 3;
        if (lf->right_duty > 80) lf->right_duty = 80;

        Set_Motor_FrontRight_Speed((uint8_t)lf->right_duty);
        Set_Motor_BackRight_Speed((uint8_t)lf->right_duty);
        Set_Motor_FrontLeft_Speed((uint8_t)lf->left_duty);
        Set_Motor_BackLeft_Speed((uint8_t)lf->left_duty);
    }
}

/* ===================================================================
 *  停车
 * =================================================================== */
void LineFollower_Stop(LineFollower_t *lf) {
    Motor_StopAll();
    Motor_SetAllForward();
    if (lf) {
        lf->turn_state     = LF_TURN_IDLE;
        lf->rpm_diff       = 0;
        lf->last_rpm_diff  = 0;
        lf->left_duty      = 0;
        lf->right_duty     = 0;
        lf->pos_integral   = 0;
        lf->pos_prev       = 0;
    }
}

int16_t LineFollower_GetPosition(LineFollower_t *lf) { return lf ? lf->position : 0; }
float   LineFollower_GetRpmDiff(LineFollower_t *lf)   { return lf ? lf->rpm_diff : 0; }
bool    LineFollower_IsLineLost(LineFollower_t *lf)    { return lf ? lf->line_lost : true; }
