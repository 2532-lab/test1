/**
 * @file    line_follower.c
 * @brief   五路灰度传感器循迹 —— P 控制 + 直角弯 pivot
 *
 * ======================== 整体架构 ========================
 *
 *   每帧调用一次 LineFollower_Update()，内部分三个阶段：
 *     1. PIVOT 原地旋转（传感器引导）
 *     2. 直角弯检测（外侧两路同黑 → 进入 PIVOT）
 *     3. 正常循迹（P 控制器 = 位置偏差 → 左右差速）
 *
 * ======================== 传感器映射 ========================
 *
 *   S1(最左)     S2        S3(中)     S4        S5(最右)
 *   PB27         PA29      PB23       PB12      PA12
 *
 *   电平: 0=黑线, 1=白面
 *   权重: -2000 / -1000 / 0 / +1000 / +2000
 *
 * ======================== 直角弯检测 ========================
 *
 *   右直角: (S4==0 或 S5==0) 且 S1+S2 全白 → turn_dir=+1
 *   左直角: (S1==0 或 S2==0) 且 S4+S5 全白 → turn_dir=-1
 *   启动保护: 上电 30 帧内不检测
 *
 * ======================== PIVOT 旋转 ========================
 *
 *   右转: Motor_SetAllForward() + Motor_SetRightReverse() → 车右转
 *   左转: Motor_SetAllForward() + Motor_SetLeftReverse()  → 车左转
 *   停止: |pos|<600 连续3帧 或 超时800帧
 *
 * ======================== P 控制器公式 ========================
 *
 *   rpm_diff = Kp × position
 *   dp       = rpm_diff × (50 / max_rpm_diff)
 *   left     = bp + dp, right = bp - dp
 *   bp       = 33 - |pos|×0.018 (动态基础速度)
 *
 * ======================== 依赖 ========================
 *
 *   motor.h      全部方向/PWM 操作通过 motor.c API，不直接碰 GPIO
 *   grayscale.h  传感器读数 + 位置计算
 *   PID.h        增量式 PID（速度环存根，未启用）
 */

#include "line_follower.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ===================================================================
 *  SpeedCtrl 速度闭环存根（未启用）
 * =================================================================== */

static SpeedCtrlCfg_t sc_def(void)
{
    SpeedCtrlCfg_t c;
    c.Kp = 1.5f; c.Ki = 1.0f; c.Kd = 0;
    c.dt = 0.01f; c.max_duty = 80;
    return c;
}

void SpeedCtrl_Init(SpeedCtrl_t *s, const SpeedCtrlCfg_t *c)
{
    memset(s, 0, sizeof(*s));
    s->cfg = c ? *c : sc_def();
    IncPID_Init(&s->pid, s->cfg.Kp, s->cfg.Ki, s->cfg.Kd,
                s->cfg.dt, 0, 80);
    IncPID_SetOutput(&s->pid, 25);
    s->output_duty = 25;
}

int16_t SpeedCtrl_Update(SpeedCtrl_t *s, float t, float a)
{
    (void)t; (void)a; (void)s;
    return 0;
}

/* ===================================================================
 *  默认配置
 * =================================================================== */

LineFollowerCfg_t LineFollower_GetDefaultCfg(void)
{
    LineFollowerCfg_t c;
    c.Kp_pos       = 0.20f;
    c.Ki_pos       = 0;
    c.Kd_pos       = 0;
    c.deadband_pos = 2.0f;
    c.max_rpm_diff = 55.0f;
    c.speed_cfg    = sc_def();
    c.base_rpm     = 10.0f;
    c.dt           = 0.01f;
    return c;
}

void LineFollower_Init(LineFollower_t *lf, const LineFollowerCfg_t *cfg)
{
    if (!lf) return;
    memset(lf, 0, sizeof(*lf));
    lf->cfg = cfg ? *cfg : LineFollower_GetDefaultCfg();
    SpeedCtrl_Init(&lf->speed_left,  &lf->cfg.speed_cfg);
    SpeedCtrl_Init(&lf->speed_right, &lf->cfg.speed_cfg);
}

/* ===================================================================
 *  主更新函数
 * =================================================================== */
void LineFollower_Update(LineFollower_t *lf, float rl, float rr)
{
    if (!lf) return;
    float enc_l = rl, enc_r = rr;

    /* 读取传感器 */
    Grayscale_Read(&lf->gs);
    lf->position = Grayscale_GetPosition(&lf->gs);
    float pos = (float)lf->position;

    /* ================================================================
     *  阶段1：PIVOT 原地旋转
     * ================================================================ */
    if (lf->turn_state == LF_TURN_PIVOT) {

        if (lf->turn_timeout > 40
            && lf->gs.line_detected
            && fabsf(pos) < 600.0f) {
            if (++lf->turn_found_cnt > 3) {
                /* 旋转完成 */
                Motor_StopAll();
                Motor_SetAllForward();
                lf->turn_state     = LF_TURN_IDLE;
                lf->run_frames     = 0;
                lf->rpm_diff       = 0;
                lf->lost_count     = 0;
                lf->line_lost      = false;
                lf->turn_found_cnt = 0;
                return;
            }
        } else {
            lf->turn_found_cnt = 0;
        }

        if (++lf->turn_timeout > 800) {
            Motor_StopAll();
            Motor_SetAllForward();
            lf->turn_state = LF_TURN_IDLE;
            lf->run_frames = 0;
            return;
        }

        /* 执行 pivot：一侧前进 + 另一侧倒退 */
        uint8_t sd = 65;
        if (lf->turn_dir > 0) {
            /* 右转：右侧倒退 + 左侧前进 */
            Motor_SetAllForward();
            Motor_SetRightReverse();
        } else {
            /* 左转：左侧倒退 + 右侧前进 */
            Motor_SetAllForward();
            Motor_SetLeftReverse();
        }
        Motor_SetAllSpeed(sd);
        lf->left_duty = lf->right_duty = sd;
        return;
    }

    /* ================================================================
     *  阶段2：直角弯检测
     * ================================================================ */
    if (lf->gs.line_detected) lf->run_frames++;

    if (lf->turn_state == LF_TURN_IDLE && lf->run_frames > 30) {
        bool right_turn = (!lf->gs.raw.s4 || !lf->gs.raw.s5) &&
                          (lf->gs.raw.s1 && lf->gs.raw.s2);
        bool left_turn  = (!lf->gs.raw.s1 || !lf->gs.raw.s2) &&
                          (lf->gs.raw.s4 && lf->gs.raw.s5);

        if (right_turn || left_turn) {
            lf->turn_state     = LF_TURN_PIVOT;
            lf->turn_dir       = right_turn ? 1 : -1;
            lf->turn_timeout   = 0;
            lf->turn_found_cnt = 0;
            Motor_StopAll();
            return;
        }
    }

    /* ================================================================
     *  阶段3：正常循迹 —— P 控制器
     * ================================================================ */

    /* 丢线处理 */
    if (!lf->gs.line_detected) {
        lf->lost_count++;
        lf->line_lost = true;
        if (lf->lost_count > 300) {
            LineFollower_Stop(lf);
            return;
        }
        lf->rpm_diff = (lf->run_frames > 30)
                       ? lf->last_rpm_diff * 0.5f
                       : 0;
        goto out;
    }

    lf->lost_count = 0;
    lf->line_lost  = false;
    {
        float kp = lf->cfg.Kp_pos;
        lf->rpm_diff = kp * pos;
        if (lf->rpm_diff >  lf->cfg.max_rpm_diff)
            lf->rpm_diff =  lf->cfg.max_rpm_diff;
        if (lf->rpm_diff < -lf->cfg.max_rpm_diff)
            lf->rpm_diff = -lf->cfg.max_rpm_diff;
        if (fabsf(lf->rpm_diff) < lf->cfg.deadband_pos)
            lf->rpm_diff = 0;
        lf->last_rpm_diff = lf->rpm_diff;
    }

out:
    /* PWM 输出 */
    {
        float dp = lf->rpm_diff * (50.0f / lf->cfg.max_rpm_diff);
        if (dp >  42) dp =  42;
        if (dp < -42) dp = -42;

        float bp = 33.0f - fabsf(pos) * 0.018f;
        if (bp < 18.0f) bp = 18.0f;

        { float target_rpm = bp * 2.8f;
          float avg_rpm    = (enc_l + enc_r) * 0.5f;
          float speed_err  = target_rpm - avg_rpm;
          float trim       = 0.08f * speed_err;
          if (trim >  5.0f) trim =  5.0f;
          if (trim < -5.0f) trim = -5.0f;
          bp += trim;
          if (bp < 10.0f) bp = 10.0f;
          if (bp > 40.0f) bp = 40.0f; }

        lf->left_duty  = (int16_t)(bp + dp);
        lf->right_duty = (int16_t)(bp - dp);

        if (lf->left_duty  < 3)  lf->left_duty  = 3;
        if (lf->left_duty  > 80) lf->left_duty  = 80;
        if (lf->right_duty < 3)  lf->right_duty = 3;
        if (lf->right_duty > 80) lf->right_duty = 80;

        /* 宏定义已修正：right→右轮, left→左轮，不再对调 */
        Motor_turn_up(0,
                      (uint8_t)lf->right_duty, (uint8_t)lf->right_duty,
                      (uint8_t)lf->left_duty,  (uint8_t)lf->left_duty);
    }
}

/* ===================================================================
 *  停车
 * =================================================================== */
void LineFollower_Stop(LineFollower_t *lf)
{
    Motor_StopAll();
    Motor_SetAllForward();
    if (lf) {
        lf->turn_state    = LF_TURN_IDLE;
        lf->rpm_diff      = 0;
        lf->last_rpm_diff = 0;
        lf->left_duty     = 0;
        lf->right_duty    = 0;
    }
}

int16_t LineFollower_GetPosition(LineFollower_t *lf)
    { return lf ? lf->position : 0; }
float   LineFollower_GetRpmDiff(LineFollower_t *lf)
    { return lf ? lf->rpm_diff : 0; }
bool    LineFollower_IsLineLost(LineFollower_t *lf)
    { return lf ? lf->line_lost : true; }
