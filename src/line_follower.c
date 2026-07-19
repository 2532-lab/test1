/**
 * @file    line_follower.c
 * @brief   灰度循迹 —— P控制 + 传感器引导直角弯（电机宏定义已修正）
 *
 * == 整体架构 ==
 *   每帧调用一次 LineFollower_Update()，内部分三个阶段：
 *     阶段1. PIVOT — 传感器引导原地旋转，转到线居中为止
 *     阶段2. 触发检测 — 外侧两路同黑+对侧全白 → 立即进入PIVOT
 *     阶段3. 正常循迹 — 纯P控制，position越大差速越大，动态降速
 *
 * == 电机宏定义（已修正）==
 *   函数名 = 物理轮子，Motor_turn_up 参数不再左右对调
 */

#include "line_follower.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ===================================================================
 *  方向 GPIO 控制（H 桥：Pin1=0 Pin2=1 → 前进）
 * =================================================================== */

/** 左侧两轮倒退（左转 pivot 时使用） */
static void mdir_left_rev(void) {
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_1_PORT, GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_2_PORT, GPIO_MOTOR_PIN_front_left_2_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_1_PORT,  GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_2_PORT,  GPIO_MOTOR_PIN_back_left_2_PIN);
}

/** 右侧两轮倒退（右转 pivot 时使用） */
static void mdir_right_rev(void) {
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT, GPIO_MOTOR_PIN_front_right_2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT, GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_1_PORT,  GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_2_PORT,  GPIO_MOTOR_PIN_back_right_2_PIN);
}

/** 全部四轮前进方向 */
static void mdir_fwd(void) {
    /* 右前轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT, GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT, GPIO_MOTOR_PIN_front_right_2_PIN);
    /* 右后轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_1_PORT,  GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_2_PORT,  GPIO_MOTOR_PIN_back_right_2_PIN);
    /* 左前轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_1_PORT,  GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_2_PORT,  GPIO_MOTOR_PIN_front_left_2_PIN);
    /* 左后轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_1_PORT,   GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_2_PORT,   GPIO_MOTOR_PIN_back_left_2_PIN);
}

/** 全部 PWM 置零 */
static void mstop(void) {
    Set_Motor_FrontLeft_Speed(0);  Set_Motor_BackLeft_Speed(0);
    Set_Motor_FrontRight_Speed(0); Set_Motor_BackRight_Speed(0);
}

/* ===================================================================
 *  SpeedCtrl 存根（速度闭环——未启用）
 * =================================================================== */

static SpeedCtrlCfg_t sc_def(void) {
    SpeedCtrlCfg_t c;
    c.Kp=1.5f; c.Ki=1.0f; c.Kd=0; c.dt=0.01f; c.max_duty=80;
    return c;
}

void SpeedCtrl_Init(SpeedCtrl_t *s, const SpeedCtrlCfg_t *c) {
    memset(s,0,sizeof(*s));
    s->cfg = c ? *c : sc_def();
    IncPID_Init(&s->pid, s->cfg.Kp, s->cfg.Ki, s->cfg.Kd, s->cfg.dt, 0, 80);
    IncPID_SetOutput(&s->pid, 25);
    s->output_duty = 25;
}

int16_t SpeedCtrl_Update(SpeedCtrl_t *s, float t, float a) {
    (void)t; (void)a; (void)s; return 0;
}

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
    c.speed_cfg    = sc_def();
    c.base_rpm     = 10.0f;
    c.dt           = 0.01f;
    return c;
}

void LineFollower_Init(LineFollower_t *lf, const LineFollowerCfg_t *cfg) {
    if (!lf) return;
    memset(lf, 0, sizeof(*lf));
    lf->cfg = cfg ? *cfg : LineFollower_GetDefaultCfg();
    SpeedCtrl_Init(&lf->speed_left,  &lf->cfg.speed_cfg);
    SpeedCtrl_Init(&lf->speed_right, &lf->cfg.speed_cfg);
}

/* ===================================================================
 *  主更新函数
 * =================================================================== */
void LineFollower_Update(LineFollower_t *lf, float rl, float rr) {
    if (!lf) return;
    float enc_l = rl, enc_r = rr;

    /* ---- 读取灰度传感器 ---- */
    Grayscale_Read(&lf->gs);
    lf->position = Grayscale_GetPosition(&lf->gs);
    float pos = (float)lf->position;

    /* ================================================================
     *  阶段1：PIVOT 原地旋转
     *
     *  turn_dir=+1（右转）→ 右侧倒退 + 左侧前进 → 车体右转
     *  turn_dir=-1（左转）→ 左侧倒退 + 右侧前进 → 车体左转
     * ================================================================ */
    if (lf->turn_state == LF_TURN_PIVOT) {

        if (lf->turn_timeout > 40
            && lf->gs.line_detected
            && fabsf(pos) < 600.0f) {
            if (++lf->turn_found_cnt > 3) {
                mstop();
                mdir_fwd();
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
            mstop();
            mdir_fwd();
            lf->turn_state = LF_TURN_IDLE;
            lf->run_frames = 0;
            return;
        }

        uint8_t sd = 65;
        if (lf->turn_dir > 0) {
            /* 右转：右侧倒退 + 左侧前进 */
            mdir_fwd();
            mdir_right_rev();
        } else {
            /* 左转：左侧倒退 + 右侧前进 */
            mdir_fwd();
            mdir_left_rev();
        }
        Set_Motor_FrontLeft_Speed(sd);
        Set_Motor_BackLeft_Speed(sd);
        Set_Motor_FrontRight_Speed(sd);
        Set_Motor_BackRight_Speed(sd);
        lf->left_duty = lf->right_duty = sd;
        return;
    }

    /* ================================================================
     *  阶段2：直角弯检测
     *
     *  传感器映射：S1/S2=右侧, S4/S5=左侧
     *  右直角：右侧(S1/S2)有黑线 + 左侧(S4+S5)全白
     *  左直角：左侧(S4/S5)有黑线 + 右侧(S1+S2)全白
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
            mstop();
            return;
        }
    }

    /* ================================================================
     *  阶段3：正常循迹 —— 纯 P 控制器
     * ================================================================ */

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
        if (lf->rpm_diff >  lf->cfg.max_rpm_diff) lf->rpm_diff =  lf->cfg.max_rpm_diff;
        if (lf->rpm_diff < -lf->cfg.max_rpm_diff) lf->rpm_diff = -lf->cfg.max_rpm_diff;
        if (fabsf(lf->rpm_diff) < lf->cfg.deadband_pos) lf->rpm_diff = 0;
        lf->last_rpm_diff = lf->rpm_diff;
    }

out:
    /* ---- PWM 输出（宏定义已修正，不再左右对调）---- */
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

        /* 参数直接对应物理轮子：right_duty→右轮, left_duty→左轮 */
        Motor_turn_up(0,
                      (uint8_t)lf->right_duty, (uint8_t)lf->right_duty,
                      (uint8_t)lf->left_duty,  (uint8_t)lf->left_duty);
    }
}

/* ===================================================================
 *  停止函数
 * =================================================================== */
void LineFollower_Stop(LineFollower_t *lf) {
    mstop();
    mdir_fwd();
    if (lf) {
        lf->turn_state    = LF_TURN_IDLE;
        lf->rpm_diff      = 0;
        lf->last_rpm_diff = 0;
        lf->left_duty     = 0;
        lf->right_duty    = 0;
    }
}

int16_t LineFollower_GetPosition(LineFollower_t *lf) { return lf ? lf->position : 0; }
float   LineFollower_GetRpmDiff(LineFollower_t *lf)   { return lf ? lf->rpm_diff : 0; }
bool    LineFollower_IsLineLost(LineFollower_t *lf)    { return lf ? lf->line_lost : true; }
