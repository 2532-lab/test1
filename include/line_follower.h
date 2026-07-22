/**
 * @file    line_follower.h
 * @brief   灰度循迹模块 —— 数据结构与接口
 *
 * ======================== 模块组成 ========================
 *
 *   1. SpeedCtrl  速度闭环（增量式 PID）
 *     接口保留但当前未启用。循迹直接输出 PWM 占空比（开环），
 *     跳过编码器速度反馈。恢复方法见 SpeedCtrl_Update。
 *
 *   2. LineFollower  主循迹控制器
 *     P 控制（位置偏差 → 差速）+ 传感器引导直角弯 PIVOT + 丢线处理
 *
 * ======================== 典型用法 ========================
 *
 * @code
 *   LineFollower_t lf;
 *   LineFollower_Init(&lf, NULL);           // 使用默认参数
 *   Motor_PWM_Init();
 *   Encoder_Init();
 *
 *   while (1) {
 *       float rpm_l, rpm_r;
 *       Encoder_GetSpeed(ENCODER_C2, 0.01f, &rpm_l);
 *       Encoder_GetSpeed(ENCODER_C1, 0.01f, &rpm_r);
 *       LineFollower_Update(&lf, rpm_l, rpm_r);
 *       delay_ms(2);                        // ~500Hz 控制频率
 *   }
 * @endcode
 */

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>
#include <stdbool.h>
#include "PID.h"
#include "grayscale.h"

/* ===================================================================
 *  速度闭环 (SpeedCtrl) —— 增量式 PID（未启用）
 * =================================================================== */

typedef struct {
    float   Kp, Ki, Kd;     /**< PID 增益 */
    float   dt;              /**< 控制周期（秒） */
    int16_t max_duty;        /**< 输出占空比上限（%） */
} SpeedCtrlCfg_t;

typedef struct {
    SpeedCtrlCfg_t cfg;      /**< 配置参数 */
    IncPID_t       pid;      /**< 增量式 PID 实例 */
    float    target_rpm;     /**< 目标转速（rpm） */
    float    actual_rpm;     /**< 实际转速（编码器测得） */
    int16_t  output_duty;    /**< 输出占空比（%） */
} SpeedCtrl_t;

void SpeedCtrl_Init(SpeedCtrl_t *sc, const SpeedCtrlCfg_t *cfg);
int16_t SpeedCtrl_Update(SpeedCtrl_t *sc, float target_rpm, float actual_rpm);

/* ===================================================================
 *  灰度循迹 (LineFollower)
 * =================================================================== */

/** 循迹器配置参数 */
typedef struct {
    float   Kp_pos;          /**< 位置环 P 增益（默认 0.20） */
    float   Ki_pos;          /**< 位置环 I 增益（当前 P 控制，未用） */
    float   Kd_pos;          /**< 位置环 D 增益（当前 P 控制，未用） */
    float   deadband_pos;    /**< 输出死区（rpm），< 此值视为 0 */
    float   max_rpm_diff;    /**< 差速输出上限（rpm），默认 55 */
    SpeedCtrlCfg_t speed_cfg; /**< 速度环配置（未启用） */
    float   base_rpm;        /**< 基准转速（已被 PWM 直接控制覆盖） */
    float   dt;              /**< 控制周期（秒） */
} LineFollowerCfg_t;

/** 转向状态 */
enum {
    LF_TURN_IDLE = 0,   /**< 正常循迹 */
    LF_TURN_BRAKE,       /**< 直角刹车：倒转减速，消耗惯性 */
    LF_TURN_PIVOT,       /**< 直角弯原地旋转中 */
    LF_TURN_FWD          /**< 保留 */
};

/** 循迹器运行时句柄 */
typedef struct {
    LineFollowerCfg_t cfg;
    SpeedCtrl_t speed_left;      /**< 左轮速度环 */
    SpeedCtrl_t speed_right;     /**< 右轮速度环 */
    Grayscale_t gs;               /**< 灰度传感器句柄 */

    /* 位置与差速 */
    int16_t position;             /**< 当前加权位置 [-2000, +2000] */
    float   rpm_diff;             /**< 当前目标差速（rpm） */
    float   last_rpm_diff;        /**< 上一帧差速（丢线恢复用） */

    /* PWM */
    int16_t left_duty;            /**< 左轮 PWM 占空比 [3, 80]% */
    int16_t right_duty;           /**< 右轮 PWM 占空比 [3, 80]% */

    /* 丢线 */
    bool     line_lost;           /**< 当前是否丢线（5 路全白） */
    uint16_t lost_count;          /**< 连续丢线帧数 */

    /* PID 状态 */
    float    pos_integral;        /**< 位置积分累加器 */
    float    pos_prev;            /**< 上一帧位置（D 项备用） */

    /* 直角弯状态机 */
    uint8_t  turn_state;          /**< LF_TURN_IDLE / LF_TURN_PIVOT */
    int8_t   turn_dir;            /**< +1=右转, -1=左转 */
    uint16_t turn_timeout;        /**< PIVOT 帧计数（超时=800帧） */
    uint16_t run_frames;          /**< 启动后累计帧数（防误触发） */
    uint8_t  turn_found_cnt;      /**< PIVOT 寻线去抖计数器 */
    uint8_t  turn_fwd_count;      /**< PIVOT 退出后直行恢复帧计数 */
    int8_t   turn_candidate_dir;   /**< 直角弯候选方向：+1=右, -1=左, 0=无 */
    uint8_t  turn_candidate_cnt;   /**< 直角弯候选连续帧计数（去抖） */
} LineFollower_t;

/* ===================================================================
 *  API
 * =================================================================== */

void LineFollower_Init(LineFollower_t *lf, const LineFollowerCfg_t *cfg);
void LineFollower_Update(LineFollower_t *lf, float rpm_left, float rpm_right);
void LineFollower_Stop(LineFollower_t *lf);
int16_t LineFollower_GetPosition(LineFollower_t *lf);
float   LineFollower_GetRpmDiff(LineFollower_t *lf);
bool    LineFollower_IsLineLost(LineFollower_t *lf);
LineFollowerCfg_t LineFollower_GetDefaultCfg(void);

#endif /* LINE_FOLLOWER_H */
