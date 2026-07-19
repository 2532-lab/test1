#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>
#include <stdbool.h>
#include "PID.h"
#include "grayscale.h"

typedef struct {
    float   Kp, Ki, Kd, dt;
    int16_t max_duty;
} SpeedCtrlCfg_t;

typedef struct {
    SpeedCtrlCfg_t cfg;
    IncPID_t       pid;
    float    target_rpm;
    float    actual_rpm;
    int16_t  output_duty;
} SpeedCtrl_t;

void SpeedCtrl_Init(SpeedCtrl_t *sc, const SpeedCtrlCfg_t *cfg);
int16_t SpeedCtrl_Update(SpeedCtrl_t *sc, float target_rpm, float actual_rpm);

typedef struct {
    float   Kp_pos, Ki_pos, Kd_pos;
    float   deadband_pos;
    float   max_rpm_diff;
    SpeedCtrlCfg_t speed_cfg;
    float   base_rpm;
    float   dt;
} LineFollowerCfg_t;

enum {
    LF_TURN_IDLE = 0,
    LF_TURN_PIVOT,
    LF_TURN_FWD
};

typedef struct {
    LineFollowerCfg_t cfg;
    SpeedCtrl_t speed_left;
    SpeedCtrl_t speed_right;
    Grayscale_t gs;

    int16_t position;
    float   rpm_diff;
    float   last_rpm_diff;
    int16_t left_duty;
    int16_t right_duty;

    bool     line_lost;
    uint16_t lost_count;

    uint8_t  turn_state;
    int8_t   turn_dir;
    uint16_t turn_timeout;
    uint16_t run_frames;
    uint8_t  turn_found_cnt;
} LineFollower_t;

void LineFollower_Init(LineFollower_t *lf, const LineFollowerCfg_t *cfg);
void LineFollower_Update(LineFollower_t *lf, float rpm_left, float rpm_right);
void LineFollower_Stop(LineFollower_t *lf);
int16_t LineFollower_GetPosition(LineFollower_t *lf);
float   LineFollower_GetRpmDiff(LineFollower_t *lf);
bool    LineFollower_IsLineLost(LineFollower_t *lf);
LineFollowerCfg_t LineFollower_GetDefaultCfg(void);

#endif
