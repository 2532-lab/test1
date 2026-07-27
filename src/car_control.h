/*
 * car_control.h - 两轮步进电机小车差速控制。
 *
 * 左右轮驱动仍然使用 STEP 脉冲和 DIR 电平。本模块将带符号的左右轮
 * 转速指令转换为现有步进驱动接口，并可选使用 MPU6050 实现短时偏航保持。
 */

#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#include "app_imu.h"
#include "drivers/stepper_pulse.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 正的 linear_rpm 表示前进。由于两个电机左右镜像安装，这里的电气方向
 * 有意设置为相反。如果实车收到正向指令后向后运动，请交换下面两个定义。
 */
#define CAR_LEFT_FORWARD_DIR       STEPPER_DIR_CW
#define CAR_RIGHT_FORWARD_DIR      STEPPER_DIR_CCW

/* ============================================================================
 * 小车位置运动参数
 * ============================================================================ */

/* 上电后自动前进的圈数。 */
#define CAR_FORWARD_TURNS          5U

/* 小车位置模式的目标速度，单位为 RPM。 */
#define CAR_FORWARD_RPM            100U

/* ==========================================================================
 * ★ 核心调节参数 — 调试时必须理解每个参数的含义和取值范围
 * ========================================================================== */

/*
 * CAR_MAX_RPM — 最高行驶转速 (RPM)
 *   取值范围: [100, 3000]
 *   默认值:   1200
 *   作用:     限制左右车轮的最大转速。该值越大，小车极速越高，
 *             但电机力矩随转速升高而下降，可能发生堵转（电机啸叫但不转）。
 *   调大: 需要更高速度，且电机当前未堵转、力矩足够。
 *   调小: 电机发生堵转（转不动），或需要更细腻的低速控制。
 *   注意: 此值会被 clamp_command() 用于限幅，不要超过 STEPPER_PULSE_MAX_RPM (3000)。
 */
#define CAR_MAX_RPM                1200

/*
 * CAR_MAX_TILT_DEG — 最大允许倾斜角度 (度)
 *   取值范围: [10.0, 60.0]
 *   默认值:   35.0
 *   作用:     当小车 roll 或 pitch 超过此角度时，判定为倾倒/受阻，
 *             触发紧急停止 (Car_EmergencyStop)，防止车轮空转损坏电机。
 *   调大: 路面坡度较大，或允许更激进的车身倾斜。
 *   调小: 平坦路面运行，希望更早检测到倾倒。
 *   注意: 过小会导致正常加速/刹车时误触发急停。
 */
#define CAR_MAX_TILT_DEG           59.0f

/*
 * CAR_HEADING_KP — 航向保持 比例系数
 *   取值范围: [0.5, 50.0]
 *   默认值:   6.0
 *   作用:     直行时根据偏航角误差 yaw_error 产生修正转速。
 *             correction = -(KP * yaw_error + KD * gyro_z)
 *             该值越大，对偏航的纠正越"硬"，回正越快。
 *   调大: 小车直行容易跑偏，需要更强的航向修正力。
 *   调小: 航向修正过于剧烈，导致小车左右摆动（振荡）。
 *   注意: KP 过大 + KD 过小 = 典型欠阻尼振荡，小车左右摇摆前进。
 */
#define CAR_HEADING_KP             6.0f

/*
 * CAR_HEADING_KD — 航向保持 微分系数
 *   取值范围: [0.0, 5.0]
 *   默认值:   0.25
 *   作用:     根据陀螺仪 Z 轴角速度 gyro_z 产生阻尼修正，
 *             抑制 KP 修正带来的超调和振荡。
 *             本质上是对"正在转多快"进行惩罚，提前刹车防止过头。
 *   调大: 航向修正后小车左右摆动（过冲），需要增加阻尼。
 *   调小: 航向修正太"软"，回正过程拖得很长。
 *   注意: KD=0 表示纯比例控制，几乎一定会振荡。
 */
#define CAR_HEADING_KD             0.25f

/*
 * CAR_HEADING_CORRECTION_MAX — 航向修正量上限 (RPM)
 *   取值范围: [10, 500]
 *   默认值:   180
 *   作用:     限制航向修正最多能"抢夺"多少转速去做转向修正。
 *             防止突然的大角度偏航导致一侧车轮骤停或反转。
 *   调大: 偏航修正量不够，小车仍然跑偏。
 *   调小: 偏航修正太激进，影响直行速度的平稳性。
 *   注意: 该值不应超过 CAR_MAX_RPM，否则修正量可能超过物理转速上限。
 */
#define CAR_HEADING_CORRECTION_MAX 180

typedef struct {
    int16_t linear_rpm;
    int16_t turn_rpm;
    bool    heading_hold;
    bool    fault;
} CarStatus_t;

/* 初始化小车状态，并让左右车轮保持停止。 */
void Car_Init(void);

/*
 * 设置差速驱动指令。
 *
 * turn_rpm 是差速量：正值使左轮快于右轮，负值相反。非零转向指令
 * 会暂时关闭偏航保持，直到下一次直行指令到来。
 *
 * 当锁存的安全故障处于活动状态时返回 false。
 */
bool Car_SetVelocity(int16_t linear_rpm, int16_t turn_rpm);

/* 停止左右车轮，但不锁存故障。 */
void Car_Stop(void);

/* 停止左右车轮，并锁存故障，直到调用 Car_ClearFault()。 */
void Car_EmergencyStop(void);

/* 清除锁存故障；调用后小车仍保持停止。 */
void Car_ClearFault(void);

/* 开启或关闭直行指令的偏航保持。 */
void Car_SetHeadingHold(bool enable);

/*
 * 在 IMU_Update() 后运行一次控制更新。该函数与 IMU_Update() 分开，
 * 调用者可以自行选择调度周期。
 */
bool Car_Update(const IMU_Data_t *imu, bool imu_valid);

/* 读取当前指令和安全状态。 */
void Car_GetStatus(CarStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* CAR_CONTROL_H */
