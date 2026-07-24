/*
 * stepper_pulse.h — 步进电机脉冲控制 (硬件 PWM + 步数计数 ISR)
 *
 * 目标芯片:    MSPM0G3507
 * 依赖:        SysConfig → ti_msp_dl_config.h (cloud_1_TILT / cloud_2_PAN)
 *
 * === ★ 双模式架构 ===
 *
 *   速度模式 (stepper_pulse_set_speed):
 *     持续 PWM 输出, 同方向调速不停止脉冲序列。
 *     适用场景: gimbal_track_velocity() 视觉追踪。
 *
 *   位置模式 (stepper_pulse_move):
 *     下发目标步数 → 启动 PWM → ISR 计数 → 到达后自动停止。
 *     主循环调用后立即返回, 非阻塞轮询 stepper_pulse_is_busy()。
 *     适用场景: gimbal_control_angle() 绝对角度控制。
 *
 *   关键差异:
 *     - 速度模式需要主循环持续调用调速, 永不自动停止
 *     - 位置模式一次下发, ISR 自动停止, 不阻塞主循环
 *
 *   云台轴映射 (以 SysConfig 为准):
 *     TILT (俯仰): cloud_1_TILT (TIMA0) CCP2 = PB0  PULSE
 *                   cloud_direction TILT            = PB13 DIR
 *     PAN  (水平): cloud_2_PAN  (TIMG6) CCP0 = PB2  PULSE
 *                   cloud_direction PAN             = PB3  DIR
 *
 *   频率控制: 修改定时器周期 (load value) → 改变 PWM 频率 → 改变转速
 *   占空比:   固定 50% → compare = period / 2
 *   方向控制: GPIO 直接置高/低
 *
 * === 使用方式 ===
 *   1. SYSCFG_DL_init() 自动初始化 cloud 定时器和 GPIO
 *   2. 调用 stepper_pulse_init() 初始化方向引脚 + 使能步数计数 ISR
 *   3. 速度追踪: 每 20ms 调用 stepper_pulse_set_speed(axis, dir, rpm)
 *   4. 位置控制: 调用 stepper_pulse_move(axis, dir, rpm, steps)
 *      然后轮询 stepper_pulse_is_busy(axis) 等待完成
 */

#ifndef __STEPPER_PULSE_H
#define __STEPPER_PULSE_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 轴选择器
 * ========================================================================== */
#define STEPPER_AXIS_PAN    0
#define STEPPER_AXIS_TILT   1

/* ==========================================================================
 * 方向定义
 * ========================================================================== */
#define STEPPER_DIR_CW      0
#define STEPPER_DIR_CCW     1

/* ==========================================================================
 * 运动参数
 * ========================================================================== */
#define STEPPER_PULSE_DEFAULT_RPM       1200
#define STEPPER_PULSE_MAX_RPM           3000//最高转速限制，高转速下力矩变小，有可能带不动载物，为了防止发生堵转，出现堵转就调小，调大：若红球移动极快，云台慢一拍，且此时电机并没有堵转
#define STEPPER_PULSE_MIN_RPM             9
#define STEPPER_PULSES_PER_REV          3200   /* 200 全步/转 × 16 细分 */
#define STEPPER_STEP_DEG             0.1125f   /* 360° / 3200               */
#define STEPPER_RPM_STOP                   0

/* ==========================================================================
 * 公开 API — 初始化
 * ========================================================================== */

/**
 * @brief   初始化步进脉冲控制模块。
 *
 * 必须在 SYSCFG_DL_init() 之后调用一次。
 * 云台 PWM 定时器由 SysConfig 自动初始化, 本函数额外:
 *   1. 配置 PAN (PB3) / TILT (PB13) 方向引脚为 GPIO 数字输出
 *   2. 使能 TIMA0 (TILT) 和 TIMG6 (PAN) 的零事件中断
 *   3. 使能 NVIC 中断
 *   4. 清零所有状态变量
 */
void stepper_pulse_init(void);

/* ==========================================================================
 * 公开 API — 速度模式 (用于 gimbal_track_velocity 视觉追踪)
 * ========================================================================== */

/**
 * @brief   速度模式 — 动态调速 (修改 PWM 周期, 不断脉冲)。
 *
 * 同方向调速时仅修改定时器周期 → 脉冲不中断, 丝滑过渡。
 * 方向反转时执行 CCP 冻结 + 停止 + 换向 + 重启安全序列。
 * rpm==0 时停止 PWM 输出。
 *
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @param   dir    STEPPER_DIR_CW (0) 或 STEPPER_DIR_CCW (1)。
 * @param   rpm    目标转速, [STEPPER_PULSE_MIN_RPM, STEPPER_PULSE_MAX_RPM]。
 *                 0 (=STEPPER_RPM_STOP) 停止。
 *
 * @note    速度模式下 ISR 步数计数被旁路 (steps_remaining=0),
 *          电机持续运转直到主循环调用 stepper_pulse_stop()。
 */
void stepper_pulse_set_speed(uint8_t axis, uint8_t dir, uint16_t rpm);

/* ==========================================================================
 * 公开 API — 位置模式 (用于 gimbal_control_angle 角度控制)
 * ========================================================================== */

/**
 * @brief   位置模式 — 有限步数运动 (非阻塞, ISR 自动停止)。
 *
 * 下发目标步数后立即返回; ISR 在后台计数, 到达目标步数后执行
 * CCP 冻结 + 停止序列, g_busy 自动变 false。
 *
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @param   dir    STEPPER_DIR_CW (0) 或 STEPPER_DIR_CCW (1)。
 * @param   rpm    运动速度, [STEPPER_PULSE_MIN_RPM, STEPPER_PULSE_MAX_RPM]。
 * @param   steps  目标脉冲数 (步数), 0 表示不动作。
 *
 * @note    如果轴忙, 先安全停止当前运动再下发新目标。
 * @note    主循环通过 stepper_pulse_is_busy() 非阻塞轮询完成状态。
 *
 * ISR 在每个 PWM 零事件:
 *   - 递减 g_steps_remaining
 *   - 更新 g_position_steps (CCW=+1, CW=-1)
 *   - steps_remaining==0 时: CCP 冻结 → g_busy=false → stopCounter → clearIRQ
 */
void stepper_pulse_move(uint8_t axis, uint8_t dir, uint16_t rpm, uint32_t steps);

/* ==========================================================================
 * 公开 API — 停止
 * ========================================================================== */

/**
 * @brief   立即停止指定轴的 PWM 输出 (CCP 安全序列)。
 *
 * 停止顺序: CCP 冻结 (CCR=PERIOD_MAX) → g_busy=false → stopCounter → clearIRQ。
 *
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @note    同时清零 g_current_rpm 和 g_steps_remaining。
 */
void stepper_pulse_stop(uint8_t axis);

/**
 * @brief   双轴紧急停止 — 禁用 ISR 步数计数后批量清零所有状态。
 *
 * 流程: g_step_count_enabled=false → stopCounter 双轴 → clearIRQ
 *       → 清零全局变量 → g_step_count_enabled=true。
 */
void stepper_pulse_emergency_stop(void);

/* ==========================================================================
 * 公开 API — 状态查询
 * ========================================================================== */

/**
 * @brief   查询指定轴是否正在运动中。
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @retval  true=运动中 (位置模式: 未达目标; 速度模式: 未停止)。
 */
bool stepper_pulse_is_busy(uint8_t axis);

/**
 * @brief   查询指定轴的当前转速。
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @retval  当前 RPM 值, 停止时返回 0。
 */
uint16_t stepper_pulse_get_current_rpm(uint8_t axis);

/**
 * @brief   查询指定轴的当前运动方向。
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @retval  STEPPER_DIR_CW (0) 或 STEPPER_DIR_CCW (1)。
 */
uint8_t stepper_pulse_get_current_dir(uint8_t axis);

/**
 * @brief   查询指定轴的累计步数 (带符号 int32_t)。
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @retval  CCW 方向递增, CW 方向递减。
 */
int32_t stepper_pulse_get_position_steps(uint8_t axis);

/**
 * @brief   查询指定轴的当前累计角度。
 *
 * 换算: deg = g_position_steps × 360° / STEPPER_PULSES_PER_REV
 *
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 * @retval  角度值 (度), 分辨率 0.1125° (3200 PPR @ 16细分)。
 */
float stepper_pulse_get_position_deg(uint8_t axis);

/**
 * @brief   标记当前位置为逻辑零点 — 重置累计步数计数器为 0。
 * @param   axis   STEPPER_AXIS_PAN (0) 或 STEPPER_AXIS_TILT (1)。
 */
void stepper_pulse_set_position_zero(uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_PULSE_H */
