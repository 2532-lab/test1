/*
 * gimbal_control.h — 二轴步进电机云台控制 (头文件)
 *
 * 目标芯片:    MSPM0G3507 (仅使用 TI DriverLib，严禁 STM32 HAL / 标准库)
 * 依赖模块:    stepper_pulse.h → 脉冲+方向+步数计数 底层控制
 *              delay.h        → millis() 系统时钟
 *
 * === ★ 双模式架构 ===
 *
 *   角度/位置模式 (gimbal_control_angle):
 *     下发绝对目标角度 → stepper_pulse_move(steps) → ISR 计数 → 自动停止。
 *     非阻塞: 调用后立即返回, 主循环轮询 gimbal_is_moving()。
 *     适用场景: 绝对定位、手动控制、标定、主循环不阻塞的场合。
 *
 *   速度追踪模式 (gimbal_track_velocity):
 *     外部偏差 → EMA滤波 → P控制器 → RPM → stepper_pulse_set_speed()。
 *     连续调速, 永不自动停止, 需主循环持续调用 (建议 ~20ms 间隔)。
 *     适用场景: 外部传感器/上位机实时目标追踪。
 *
 *   关键差异:
 *     - 角度模式: 一次下发, ISR 自动完成, 主循环可处理其他任务
 *     - 速度模式: 持续驱动, 适合响应式追踪但需定时调用
 *
 * === ★ 快速上手 (完整启动序列) ===
 *
 *   在你的 main.c 中:
 *
 *   #include "ti_msp_dl_config.h"
 *   #include "gimbal_control.h"
 *   #include "delay.h"
 *
 *   // SysTick 毫秒计数器 (delay.h 中已 extern 声明)
 *   volatile uint32_t sys_tick_ms = 0;
 *   void SysTick_Handler(void) { sys_tick_ms++; }
 *
 *   int main(void) {
 *       // 1. 硬件初始化
 *       SYSCFG_DL_init();
 *       SysTick_Config(CPUCLK_FREQ / 1000U);
 *
 *       // 2. 云台初始化
 *       gimbal_init();
 *       gimbal_pan.rpm  = GIMBAL_EXAMPLE_SPEED_RPM;
 *       gimbal_tilt.rpm = GIMBAL_EXAMPLE_SPEED_RPM;
 *       gimbal_reset_zero();
 *
 *       // 3. 非阻塞主循环
 *       while (1) {
 *           // ... 你的状态机 / 传感器读取 / 通信 ...
 *       }
 *   }
 *
 * === ★ 非阻塞状态机模板 ===
 *
 *   typedef enum { STATE_MOVE, STATE_WAIT, STATE_PAUSE } state_t;
 *   state_t state = STATE_MOVE;
 *   uint32_t pause_start = 0;
 *
 *   while (1) {
 *       uint32_t now = millis();
 *       switch (state) {
 *       case STATE_MOVE:
 *           gimbal_control_angle(90.0f, 45.0f);   // 下发目标, 立即返回
 *           state = STATE_WAIT;
 *           break;
 *       case STATE_WAIT:
 *           if (!gimbal_is_moving()) {             // 非阻塞轮询
 *               pause_start = now;
 *               state = STATE_PAUSE;
 *           }
 *           break;
 *       case STATE_PAUSE:
 *           if (now - pause_start >= GIMBAL_EXAMPLE_PAUSE_MS) {
 *               state = STATE_MOVE;
 *           }
 *           break;
 *       }
 *       // ★ 你的其他非阻塞任务...
 *   }
 */

#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "stepper_pulse.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 追踪参数 — 可根据实际硬件调优
 * ========================================================================== */

/* 像素 → RPM 比例系数 (P 控制器增益) */
#define GIMBAL_TRACK_KP_PAN          30.0f
#define GIMBAL_TRACK_KP_TILT         30.0f

/* 死区 (像素) — 误差小于此值不动作, 防止微振 */
#define GIMBAL_DEADBAND_PAN             3
#define GIMBAL_DEADBAND_TILT            3

/* EMA 滤波系数 (0.0 ~ 1.0)
 * 越小 = 越平滑但响应慢, 越大 = 响应快但噪声多
 * 0.15 → 适合 50Hz 更新率, 截止频率 ≈ 1.3 Hz */
#define GIMBAL_EMA_ALPHA            0.15f

/* 误差缩小时的 EMA 滤波系数 (非对称滤波)
 * 当 |raw_error| < |ema_error| 时使用较高 alpha → 更快的减速响应 */
#define GIMBAL_EMA_ALPHA_DECAY      0.30f

/* RPM 变化率限制 (RPM / 20ms)
 * 防止转速突变导致丢步。例如 200 RPM/周期 = 10000 RPM/s 加速率 */
#define GIMBAL_RPM_SLEW_RATE          200

/* 最大追踪 RPM — 超过此值钳制 */
#define GIMBAL_TRACK_MAX_RPM         2000

/* 最小追踪 RPM — 低于此值停止 (步进电机太慢会抖) */
#define GIMBAL_TRACK_MIN_RPM           80

/* 目标丢失超时 (ms) — 超过此时间无新数据则自动停止 */
#define GIMBAL_LOST_TIMEOUT_MS        200

/* ==========================================================================
 * 角度控制参数 (位置模式)
 * ========================================================================== */

/* 角度模式默认 RPM (适中速度, 兼顾精度和响应) */
#define GIMBAL_ANGLE_DEFAULT_RPM       300

/* 角度限位 (16细分: 3200 PPR, 0.1125°/步) */
#define GIMBAL_PAN_ANGLE_MAX         180.0f
#define GIMBAL_TILT_ANGLE_MAX        100.0f
#define GIMBAL_DEG_PER_REV           360.0f
#define GIMBAL_ANGLE_DEADBAND         0.12f  /* 略大于单步精度 0.1125° */

/* ==========================================================================
 * 示例: 运动测试参数 (从 main.c 提取, 供参考)
 * ==========================================================================
 *
 * 这些参数来自实际测试验证过的配置, 可根据你的需求调整。
 * 复制到你的 main.c 或配置头文件中直接使用。
 */

/* 摆动幅度 (度) — 建议取 0.1125° 的整数倍以对齐步距 */
#define angle_PAN     90.0f
#define angle_TILT    45.0f

/* 运动转速 (RPM) — 首次测试建议用低速 30 RPM 验证方向 */
#define speed_RPM              30

/* 运动段间暂停 (ms) — 等待机械振动衰减 */
#define part_pause_MS             1600//1600

/* 上电稳定等待 (ms) — 让驱动器硬件就绪 */
#define init_STABLE_MS        100

/* ==========================================================================
 * 堵转检测参数 (软件备用)
 * ========================================================================== */
#define STALL_POLL_INTERVAL_MS           10
#define STALL_INTEGRAL_MAX              50.0f
#define STALL_OUTPUT_RUNAWAY_DEG       360.0f
#define STALL_CONFIRM_COUNT               2

/* ==========================================================================
 * 堵转故障状态
 * ========================================================================== */
typedef enum {
    STALL_FAULT_NONE = 0,
    STALL_FAULT_HARDWARE,
    STALL_FAULT_INTEGRAL,
    STALL_FAULT_OUTPUT_RUNAWAY,
    STALL_FAULT_EMERGENCY_STOP,
} stall_fault_t;

/* ==========================================================================
 * 简化的 PID 结构体 (仅用于位置模式)
 * ========================================================================== */
typedef struct {
    float Kp, Ki, Kd;
    float error, last_error;
    float integral, output;
} gimbal_pid_t;

/* ==========================================================================
 * 逐轴追踪状态
 * ========================================================================== */
typedef struct {
    float       angle;              /* 当前角度 (度)                        */
    float       target_angle;       /* 目标角度 (度), 位置模式用            */
    gimbal_pid_t pid;               /* PID 控制器 (保留)                     */
    uint8_t     motor_addr;         /* 轴索引 (复用)                         */
    void       *uart;               /* 保留字段                              */
    uint16_t    rpm;                /* 默认 RPM                              */
    uint8_t     accel;              /* 保留                                  */
    float       angle_limit;        /* 角度限位 (±)                          */
    int32_t     target_point;       /* 保留 (兼容旧接口)                      */
    int32_t     current_point;      /* 保留                                  */

    /* ── 速度追踪状态 ── */
    float       ema_error;          /* EMA 滤波后的误差值                     */
    uint16_t    current_rpm;        /* 当前输出的 RPM (经限幅处理)            */
    int32_t     last_data_ms;       /* 上次收到数据的时刻 (ms)                */

    /* ── 堵转监控 ── */
    uint8_t     stall_confirm_count;
    stall_fault_t fault;
} gimbal_axis_t;

/* ==========================================================================
 * 全局状态
 * ========================================================================== */
extern gimbal_axis_t gimbal_pan;
extern gimbal_axis_t gimbal_tilt;

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
 * @brief   初始化双轴云台控制模块。
 *
 * 必须在 SYSCFG_DL_init() 之后调用一次。
 * 内部调用 stepper_pulse_init() 配置 DIR 引脚 + 使能步数计数 ISR,
 * 并重置 PAN/TILT 两轴的追踪状态。
 */
void gimbal_init(void);

/**
 * @brief   标记当前双轴位置为逻辑零点。
 *
 * 清零硬件步数计数器 + 软件角度值。
 * 通常在初始标定或需要重新建立位置基准时调用。
 */
void gimbal_reset_zero(void);

/**
 * @brief   兼容接口 — 标记当前位置为零点。
 * @param   store   保留参数 (当前未使用, 始终清零)。
 */
void gimbal_set_zero(bool store);

/* ==========================================================================
 * ★ 核心 API: 角度/位置控制模式 (非阻塞)
 * ========================================================================== */

/**
 * @brief   绝对角度位置控制 — 非阻塞, ISR 自动停止 (★ 主要的云台控制 API)。
 *
 * 内部流程:
 *   1. 从 stepper_pulse_get_position_deg() 读取当前角度
 *   2. 计算角度差: delta = target - current
 *   3. 死区检查: |delta| <= GIMBAL_ANGLE_DEADBAND → 跳过
 *   4. 角度差 → 步数: steps = |delta| × PPR / 360 (四舍五入)
 *   5. stepper_pulse_move(axis, dir, rpm, steps) 下发运动
 *   6. 基于步进实际位置更新 .angle 字段
 *   7. 立即返回 (不等待运动完成!)
 *
 * @param   pan_deg    水平目标角度 (度), [-GIMBAL_PAN_ANGLE_MAX, +GIMBAL_PAN_ANGLE_MAX]。
 * @param   tilt_deg   俯仰目标角度 (度), [-GIMBAL_TILT_ANGLE_MAX, +GIMBAL_TILT_ANGLE_MAX]。
 *
 * @note    主循环通过 gimbal_is_moving() 轮询运动状态。
 * @note    运动过程中再次调用会覆盖之前的目标 (先安全停止再下发新目标)。
 */
void gimbal_control_angle(float pan_deg, float tilt_deg);

/**
 * @brief   单轴水平角度控制 (保持 TILT 当前位置不变)。
 * @param   pan_deg   水平目标角度 (度)。
 */
void gimbal_control_pan_only(float pan_deg);

/**
 * @brief   单轴俯仰角度控制 (保持 PAN 当前位置不变)。
 * @param   tilt_deg  俯仰目标角度 (度)。
 */
void gimbal_control_tilt_only(float tilt_deg);

/**
 * @brief   查询是否有任何轴正在运动中。
 * @retval  true 表示至少一轴正在执行 stepper_pulse_move()。
 */
bool gimbal_is_moving(void);

/**
 * @brief   查询 PAN 轴当前角度 (从硬件步数计数器读取)。
 * @retval  角度值 (度), 分辨率 0.1125°。
 */
float gimbal_get_current_angle_pan(void);

/**
 * @brief   查询 TILT 轴当前角度 (从硬件步数计数器读取)。
 * @retval  角度值 (度), 分辨率 0.1125°。
 */
float gimbal_get_current_angle_tilt(void);

/* ==========================================================================
 * ★ 核心 API: 速度追踪模式 (用于 CanMV 视觉追踪)
 * ========================================================================== */

/**
 * @brief   速度追踪核心 — 以像素误差驱动连续调速 (★ 丝滑追踪)。
 *
 * 内部处理: 超时检测 → 死区过滤 → 非对称 EMA 低通滤波 → P 控制器
 *          → RPM 变化率限制 → 原始误差定方向 → stepper_pulse_set_speed()。
 *
 * @param   error_x   水平像素偏差 (目标.cx - 160), 范围约 ±160 像素。
 * @param   error_y   垂直像素偏差 (目标.cy - 120), 范围约 ±120 像素。
 * @param   now_ms    当前系统毫秒时间戳 (millis()), 用于超时检测。
 *
 * @note    在主循环中以固定频率调用 (建议 50Hz = 20ms 间隔)。
 * @note    方向由原始误差决定, 不受 EMA 滞后影响。
 * @note    超过 GIMBAL_LOST_TIMEOUT_MS 无新数据时自动停止。
 */
void gimbal_track_velocity(int16_t error_x, int16_t error_y, uint32_t now_ms);

/**
 * @brief   平滑停止双轴速度追踪 (清零 EMA 误差 + 停止 PWM 输出)。
 */
void gimbal_track_stop(void);

/**
 * @brief   查询速度追踪是否活跃 (有轴正在运转)。
 * @retval  true 表示追踪中。
 */
bool gimbal_track_is_active(void);

/* ==========================================================================
 * 调试接口 — 获取追踪内部状态 (用于 OLED/TFT 显示)
 * ========================================================================== */

/**
 * @brief   获取 PAN 轴 EMA 滤波后的像素误差值 (调试用)。
 * @retval  EMA 滤波后的误差值 (像素)。
 */
float gimbal_get_filtered_error_x(void);

/**
 * @brief   获取 TILT 轴 EMA 滤波后的像素误差值 (调试用)。
 * @retval  EMA 滤波后的误差值 (像素)。
 */
float gimbal_get_filtered_error_y(void);

/**
 * @brief   获取 PAN 轴当前输出的 RPM 值 (调试用)。
 * @retval  当前 RPM。
 */
uint16_t gimbal_get_output_rpm_pan(void);

/**
 * @brief   获取 TILT 轴当前输出的 RPM 值 (调试用)。
 * @retval  当前 RPM。
 */
uint16_t gimbal_get_output_rpm_tilt(void);

/* ==========================================================================
 * 兼容 API: 速度模式 (已废弃, 保留编译兼容)
 * ========================================================================== */

/**
 * @brief   速度模式 — 直接发送 RPM 持续运转 (已废弃)。
 *
 * 保留用于编译兼容和简单测试。
 * 新代码请使用:
 *   - gimbal_control_angle()    用于绝对角度定位 (非阻塞)
 *   - gimbal_track_velocity()   用于视觉追踪 (平滑连续调速)
 *
 * @param   v1   PAN  轴 RPM (正=CCW, 负=CW, |v1|<=0.01 则停止)。
 * @param   v2   TILT 轴 RPM (正=CCW, 负=CW, |v2|<=0.01 则停止)。
 *
 * @deprecated  本函数阻塞主循环, 且无 EMA 滤波和死区处理。
 */
void gimbal_control_velocity(float v1, float v2);

/* ==========================================================================
 * 堵转保护
 * ========================================================================== */

/**
 * @brief   初始化堵转检测状态, 清零故障标志和确认计数。
 */
void gimbal_stall_monitor_init(void);

/**
 * @brief   堵转检测轮询 (占位, 后续可扩展编码器反馈或电流检测)。
 */
void gimbal_stall_monitor_tick(void);

/**
 * @brief   查询是否有任何轴处于堵转故障状态。
 * @retval  true 表示至少一轴故障。
 */
bool gimbal_is_any_stalled(void);

/**
 * @brief   查询 PAN 轴是否堵转。
 * @retval  true 表示 PAN 轴故障。
 */
bool gimbal_is_pan_stalled(void);

/**
 * @brief   查询 TILT 轴是否堵转。
 * @retval  true 表示 TILT 轴故障。
 */
bool gimbal_is_tilt_stalled(void);

/**
 * @brief   获取 PAN 轴当前故障类型。
 * @retval  stall_fault_t 枚举值。
 */
stall_fault_t gimbal_get_pan_fault(void);

/**
 * @brief   获取 TILT 轴当前故障类型。
 * @retval  stall_fault_t 枚举值。
 */
stall_fault_t gimbal_get_tilt_fault(void);

/**
 * @brief   清除双轴堵转故障标志 (复位为 STALL_FAULT_NONE)。
 */
void gimbal_clear_stall_fault(void);

/**
 * @brief   堵转回调 (弱符号, 用户可覆盖实现自定义处理)。
 * @param   motor_addr   轴索引 (STEPPER_AXIS_PAN / STEPPER_AXIS_TILT)。
 * @param   fault        故障类型 (stall_fault_t)。
 */
void gimbal_stall_callback(uint8_t motor_addr, stall_fault_t fault);

/* ==========================================================================
 * 工具函数
 * ========================================================================== */

/**
 * @brief   双轴紧急停止 — 清零所有状态并停止 PWM 输出。
 *
 * 同时清零 EMA 误差、RPM, 调用 stepper_pulse_emergency_stop()。
 */
void gimbal_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_CONTROL_H */
