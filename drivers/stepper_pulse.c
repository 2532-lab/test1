/*
 * stepper_pulse.c — 步进电机脉冲控制 (硬件 PWM + 步数计数 ISR)
 *
 * 目标芯片:    MSPM0G3507
 * 依赖:        SysConfig → cloud_1_TILT (TIMA0) / cloud_2_PAN (TIMG6)
 *
 * === ★ 双模式架构 ===
 *
 *   速度模式 (stepper_pulse_set_speed):
 *     定时器 CCP 引脚直接输出 PWM 方波 → 纯硬件, 零 CPU 开销, 零抖动。
 *     DL_TimerX_setLoadValue() 改周期 → 调速。
 *     占空比固定 50% (compare = period / 2), 仅通过周期控制频率:
 *       pulse_freq = RPM × PULSES_PER_REV / 60
 *       period     = timer_clock / pulse_freq
 *     同方向调速不停止 PWM → 丝滑过渡。
 *     适用场景: gimbal_track_velocity() 视觉追踪 (CanMV 像素误差驱动)。
 *
 *   位置模式 (stepper_pulse_move):
 *     下发目标步数 → 启动 PWM → 定时器零事件 ISR 计数每步 →
 *     到达目标步数后 ISR 自动停止 PWM → 主循环轮询 stepper_pulse_is_busy()。
 *     每次零事件 ISR:
 *       - 清中断标志
 *       - 递减 steps_remaining
 *       - 更新 position_steps (方向感知)
 *       - steps_remaining == 0 → 停止定时器 + 清 busy
 *     适用场景: gimbal_control_angle() 绝对角度定位。
 *
 *   关键差异:
 *     - 速度模式: 永不自动停止, 需要主循环显式调用 stop
 *     - 位置模式: ISR 自动停止, 主循环无需干预
 *
 *   TILT 轴: cloud_1_TILT (TIMA0), CCP2 = PB0, DIR = PB13
 *   PAN  轴: cloud_2_PAN  (TIMG6), CCP0 = PB2, DIR = PB3
 *
 *   TIMA0 和 TIMG6 由 SysConfig 在 SYSCFG_DL_init() 中完成:
 *     时钟配置、PWM 模式、GPIO 复用为外设功能、初始周期和占空比。
 *   本模块负责: 运行时调速、步数计数 ISR、位置跟踪。
 */

#include "stepper_pulse.h"
#include "ti_msp_dl_config.h"

/* ==========================================================================
 * 定时器时钟: BUSCLK = 32 MHz (SysConfig 默认)
 * ========================================================================== */
#define TIMER_CLK_HZ            32000000U//U:unsigned

/* 周期安全范围 */
#define PERIOD_MIN                  10U
#define PERIOD_MAX               65535U

/* ==========================================================================
 * 每轴实例引用 (来自 SysConfig 生成宏)
 * ========================================================================== */

/* TILT 轴: cloud_1_TILT = TIMA0 */
#define TILT_TIMER              cloud_1_TILT_INST

/* PAN 轴: cloud_2_PAN = TIMG6 */
#define PAN_TIMER               cloud_2_PAN_INST

/* DIR 引脚: cloud_direction GPIO 组 */
#define TILT_DIR_PORT           cloud_direction_PORT
#define TILT_DIR_PIN            cloud_direction_TILT_PIN
#define PAN_DIR_PORT            cloud_direction_PORT
#define PAN_DIR_PIN             cloud_direction_PAN_PIN

/* ==========================================================================
 * 每轴运动状态
 * ========================================================================== */
static volatile bool     g_busy[2];
static volatile uint16_t g_current_rpm[2];//实际转速变量
static volatile uint8_t  g_current_dir[2];

/* ── 位置模式状态 (ISR 访问, volatile 确保编译器不优化) ── */
static volatile uint32_t g_steps_remaining[2];   /* 剩余步数, ISR 递减   */
static volatile int32_t  g_position_steps[2];    /* 累计步数, 带符号     */
static volatile bool     g_step_count_enabled;    /* ISR 步数计数全局使能 */

/* ==========================================================================
 * 工具函数: RPM → 定时器周期
 * ==========================================================================
 *
 * period = CLK × 60 / (RPM × PPR)
 * 推导: pulse_freq = RPM × PPR / 60,  period = CLK / pulse_freq
 */
static uint16_t rpm_to_period(uint16_t rpm)
{
    uint32_t period;
    //输入限幅，防止单片机传错数据，防止传的太高会发生堵转
    if (rpm == 0) return 0;
    if (rpm > STEPPER_PULSE_MAX_RPM) rpm = STEPPER_PULSE_MAX_RPM;
    if (rpm < STEPPER_PULSE_MIN_RPM) rpm = STEPPER_PULSE_MIN_RPM;

    period = (TIMER_CLK_HZ * 60UL) / ((uint32_t)rpm * STEPPER_PULSES_PER_REV);//UL:unsigned long
    //输出限幅，防止周期太小，使单片机不停的进中断，
    if (period == 0) return PERIOD_MIN;
    if (period < PERIOD_MIN) period = PERIOD_MIN;
    if (period > PERIOD_MAX) period = PERIOD_MAX;

    return (uint16_t)period;
}

/* ==========================================================================
 * 内部: 设置 DIR 引脚电平（方向）
 * ========================================================================== */
//水平电机
static void set_dir_pan(uint8_t dir)
{
    if (dir == STEPPER_DIR_CCW) {
        DL_GPIO_setPins(PAN_DIR_PORT, PAN_DIR_PIN);//逆时针电平拉高
    } else {
        DL_GPIO_clearPins(PAN_DIR_PORT, PAN_DIR_PIN);
    }
}
//垂直电机 (极性与 PAN 相同: CCW→HIGH, CW→LOW)
static void set_dir_tilt(uint8_t dir)
{
    if (dir == STEPPER_DIR_CCW) {
        DL_GPIO_setPins(TILT_DIR_PORT, TILT_DIR_PIN);
    } else {
        DL_GPIO_clearPins(TILT_DIR_PORT, TILT_DIR_PIN);
    }
}

/* ==========================================================================
 * 内部: 配置并启动 PWM 输出
 * ========================================================================== */

/*
 * tima_config_pwm — 配置 TIMA0 (cloud_1_TILT) 的周期和占空比
 *
 * TIMA0 是 16-bit TimerA, 使用 DL_TimerA_* API。
 * SysConfig 已完成时钟和 GPIO 复用初始化, 这里只改周期和占空比。
 */
static void tima_config_pwm(uint16_t period)
{
    uint16_t compare = period / 2U;   /* 50% duty */

    DL_TimerA_stopCounter(TILT_TIMER);//修改定时器配置之前，先暂停定时器技术

    /*
     * ★ 冻结 CCP: 设 CCR=PERIOD_MAX, 输出锁死高电平,
     *   后续 setTimerCount/setLoadValue 不会产生伪边沿。
     */
     //通过将CCR比较器改为最大值，
    DL_TimerA_setCaptureCompareValue(TILT_TIMER,
        PERIOD_MAX, DL_TIMER_CC_2_INDEX);

    /*
     * ★ EDGE_ALIGN_UP (向上计数): CNT 从 0 开始, 计数到 LOAD 后回绕。
     *   setTimerCount=0 确保第一个 PWM 周期从底部开始, 是完整周期。
     *   此时 CCR=PERIOD_MAX → 输出无变化 → 驱动器不收伪脉冲。
     */
    DL_TimerA_setTimerCount(TILT_TIMER, 0);//向上计数模式，从0开始

    DL_TimerA_setLoadValue(TILT_TIMER, period);

    /*
     * ★ CC3=0: 保留 (不影响 CZC, 零事件由 SysConfig 的 ZCOND 控制)。
     *   与 CC2 (PWM 比较) 分离，避免零事件与比较匹配碰撞导致 31ns 窄脉冲。
     */
    DL_TimerA_setCaptureCompareValue(TILT_TIMER, 0, DL_TIMER_CC_3_INDEX);

    DL_TimerA_setCaptureCompareValue(TILT_TIMER, compare,
                                      DL_TIMER_CC_2_INDEX);

    DL_TimerA_clearInterruptStatus(TILT_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(TILT_TIMER);
}

/*
 * timg_config_pwm — 配置 TIMG6 (cloud_2_PAN) 的周期和占空比
 */
static void timg_config_pwm(uint16_t period)
{
    uint16_t compare = period / 2U;   /* 50% duty */

    DL_TimerG_stopCounter(PAN_TIMER);

    /*
     * ★ 冻结 CCP: 设 CCR=PERIOD_MAX, 输出锁死高电平,
     *   后续 setTimerCount/setLoadValue 不会产生伪边沿。
     */
    DL_TimerG_setCaptureCompareValue(PAN_TIMER,
        PERIOD_MAX, DL_TIMER_CC_0_INDEX);

    /*
     * ★ EDGE_ALIGN_UP (向上计数): CNT 从 0 开始, 计数到 LOAD 后回绕。
     *   setTimerCount=0 确保第一个 PWM 周期从底部开始, 是完整周期。
     *   此时 CCR=PERIOD_MAX → 输出无变化 → 驱动器不收伪脉冲。
     */
    DL_TimerG_setTimerCount(PAN_TIMER, 0);//向上计数模式，从0开始

    DL_TimerG_setLoadValue(PAN_TIMER, period);

    /*
     * ★ CC1=0: 保留 (不影响 CZC, 零事件由 SysConfig 的 ZCOND 控制)。
     *   与 CC0 (PWM 比较) 分离，避免零事件与比较匹配碰撞导致 31ns 窄脉冲。
     */
    DL_TimerG_setCaptureCompareValue(PAN_TIMER, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_setCaptureCompareValue(PAN_TIMER, compare,
                                      DL_TIMER_CC_0_INDEX);

    DL_TimerG_clearInterruptStatus(PAN_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);

    DL_TimerG_startCounter(PAN_TIMER);
}

/* ==========================================================================
 * ★ 步数计数 ISR
 * ==========================================================================
 *
 * 每个 PWM 零事件 (计数器回零) = 1 个完整脉冲 = 1 步。
 *
 * ISR 设计约束:
 *   - 极短执行时间: 仅清标志、递减计数、更新位置、到达目标时停止
 *   - 无浮点运算、无函数调用 (除 DL API)
 *   - volatile 变量确保编译器不优化掉读写
 *
 * 并发安全:
 *   - g_steps_remaining: 主循环仅在非 busy 时写入; ISR 仅递减并检测零
 *   - g_busy: 主循环写入 true; ISR 写入 false → 无同时写入
 *   - g_position_steps: ISR 写入; 主循环仅读取 → 安全
 */

/*
 * TIMA0_IRQHandler — TILT 轴 (TIMA0) 中断服务例程
 */
void TIMA0_IRQHandler(void)
{
    uint32_t pending = DL_TimerA_getPendingInterrupt(TILT_TIMER);

    /* 零事件中断: 每个 PWM 周期触发一次 = 1 步 */
    if (pending & DL_TIMER_INTERRUPT_ZERO_EVENT) {
        //清除零事件中断标志位
        DL_TimerA_clearInterruptStatus(TILT_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);

        /*
         * ★ 仅当轴处于活跃状态时才更新位置/步数。
         *    stopCounter 是异步操作, busy=false 后定时器可能仍会产生
         *    1~N 个额外零事件; 用 g_busy 守卫防止这些"幽灵脉冲"
         *    被计入位置, 否则高速时误差可达 1.7°+。
         */
        if (g_step_count_enabled && g_busy[STEPPER_AXIS_TILT]) {
            //方向记账
            if (g_current_dir[STEPPER_AXIS_TILT] == STEPPER_DIR_CCW) {
                g_position_steps[STEPPER_AXIS_TILT]++;
            } else {
                g_position_steps[STEPPER_AXIS_TILT]--;
            }

            /* 位置模式: 递减步数, 到达目标后自动停止 */
            if (g_steps_remaining[STEPPER_AXIS_TILT] > 0) {
                g_steps_remaining[STEPPER_AXIS_TILT]--;

                if (g_steps_remaining[STEPPER_AXIS_TILT] == 0) {
                    /*
                     * ★ 停止顺序至关重要:
                     *   1. 冻结 CCP (CCR=PERIOD_MAX) → 输出锁死高电平
                     *   2. g_busy=false → 软件拒绝后续幽灵中断
                     *   3. stopCounter → 异步停止, 但 CCP 已冻结无伪边沿
                     *   4. clearInterruptStatus → 清理残留标志
                     */
                    DL_TimerA_setCaptureCompareValue(TILT_TIMER,
                        PERIOD_MAX, DL_TIMER_CC_2_INDEX);
                    g_busy[STEPPER_AXIS_TILT]         = false;
                    g_current_rpm[STEPPER_AXIS_TILT]   = 0;
                    DL_TimerA_stopCounter(TILT_TIMER);
                    DL_TimerA_clearInterruptStatus(TILT_TIMER,
                        DL_TIMER_INTERRUPT_ZERO_EVENT);
                }
            }
        }
    }
}

/*
 * TIMG6_IRQHandler — PAN 轴 (TIMG6) 中断服务例程
 */
void TIMG6_IRQHandler(void)
{
    uint32_t pending = DL_TimerG_getPendingInterrupt(PAN_TIMER);

    /* 零事件中断: 每个 PWM 周期触发一次 = 1 步 */
    if (pending & DL_TIMER_INTERRUPT_ZERO_EVENT) {
        DL_TimerG_clearInterruptStatus(PAN_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);

        /*
         * ★ 仅当轴处于活跃状态时才更新位置/步数。
         *    stopCounter 是异步操作, busy=false 后定时器可能仍会产生
         *    1~N 个额外零事件; 用 g_busy 守卫防止这些"幽灵脉冲"
         *    被计入位置, 否则高速时误差可达 1.7°+。
         */
        if (g_step_count_enabled && g_busy[STEPPER_AXIS_PAN]) {
            if (g_current_dir[STEPPER_AXIS_PAN] == STEPPER_DIR_CCW) {
                g_position_steps[STEPPER_AXIS_PAN]++;
            } else {
                g_position_steps[STEPPER_AXIS_PAN]--;
            }

            /* 位置模式: 递减步数, 到达目标后自动停止 */
            if (g_steps_remaining[STEPPER_AXIS_PAN] > 0) {
                g_steps_remaining[STEPPER_AXIS_PAN]--;

                if (g_steps_remaining[STEPPER_AXIS_PAN] == 0) {
                    /*
                     * ★ 停止顺序至关重要:
                     *   1. 冻结 CCP (CCR=PERIOD_MAX) → 输出锁死高电平
                     *   2. g_busy=false → 软件拒绝后续幽灵中断
                     *   3. stopCounter → 异步停止, 但 CCP 已冻结无伪边沿
                     *   4. clearInterruptStatus → 清理残留标志
                     */
                    DL_TimerG_setCaptureCompareValue(PAN_TIMER,
                        PERIOD_MAX, DL_TIMER_CC_0_INDEX);
                    g_busy[STEPPER_AXIS_PAN]         = false;
                    g_current_rpm[STEPPER_AXIS_PAN]   = 0;
                    DL_TimerG_stopCounter(PAN_TIMER);
                    DL_TimerG_clearInterruptStatus(PAN_TIMER,
                        DL_TIMER_INTERRUPT_ZERO_EVENT);
                }
            }
        }
    }
}

/* ==========================================================================
 * stepper_pulse_init — 初始化 DIR 引脚 + 使能步数计数 ISR
 * ==========================================================================
 *
 * 注意: cloud_1_TILT 和 cloud_2_PAN 的定时器、时钟、CCP 引脚复用
 * 已由 SYSCFG_DL_init() → SYSCFG_DL_cloud_1_TILT_init() /
 * SYSCFG_DL_cloud_2_PAN_init() 完成。
 *
 * 本函数额外:
 *   1. 配置 DIR 引脚为 GPIO 数字输出
 *   2. 使能定时器零事件中断
 *   3. 使能 NVIC
 *   4. 清零所有状态变量
 */
void stepper_pulse_init(void)
{
    /*
     * DIR 引脚已由 SYSCFG_DL_init() 初始化为 GPIO 数字输出,
     * 此处仅需确保上电默认低电平。使用 SysConfig 生成的宏,
     * 避免裸 PINCM 数字 (IOMUX_PINCM16=PB0 会误杀 TILT PULSE!)。
     */
    DL_GPIO_clearPins(PAN_DIR_PORT, PAN_DIR_PIN);
    DL_GPIO_clearPins(TILT_DIR_PORT, TILT_DIR_PIN);

    /* ── 清零状态变量 ── */
    g_busy[0]              = false;//标记电机当前有没有转动，而0/1是代表两个电机
    g_busy[1]              = false;
    g_current_rpm[0]       = 0;//配置当前实时转速
    g_current_rpm[1]       = 0;
    g_current_dir[0]       = STEPPER_DIR_CW;//默认为顺时针
    g_current_dir[1]       = STEPPER_DIR_CW;
    g_steps_remaining[0]   = 0;//配置剩余需要走的步数
    g_steps_remaining[1]   = 0;
    g_position_steps[0]    = 0;//当前绝对位置坐标
    g_position_steps[1]    = 0;
    g_step_count_enabled   = true;

    /* ── 使能 TIMA0 零事件中断 (TILT 轴) ── */
    DL_TimerA_enableInterrupt(TILT_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(cloud_1_TILT_INST_INT_IRQN);

    /* ── 使能 TIMG6 零事件中断 (PAN 轴) ── */
    DL_TimerG_enableInterrupt(PAN_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(cloud_2_PAN_INST_INT_IRQN);
}

/* ==========================================================================
 * stepper_pulse_set_speed — 动态调速 (速度模式)
 * ==========================================================================
 *
 * 速度模式下旁路步数计数: 设置 steps_remaining = 0, ISR 不会干预。
 * 电机持续运转直到主循环调用 stepper_pulse_stop()。
 */
 //axis:电机号，控制哪个电机
 //
void stepper_pulse_set_speed(uint8_t axis, uint8_t dir, uint16_t rpm)
{
    if (axis > 1) return;

    uint8_t  old_dir = g_current_dir[axis];
    uint16_t period;

    g_current_rpm[axis] = rpm;

    /* ── rpm==0: 停止 ── */
    if (rpm == STEPPER_RPM_STOP) {
        if (g_busy[axis]) {
            stepper_pulse_stop(axis);
        }
        return;
    }

    period = rpm_to_period(rpm);

    /* ── 速度模式: 旁路步数计数 (防止 ISR 误停) ── */
    g_steps_remaining[axis] = 0;

    /* ── 轴空闲: 设置方向并启动 PWM ── */
    if (!g_busy[axis]) {
        g_current_dir[axis] = dir;
        g_busy[axis] = true;               /* ★ 先标 busy, 再启动定时器 */
        if (axis == STEPPER_AXIS_PAN) {
            set_dir_pan(dir);
            timg_config_pwm(period);
        } else {
            set_dir_tilt(dir);
            tima_config_pwm(period);
        }
        return;
    }

    /* ── 轴运动中 ── */
    if (dir == old_dir) {
        /* ★ 同方向: 仅修改周期, 不停止 PWM 输出! */
        if (axis == STEPPER_AXIS_PAN) {
            DL_TimerG_setLoadValue(PAN_TIMER, period);
            DL_TimerG_setCaptureCompareValue(PAN_TIMER, period / 2U,
                                              DL_TIMER_CC_0_INDEX);
        } else {
            DL_TimerA_setLoadValue(TILT_TIMER, period);
            DL_TimerA_setCaptureCompareValue(TILT_TIMER, period / 2U,
                                              DL_TIMER_CC_2_INDEX);
        }
    } else {
        /*
         * 方向反转: stopCounter → CCR冻结 → g_busy=false →
         *           clearIRQ → 换向 → g_busy=true → 重启
         *
         * ★ 必须先停计数器再冻结 CCR (防止 ~50% 幽灵边沿),
         *   再设 g_busy=false (防止丢真实步数到位置跟踪)。
         */
        g_current_dir[axis] = dir;
        if (axis == STEPPER_AXIS_PAN) {
            DL_TimerG_stopCounter(PAN_TIMER);
            DL_TimerG_setCaptureCompareValue(PAN_TIMER,
                PERIOD_MAX, DL_TIMER_CC_0_INDEX);
            g_busy[axis] = false;
            DL_TimerG_clearInterruptStatus(PAN_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
            set_dir_pan(dir);
        } else {
            DL_TimerA_stopCounter(TILT_TIMER);
            DL_TimerA_setCaptureCompareValue(TILT_TIMER,
                PERIOD_MAX, DL_TIMER_CC_2_INDEX);
            g_busy[axis] = false;
            DL_TimerA_clearInterruptStatus(TILT_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
            set_dir_tilt(dir);
        }
        /* 重启: tima/timg_config_pwm 内部会解冻 CCR=period/2 */
        g_busy[axis] = true;
        if (axis == STEPPER_AXIS_PAN) {
            timg_config_pwm(period);
        } else {
            tima_config_pwm(period);
        }
    }
}

/* ==========================================================================
 * stepper_pulse_move — 有限步数运动 (★ 位置模式核心, 非阻塞)
 * ==========================================================================
 */

void stepper_pulse_move(uint8_t axis, uint8_t dir, uint16_t rpm, uint32_t steps)
{
    if (axis > 1) return;
    if (steps == 0) return;

    uint16_t period;

    /* ── 如果轴忙, 先停止当前运动 ── */
    if (g_busy[axis]) {
        stepper_pulse_stop(axis);
    }

    period = rpm_to_period(rpm);

    /* ── 设置方向 ── */
    g_current_dir[axis] = dir;
    if (axis == STEPPER_AXIS_PAN) {
        set_dir_pan(dir);
    } else {
        set_dir_tilt(dir);
    }

    /* ── 设置剩余步数 (必须在启动定时器之前!) ── */
    g_steps_remaining[axis] = steps;

    /* ── ★ 标记 busy 和 rpm 必须设在定时器启动之前!
     *     否则 ISR 可能在标志位设置前就完成运动,
     *     导致主循环覆盖 ISR 写入的值 → 状态卡死。 ── */
    g_current_rpm[axis] = rpm;
    g_busy[axis]        = true;

    /* ── 配置 PWM 并启动 ── */
    if (axis == STEPPER_AXIS_PAN) {
        timg_config_pwm(period);
    } else {
        tima_config_pwm(period);
    }

    /* 立即返回 — ISR 在后台计数, 到达目标步数后自动停止 */
}

/* ==========================================================================
 * stepper_pulse_stop — 停止指定轴
 * ========================================================================== */
void stepper_pulse_stop(uint8_t axis)
{
    if (axis > 1) return;

    /*
     * ★ 停止顺序 (v5 修正):
     *   1. stopCounter → 先停计数器 (停后改 CCR 无输出跳变)
     *   2. CCR=PERIOD_MAX → 冻结输出 (防御性, 此时已安全)
     *   3. g_busy=false → 软件拒收幽灵 ISR
     *   4. clearIRQ → 清理残留标志
     *
     *   ★ 必须 stopCounter 在 CCR 冻结之前! 否则 ~50% 概率
     *     产生 LOW→HIGH 幽灵边沿被驱动器当作一步。
     */
    if (axis == STEPPER_AXIS_PAN) {
        DL_TimerG_stopCounter(PAN_TIMER);
        DL_TimerG_setCaptureCompareValue(PAN_TIMER,
            PERIOD_MAX, DL_TIMER_CC_0_INDEX);
        g_busy[axis] = false;
        DL_TimerG_clearInterruptStatus(PAN_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    } else {
        DL_TimerA_stopCounter(TILT_TIMER);
        DL_TimerA_setCaptureCompareValue(TILT_TIMER,
            PERIOD_MAX, DL_TIMER_CC_2_INDEX);
        g_busy[axis] = false;
        DL_TimerA_clearInterruptStatus(TILT_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    }

    g_current_rpm[axis]      = 0;
    g_steps_remaining[axis]  = 0;
}

/* ==========================================================================
 * stepper_pulse_emergency_stop — 双轴急停
 * ========================================================================== */
void stepper_pulse_emergency_stop(void)
{
    /* ★ 先禁用 ISR 步数计数, 确保任何挂起的中断看到一致状态 */
    g_step_count_enabled = false;

    DL_TimerA_stopCounter(TILT_TIMER);
    DL_TimerG_stopCounter(PAN_TIMER);

    /* 清除挂起的中断标志 (防止 stale IRQ) */
    DL_TimerA_clearInterruptStatus(TILT_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_clearInterruptStatus(PAN_TIMER, DL_TIMER_INTERRUPT_ZERO_EVENT);

    /* 批量清零所有状态 (ISR 已被 g_step_count_enabled 阻挡, 安全) */
    g_busy[0]             = false;
    g_busy[1]             = false;
    g_current_rpm[0]      = 0;
    g_current_rpm[1]      = 0;
    g_steps_remaining[0]  = 0;
    g_steps_remaining[1]  = 0;

    /* 恢复 ISR (下次 stepper_pulse_move 可用) */
    g_step_count_enabled = true;
}

/* ==========================================================================
 * stepper_pulse_set_position_zero — 标记当前位置为逻辑零点
 * ========================================================================== */
void stepper_pulse_set_position_zero(uint8_t axis)
{
    if (axis > 1) return;
    g_position_steps[axis] = 0;
}

/* ==========================================================================
 * 查询函数
 * ========================================================================== */
bool stepper_pulse_is_busy(uint8_t axis)
{
    if (axis > 1) return false;
    return g_busy[axis];
}

uint16_t stepper_pulse_get_current_rpm(uint8_t axis)
{
    if (axis > 1) return 0;
    return g_current_rpm[axis];
}

uint8_t stepper_pulse_get_current_dir(uint8_t axis)
{
    if (axis > 1) return 0;
    return g_current_dir[axis];
}

int32_t stepper_pulse_get_position_steps(uint8_t axis)
{
    if (axis > 1) return 0;
    return g_position_steps[axis];
}

float stepper_pulse_get_position_deg(uint8_t axis)
{
    if (axis > 1) return 0.0f;
    return (float)g_position_steps[axis] * 360.0f / (float)STEPPER_PULSES_PER_REV;
}
