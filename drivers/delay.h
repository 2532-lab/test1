/*
 * delay.h — 毫秒/微秒延时与系统时钟（基于 SysTick）
 *
 * 目标芯片: MSPM0G3507
 * 依赖:     SysTick 配置为 1ms 周期, CPUCLK_FREQ = 32 MHz
 *
 * === ★ 系统时钟参数说明 ===
 *
 *   CPUCLK_FREQ (32,000,000 Hz):
 *     由 SysConfig 在 ti_msp_dl_config.h 中定义，表示 CPU 主频。
 *     取值范围: 取决于硬件晶振和 PLL 配置 (MSPM0G3507 默认 32MHz)。
 *     作用:     决定 delay_us() 的精度和所有定时器的时基。
 *     注意:     修改 SYSPLL 配置后必须同步修改此值，否则所有延时不准。
 *
 *   SysTick 周期:
 *     通过 SysTick_Config(CPUCLK_FREQ / 1000U) 配置为 1ms 中断周期。
 *     sys_tick_ms 每 1ms 递增 1，用作系统时间基准。
 *     millis() 返回此值 — 约 49.7 天回绕到 0 (uint32_t 最大值)。
 *     使用 (now - last >= INTERVAL) 模式可安全处理溢出。
 *
 * === 使用方式 ===
 *   在 main.c 中:
 *     1. 声明 volatile uint32_t sys_tick_ms = 0;
 *     2. 实现 SysTick_Handler:  void SysTick_Handler(void) { sys_tick_ms++; }
 *     3. main() 中:  SysTick_Config(CPUCLK_FREQ / 1000U);
 *   此后即可使用 millis() 获取系统毫秒时间戳。
 */

#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 全局 SysTick 毫秒计数器
 * ========================================================================== */

/**
 * @brief  SysTick 毫秒计数器 — 由 SysTick_Handler 递增。
 * @note   需在 main.c 或任意源文件中定义: volatile uint32_t sys_tick_ms = 0;
 * @note   需在 main() 中调用 SysTick_Config(CPUCLK_FREQ / 1000U) 启动。
 */
extern volatile uint32_t sys_tick_ms;

/**
 * @brief   获取系统上电以来的毫秒时间戳 (非阻塞)。
 * @retval  自 SysTick 启动后的毫秒数 (uint32_t, 溢出周期 ≈ 49.7 天)。
 *
 * 典型用法 (溢出安全):
 *   uint32_t now = millis();
 *   if (now - last_time >= INTERVAL_MS) { last_time = now; ... }
 *
 * ★ 调度参数说明:
 *   empty.c 主循环中 IMU_Update() 以 5ms 间隔调用 (200Hz)。
 *   该周期与 app_imu 中 s_filter.dt = 0.005 严格匹配。
 *   修改调度速率需同步修改 s_filter.dt，否则互补滤波器不收敛。
 */
static inline uint32_t millis(void)
{
    return sys_tick_ms;
}

/* ==========================================================================
 * 阻塞延时 (仅调试用, 主循环中禁止长时间阻塞)
 * ========================================================================== */

/*
 * delay_ms — 毫秒级阻塞延时。
 *
 *   必须在 SysTick_Config() 之后才能使用。
 *   实现: 忙等轮询 sys_tick_ms，不依赖 WFE 指令（避免调试器暂停 SysTick
 *        导致永久睡眠，以及 Cortex-M0+ WFE 事件寄存器竞争）。
 *
 *   ★ CCS 调试注意:
 *     CCS 默认在断点处暂停 SysTick。如果 delay_ms() 在调试期间卡死，
 *     请在 CCS → 项目属性 → MSPM0 调试设置中取消 "Pause SysTick while debugging"。
 *
 *   ★ 性能注意:
 *     阻塞延时占用 CPU 100%，仅用于初始化阶段（如 MPU6050 上电等待）。
 *     主循环中禁止使用 delay_ms()，应使用 millis() + 状态机实现非阻塞定时。
 */
void delay_ms(uint32_t ms);

/*
 * delay_us — 微秒级阻塞延时。
 *
 *   使用 delay_cycles() 实现，基于 CPUCLK_FREQ 换算。
 *   精度: ±10% @ 32 MHz（因为 delay_cycles 的周期计数精度约为 ±4 cycles）。
 *   适用范围: 1us ~ 数千 us（长延时请用 delay_ms）。
 *
 *   ★ CPUCLK_FREQ 影响:
 *     delay_cycles(n) ≈ n × 4 个 CPU 周期。
 *     32 MHz → 1 μs = 32 cycles → delay_cycles(8)。
 *     如果修改 CPU 时钟频率，必须在调用 delay_us 之前更新 CPUCLK_FREQ 宏。
 */
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */
