/*
 * delay.h — 毫秒/微秒延时 + 系统时钟 (基于 SysTick)
 *
 * 目标芯片: MSPM0G3507
 * 依赖:     SysTick 配置为 1ms 周期, CPUCLK_FREQ = 32 MHz
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
 * 典型用法:
 *   uint32_t now = millis();
 *   if (now - last_time >= INTERVAL_MS) { ... }  // 溢出安全
 */
static inline uint32_t millis(void)
{
    return sys_tick_ms;
}

/* ==========================================================================
 * 阻塞延时 (仅调试用, 主循环禁止)
 * ========================================================================== */

/*
 * delay_ms — 毫秒延时。
 * 必须在 SysTick_Config() 之后才能使用。
 */
void delay_ms(uint32_t ms);

/*
 * delay_us — 微秒延时。
 * 使用 delay_cycles() 实现粗略的微秒延时。
 * 精度: ±10% @ 32 MHz。
 */
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */
