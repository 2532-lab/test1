#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * SysTick 毫秒计数器（app_telemetry 非阻塞遥测需要）
 * ========================================================================== */

/**
 * @brief  SysTick 毫秒计数器 — 由 SysTick_Handler 递增。
 * @note   需在需调用 millis() 的源文件中定义。
 *         本变量在 delay.c 中定义，SysTick ISR 中递增。
 */
extern volatile uint32_t sys_tick_ms;

/**
 * @brief   获取系统上电以来的毫秒时间戳（非阻塞）。
 * @retval  自 SysTick 启动后的毫秒数。
 */
static inline uint32_t millis(void)
{
    return sys_tick_ms;
}

/* ==========================================================================
 * 阻塞延时
 * ========================================================================== */

/**
 * @brief 微秒级延时
 * @param us 微秒数 (1 ~ 1000000)
 * @note  基于 delay_cycles 实现，受中断影响有微小抖动
 */
void delay_us(uint32_t us);

/**
 * @brief 毫秒级延时
 * @param ms 毫秒数
 */
void delay_ms(uint32_t ms);

/**
 * @brief 秒级延时
 * @param s 秒数
 */
void delay_s(uint32_t s);

#ifdef __cplusplus
}
#endif

#endif /* _DELAY_H_ */
