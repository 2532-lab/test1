/*
 * delay.c — 毫秒/微秒延时实现
 *
 * 目标芯片: MSPM0G3507
 * CPUCLK_FREQ = 32 MHz
 */

#include "delay.h"
#include "ti_msp_dl_config.h"

/* 全局 SysTick 毫秒计数器 (由 main.c 中的 SysTick_Handler 递增) */
extern volatile uint32_t sys_tick_ms;

/**
 * @brief   毫秒级阻塞延时 (基于 SysTick 计数器)。
 * @param   ms   延时毫秒数。
 * @note    使用 __WFE() 低功耗等待, 由 SysTick 中断唤醒。
 * @note    必须在 SysTick_Config() 之后才能使用。
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms) {
        /* 等待 SysTick 递增 */
        __WFE();  /* 低功耗等待事件, 被 SysTick 唤醒 */
    }
}

/**
 * @brief   微秒级阻塞延时 (基于 delay_cycles)。
 *
 * delay_cycles(n) 延迟约 n×4 个 CPU 周期。
 * 32 MHz → 1 μs = 32 cycles → delay_cycles(8)。
 *
 * @param   us   延时微秒数。
 * @note    精度: ±10% @ 32 MHz; 长延时用 delay_ms()。
 */
void delay_us(uint32_t us)
{
    /* CPUCLK_FREQ / 4,000,000 = 32,000,000 / 4,000,000 = 8 */
    uint32_t cycles_per_us = CPUCLK_FREQ / 4000000UL;
    if (cycles_per_us == 0) cycles_per_us = 1;
    uint32_t total = us * cycles_per_us;
    while (total > 0) {
        uint32_t chunk = (total > 65535UL) ? 65535UL : total;
        delay_cycles((uint32_t)chunk);
        total -= chunk;
    }
}
