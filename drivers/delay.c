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
 * @note    使用忙等 (不依赖 WFE) — 避免 CCS 调试时 SysTick 暂停导致 WFE 永久睡眠,
 *          也避免 Cortex-M0+ WFE 事件寄存器竞态。
 * @note    必须在 SysTick_Config() 之后才能使用。
 * @note    CCS 调试时如 SysTick 被暂停, 本函数仍会卡住 —
 *          请配置 CCS: Debug Properties → MSPM0 → 取消 "Halt SysTick during debug"。
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms) {
        /* 忙等 — 比 __WFE() 更可靠, 不依赖 ARM 事件寄存器。
         * 电机控制应用不关心这 100ms 初始化的功耗。 */
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
