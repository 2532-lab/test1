#include "delay.h"
#include "ti_msp_dl_config.h"

/* MSPM0G3507 默认 SYSOSC = 32MHz，MCLK = 32MHz */
#define CPU_FREQ_HZ  32000000U
#define CYCLES_PER_US (CPU_FREQ_HZ / 1000000U)   /* 1us = 32 个时钟周期 */
#define CYCLES_PER_MS (CPU_FREQ_HZ / 1000U)       /* 1ms = 32000 个时钟周期 */

/**
 * SysTick 毫秒计数器——由 SysTick_Handler 递增。
 * 需在 SysConfig 中启用 SysTick，并在中断服务函数中递增此变量。
 */
volatile uint32_t sys_tick_ms = 0;

/**
 * @brief 微秒级延时（粗糙版本，适用于 us >= 10）
 * @param us 微秒数
 */
void delay_us(uint32_t us)
{
    if (us == 0) return;

    uint64_t cycles = (uint64_t)us * CYCLES_PER_US;
    /* 单次 delay_cycles 最大约 2^24 个周期（~0.5秒），长延时拆成循环 */
    while (cycles > 0) {
        uint32_t chunk;
        if (cycles > 16000000UL) {
            chunk = 16000000UL;
        } else {
            chunk = (uint32_t)cycles;
        }
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

/**
 * @brief 毫秒级延时
 * @param ms 毫秒数
 */
void delay_ms(uint32_t ms)
{
    if (ms == 0) return;

    uint64_t cycles = (uint64_t)ms * CYCLES_PER_MS;
    while (cycles > 0) {
        uint32_t chunk;
        if (cycles > 16000000UL) {
            chunk = 16000000UL;
        } else {
            chunk = (uint32_t)cycles;
        }
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

/**
 * @brief 秒级延时
 * @param s 秒数
 */
void delay_s(uint32_t s)
{
    delay_ms(s * 1000UL);
}
