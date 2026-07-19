/**
 * @file    delay.c
 * @brief   阻塞延时实现 —— 基于 delay_cycles()
 *
 * ======================== 时钟基础 ========================
 *
 *   MSPM0G3507 默认系统时钟：
 *     SYSOSC = 32MHz（内部高速振荡器）
 *     MCLK   = 32MHz（CPU 主时钟 = SYSOSC / 1）
 *     BUSCLK = 32MHz（外设总线时钟）
 *
 *   delay_cycles(n) 消耗 n 个 MCLK 周期。
 *   Cortex-M0+ 每条指令 1~2 周期，简单循环约 3 周期/迭代。
 *
 *   换算关系：
 *     1μs = 32 个 MCLK 周期（32MHz）
 *     1ms = 32000 个 MCLK 周期
 *     1s  = 32000000 个 MCLK 周期
 *
 * ======================== 实现限制 ========================
 *
 *   delay_cycles() 内部使用 24 位计数器，最大延迟 = 2^24 - 1 ≈ 16,777,215 周期。
 *   约 0.524s（16777215/32000000）。
 *
 *   超过此值的延时会被拆分成多次循环：
 *     while (cycles > 0) {
 *         chunk = min(cycles, 16000000);   // 留 ~1% 安全裕量
 *         delay_cycles(chunk);
 *         cycles -= chunk;
 *     }
 *   每次循环有少量函数调用和判断开销（~10 cycles），
 *   长延时（>1s）的累积误差在 0.1% 以内，可接受。
 *
 * ======================== 中断影响 ========================
 *
 *   delay_cycles() 不会屏蔽中断。如果在延时过程中发生中断，
 *   ISR 执行时间会计入延时，导致实际延时 > 参数值。
 *
 *   典型中断打断量：
 *     SysTick ISR (1kHz):  ~50μs/ms → 约 5% 额外延迟
 *     UART ISR (115200):  每字节 ~87μs → 连续接收时额外延迟显著
 *     Timer ISR:          取决于处理逻辑
 *
 *   对时序敏感的外设操作（如 I2C 软件模拟），应在调用本函数前
 *   临时屏蔽中断：__disable_irq() / __enable_irq()。
 */

#include "delay.h"
#include "ti_msp_dl_config.h"

/* ===================================================================
 *  时钟常量
 * =================================================================== */

/** 系统主频：32MHz（对应 MSPM0G3507 默认 SYSOSC 配置） */
#define CPU_FREQ_HZ      32000000U

/** 1 微秒对应的时钟周期数 */
#define CYCLES_PER_US    (CPU_FREQ_HZ / 1000000U)

/** 1 毫秒对应的时钟周期数 */
#define CYCLES_PER_MS    (CPU_FREQ_HZ / 1000U)

/** 单次 delay_cycles 安全上限（留 ~4.7% 裕量，防 24-bit 溢出） */
#define MAX_CHUNK_CYCLES 16000000UL

/* ===================================================================
 *  延时实现
 * =================================================================== */

void delay_us(uint32_t us)
{
    if (us == 0) return;

    uint64_t cycles = (uint64_t)us * CYCLES_PER_US;

    while (cycles > 0) {
        uint32_t chunk = (cycles > MAX_CHUNK_CYCLES)
                         ? MAX_CHUNK_CYCLES
                         : (uint32_t)cycles;
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

void delay_ms(uint32_t ms)
{
    if (ms == 0) return;

    uint64_t cycles = (uint64_t)ms * CYCLES_PER_MS;

    while (cycles > 0) {
        uint32_t chunk = (cycles > MAX_CHUNK_CYCLES)
                         ? MAX_CHUNK_CYCLES
                         : (uint32_t)cycles;
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

void delay_s(uint32_t s)
{
    /* 直接复用 delay_ms —— 秒级精度足够 */
    delay_ms(s * 1000UL);
}
