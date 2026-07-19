/**
 * @file    delay.h
 * @brief   阻塞延时 + millis() 毫秒时间戳 —— 声明
 *
 * ======================== 延时方案对比 ========================
 *
 *   阻塞延时（delay_ms / delay_us）：
 *     原理：循环执行空指令（NOP），烧 CPU 周期。
 *     优点：精度高（±1μs），实现简单，中断仍能响应。
 *     缺点：CPU 在此期间不能做其他事。
 *     适用：初始化阶段、微秒级 GPIO 时序控制、调试。
 *
 *   非阻塞延时（millis）：
 *     原理：SysTick 每 1ms 中断一次，ISR 递增 sys_tick_ms 计数器。
 *     优点：主循环不阻塞，可以做其他事。
 *     缺点：精度受 SysTick 中断频率限制（最小 1ms）。
 *     适用：状态机超时、遥测节流、看门狗喂狗。
 *
 * ======================== SysTick 配置（由 encoder.c 管理） ========================
 *
 *   本模块声明了 extern volatile uint32_t sys_tick_ms，
 *   但它实际上在 encoder.c 的 SysTick_Handler 中递增。
 *
 *   为什么用 encoder.c 管理 SysTick 而不是这里？
 *     encoder.c 的 Encoder_Scan() 也需要 1kHz 中断来扫描
 *     C2/C3/C4 编码器。两者共用同一个 SysTick 中断，
 *     避免占用多个定时器。
 *
 *   如果 encoder.c 未编译（如调试时不需编码器），
 *   需要在其他地方初始化 SysTick 并提供 SysTick_Handler，
 *   否则 millis() 始终返回 0 且 delay_ms() 不受影响。
 */

#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 *  millis() 毫秒时间戳
 *
 *  由 SysTick ISR（encoder.c）每 1ms 递增一次。
 *  32 位无符号：最大 2^32 ms ≈ 49.7 天。
 *  溢出后从 0 重新计数，这对计算时间差是安全的
 *  （无符号减法天然处理溢出： (now - last) 在溢出后依然正确）。
 * =================================================================== */

extern volatile uint32_t sys_tick_ms;

/**
 * @brief   获取系统上电以来的毫秒时间戳（非阻塞）
 * @return  自 SysTick 启动后的毫秒数 [0, 2^32-1]
 *
 * 溢出安全：假设 last=0xFFFFFF00, now=0x00000010（溢出了）
 *   uint32_t elapsed = now - last = 0x110 = 272ms ✓
 *   无符号整数的模运算确保了差值正确。
 *
 * 注意：如果中断被屏蔽超过 1ms，计数会丢失。
 *       裸机环境下通常不会（ISR 执行时间 < 50μs），
 *       但 RTOS 下需要确认 SysTick 优先级够高。
 */
static inline uint32_t millis(void)
{
    return sys_tick_ms;
}

/* ===================================================================
 *  阻塞延时
 * =================================================================== */

/**
 * @brief   微秒级阻塞延时
 * @param   us  微秒数 [1, 1000000]
 *
 * 实现：调用 MSPM0 DriverLib 的 delay_cycles()，循环消耗 CPU 周期。
 *   CPU_FREQ = 32MHz → 1μs = 32 cycles
 *   10μs 延时 = delay_cycles(320) 约 1.1μs 额外开销
 *
 * 精度：单次 delay_cycles 最大 2^24 (~16M) 个周期 ≈ 0.5s，
 *       长延时自动拆分为多次循环。
 *
 * @note  中断会打断 delay_cycles，导致实际延时会略长于参数值。
 *        对于微秒级时序（如 DS18B20 的 one-wire 协议），
 *        需要在调用前屏蔽中断。
 */
void delay_us(uint32_t us);

/**
 * @brief   毫秒级阻塞延时
 * @param   ms  毫秒数
 *
 * 实现：delay_cycles(ms × 32000)，内部循环处理长延时。
 *       功耗较高——CPU 在全速运行空循环。
 *       如果长时间阻塞（>100ms），建议改用状态机 + millis()。
 */
void delay_ms(uint32_t ms);

/**
 * @brief   秒级阻塞延时
 * @param   s  秒数
 *
 * 等价于 delay_ms(s × 1000)。
 */
void delay_s(uint32_t s);

#ifdef __cplusplus
}
#endif

#endif /* _DELAY_H_ */
