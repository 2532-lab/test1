#include "delay.h"
#include "ti_msp_dl_config.h"

#define CPU_FREQ_HZ  32000000U
#define CYCLES_PER_US (CPU_FREQ_HZ / 1000000U)
#define CYCLES_PER_MS (CPU_FREQ_HZ / 1000U)

void delay_us(uint32_t us)
{
    if (us == 0) return;
    uint64_t cycles = (uint64_t)us * CYCLES_PER_US;
    while (cycles > 0) {
        uint32_t chunk = (cycles > 16000000UL) ? 16000000UL : (uint32_t)cycles;
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

void delay_ms(uint32_t ms)
{
    if (ms == 0) return;
    uint64_t cycles = (uint64_t)ms * CYCLES_PER_MS;
    while (cycles > 0) {
        uint32_t chunk = (cycles > 16000000UL) ? 16000000UL : (uint32_t)cycles;
        delay_cycles(chunk);
        cycles -= chunk;
    }
}

void delay_s(uint32_t s)
{
    delay_ms(s * 1000UL);
}
