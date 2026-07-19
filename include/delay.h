#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t sys_tick_ms;

static inline uint32_t millis(void)
{
    return sys_tick_ms;
}

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

#ifdef __cplusplus
}
#endif

#endif
