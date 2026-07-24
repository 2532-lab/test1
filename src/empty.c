/*
 * main.c — 双轴云台激光追踪靶心 (MSPM0G3507)
 *
 * K230 通过 UART2 发送 $TARGET,水平偏离角度,垂直偏离角度\r\n
 * MSPM0 接收后驱动双轴步进电机追踪目标。
 *
 * 状态流转: INIT(等待K230启动) → SCAN(扫描搜索) ⇄ TRACK(锁定追踪)
 */

#include "ti_msp_dl_config.h"
#include "vision_tracking.h"
#include "drivers/delay.h"
#include <stdint.h>
#include <stdbool.h>

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000U);
    Vision_Init();
    Vision_SetLaserOffset(0.0f, 0.0f);
    delay_ms(100);

    while (1) {
        Vision_Update();
    }
}
