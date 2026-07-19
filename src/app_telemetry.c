/**
 * @file    app_telemetry.c
 * @brief   通用 VOFA+ JustFloat 遥测模块实现
 *
 * 依赖：
 *   - bluetooth_uart.h  — BluetoothUART_Init / SendJustFloatNonBlocking / IsTxBusy
 *   - delay.h           — millis()
 */

#include "app_telemetry.h"
#include "bluetooth_uart.h"
#include "delay.h"
#include <stddef.h>     /* NULL */

/* ========================================================================
 * 模块内部状态
 * ======================================================================== */

static uint32_t s_interval_ms;       /* 最小发送间隔 */
static uint32_t s_last_send_ms;      /* 上次发送毫秒时间戳 */

/* ========================================================================
 * 公开接口
 * ======================================================================== */

void Telemetry_Init(uint8_t num_channels, uint8_t interval_ms)
{
    (void)num_channels;  /* 保留参数，未来可用于校验 */

    s_interval_ms  = interval_ms;
    s_last_send_ms = 0;

    BluetoothUART_Init();
}

void Telemetry_Send(const float *channels, uint8_t count)
{
    if (channels == NULL || count == 0) return;

    /* 1. 节流检查：距上次发送够 interval_ms 吗？ */
    uint32_t now = millis();
    if (now - s_last_send_ms < s_interval_ms)
        return;

    /* 2. 忙检查：TX 缓冲区还有数据吗？ */
    if (BluetoothUART_IsTxBusy())
        return;

    /* 3. 非阻塞发送 */
    if (BluetoothUART_SendJustFloatNonBlocking(channels, count))
    {
        s_last_send_ms = now;
    }
    /* 发送失败（缓冲满）则等下一帧重试 */
}
