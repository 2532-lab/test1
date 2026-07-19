/**
 * @file    app_telemetry.c
 * @brief   通用 VOFA+ JustFloat 遥测模块实现
 *
 * ======================== 工作流程 ========================
 *
 *   主循环每帧调用 Telemetry_Send()。
 *
 *   模块内部状态机：
 *
 *     收到 Send 请求
 *         │
 *     ┌───▼────────────┐
 *     │ 距上次发送      │
 *     │ ≥ interval_ms ? │──── 否 ──→ return（静默跳过）
 *     └───┬────────────┘
 *         │ 是
 *     ┌───▼────────────┐
 *     │ TX 缓冲已空 ?   │──── 否 ──→ return（上一帧还在发送中）
 *     └───┬────────────┘
 *         │ 是
 *     ┌───▼────────────┐
 *     │ 写入 TX 缓冲    │
 *     │ 更新 last_send  │
 *     └────────────────┘
 *
 *   掉帧是正常现象——控制回路 200Hz、蓝牙 20Hz，
 *   90% 的帧会被节流掉，这是预期行为而非 bug。
 */

#include "app_telemetry.h"
#include "bluetooth_uart.h"
#include "delay.h"
#include <stddef.h>

/* ===================================================================
 *  模块私有状态
 * =================================================================== */

/** 最小发送间隔（毫秒）—— 由 Telemetry_Init 设置 */
static uint32_t s_interval_ms;

/** 上次成功发送的毫秒时间戳 —— 用于节流判断 */
static uint32_t s_last_send_ms;

/* ===================================================================
 *  公开接口
 * =================================================================== */

void Telemetry_Init(uint8_t num_channels, uint8_t interval_ms)
{
    (void)num_channels;  /* 未来可用于运行时校验 count */

    s_interval_ms  = interval_ms;
    s_last_send_ms = 0;  /* 首次调用立即发送 */

    /*
     * 委托底层驱动初始化 UART_0
     * → 使能 RX 中断 + NVIC
     * TX 中断由 SendNonBlocking 按需开启
     */
    BluetoothUART_Init();
}

void Telemetry_Send(const float *channels, uint8_t count)
{
    /* 参数有效性 */
    if (channels == NULL || count == 0) return;

    /*
     * 节流检查：距上次成功发送是否达到 interval_ms
     *
     * 注意：这里用的是 millis()，依赖于 encoder.c 的 SysTick ISR。
     * 如果 encoder.c 未编译或 SysTick 未初始化，millis() 始终
     * 返回 0，s_last_send_ms 也始终为 0，now - 0 ≥ interval_ms
     * 恒成立 → 节流失效，每帧都尝试发送 → TX 缓冲区迅速填满。
     *
     * 这是有意为之的 tradeoff：如果 SysTick 没初始化，说明
     * 编码器也不需要（两者共用）。此时没有"控制周期"的概念，
     * 全线速发送也是合理的。
     */
    uint32_t now = millis();
    if (now - s_last_send_ms < s_interval_ms)
        return;

    /*
     * 忙检查：上一帧是否已发送完毕？
     *
     * IsTxBusy 检查 TX 环形缓冲是否非空。
     * 如果上一帧的 28 字节（6通道）还没被 ISR 搬完，
     * 此时再写新数据会覆盖未发送的旧数据。
     *
     * 注意：不检查 TX FIFO 本身（只检查缓冲）。
     * 缓冲空意味着 ISR 已搬完，即使 FIFO 还在移位输出最后
     * 几个字节也是安全的——新数据写入缓冲≠立即覆盖 FIFO。
     */
    if (BluetoothUART_IsTxBusy())
        return;

    /*
     * 非阻塞发送
     *
     * SendJustFloatNonBlocking 将数据写入 TX 环形缓冲后
     * 立即返回（不等待 UART 发送完成）。
     *
     * 失败（缓冲满）则静默丢弃——下次调用时可能空间已释放。
     */
    if (BluetoothUART_SendJustFloatNonBlocking(channels, count)) {
        s_last_send_ms = now;  /* 发送成功，记录时间戳 */
    }
}
