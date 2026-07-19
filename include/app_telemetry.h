/**
 * @file    app_telemetry.h
 * @brief   通用 VOFA+ JustFloat 遥测模块 —— 声明
 *
 * ======================== 设计意图 ========================
 *
 *   将 JustFloat 发送逻辑封装为独立模块，主函数只需：
 *     Telemetry_Init(6, 50);              // 6通道, 20Hz
 *     while (1) {
 *         float ch[6] = {rpm1, rpm2, rpm3, rpm4, phase, time};
 *         Telemetry_Send(ch, 6);          // 非阻塞，自动节流
 *     }
 *
 *   内部自动处理：
 *     1. 蓝牙初始化（委托 BluetoothUART_Init）
 *     2. 帧间隔节流（基于 millis 时间戳）
 *     3. 非阻塞发送（TX 环形缓冲 → ISR → TX FIFO）
 *     4. 忙检测（缓冲未空 → 跳过本帧）
 *
 * ======================== 为什么独立于 bluetooth_uart？ ========================
 *
 *   bluetooth_uart 是底层驱动（字节级 UART 操作）。
 *   app_telemetry 是应用层模块（业务级遥测管理）。
 *
 *   这种分层的好处：
 *     - 底层驱动可复用（蓝牙模块换型号不影响遥测逻辑）
 *     - 上层只需关心数据，不关心 UART 缓存和 ISR
 *     - 节流策略在应用层，驱动层不依赖时间概念
 *
 * ======================== 节流策略 ========================
 *
 *   每次 Telemetry_Send 调用时检查：
 *     1. 距上次成功发送 ≥ interval_ms？
 *     2. TX 缓冲区已空（上一帧已发完）？
 *
 *   两条都满足才发送。不满足则静默丢弃本帧。
 *
 *   这样设计的原因：
 *     - 控制周期可能比无线带宽快（200Hz 控制 → 20Hz 发送）
 *     - 自动降频不丢关键帧（丢的是中间帧，不是最新帧）
 *     - 无缓冲积压（不像队列模式会发过期数据）
 */

#ifndef APP_TELEMETRY_H_
#define APP_TELEMETRY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   初始化遥测模块
 * @param   num_channels  JustFloat 通道数（保留参数，未来校验用）
 * @param   interval_ms   最小发送间隔（毫秒）
 *
 *   内部自动调用 BluetoothUART_Init() 使能 UART_0 中断。
 *
 *   建议 interval_ms：
 *     6 通道 → 50ms (20Hz)：实时性/带宽平衡
 *     10 通道 → 100ms (10Hz)：大数据量降低频率
 *     3 通道 → 20ms (50Hz)：高频小数据
 */
void Telemetry_Init(uint8_t num_channels, uint8_t interval_ms);

/**
 * @brief   非阻塞发送一帧 JustFloat 数据
 * @param   channels  浮点数组，长度必须 ≥ count
 * @param   count     通道数量
 *
 * 每帧调用一次（通常放在主循环末尾），模块自动节流。
 * 总执行时间：< 1μs（跳过时）或 ~20μs（发送时，写 TX 缓冲）。
 */
void Telemetry_Send(const float *channels, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_H_ */
