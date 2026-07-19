/**
 * @file    bluetooth_uart.h
 * @brief   BLE 蓝牙透传模块 UART 驱动 —— API（MSPM0G3507 UART_0, 115200 baud）
 *
 * ======================== 硬件连接 ========================
 *
 *   G3507 UART_0 → L1306 桥 → VG6328A-MS 蓝牙 → 手机/PC VOFA+
 *
 *   ┌─────────────────┐     UART_0     ┌─────────────┐     UART_1     ┌──────────┐
 *   │  G3507          │ PA10 ──────→   │  L1306 (桥)  │ PA14 ──────→   │ VG6328A  │
 *   │  (本工程)       │ PA11 ←──────   │  双向透传    │ PA22 ←──────   │ BLE 5.3  │
 *   └─────────────────┘                └─────────────┘                └──────────┘
 *
 *   引脚映射（SysConfig → 物理引脚）：
 *     UART_0 RX = PA11
 *     UART_0 TX = PA10
 *
 * ======================== 驱动架构 ========================
 *
 *   接收（RX）：中断驱动 → 环形缓冲区
 *     UART0_IRQHandler 将收到的字节写入 RX 环形缓冲（64 字节），
 *     主循环通过 Available() / Read() 非阻塞读取。
 *
 *   发送（TX 阻塞）：轮询 TX FIFO
 *     SendByte / SendString / SendBytes 轮询等待 TX FIFO 空闲，
 *     逐字节写入。适合 RESPONSE 类短数据（几十字节）。
 *
 *   发送（TX 非阻塞）：中断驱动 → 环形缓冲区 → TX FIFO
 *     SendJustFloatNonBlocking 将数据写入 TX 环形缓冲（128 字节），
 *     立即返回。UART0_IRQHandler 在后台将缓冲搬入 TX FIFO。
 *     适合遥测等高频、不丢帧的流式数据。
 *
 * ======================== JustFloat 协议 ========================
 *
 *   VOFA+ 上位机的 JustFloat 协议格式：
 *     [float0][float1]...[floatN][tail: 0x00,0x00,0x80,0x7F]
 *
 *   每个 float = 4 字节（小端序，Cortex-M0+ 原生的字节序）。
 *   帧尾 4 字节 = {0x00, 0x00, 0x80, 0x7F}（固定值，VOFA+ 由此识别帧边界）。
 *   总字节 = channel_count × 4 + 4。
 *
 *   例：6 通道 → 24 + 4 = 28 字节/帧。
 *   115200 baud 下每帧耗时 ≈ 28×10/115200 ≈ 2.4ms。
 *
 * ======================== 环形缓冲区设计 ========================
 *
 *   单生产者-单消费者模式（SPSC），无需锁：
 *     RX 缓冲：ISR 写 head（生产者），主循环读 tail（消费者）
 *     TX 缓冲：主循环写 head（生产者），ISR 读 tail（消费者）
 *
 *   满/空判断（保留 1 字节防止 head==tail 歧义）：
 *     (head + 1) % SIZE == tail → 满
 *     head == tail → 空
 *     有效容量 = SIZE - 1
 *
 *   RX 缓冲 64 字节：B 足够接收 2~3 条 AT 指令或一条短命令。
 *   TX 缓冲 128 字节：足够存 4 帧 6 通道 JustFloat（4×28=112B）。
 *     如果遥测发太快（<2ms 间隔），主循环写入速度超过 ISR 搬出速度，
 *     缓冲区会满 → 丢帧（SendJustFloatNonBlocking 返回 false）。
 */

#ifndef BLUETOOTH_UART_H_
#define BLUETOOTH_UART_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 *  常量与类型
 * =================================================================== */

/** RX 环形缓冲区容量 */
#define BLUETOOTH_UART_RX_BUF_SIZE  64

/** RX 环形缓冲（SPSC：ISR 写 head，主循环读 tail） */
typedef struct {
    uint8_t  buf[BLUETOOTH_UART_RX_BUF_SIZE];
    volatile uint8_t head;   /**< ISR 写入位置 */
    uint8_t  tail;           /**< 主循环读出位置 */
} BluetoothUART_t;

/** 全局 RX 缓冲实例 */
extern volatile BluetoothUART_t g_bluetooth_uart;

/** TX 环形缓冲区容量 */
#define BLUETOOTH_UART_TX_BUF_SIZE  128

/** TX 环形缓冲（SPSC：主循环写 head，ISR 读 tail） */
typedef struct {
    uint8_t  buf[BLUETOOTH_UART_TX_BUF_SIZE];
    volatile uint8_t head;   /**< 主循环写入位置 */
    volatile uint8_t tail;   /**< ISR 读出位置 */
} BluetoothUART_TxBuf_t;

/** 全局 TX 缓冲实例 */
extern volatile BluetoothUART_TxBuf_t g_bt_tx;

/* ===================================================================
 *  初始化 / 反初始化
 * =================================================================== */

/**
 * @brief   使能 UART_0 的 NVIC 中断和 RX 接收中断
 *
 * 必须在 SYSCFG_DL_init() 之后调用。
 * 调用后 UART0_IRQHandler 激活，收到的字节自动进入 RX 环形缓冲。
 *
 * TX 中断默认关闭（按需使能）——只有 TX 缓冲有数据时才开。
 */
void BluetoothUART_Init(void);

/**
 * @brief   禁止 UART_0 所有中断，清零缓冲区
 */
void BluetoothUART_Deinit(void);

/* ===================================================================
 *  接收（非阻塞，中断驱动）
 * =================================================================== */

uint8_t BluetoothUART_Available(void);
uint8_t BluetoothUART_Read(void);
void BluetoothUART_Flush(void);
void BluetoothUART_FlushTx(void);

/* ===================================================================
 *  发送（阻塞，轮询 TX FIFO）
 * =================================================================== */

void BluetoothUART_SendByte(uint8_t byte);
void BluetoothUART_SendString(const char *str);
void BluetoothUART_SendBytes(const uint8_t *data, uint16_t len);

/* ===================================================================
 *  JustFloat 协议发送
 * =================================================================== */

/**
 * @brief   阻塞发送 JustFloat 帧（轮询，逐字节等待）
 *
 * 简单但阻塞——发送期间主循环暂停。
 * 用于调试输出、低频数据（<10Hz）。
 */
void BluetoothUART_SendJustFloat(const float *data, uint8_t channel_count);

/**
 * @brief   非阻塞发送 JustFloat 帧（中断驱动）
 *
 * 数据写入 TX 环形缓冲后立即返回，ISR 后台搬入 TX FIFO。
 *
 * @retval true  帧已写入缓冲
 * @retval false TX 缓冲空间不足，帧被丢弃
 *
 * 适用场景：高频遥测（50~100Hz），不阻塞主循环。
 */
bool BluetoothUART_SendJustFloatNonBlocking(const float *data, uint8_t channel_count);

/**
 * @brief   查询 TX 缓冲是否仍有数据等待发送
 * @retval true  发送中（ISR 还在搬数据）
 * @retval false 发送完毕（TX 缓冲空 + TX FIFO 空）
 */
bool BluetoothUART_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_UART_H_ */
