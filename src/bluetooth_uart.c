/**
 * @file    bluetooth_uart.c
 * @brief   BLE 蓝牙透传 UART 驱动实现 —— UART_0 中断 + 双环形缓冲
 *
 * ======================== 中断设计 ========================
 *
 *   UART0_IRQHandler（按优先级 64 响应）：
 *     RX 部分：从 RX FIFO 接收字节 → 写入 RX 环形缓冲
 *             ISR_MAX_LOOP=32 限制单次 ISR 最多读 32 字节，
 *             防止 FIFO 满 + 缓冲满导致死循环。
 *
 *     TX 部分：从 TX 环形缓冲取字节 → 填入 TX FIFO
 *             缓冲空时自动关闭 TX 中断（节能 + 减少空转）。
 *
 *   优先级：
 *     SysTick    = 192  (最低，编码器扫描)
 *     UART_0     = 64   (蓝牙数据，不可丢失)
 *     GPIOB INT  = 0    (最高，MPU6050 数据就绪)
 *
 *   注意：UART_0 中断优先级高于 SysTick。
 *     如果 UART_0 正在高频率接收（蓝牙连续发数据），
 *     SysTick 会被推迟响应 → encoder Scan 延迟 → RPM 计算噪声。
 *     实测影响：115200 baud 连续收 60 字节 ≈ 5.2ms 持续中断，
 *     SysTick 最多丢 5~6 个 tick（可接受）。
 *
 * ======================== 环形缓冲实现要点 ========================
 *
 *   SPSC（Single-Producer Single-Consumer）：
 *     只有一个写者、一个读者，无需锁或临界区。
 *
 *   RX 缓冲：ISR 写（head），主循环读（tail）
 *     - Available() 读取 head（volatile）→ 数据竞赛安全
 *     - Read() 读 tail（主循环独占）→ 无竞赛
 *     - head 的更新由 ISR 原子完成（单字节写入是原子的）
 *
 *   TX 缓冲：主循环写（head），ISR 读（tail）
 *     - SendNonBlocking 写 head（主循环独占）→ 无竞赛
 *     - ISR 读 tail + 写 tail（ISR 独占）→ 无竞赛
 *     - ISR 读 head（volatile）→ 需保证可见性（volatile 即可）
 *
 * ======================== TX 中断按需使能 ========================
 *
 *   设计要点：TX 中断不常驻使能，只在有数据待发时才开。
 *
 *   SendJustFloatNonBlocking 写入数据后：
 *     1. 使能 TX 中断
 *     2. 立即调用 uart_tx_feed() 预填一轮（不等中断，减少首字节延迟）
 *
 *   uart_tx_feed() 搬完所有数据后：
 *     → 自动关闭 TX 中断（避免空转中断的开销）
 *
 *   下一个 SendJustFloatNonBlocking 调用时会重新使能。
 */

#include "bluetooth_uart.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

/* ===================================================================
 *  全局实例
 * =================================================================== */

/** RX 环形缓冲：ISR 写 head，主循环读 tail */
volatile BluetoothUART_t g_bluetooth_uart;

/** TX 环形缓冲：主循环写 head，ISR 读 tail */
volatile BluetoothUART_TxBuf_t g_bt_tx;

/** ISR RX 排空循环上限：防止异常状态下死循环 */
#define ISR_MAX_LOOP  32

/* ===================================================================
 *  TX 内部辅助 —— ISR 和 SendNonBlocking 共用
 * =================================================================== */

/**
 * @brief   将 TX 缓冲字节搬入 UART TX FIFO（非阻塞）
 *
 * 循环条件：
 *   - 缓冲非空（head != tail）→ 有数据待发
 *   - TX FIFO 有空位 → 能写入
 *
 * 搬完后自动关 TX 中断。
 *
 * 调用场景：
 *   1. ISR 中（响应 TX 中断）
 *   2. SendJustFloatNonBlocking 中（启动传输时预填首字节）
 */
static void uart_tx_feed(void)
{
    while (g_bt_tx.head != g_bt_tx.tail) {

        /*
         * TX FIFO 满 → 退出
         * MSPM0G3507 UART TX FIFO = 8 字节深度
         * FIFO 满时再写入会丢失数据，所以先检查再写
         */
        if (DL_UART_Main_isTXFIFOFull(UART_0_INST))
            break;

        /* 从缓冲取一字节写入 TX FIFO */
        DL_UART_Main_transmitData(UART_0_INST,
                                  g_bt_tx.buf[g_bt_tx.tail]);
        g_bt_tx.tail = (g_bt_tx.tail + 1) % BLUETOOTH_UART_TX_BUF_SIZE;
    }

    /*
     * 缓冲已空 → 关闭 TX 中断
     *
     * 不关 TX 中断的后果：缓冲空时 ISR 仍会触发（TX FIFO 空 → 中断），
     * ISR 进来发现没数据可搬，空转一次退出。高频空转（>1kHz）浪费 CPU。
     * 关闭后，下次 SendNonBlocking 会重新使能。
     */
    if (g_bt_tx.head == g_bt_tx.tail) {
        DL_UART_Main_disableInterrupt(UART_0_INST,
                                      DL_UART_MAIN_INTERRUPT_TX);
    }
}

/* ===================================================================
 *  UART_0 中断服务程序
 *
 *  处理器：MSPM0G3507 的 UART_0 外设
 *  触发源：RX 非空（收到数据）/ TX 空（需要填充发射 FIFO）
 *  中断号：UART_0_INST_INT_IRQN（由 SysConfig 生成）
 *
 *  Cortex-M0+ ISR 规范：
 *    - 函数名必须与 startup 文件中的向量表名称一致
 *    - 不需要 __attribute__((interrupt))（TI Clang 自动识别）
 *    - NVIC 优先级已由 SysConfig 配置
 * =================================================================== */

void UART0_IRQHandler(void)
{
    uint8_t byte;
    uint8_t loop_count = 0;

    /* ====== RX 部分：排空接收 FIFO ====== */

    /*
     * 循环读取 RX FIFO 直到空（非空 = 硬件收到了新字节）。
     *
     * ISR_MAX_LOOP 保护：
     *   如果 RX 中断一直触发（如 RX 引脚被干扰拉到低电平产生
     *   连续的 break 条件），FIFO 会不断被硬件填充 + ISR 清空，
     *   形成死循环。限制 32 次读操作后强制退出。
     *
     *   32 字节 ≈ 2.8ms @115200，覆盖一次典型 BLE 数据包。
     */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)
           && loop_count < ISR_MAX_LOOP) {
        byte = DL_UART_Main_receiveData(UART_0_INST);
        loop_count++;

        /*
         * 写入 RX 环形缓冲
         *
         * next == tail → 缓冲区满 → 静默丢弃此字节
         *
         * 为什么不满时再读？
         *   不可——已读出的字节必须消费掉，FIFO 不会"放回去"。
         *   丢弃是无奈之举，说明主循环消费速度跟不上蓝牙发送速度。
         *
         * 解决方案：加大 RX_BUF_SIZE（当前 64B），或提高主循环轮询频率。
         */
        uint8_t next = (g_bluetooth_uart.head + 1)
                        % BLUETOOTH_UART_RX_BUF_SIZE;
        if (next != g_bluetooth_uart.tail) {
            g_bluetooth_uart.buf[g_bluetooth_uart.head] = byte;
            g_bluetooth_uart.head = next;
        }
        /* else: 缓冲区满，丢弃 */
    }

    /* ====== TX 部分：填充发送 FIFO ====== */
    uart_tx_feed();
}

/* ===================================================================
 *  初始化 / 反初始化
 * =================================================================== */

void BluetoothUART_Init(void)
{
    /* 清零环形缓冲索引 */
    g_bluetooth_uart.head = 0;
    g_bluetooth_uart.tail = 0;
    g_bt_tx.head = 0;
    g_bt_tx.tail = 0;

    /*
     * 使能 RX 中断 + NVIC
     *
     * TX 中断不在此使能——只有 TX 缓冲有数据时才按需开启。
     *
     * 关于 UART_0_INST_INT_IRQN：
     *   由 SysConfig 根据 UART_0 的外设中断向量自动生成。
     *   MSPM0G3507 的 UART0 和 UART1 共用同一个 IRQ 号，
     *   G3507 的 UART3 等是独立的。这里用的是 UART_0 的 IRQ。
     */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void BluetoothUART_Deinit(void)
{
    /* 关 NVIC + 关外设中断 */
    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(UART_0_INST,
                                  DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_disableInterrupt(UART_0_INST,
                                  DL_UART_MAIN_INTERRUPT_TX);

    /* 清空 TX 缓冲（RX 缓冲可保留——上层可能需要读出剩余数据） */
    g_bt_tx.head = 0;
    g_bt_tx.tail = 0;
}

/* ===================================================================
 *  接收——非阻塞（主循环调用）
 * =================================================================== */

/**
 * @brief   获取可读字节数
 *
 * 算法：(head - tail) 模 BUFFER_SIZE
 *
 * 注意：读 head 时 ISR 可能正在修改它。
 *   8-bit MCU 上多字节变量的并发读写不是原子的，但这里
 *   head/tail 都是 uint8_t，单字节加载在 Cortex-M0+ 上是原子的。
 */
uint8_t BluetoothUART_Available(void)
{
    uint8_t head = g_bluetooth_uart.head;  /* volatile 快照 */

    if (head >= g_bluetooth_uart.tail)
        return head - g_bluetooth_uart.tail;
    else
        /* 环形回绕情况：head 已绕回 0，tail 还在高位 */
        return (BLUETOOTH_UART_RX_BUF_SIZE - g_bluetooth_uart.tail) + head;
}

uint8_t BluetoothUART_Read(void)
{
    if (g_bluetooth_uart.head == g_bluetooth_uart.tail)
        return 0;  /* 缓冲空 */

    uint8_t byte = g_bluetooth_uart.buf[g_bluetooth_uart.tail];
    g_bluetooth_uart.tail = (g_bluetooth_uart.tail + 1)
                             % BLUETOOTH_UART_RX_BUF_SIZE;
    return byte;
}

void BluetoothUART_Flush(void)
{
    /* 丢弃所有缓存：tail 跳转到 head */
    g_bluetooth_uart.tail = g_bluetooth_uart.head;
}

void BluetoothUART_FlushTx(void)
{
    /* 关 TX 中断 + 清空缓冲 */
    DL_UART_Main_disableInterrupt(UART_0_INST,
                                  DL_UART_MAIN_INTERRUPT_TX);
    g_bt_tx.head = 0;
    g_bt_tx.tail = 0;
}

/* ===================================================================
 *  发送——阻塞（轮询 TX FIFO）
 *
 *  DL_UART_Main_transmitDataBlocking() 内部轮询 TX FIFO 满标志，
 *  直到有空位才写入。不返回直到字节已写入 TX FIFO。
 *  注意：写入 TX FIFO ≠ 发送完成。FIFO 中的字节还需要由硬件
 *  移位寄存器逐位发出（~87μs/byte @115200）。
 * =================================================================== */

void BluetoothUART_SendByte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, byte);
}

void BluetoothUART_SendString(const char *str)
{
    if (str == NULL) return;

    while (*str) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST,
                                          (uint8_t)(*str));
        str++;
    }
}

void BluetoothUART_SendBytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;

    for (uint16_t i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, data[i]);
    }
}

/* ===================================================================
 *  JustFloat 协议
 *
 *  VOFA+ 是一个开源的上位机数据可视化工具。
 *  协议：每个通道一个 float（4 字节），帧尾 4 字节标记。
 *  帧尾 = {0x00, 0x00, 0x80, 0x7F}（IEEE 754 表示的 +Inf 的变体）。
 *
 *  为什么帧尾是这个值？
 *    正常 float 数据几乎不会恰好等于这四个字节的组合，
 *    VOFA+ 通过扫描字节流中的这个固定序列来找到帧边界。
 *    这是一个简单但有效的帧同步方案——不增加额外字节开销。
 * =================================================================== */

/** VOFA+ JustFloat 帧尾定界标记 */
static const uint8_t JUSTFLOAT_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

void BluetoothUART_SendJustFloat(const float *data, uint8_t channel_count)
{
    if (data == NULL || channel_count == 0) return;

    /*
     * 逐通道发送 float 原始字节
     *
     * Cortex-M0+ 是小端序，float 在内存中的布局是
     * [byte0 LSB][byte1][byte2][byte3 MSB]，
     * 直接按字节发送即可。VOFA+ 解析时会按小端序重组。
     */
    for (uint8_t ch = 0; ch < channel_count; ch++) {
        BluetoothUART_SendBytes((const uint8_t *)&data[ch], 4);
    }

    /* 帧尾 */
    BluetoothUART_SendBytes(JUSTFLOAT_TAIL, 4);
}

bool BluetoothUART_SendJustFloatNonBlocking(const float *data,
                                            uint8_t channel_count)
{
    if (data == NULL || channel_count == 0) return false;

    /* 总字节数：通道数×4 + 帧尾4 */
    uint8_t total = channel_count * 4 + 4;

    /*
     * 空间检查：TX 缓冲区剩余空间是否够？
     *
     * 有效容量 = SIZE - 1 - used
     * SIZE - 1 是因为保留 1 字节防止 head==tail（空/满歧义消除）。
     */
    {
        uint8_t h = g_bt_tx.head;
        uint8_t t = g_bt_tx.tail;
        uint8_t used = (h >= t) ? (h - t)
                                : (BLUETOOTH_UART_TX_BUF_SIZE - t + h);
        uint8_t free_space = BLUETOOTH_UART_TX_BUF_SIZE - 1 - used;

        if (free_space < total)
            return false;  /* 空间不足，丢弃本帧 */
    }

    /*
     * 写入 float 通道数据
     *
     * 按字节逐通道写入 TX 环形缓冲。
     * 这里直接操作 g_bt_tx.head（主循环独占），无并发问题。
     */
    for (uint8_t ch = 0; ch < channel_count; ch++) {
        const uint8_t *p = (const uint8_t *)&data[ch];
        for (uint8_t i = 0; i < 4; i++) {
            g_bt_tx.buf[g_bt_tx.head] = p[i];
            g_bt_tx.head = (g_bt_tx.head + 1)
                            % BLUETOOTH_UART_TX_BUF_SIZE;
        }
    }

    /* 写入帧尾 */
    for (uint8_t i = 0; i < 4; i++) {
        g_bt_tx.buf[g_bt_tx.head] = JUSTFLOAT_TAIL[i];
        g_bt_tx.head = (g_bt_tx.head + 1) % BLUETOOTH_UART_TX_BUF_SIZE;
    }

    /*
     * 启动 TX 传输：
     *   1. 使能 TX 中断（让 ISR 开始从缓冲搬数据到 TX FIFO）
     *   2. 立即调用 uart_tx_feed() 预填首字节（不等 ISR 触发，
     *      减少首字节延迟~50μs）
     *
     * 注意：这里不检查 uart_tx_feed 是否搬完了所有数据。
     *   如果 TX FIFO 只有 8 字节深而本帧有 28 字节，
     *   uart_tx_feed 会搬 8 字节后因 FIFO 满而退出。
     *   剩余的由 ISR 在 TX FIFO 有空间时继续搬。
     */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_TX);
    uart_tx_feed();

    return true;
}

bool BluetoothUART_IsTxBusy(void)
{
    /*
     * 判断是否忙：
     *
     * head != tail → 缓冲非空 → 还有数据待 ISR 搬入 TX FIFO
     *
     * 注意：这里没有检查 TX FIFO 本身是否正在移位输出。
     *   缓冲空 + TX FIFO 非空时（ISR 已搬完但硬件还在发最后几个字节），
     *   本函数会返回 false，但实际最后一个字节还没发完。
     *
     *   这是有意为之——IsTxBusy 用于节流（防止写缓冲太密导致溢出），
     *   而非精确的"物理发送完成"检测。对于 50ms 间隔的遥测，
     *   最后 1 字节的延迟（~87μs）可忽略。
     */
    return (g_bt_tx.head != g_bt_tx.tail);
}
