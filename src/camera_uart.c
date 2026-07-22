/**
 * @file    camera_uart.c
 * @brief   K230 摄像头 UART2 接收驱动实现
 *
 * UART2 中断服务程序：将收到的字节写入环形缓冲。
 * 主循环通过 Available()/Read() 非阻塞消费。
 *
 * ISR 保护：单次 ISR 最多读 32 字节，防止死循环。
 * 缓冲满时静默丢弃（摄像头数据是连续流，丢一帧可接受）。
 */

#include "camera_uart.h"
#include "ti_msp_dl_config.h"

/* ===================================================================
 *  环形缓冲
 * =================================================================== */

static uint8_t  s_rx_buf[CAMERA_UART_RX_BUF_SIZE];
static volatile uint16_t s_head = 0;  /* ISR 写入 */
static uint16_t s_tail = 0;           /* 主循环读出 */

#define ISR_MAX_LOOP  32

/* ===================================================================
 *  UART2 中断服务程序
 * =================================================================== */

void camera_INST_IRQHandler(void)
{
    uint8_t loop = 0;

    while (!DL_UART_Main_isRXFIFOEmpty(camera_INST) && loop < ISR_MAX_LOOP) {
        uint8_t byte = DL_UART_Main_receiveData(camera_INST);
        loop++;

        uint16_t next = (s_head + 1) % CAMERA_UART_RX_BUF_SIZE;
        if (next != s_tail) {
            s_rx_buf[s_head] = byte;
            s_head = next;
        }
        /* else: 缓冲满，丢弃（摄像头流数据，丢一帧不致命） */
    }
}

/* ===================================================================
 *  公开 API
 * =================================================================== */

void CameraUART_Init(void)
{
    s_head = 0;
    s_tail = 0;

    DL_UART_Main_enableInterrupt(camera_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(camera_INST_INT_IRQN);
}

uint16_t CameraUART_Available(void)
{
    uint16_t head = s_head;
    if (head >= s_tail)
        return head - s_tail;
    else
        return (CAMERA_UART_RX_BUF_SIZE - s_tail) + head;
}

uint8_t CameraUART_Read(void)
{
    if (s_head == s_tail) return 0;

    uint8_t byte = s_rx_buf[s_tail];
    s_tail = (s_tail + 1) % CAMERA_UART_RX_BUF_SIZE;
    return byte;
}

void CameraUART_Flush(void)
{
    s_tail = s_head;
}
