/**
 * @file    camera_uart.h
 * @brief   K230 摄像头 UART2 接收驱动（中断 + 环形缓冲）
 *
 * 硬件：UART2 (PB18=RX, PB17=TX), 115200-8N1
 * 用途：接收 K230 的 $TARGET 靶子定位数据帧
 *
 * 设计：轻量版，仅 RX 接收（TX 不需要发数据给摄像头）。
 *       使用 128 字节环形缓冲，中断驱动接收。
 */

#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 *  常量
 * =================================================================== */

/** RX 环形缓冲容量 */
#define CAMERA_UART_RX_BUF_SIZE  256

/* ===================================================================
 *  API
 * =================================================================== */

/**
 * @brief  初始化摄像头 UART（使能 UART2 RX 中断）
 * @note   必须在 SYSCFG_DL_init() 之后调用
 */
void CameraUART_Init(void);

/**
 * @brief  获取可读字节数（非阻塞）
 * @return 缓冲区中待读取的字节数
 */
uint16_t CameraUART_Available(void);

/**
 * @brief  读取一个字节（非阻塞）
 * @return 字节值，缓冲空时返回 0
 * @note   先调用 Available() 确认有数据再读
 */
uint8_t CameraUART_Read(void);

/**
 * @brief  清空接收缓冲
 */
void CameraUART_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_UART_H */
