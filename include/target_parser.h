/**
 * @file    target_parser.h
 * @brief   靶子定位协议解析 — $TARGET 帧解码
 *
 * 解析 K230 摄像头通过 UART2 发送的 $TARGET 协议帧。
 */

#ifndef TARGET_PARSER_H
#define TARGET_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 *  数据结构
 * =================================================================== */

/** 靶子定位数据 */
typedef struct {
    bool    valid;      /**< 是否检测到靶子（false = NONE） */
    float   h_deg;      /**< 水平角 (°)，正=偏右 */
    float   v_deg;      /**< 垂直角 (°)，正=偏下 */
    int     circles;    /**< 检测到的同心圆数（2~5） */
} TargetData_t;

/* ===================================================================
 *  API
 * =================================================================== */

/**
 * @brief  喂入一个字节
 * @return  1 = 解析出完整帧（out 有效）
 * @return  0 = 继续喂字节（帧不完整或非 $TARGET 行已跳过）
 * @return -1 = 收到完整一行但不是 $TARGET 格式（可调 TargetParser_GetLastLine 查看原文）
 */
int TargetParser_Feed(uint8_t byte, TargetData_t *out);

/**
 * @brief  获取最近收到的一行原始文本（调试用）
 */
const char *TargetParser_GetLastLine(void);

/**
 * @brief  重置解析器状态
 */
void TargetParser_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_PARSER_H */
