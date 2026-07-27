/**
 * @file    ADC.h
 * @brief   MSPM0G3507 ADC 驱动
 *
 * 提供 ADC 初始化、软件触发采样以及 ADC 输入电压换算等通用功能。
 *
 * 硬件连接:
 *   - PA15 (ADC1_CH0) ← 锂电池分压输出
 *   - 分压电路: 100kΩ (上) + 10kΩ (下), 分压比 11:1
 *   - 参考电压: VDDA = 3.3V
 *   - ADC 分辨率: 12-bit (0~4095)
 *
 * 典型使用流程:
 *   1. SYSCFG_DL_init();    // 系统初始化 (SysConfig 生成)
 *   2. ADC_Init();          // ADC 配置为软件触发模式
 *   3. raw = ADC_GetSamplingValue();  // 获取原始值
 *   4. v   = ADC_RawToInputVoltage(raw); // 换算为 ADC 输入端电压
 *
 * 锂电池电压换算通过 ADC_RawToBatteryVoltage() 单独提供，
 * 其内部包含本工程电阻分压电路的换算参数。
 */

#ifndef ADC_H
#define ADC_H

#include "ti_msp_dl_config.h"

/**
 * @brief   ADC 初始化函数
 *
 * 在 SYSCFG_DL_init() 之后调用，将 SysConfig 生成的 AUTO_NEXT (自动连续转换)
 * 覆盖为软件触发模式，并配置 PA15 为模拟输入、设置采样时间。
 *
 * @note    必须在每次系统复位后、使用 ADC 之前调用一次。
 * @note    本函数会先停止正在进行的转换，等待完成后再重新配置。
 */
void     ADC_Init(void);

/**
 * @brief   执行一次 ADC 软件触发采样，返回 12-bit 原始值
 *
 * 通过软件触发启动一次 ADC 转换，阻塞等待转换完成后返回结果。
 *
 * @return  uint16_t  ADC 原始采样值，范围 0 ~ 4095 (12-bit 分辨率)
 * @note    本函数为阻塞调用，转换期间会忙等，不适合需要低延迟的场景。
 * @note    返回值对应 ADC 输入端电压: V_in = raw × 3.3V / 4096
 */
uint16_t ADC_GetSamplingValue(void);

/**
 * @brief   将 ADC 原始值换算为 ADC 输入端电压
 *
 * 换算公式: V_in = raw × VREF / 4096
 *
 * @param   raw  ADC 原始采样值 (0 ~ 4095)
 * @return  float ADC 输入端电压，单位: 伏特 (V)
 * @note    该函数只反映 ADC 引脚电压，不包含外部电阻分压或其他传感器的换算。
 */
float    ADC_RawToInputVoltage(uint16_t raw);

/**
 * @brief   将 ADC 原始值换算为锂电池电压
 *
 * 先将原始值换算为 ADC 输入端电压，再乘以当前工程的电池分压比。
 *
 * @param   raw  ADC 原始采样值 (0 ~ 4095)
 * @return  float 锂电池总电压，单位: 伏特 (V)
 * @note    本函数仅适用于当前工程中 PA15 上的 100kΩ/10kΩ 电阻分压电路。
 */
float    ADC_RawToBatteryVoltage(uint16_t raw);

#endif
