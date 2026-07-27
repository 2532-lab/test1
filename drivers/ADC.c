/**
 * @file    ADC.c
 * @brief   ADC 驱动实现 —— 通用 ADC API + 锂电池电压测量
 *
 * 本模块使用 MSPM0G3507 的 ADC1 (12-bit SAR ADC) 外设，配合外部电阻分压电路
 * 实现软件触发的单次采样。原始采样和 ADC 输入端电压换算属于通用 API，
 * 锂电池总电压换算单独封装在 ADC_RawToBatteryVoltage() 中。
 *
 * 硬件说明:
 *   - ADC 输入端连接电阻分压网络的中间节点
 *   - 上分压电阻 R_TOP = 100kΩ, 下分压电阻 R_BOTTOM = 10kΩ
 *   - 分压比 = (100k + 10k) / 10k = 11.0
 *   - 参考电压 VDDA = 3.3V
 *
 * ADC 时序:
 *   - 戴维南等效电阻 ≈ 9kΩ, 采样电容 ≈ 12.5pF
 *   - 12-bit 分辨率要求采样时间 ≥ 33 个 ADC 时钟周期
 *   - 本模块设置采样时钟数 = 64 (留有裕量)
 */

#include "ADC.h"

/* ── 通用 ADC 参数 ────────────────────────────────────────── */
#define ADC_REFERENCE_VOLTAGE 3.30f  /**< ADC 参考电压 VDDA (V)     */
#define ADC_FULL_SCALE        4096.0f /**< 12-bit ADC 满量程 (2^12) */

/* ── 当前工程的锂电池分压电路参数 ─────────────────────────── */
#define BATTERY_R_TOP         100.0f  /**< 上分压电阻 (kΩ) */
#define BATTERY_R_BOTTOM       10.0f  /**< 下分压电阻 (kΩ) */

/** @brief 锂电池分压比: (100k + 10k) / 10k = 11.0 */
#define BATTERY_DIVIDER_RATIO \
    ((BATTERY_R_TOP + BATTERY_R_BOTTOM) / BATTERY_R_BOTTOM)

/**
 * @brief ADC 采样时钟数
 *
 * 戴维南等效电阻 ≈ 9kΩ × 采样电容 12.5pF → 时间常数 τ ≈ 112.5ns
 * 12-bit 分辨率需要 ≥ 9τ ≈ 1.01μs，对应 ≥ 33 个 ADC 时钟周期 (@32MHz)
 * 取 64 留有足够裕量。
 */
#define SAMPLE_CLOCKS 64


/**
 * @brief   ADC 模块初始化
 *
 * 在 SYSCFG_DL_init() 之后调用。SysConfig 默认将 ADC 配置为 AUTO_NEXT
 * (自动连续转换) 模式，本函数将其覆盖为软件触发模式 (TRIGGER_NEXT)，
 * 以便在 main 循环中由软件主动控制每次转换的时机。
 *
 * 具体操作:
 *   1. 停止当前正在进行的 ADC 转换并等待完成
 *   2. 将 PA15 配置为模拟功能引脚 (ADC1_CH0)
 *   3. 设置采样时钟数为 64 (确保 12-bit 精度)
 *   4. 重新配置转换存储器 (ADCMEM_0):
 *      - 输入通道: CH0 (PA15)
 *      - 参考电压: VDDA (3.3V)
 *      - 触发模式: 软件触发
 *      - 禁止平均和窗口比较
 *   5. 重新使能 ADC 转换
 *
 * @note   必须在 SYSCFG_DL_init() 之后调用，否则 SysConfig 的配置会被覆盖
 */
void ADC_Init(void)
{
    /* 停止当前转换并等待完成，确保后续配置安全 */
    DL_ADC12_disableConversions(ADC1_INST);
    while (DL_ADC12_getStatus(ADC1_INST) & DL_ADC12_STATUS_CONVERSION_ACTIVE);

    /* 将 PA15 配置为 ADC 模拟输入引脚 */
    DL_GPIO_initPeripheralAnalogFunction(GPIO_ADC1_IOMUX_C0);

    /* 设置采样时间 = 64 个 ADC 时钟周期，保证 12-bit 精度 */
    DL_ADC12_setSampleTime0(ADC1_INST, SAMPLE_CLOCKS);

    /* 配置转换存储器 0:
     *   - 输入通道: CH0 (PA15)
     *   - 参考电压: VDDA (3.3V)
     *   - 采样触发源: SCOMP0 (采样定时器 0)
     *   - 禁止硬件平均
     *   - 禁止 burnout 电流源
     *   - 触发模式: 软件触发 (由 DL_ADC12_startConversion 启动)
     *   - 禁止窗口比较器
     */
    DL_ADC12_configConversionMem(ADC1_INST, ADC1_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    /* 重新使能 ADC 转换 */
    DL_ADC12_enableConversions(ADC1_INST);
}


/**
 * @brief   执行一次软件触发采样
 *
 * 通过 DL_ADC12_startConversion() 软件触发一次 ADC 转换，
 * 然后阻塞等待转换完成，最后返回转换结果。
 *
 * 时序:
 *   1. 软件触发 → ADC 开始采样 (SAMPLE_CLOCKS 个周期)
 *   2. 采样结束 → ADC 开始逐次逼近转换 (12 个周期)
 *   3. 转换完成 → 结果存入 ADCMEM_0
 *   4. 函数返回原始值
 *
 * @return  uint16_t  12-bit ADC 原始值，范围 0 ~ 4095
 *          - 0:     输入电压 = 0V
 *          - 4095:  输入电压 = VDDA = 3.3V
 * @note   本函数为阻塞调用，在转换完成前会持续忙等。
 *         对于需要低功耗或非阻塞的场景，可改用中断方式。
 */
uint16_t ADC_GetSamplingValue(void)
{
    /* 软件触发一次 ADC 转换 */
    DL_ADC12_startConversion(ADC1_INST);
    /* 等待转换完成 (忙等) */
    while (DL_ADC12_getStatus(ADC1_INST) & DL_ADC12_STATUS_CONVERSION_ACTIVE);
    /* 读取并返回转换结果 */
    return (uint16_t)DL_ADC12_getMemResult(ADC1_INST, ADC1_ADCMEM_0);
}


/**
 * @brief   将 ADC 原始值换算为 ADC 输入端电压
 *
 * 公式: V_in = raw × ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE
 *
 * @param   raw  ADC 原始采样值 (0 ~ 4095)
 * @return  float ADC 输入端电压，单位: 伏特 (V)
 */
float ADC_RawToInputVoltage(uint16_t raw)
{
    return (float)raw * ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE;
}


/**
 * @brief   将 ADC 原始值换算为锂电池电压
 *
 * 先换算 ADC 输入端电压，再乘以当前工程的电池分压比。
 *
 * @param   raw  ADC 原始采样值 (0 ~ 4095)
 * @return  float 锂电池总电压，单位: 伏特 (V)
 */
float ADC_RawToBatteryVoltage(uint16_t raw)
{
    return ADC_RawToInputVoltage(raw) *
           BATTERY_DIVIDER_RATIO;
}
