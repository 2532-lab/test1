/**
 * @file    grayscale.c
 * @brief   五路灰度传感器模块实现
 *
 * ======================== 传感器初始化 ========================
 *
 *   SysConfig 已将 5 路 GPIO 配置为数字输入（内部下拉）。
 *   SYSCFG_DL_init() 调用过程中自动完成 IOMUX 配置。
 *
 *   Grayscale_Init() 当前为空——所有 GPIO 初始化已由
 *   SysConfig 生成代码完成。保留此函数以便将来可能需要
 *   运行时校准（如自动调节阈值）。
 *
 * ======================== 位置计算说明 ========================
 *
 *   加权平均法：
 *
 *   对每条检测到黑线的传感器（低电平=0），用其位置权重累加，
 *   最终除以黑线传感器数量得到加权平均位置。
 *
 *   为什么不用简单的"中间传感器编号"？
 *     当黑线跨越两个传感器时（如 S2 和 S3 都在黑线上），
 *     简单取中值会丢失精度。加权平均能为"半位"情况提供
 *     更平滑的位置输出，有利于 PID 控制器的稳定。
 *
 *   特殊情况：
 *     - 全白（line_count=0）→ 返回上次位置（防止跳跃）
 *     - 全黑（line_count=5）→ 加权平均 = 0（传感器太近或线太宽）
 *
 * ======================== 滤波说明 ========================
 *
 *   Grayscale_GetPositionFiltered() 对 position 做中值滤波。
 *   连续采样 samples 次（建议 3~5），排序后取中值。
 *
 *   为什么要滤波？
 *     TCRT5000 的输出在黑白边界处可能抖动（比较器阈值附近），
 *     瞬时读到错误的 0/1 会破坏位置计算。
 *     中值滤波可以有效去除这种单次毛刺。
 *
 *   中值 vs 均值：
 *     均值滤波：毛刺会污染结果（10% 的坏点使结果偏 10%）。
 *     中值滤波：只要坏点 < 50%，结果完全不受影响。
 *     缺点：需要排序，O(n²) 复杂度（n≤9 时可接受）。
 */

#include "grayscale.h"
#include "ti_msp_dl_config.h"

/* ===================================================================
 *  传感器权重表
 *
 *  下标 0~4 对应 S1~S5，值 -2000~+2000。
 *  物理意义：传感器离车体中线的距离（归一化到 ±2000）。
 *
 *  为什么用 ±2000 而不是 ±1000？
 *    给 PID 控制器更大的数值范围，避免浮点精度损失。
 *    position=0 表示线在正中间，±2000 表示线在传感器范围边缘。
 * =================================================================== */

static const int16_t GS_WEIGHT[5] = {
     2000,   /* S1: 最右（经实车验证） */
     1000,   /* S2 */
        0,   /* S3: 中央 */
    -1000,   /* S4 */
    -2000    /* S5: 最左 */
};

/* ===================================================================
 *  初始化
 * =================================================================== */

void Grayscale_Init(void)
{
    /* SysConfig 已通过 SYSCFG_DL_GPIO_init() 完成所有 GPIO 初始化 */
}

/* ===================================================================
 *  传感器读取
 * =================================================================== */

/**
 * @brief   读取单个传感器电平
 * @return  1=高电平(白), 0=低电平(黑)
 *
 * DL_GPIO_readPins 返回引脚掩码（非零=高, 零=低），
 * 归一化到 1/0 便于后续逻辑判断。
 */
static inline uint8_t GS_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0) ? 1 : 0;
}

void Grayscale_Read(Grayscale_t *gs)
{
    /* 逐路读取 5 个传感器的电平（1=白, 0=黑） */
    gs->raw.s1 = GS_ReadPin(GS_S1_PORT, GS_S1_PIN);
    gs->raw.s2 = GS_ReadPin(GS_S2_PORT, GS_S2_PIN);
    gs->raw.s3 = GS_ReadPin(GS_S3_PORT, GS_S3_PIN);
    gs->raw.s4 = GS_ReadPin(GS_S4_PORT, GS_S4_PIN);
    gs->raw.s5 = GS_ReadPin(GS_S5_PORT, GS_S5_PIN);

    /*
     * 统计黑线传感器数量
     *
     * (!raw.sx) 将 0→1（黑线=有效）、1→0（白面=不计）。
     * 累加得到 line_count = 0~5。
     */
    gs->line_count = (!gs->raw.s1) + (!gs->raw.s2) + (!gs->raw.s3)
                   + (!gs->raw.s4) + (!gs->raw.s5);

    gs->line_detected = (gs->line_count > 0) ? 1 : 0;
}

/* ===================================================================
 *  位置计算 —— 加权平均法
 * =================================================================== */

static void GS_ReadToArray(const GrayscaleData_t *raw, uint8_t *arr)
{
    arr[0] = raw->s1;
    arr[1] = raw->s2;
    arr[2] = raw->s3;
    arr[3] = raw->s4;
    arr[4] = raw->s5;
}

int16_t Grayscale_GetPosition(Grayscale_t *gs)
{
    /* 全白 → 保持上次位置（避免 position 跳变） */
    if (!gs->line_detected) {
        return gs->position;
    }

    uint8_t arr[5];
    GS_ReadToArray(&gs->raw, arr);

    int32_t sum_weighted = 0;
    uint8_t sum_count    = 0;

    for (uint8_t i = 0; i < 5; i++) {
        if (arr[i] == 0) {             /* 低电平 = 黑线 */
            sum_weighted += GS_WEIGHT[i];
            sum_count++;
        }
    }

    if (sum_count > 0) {
        gs->position = (int16_t)(sum_weighted / sum_count);
    }

    return gs->position;
}

/* ===================================================================
 *  中值滤波位置读取
 *
 *  采样 samples 次，每次完整执行 Read + GetPosition，
 *  收集 position 数组，排序后取中值。
 *
 *  samples 建议值：3（快速，适合高速）或 5（稳定，适合低速）。
 *  最大 9 次（防栈溢出——int16_t[9] = 18 字节，安全）。
 * =================================================================== */

int16_t Grayscale_GetPositionFiltered(Grayscale_t *gs, uint8_t samples)
{
    if (samples == 0) return gs->position;
    if (samples > 9)  samples = 9;

    int16_t buf[9];

    /* 连续采样 */
    for (uint8_t i = 0; i < samples; i++) {
        Grayscale_Read(gs);               /* 读传感器 */
        Grayscale_GetPosition(gs);        /* 计算位置 */
        buf[i] = gs->position;
    }

    /* 冒泡排序（n≤9 时效率足够，比快排简单且不递归） */
    for (uint8_t i = 0; i < samples - 1; i++) {
        for (uint8_t j = 0; j < samples - 1 - i; j++) {
            if (buf[j] > buf[j + 1]) {
                int16_t tmp   = buf[j];
                buf[j]        = buf[j + 1];
                buf[j + 1]    = tmp;
            }
        }
    }

    /* 取中值（奇数个 → 中间；偶数个 → 中间两数均值） */
    if (samples & 1) {
        return buf[samples / 2];
    } else {
        return (int16_t)((buf[samples / 2 - 1] + buf[samples / 2]) / 2);
    }
}
