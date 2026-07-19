/*
 * tracking_main.c — 步进电机云台 CanMV 矩形追踪 (主程序)
 *
 * 目标芯片:    MSPM0G3507 (TI DriverLib, 严禁 STM32 HAL / 标准库)
 * 依赖:        SysConfig → ti_msp_dl_config.h
 *              drivers/gimbal_control.h  → gimbal_track_velocity() 速度追踪
 *              drivers/delay.h           → millis() 系统时钟
 *              canmv/shape_detect.py     → CanMV K230 形状检测 (已存在)
 *
 * === 系统架构 ===
 *
 *   [CanMV K230 庐山派]                  [MSPM0G3507]
 *   shape_detect.py                     tracking_main.c
 *   det_w=480 det_h=270                      │
 *       │                                UART RX ISR (10 字节协议)
 *   findContours()                       ring buffer (256B)
 *   classify()                                │
 *   EMA + temporal persist              parse_canmv_packets()
 *       │                                     │
 *   uart_send() ──UART2 115200──→       filter: rectangle/square
 *   10 字节/帧                                │
 *                                      gimbal_track_velocity()
 *                                      (50Hz, EMA+P控制+调速)
 *                                            │
 *                                      stepper_pulse_set_speed()
 *                                            │
 *                                      TIMA0(TILT)/TIMG6(PAN) PWM
 *
 * === CanMV → MSPM0 UART 协议 (shape_detect.py:32-44) ===
 *
 *   帧格式 (10 字节):
 *     [0x55] [0xAA] [code] [errX_H] [errX_L] [errY_H] [errY_L] [CHK] [0x0D] [0x0A]
 *      ───────┬───────  ───────────────────┬───────────────────  ──┬──  ───┬───
 *         帧头标记              负载 (5 字节)                  校验和  CR   LF
 *
 *   code:   形状编码 (1=三角, 2=正方, 3=矩形, 4=圆)
 *   errX:   int16_t big-endian, 水平像素偏差 (cx - 240, 范围 ±240)
 *   errY:   int16_t big-endian, 垂直像素偏差 (cy - 135, 范围 ±135)
 *   CHK:    (code + errX_H + errX_L + errY_H + errY_L) & 0xFF
 *
 *   本程序仅追踪矩形 (code=3) 和正方形 (code=2)。
 *
 * === 快速启动 ===
 *
 *   1. SysConfig 中配置 UART (名称: cloud_uart_canmv):
 *        RX 引脚: 任意可用 GPIO (如 PA10)
 *        波特率: 115200, 8N1
 *        RX 中断: 使能
 *
 *   2. 将 src/tracking_main.c 加入 CCS 工程编译
 *
 *   3. 接线: CanMV K230 UART2 TX(IO11) → MSPM0 RX
 *
 *   4. 上电顺序:
 *        a. 步进电机驱动器上电
 *        b. MSPM0 上电 → gimbal_init() → SysTick → UART → 进入追踪循环
 *        c. CanMV 上电 → shape_detect.py 自动运行 → 开始发 UART 数据
 *        d. 将矩形放在摄像头前 → 云台自动追踪!
 *
 * === 调参指南 ===
 *
 *   追踪参数在 gimbal_control.h 中调整:
 *     GIMBAL_TRACK_KP_PAN/TILT  → P 增益 (默认 30 RPM/px)
 *     GIMBAL_DEADBAND_PAN/TILT  → 像素死区 (默认 3px, 480p 建议 5px)
 *     GIMBAL_EMA_ALPHA          → EMA 平滑系数 (默认 0.15)
 *     GIMBAL_TRACK_MAX_RPM      → 最大追踪转速 (默认 2000)
 *     GIMBAL_LOST_TIMEOUT_MS    → 目标丢失超时 (默认 200ms)
 *
 *   首次测试建议:
 *     - 用低速: gimbal_pan.rpm = 60; gimbal_tilt.rpm = 60;
 *     - 增大死区: GIMBAL_DEADBAND_PAN/TILT = 8
 *     - 确认方向正确后再逐步调高 RPM
 */

/* ==========================================================================
 * 系统头文件
 * ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* ==========================================================================
 * 项目模块
 * ========================================================================== */
#include "drivers/gimbal_control.h"
#include "drivers/delay.h"

/* ==========================================================================
 * SysTick — 1ms 时钟 (delay.h 依赖)
 * ========================================================================== */
volatile uint32_t sys_tick_ms = 0;

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/* ==========================================================================
 * UART 定义 — 在 SysConfig 中创建名为 cloud_uart_canmv 的 UART 外设
 * ========================================================================== */
#define TRACK_UART              cloud_uart_canmv_INST
#define TRACK_UART_IRQN         cloud_uart_canmv_INST_INT_IRQN

/* ==========================================================================
 * CanMV 协议常量 (与 shape_detect.py 保持一致)
 * ========================================================================== */
#define CANMV_FRAME_HEADER1     0x55    /* 帧头第 1 字节 */
#define CANMV_FRAME_HEADER2     0xAA    /* 帧头第 2 字节 */
#define CANMV_FRAME_TAIL_CR     0x0D    /* 帧尾 CR */
#define CANMV_FRAME_TAIL_LF     0x0A    /* 帧尾 LF */
#define CANMV_FRAME_SIZE        10      /* 总帧长 (字节) */

/* 形状编码 (与 shape_detect.py:175 一致) */
#define SHAPE_TRIANGLE          1
#define SHAPE_SQUARE            2
#define SHAPE_RECTANGLE         3
#define SHAPE_CIRCLE            4

/*
 * 追踪目标过滤: 仅追踪矩形和正方形。
 * 如需追踪所有形状, 设为 false。
 */
#define TRACK_ONLY_RECTANGLES   true

/* ==========================================================================
 * UART 环形缓冲区 (中断驱动接收)
 * ========================================================================== */
#define UART_RX_BUF_SIZE        256

static volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;
static volatile bool     uart_rx_overflow = false;

/* ==========================================================================
 * 数据包解析 — 10 字节 CanMV 协议
 * ========================================================================== */

/* 解析状态机 */
typedef enum {
    PARSE_WAIT_H1 = 0,      /* 等待 0x55 */
    PARSE_WAIT_H2,          /* 等待 0xAA */
    PARSE_DATA,             /* 收集 5 字节负载 (code + errX_H/L + errY_H/L + chk) */
    PARSE_WAIT_CR,          /* 验证 0x0D */
    PARSE_WAIT_LF,          /* 验证 0x0A */
} canmv_parse_state_t;

/* 全局追踪数据 (lock-free: ISR/parser 写入, 主循环读取) */
static volatile int16_t  g_error_x = 0;
static volatile int16_t  g_error_y = 0;
static volatile uint8_t  g_shape_code = 0;
static volatile bool     g_tracking_data_ready = false;
static volatile uint32_t g_last_valid_packet_ms = 0;

/* 统计计数器 (调试用) */
static volatile uint32_t g_packets_received  = 0;
static volatile uint32_t g_packets_bad_chk   = 0;
static volatile uint32_t g_packets_bad_frame = 0;
static volatile uint32_t g_packets_filtered  = 0;  /* 被形状过滤的帧数 */

/* ==========================================================================
 * 环形缓冲区操作
 * ========================================================================== */

static inline bool uart_rx_available(void)
{
    return (uart_rx_head != uart_rx_tail);
}

static uint8_t uart_rx_read(void)
{
    while (!uart_rx_available()) { __WFE(); }
    uint8_t byte = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUF_SIZE;
    return byte;
}

/* ==========================================================================
 * CanMV 数据包解析器
 * ========================================================================== */

/**
 * @brief   从环形缓冲区消费数据, 解析 CanMV 10 字节帧。
 *
 * 协议: [0x55] [0xAA] [code] [errX_H] [errX_L] [errY_H] [errY_L] [CHK] [CR] [LF]
 *
 * 状态机:
 *   PARSE_WAIT_H1 → PARSE_WAIT_H2 → PARSE_DATA (5B) → PARSE_WAIT_CR → PARSE_WAIT_LF
 *
 * 校验检查:
 *   - Byte 0 == 0x55, Byte 1 == 0xAA (帧头)
 *   - Byte 8 == 0x0D, Byte 9 == 0x0A (帧尾)
 *   - Byte 7 == SUM(Byte2..Byte6) & 0xFF (校验和)
 *
 * 形状过滤: 仅接受矩形 (code=3) 和正方形 (code=2)。
 */
static void parse_canmv_packets(void)
{
    static canmv_parse_state_t state = PARSE_WAIT_H1;
    static uint8_t  data_buf[CANMV_FRAME_SIZE];
    static uint8_t  data_idx = 0;

    while (uart_rx_available()) {
        uint8_t byte = uart_rx_read();

        switch (state) {

        case PARSE_WAIT_H1:
            if (byte == CANMV_FRAME_HEADER1) {
                data_buf[0] = byte;
                data_idx    = 1;
                state       = PARSE_WAIT_H2;
            }
            break;

        case PARSE_WAIT_H2:
            if (byte == CANMV_FRAME_HEADER2) {
                data_buf[1] = byte;
                data_idx    = 2;
                state       = PARSE_DATA;
            } else {
                /* 不是 0xAA → 重新同步 (可能前一帧损坏) */
                state = PARSE_WAIT_H1;
                if (byte == CANMV_FRAME_HEADER1) {
                    data_buf[0] = byte;
                    data_idx    = 1;
                    state       = PARSE_WAIT_H2;
                }
            }
            break;

        case PARSE_DATA:
            /* 收集 6 字节负载: code + errX_H + errX_L + errY_H + errY_L + chk */
            data_buf[data_idx] = byte;
            data_idx++;
            if (data_idx >= (CANMV_FRAME_SIZE - 2)) {
                /* 已收集 8 字节 (header×2 + payload×5 + chk×1),
                 * 剩余 2 字节 (CR + LF) 由后续状态处理 */
                state = PARSE_WAIT_CR;
            }
            break;

        case PARSE_WAIT_CR:
            if (byte == CANMV_FRAME_TAIL_CR) {
                data_buf[8] = byte;
                state = PARSE_WAIT_LF;
            } else {
                g_packets_bad_frame++;
                /* 重新同步 */
                state = PARSE_WAIT_H1;
                if (byte == CANMV_FRAME_HEADER1) {
                    data_buf[0] = byte;
                    data_idx    = 1;
                    state       = PARSE_WAIT_H2;
                }
            }
            break;

        case PARSE_WAIT_LF:
            if (byte == CANMV_FRAME_TAIL_LF) {
                data_buf[9] = byte;

                /* 帧完整 → 验证校验和 */
                uint8_t calc_chk = (uint8_t)(
                    data_buf[2] + data_buf[3] + data_buf[4]   /* code + errX */
                    + data_buf[5] + data_buf[6]              /* errY */
                );
                uint8_t recv_chk = data_buf[7];

                if (calc_chk == recv_chk) {
                    uint8_t  code = data_buf[2];
                    int16_t  ex   = (int16_t)(((uint16_t)data_buf[3] << 8)
                                             | (uint16_t)data_buf[4]);
                    int16_t  ey   = (int16_t)(((uint16_t)data_buf[5] << 8)
                                             | (uint16_t)data_buf[6]);

                    /*
                     * 形状过滤: 仅追踪矩形 (3) 和正方形 (2)。
                     * 三角形 (1) 和圆形 (4) 被丢弃。
                     */
                    if (!TRACK_ONLY_RECTANGLES
                        || code == SHAPE_RECTANGLE
                        || code == SHAPE_SQUARE)
                    {
                        g_shape_code  = code;
                        g_error_x     = ex;
                        g_error_y     = ey;
                        g_tracking_data_ready = true;
                        g_last_valid_packet_ms = sys_tick_ms;
                        g_packets_received++;
                    } else {
                        g_packets_filtered++;
                    }
                } else {
                    g_packets_bad_chk++;
                }
            } else {
                g_packets_bad_frame++;
                /*
                 * ★ 重新同步: 当前字节可能恰好是下一帧的 0x55 帧头。
                 *   其他错误分支 (PARSE_WAIT_H2, PARSE_WAIT_CR) 有相同处理,
                 *   PARSE_WAIT_LF 之前遗漏了此检查 → 每 ~1100 帧丢 1 帧。
                 */
                if (byte == CANMV_FRAME_HEADER1) {
                    data_buf[0] = byte;
                    data_idx    = 1;
                    state       = PARSE_WAIT_H2;
                    break;      /* 不执行下面的重置! */
                }
            }

            /* 重置状态机 (成功帧 / 非 0x55 失败帧) */
            state    = PARSE_WAIT_H1;
            data_idx = 0;
            break;

        } /* switch(state) */
    } /* while(uart_rx_available()) */
}

/* ==========================================================================
 * UART RX 中断服务例程
 * ========================================================================== */

/**
 * @brief   UART RX ISR — 将接收字节写入环形缓冲区。
 *
 * 使用 FIFO 模式排空所有可用字节 (MSPM0G3507 UART 默认带 FIFO)。
 * 缓冲区满时丢弃并标记溢出 (丢帧优于崩溃)。
 */
void cloud_uart_canmv_INST_IRQHandler(void)
{
    uint32_t status = DL_UART_getPendingInterrupt(TRACK_UART);

    if (status & DL_UART_INTERRUPT_RX_RXDATA) {
        DL_UART_clearInterruptStatus(TRACK_UART, DL_UART_INTERRUPT_RX_RXDATA);

        /* 排空 RX FIFO */
        while (!DL_UART_isRXFIFOEmpty(TRACK_UART)) {
            uint8_t byte = (uint8_t)DL_UART_receiveData(TRACK_UART);

            uint16_t next_head = (uart_rx_head + 1) % UART_RX_BUF_SIZE;
            if (next_head != uart_rx_tail) {
                uart_rx_buf[uart_rx_head] = byte;
                uart_rx_head = next_head;
            } else {
                uart_rx_overflow = true;
            }
        }
    }

    /* 错误标志: 清除并记录 (防止中断风暴) */
    if (status & (DL_UART_INTERRUPT_RX_FRAMING_ERROR
                | DL_UART_INTERRUPT_RX_PARITY_ERROR
                | DL_UART_INTERRUPT_RX_OVERRUN_ERROR)) {
        DL_UART_clearInterruptStatus(TRACK_UART,
            (DL_UART_INTERRUPT_RX_FRAMING_ERROR
           | DL_UART_INTERRUPT_RX_PARITY_ERROR
           | DL_UART_INTERRUPT_RX_OVERRUN_ERROR));
    }
}

/* ==========================================================================
 * UART 初始化
 * ========================================================================== */

static void tracking_uart_init(void)
{
    /* 清空所有挂起中断 */
    DL_UART_clearInterruptStatus(TRACK_UART,
        DL_UART_INTERRUPT_RX_RXDATA
      | DL_UART_INTERRUPT_RX_FRAMING_ERROR
      | DL_UART_INTERRUPT_RX_PARITY_ERROR
      | DL_UART_INTERRUPT_RX_OVERRUN_ERROR);

    /* 使能 RX 中断 */
    DL_UART_enableInterrupt(TRACK_UART, DL_UART_INTERRUPT_RX_RXDATA);

    /* 清零统计 (必须在 NVIC_EnableIRQ 之前, 防止 ISR 在清零前触发) */
    g_packets_received  = 0;
    g_packets_bad_chk   = 0;
    g_packets_bad_frame = 0;
    g_packets_filtered  = 0;
    uart_rx_overflow    = false;

    /* NVIC 使能 */
    NVIC_EnableIRQ(TRACK_UART_IRQN);
}

/* ==========================================================================
 * 主函数 — 50Hz 矩形追踪循环
 * ========================================================================== */

/*
 * 追踪循环间隔: 20ms = 50Hz。
 * 与 CanMV shape_detect.py 的实际帧率 (~30-50 FPS) 匹配。
 */
#define TRACK_LOOP_INTERVAL_MS      20

int main(void)
{
    /* ── 1. SysConfig 硬件初始化 ── */
    SYSCFG_DL_init();

    /* ── 2. 云台初始化 ── */
    gimbal_init();

    /* ── 3. SysTick 1ms ── */
    SysTick_Config(CPUCLK_FREQ / 1000U);

    /* ── 4. 零点标定 ── */
    gimbal_reset_zero();

    /*
     * ── 5. 设置追踪 RPM ──
     * 首次测试用低速 (60 RPM) 确认方向正确后再调高。
     * gimbal_track_velocity() 内部会自动调速 (0 ~ TRACK_MAX_RPM)。
     * 此处的 rpm 仅用于 gimbal_control_angle() 位置模式, 本程序不使用。
     */
    gimbal_pan.rpm  = 600;
    gimbal_tilt.rpm = 600;

    /* ── 6. UART 初始化 ── */
    tracking_uart_init();

    /* ── 7. 堵转检测初始化 ── */
    gimbal_stall_monitor_init();

    /* ── 8. 上电稳定等待 ── */
    delay_ms(100);

    /*
     * ── 9. ★ 50Hz 追踪主循环 ──
     *
     * 数据流:
     *   CanMV → UART ISR → ring buffer → parse_canmv_packets()
     *   → g_error_x/y → gimbal_track_velocity() → stepper_pulse_set_speed()
     *
     * 时序:
     *   - parse_canmv_packets(): 每次循环调用, 非阻塞消费 ring buffer
     *   - gimbal_track_velocity(): 每 20ms 调用一次 (50Hz)
     *   - 其余时间 CPU 休眠 (__WFE), 由 SysTick 或 UART RX 唤醒
     *
     * gimbal_track_velocity() 内部处理:
     *   超时检测 (200ms 无数据 → error=0) → 死区 → 非对称 EMA
     *   → P 控制 → RPM 变化率限制 → 原始误差定方向 → set_speed()
     */
    uint32_t last_loop_ms = millis();

    while (1) {
        uint32_t now = millis();

        /* ★ 持续消费 UART 缓冲 (非阻塞), 不放在 20ms 分支内 */
        parse_canmv_packets();

        /* 固定间隔追踪更新 (50Hz) */
        if ((now - last_loop_ms) >= TRACK_LOOP_INTERVAL_MS) {
            last_loop_ms = now;

            /*
             * 读取最新追踪数据 (lock-free, int16_t 在 Cortex-M0+ 上原子对齐读取)。
             *
             * g_tracking_data_ready 确保只在新数据到达时更新追踪引擎。
             * 无新数据时传 (0,0) 以保证超时检测持续运行:
             *   超过 GIMBAL_LOST_TIMEOUT_MS (200ms) → 引擎自动停止电机。
             */
            int16_t ex, ey;
            if (g_tracking_data_ready) {
                ex = g_error_x;
                ey = g_error_y;
                g_tracking_data_ready = false;
            } else {
                ex = 0;
                ey = 0;
            }

            /*
             * ★ 核心: 速度追踪
             *
             * 第三个参数 now 传递给 gimbal_track_velocity() 用于超时检测。
             * 超过 GIMBAL_LOST_TIMEOUT_MS 无数据 → 内部强制 error=0 → 电机平滑停止。
             *
             * 好处:
             *   - CanMV 断电/遮挡 → 云台自动停止 (无猝停)
             *   - CanMV 恢复 → 云台自动恢复追踪
             *   - 主循环无需任何额外逻辑
             */
            gimbal_track_velocity(ex, ey, now);
        }

        /* 低功耗等待 — SysTick(1ms) 或 UART RX 中断唤醒 */
        __WFE();
    }

    return 0;
}
