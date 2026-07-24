/*
 * vision_tracking.c — 视觉追踪模块实现
 *
 * 封装全部底层细节:
 *   - UART2 RX 中断 + 环形缓冲区
 *   - K230 $TARGET 协议解析 (canmv 2字段格式)
 *   - 追踪状态机 (INIT → SCANNING ↔ TRACKING)
 *   - EMA 平滑 + 角度突变保护 + 激光偏差补偿
 *   - HMC5883L 磁力计 PAN 轴扰动补偿 (heading-rate feed-forward)
 *   - 扫描运动看门狗 + 堵转检测
 *
 * 依赖: ti_msp_dl_config.h, drivers/gimbal_control.h, drivers/delay.h,
 *       app_imu_9dof.h
 */

#include "vision_tracking.h"
#include "ti_msp_dl_config.h"
#include "drivers/gimbal_control.h"
#include "drivers/delay.h"
#include "app_imu_9dof.h"
#include <math.h>
#include <string.h>

/* ==========================================================================
 * 1. SysTick 毫秒时钟 (delay.h 依赖)
 * ========================================================================== */

volatile uint32_t sys_tick_ms = 0;        /* CCS 中查看: 运行中应持续递增 */
volatile uint32_t g_systick_count = 0;    /* SysTick 触发总次数 (>0 = 正常) */

void SysTick_Handler(void) {
    sys_tick_ms++;
    g_systick_count++;
}

/* ==========================================================================
 * 2. 可调参数 — 见 vision_tracking.h 第一节
 * ========================================================================== */

/* ==========================================================================
 * 3. 激光-摄像头偏差补偿 (volatile: 调试器在线调参)
 * ==========================================================================
 *
 * 追踪公式: target = current + (EMA偏移 + offset) × gain
 * ★ offset 乘以 TRACK_GAIN 后生效, 修改 GAIN 需重新标定 offset。
 */
static volatile float g_pan_offset  = 0.0f;
static volatile float g_tilt_offset = 0.0f;

/* ==========================================================================
 * 3b. HMC5883L 磁力计 PAN 轴增稳 (heading-rate feed-forward)
 * ==========================================================================
 *
 * HMC5883L 安装在云台上, 测量摄像头当前绝对磁航向。
 * 每 20ms 周期计算 heading 变化率, 减去自行命令的 PAN 位移,
 * 剩余部分 = 外部扰动 (风吹/底盘旋转), 直接前馈补偿到 PAN 目标。
 *
 * 公式:
 *   heading_delta = hdg_now - hdg_prev        (总航向变化)
 *   pan_delta     = pan_now - pan_prev        (自行命令的位移)
 *   disturbance   = heading_delta - pan_delta (外部扰动)
 *   pan_ff        = -disturbance × KP_HEADING (前馈修正)
 *
 * KP_HEADING: 前馈增益。=1.0 全量抵消扰动, <1.0 保留部分给视觉闭环。
 */

static bool     s_imu_ok = false;        /* 9-DOF IMU 可用 */
static float    s_prev_yaw   = 0.0f;     /* 上周期 yaw (°) */
static float    s_prev_pitch = 0.0f;     /* 上周期 pitch (°) */
static float    s_prev_pan   = 0.0f;     /* 上周期 PAN 步进角 (°) */
static float    s_prev_tilt  = 0.0f;     /* 上周期 TILT 步进角 (°) */
static bool     s_imu_first  = true;     /* 首次采样跳过差分 */
static IMU_9DOF_Data_t s_imu_data;       /* 最新 IMU 融合数据 */

/* NaN/Inf 快速检测 (多个函数共用, 必须在所有调用者之前定义) */
static inline bool bad_float(float x) { return x != x || x > 1e10f || x < -1e10f; }

/** @brief 计算 IMU 双轴前馈修正量, 返回 pan_ff, *tilt_ff */
static float imu_compute_feedforward(float cur_pan, float cur_tilt, float *tilt_ff)
{
    *tilt_ff = 0.0f;
    if (!s_imu_ok) return 0.0f;
    /* ★ IMU 传感器数据 NaN 纵深防御: 陀螺/磁力计偶发 glitch 产生 NaN → 前馈变成 NaN
     *   → 下游 target NaN 守卫虽拦截但 s_pan_ff 静态变量残留 NaN → 所有 TRACK 帧被拒。
     *   在入口处拦截, 防止 NaN 进入计算链。 */
    if (bad_float(s_imu_data.yaw) || bad_float(s_imu_data.pitch)) return 0.0f;
    if (s_imu_first) {
        s_prev_yaw   = s_imu_data.yaw;
        s_prev_pitch = s_imu_data.pitch;
        s_prev_pan   = cur_pan;
        s_prev_tilt  = cur_tilt;
        s_imu_first  = false;
        return 0.0f;
    }

    /* ── PAN 轴: yaw 变化 - 自行位移 = 扰动 → 前馈 ──
     * ★ yaw 在 [0,360), 必须 wrap 到 ±180° 防 0/360 环绕产生 ~360° 虚假前馈 */
    float yaw_delta = s_imu_data.yaw - s_prev_yaw;
    while (yaw_delta > 180.0f) yaw_delta -= 360.0f;
    while (yaw_delta < -180.0f) yaw_delta += 360.0f;
    float pan_delta = cur_pan - s_prev_pan;
    while (pan_delta > 180.0f) pan_delta -= 360.0f;
    while (pan_delta < -180.0f) pan_delta += 360.0f;
    float pan_ff = -(yaw_delta - pan_delta) * KP_IMU_PAN;

    /* ── TILT 轴: pitch 变化 - 自行位移 = 扰动 → 前馈 ──
     * ★ v9 修复: IMU pitch 正=相机↑, TILT 电机角度正=相机↓, 两者符号相反。
     *   因此用 + (而非 -) 来抵消电机自身运动: pitch_delta≈-tilt_delta→和≈0。
     *   风推相机↑ (pitch>0) → tilt_ff>0 → tt 增大 → TILT 向下对抗扰动。 */
    float pitch_delta = s_imu_data.pitch - s_prev_pitch;
    float tilt_delta  = cur_tilt - s_prev_tilt;
    *tilt_ff = (pitch_delta + tilt_delta) * KP_IMU_TILT;

    s_prev_yaw   = s_imu_data.yaw;
    s_prev_pitch = s_imu_data.pitch;
    s_prev_pan   = cur_pan;
    s_prev_tilt  = cur_tilt;
    return pan_ff;
}

/* ==========================================================================
 * 4. UART2 环形缓冲区 (中断驱动)
 * ========================================================================== */

#define UART_RX_BUF_SIZE         2048
#define MAX_LINE_LEN             64

static volatile uint8_t  g_rx_buf[UART_RX_BUF_SIZE];

/* ★ 以下变量非 static — 可在 CCS Expressions 窗口直接查看 */
volatile uint16_t g_rx_head = 0;          /* ISR 写, 主循环读 — 非零=ISR 触发过 */
volatile uint16_t g_rx_tail = 0;          /* 主循环写, ISR 读 */
volatile uint32_t g_uart_isr_rx_count = 0; /* ISR RX 路径触发次数 (>0 = 收到数据!) */
volatile uint32_t g_uart_isr_err_count = 0;/* Framing/Parity/Overrun 错误次数 */
volatile uint32_t g_rx_total_bytes  = 0;   /* ISR 累计接收字节总数 (>0 = 物理连接正常) */
volatile bool     g_rx_overflow = false;

static char g_line_buf[MAX_LINE_LEN];

static inline bool rx_available(void) { return g_rx_head != g_rx_tail; }

static uint8_t rx_read(void) {
    while (!rx_available()) {
        /* 忙等 — 不用 __WFE(), 避免 Cortex-M0+ 事件寄存器竞态导致睡死 */
    }
    uint8_t b = g_rx_buf[g_rx_tail];
    g_rx_tail = (g_rx_tail + 1) % UART_RX_BUF_SIZE;
    return b;
}

/*
 * UART2 RX ISR — MSPM0 UART Main 电平触发中断
 *
 * ★ 关键: MSPM0 UART Main 的 RX 中断是**电平触发** — FIFO 里有数据就自动
 *   pending, 读空就自动清除。不需要 getPendingInterrupt/clearInterruptStatus,
 *   加这些反而可能因时序问题跳过读 FIFO → 数据永远收不到。
 *
 * 正确写法: 循环读 FIFO 直到为空, 加 loop<64 守卫防死循环。
 */
void UART2_IRQHandler(void)
{
    uint8_t loop = 0;
    while (!DL_UART_Main_isRXFIFOEmpty(cloud_uart_vision_INST) && loop < 64) {
        uint8_t b = (uint8_t)DL_UART_Main_receiveData(cloud_uart_vision_INST);
        uint16_t next = (g_rx_head + 1) % UART_RX_BUF_SIZE;
        if (next != g_rx_tail) { g_rx_buf[g_rx_head] = b; g_rx_head = next; }
        else { g_rx_overflow = true; }
        g_rx_total_bytes++;
        loop++;
    }
    g_uart_isr_rx_count++;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 协议解析结果 (文件作用域, 在 uart_rx_flush / parse_packets / state_machine
 * 之间共享。非 static — CCS Expressions 窗口可直接查看 g_parsed)
 * ═══════════════════════════════════════════════════════════════════════════ */
struct {
    float    h_deg, v_deg;
    int      circles;
    bool     updated, target_lost;
    bool     has_angle;           /* ★ 独立标志: 仅有效角度帧置 true, NONE 帧绝不覆盖 */
    uint32_t last_packet_ms;
    uint32_t cnt_ok, cnt_none, cnt_bad, cnt_overflow;
} g_parsed = {0};

/* 前置声明 (定义在 parse_packets() 上方) */
static void parse_state_reset(void);

/** @brief 冲洗环形缓冲区: 排空硬件FIFO + 复位读写指针 + 清溢出标志
 *
 * 调用时机:
 *   1. Vision_Init() 末尾 — 丢弃 IMU 校准期间累积的陈旧数据
 *   2. 调试时手动调用 — CCS Expressions 中执行即可重置通信状态
 *
 * 安全: 冲洗期间临时关闭 NVIC, 防止 ISR 并发写入。
 */
static void uart_rx_flush(void)
{
    /* 1. 暂停 UART RX 中断 */
    NVIC_DisableIRQ(cloud_uart_vision_INST_INT_IRQN);

    /* 2. 排空 UART 硬件 RX FIFO (丢弃积压的陈旧字节) */
    while (!DL_UART_Main_isRXFIFOEmpty(cloud_uart_vision_INST)) {
        (void)DL_UART_Main_receiveData(cloud_uart_vision_INST);
    }

    /* 3. 清除所有挂起的 UART 中断标志 */
    DL_UART_Main_clearInterruptStatus(cloud_uart_vision_INST,
        DL_UART_MAIN_INTERRUPT_RX
      | DL_UART_MAIN_INTERRUPT_FRAMING_ERROR
      | DL_UART_MAIN_INTERRUPT_PARITY_ERROR
      | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);

    /* 4. 复位环形缓冲区读写指针 + 溢出标志 */
    g_rx_head     = 0;
    g_rx_tail     = 0;
    g_rx_overflow = false;

    /* 5. 复位解析器状态机 (防止中间状态残留) */
    parse_state_reset();

    /* 6. 清零解析结果 (has_angle/updated/target_lost 等全部复位) */
    g_parsed.has_angle   = false;
    g_parsed.updated      = false;
    g_parsed.target_lost  = false;
    g_parsed.last_packet_ms = 0;
    /* 注: 统计计数器 (cnt_ok/cnt_none/cnt_bad/cnt_overflow) 不清零,
     *     保留用于上电后的整体诊断 */

    /* 7. 恢复 UART RX 中断 — 此后收到的数据为"干净"帧 */
    NVIC_EnableIRQ(cloud_uart_vision_INST_INT_IRQN);
}

static void uart_init(void)
{
    DL_UART_Main_clearInterruptStatus(cloud_uart_vision_INST,
        DL_UART_MAIN_INTERRUPT_RX
      | DL_UART_MAIN_INTERRUPT_FRAMING_ERROR
      | DL_UART_MAIN_INTERRUPT_PARITY_ERROR
      | DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
    DL_UART_Main_enableInterrupt(cloud_uart_vision_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(cloud_uart_vision_INST_INT_IRQN);
}

/* ==========================================================================
 * 5. ASCII 浮点/整数解析 (轻量级, 避免 sscanf)
 * ========================================================================== */

static bool parse_float(const char *str, float *val)
{
    if (!str || !val || !*str) return false;
    float r = 0.0f, sign = 1.0f, dec = 0.0f, div = 1.0f;
    bool ok = false, in_dec = false;
    const char *p = str;
    if (*p == '-') { sign = -1.0f; p++; } else if (*p == '+') { p++; }
    while (*p && *p != ',' && *p != '\r' && *p != '\n') {
        if (*p == '.') { if (in_dec) return false; in_dec = true; p++; continue; }
        if (*p < '0' || *p > '9') return false;
        ok = true;
        if (in_dec) { div *= 10.0f; dec = dec * 10.0f + (float)(*p - '0'); }
        else        { r = r * 10.0f + (float)(*p - '0'); }
        p++;
    }
    if (!ok) return false;
    *val = sign * (r + dec / div);
    return (*val == *val && *val < 1e10f && *val > -1e10f);  /* NaN/Inf 拒绝 */
}

/** @brief 轻量级整数解析 (用于同心圆数 n), 与 parse_float 相同的终止规则 */
static bool parse_int(const char *str, int *val)
{
    if (!str || !val || !*str) return false;
    int r = 0, sign = 1;
    bool ok = false;
    const char *p = str;
    if (*p == '-') { sign = -1; p++; } else if (*p == '+') { p++; }
    while (*p && *p != ',' && *p != '\r' && *p != '\n') {
        if (*p < '0' || *p > '9') return false;
        ok = true;
        r = r * 10 + (int)(*p - '0');
        p++;
    }
    if (!ok) return false;
    *val = sign * r;
    return true;
}

/* ==========================================================================
 * 6. K230 $TARGET 协议解析状态机
 * ========================================================================== */

typedef enum { PS_WAIT_DOLLAR, PS_MATCH_TARGET, PS_READ_LINE, PS_WAIT_LF } ParseState_t;

/* ── 解析器内部状态 (文件作用域, 允许外部复位) ── */
static ParseState_t s_parse_state = PS_WAIT_DOLLAR;
static uint8_t      s_parse_midx  = 0;
static uint8_t      s_parse_lidx  = 0;
static bool         s_parse_overflow_line = false;

/** @brief 复位解析器内部状态 (环形缓冲区冲洗后调用) */
static void parse_state_reset(void)
{
    s_parse_state         = PS_WAIT_DOLLAR;
    s_parse_midx          = 0;
    s_parse_lidx          = 0;
    s_parse_overflow_line = false;
}

static void parse_packets(void)
{
    /* 同步溢出统计 — 先读后清再递增, 减少 ISR 竞态窗口 */
    if (g_rx_overflow) {
        g_rx_overflow = false;
        g_parsed.cnt_overflow++;
    }

    while (rx_available()) {
        uint8_t b = rx_read();
        switch (s_parse_state) {

        case PS_WAIT_DOLLAR:
            if (b == '$') { s_parse_midx = 0; s_parse_lidx = 0; s_parse_overflow_line = false; s_parse_state = PS_MATCH_TARGET; }
            break;

        case PS_MATCH_TARGET: {
            static const char hdr[] = "TARGET,";
            if (b == (uint8_t)hdr[s_parse_midx]) {
                if (++s_parse_midx >= 7) { s_parse_state = PS_READ_LINE; s_parse_lidx = 0; s_parse_overflow_line = false; }
            } else if (b == '\r' || b == '\n') {
                s_parse_state = PS_WAIT_DOLLAR; g_parsed.cnt_bad++;
            } else if (b == '$') {
                s_parse_midx = 0; s_parse_lidx = 0; s_parse_overflow_line = false;
            } else {
                s_parse_state = PS_WAIT_DOLLAR; g_parsed.cnt_bad++;
            }
            break;
        }

        case PS_READ_LINE:
            if (b == '\r') {
                g_line_buf[s_parse_lidx] = '\0';
                /* NONE 检测 */
                if (s_parse_lidx == 4 && g_line_buf[0]=='N' && g_line_buf[1]=='O'
                    && g_line_buf[2]=='N' && g_line_buf[3]=='E') {
                    g_parsed.target_lost = true;
                    g_parsed.last_packet_ms = sys_tick_ms;
                    g_parsed.cnt_none++;
                } else if (!s_parse_overflow_line && s_parse_lidx > 0) {
                    /* 协议: 自动兼容 2字段(canmv暗色斑块) / 3字段(红色同心圆)
                     *   $TARGET,h,v\r\n     → circles=0
                     *   $TARGET,h,v,n\r\n   → circles=n
                     * 用逗号数量自动鉴别, 不再拒绝3字段帧 */
                    char *c1 = NULL, *c2 = NULL;
                    for (uint8_t i = 0; i < s_parse_lidx; i++) {
                        if (g_line_buf[i] == ',') {
                            if (!c1)      { g_line_buf[i] = '\0'; c1 = &g_line_buf[i]; }
                            else if (!c2) { g_line_buf[i] = '\0'; c2 = &g_line_buf[i]; }
                            else break;   /* 超过2个逗号 → 格式错误, c1/c2 已记录 */
                        }
                    }
                    if (c1) {
                        char *fh = g_line_buf;          /* h_deg 字段 */
                        char *fv = c1 + 1;              /* v_deg 字段 (可能含 ,n)  */
                        int circles = 0;

                        if (c2) {
                            /* 3字段格式: $TARGET,h,v,n\r\n */
                            char *fn = c2 + 1;          /* n 字段 */
                            if (!parse_int(fn, &circles)) {
                                /* n 解析失败不拒绝整帧, circles=0 降级 */
                                circles = 0;
                            }
                        }

                        float h, v;
                        if (parse_float(fh, &h) && parse_float(fv, &v)) {
                            float ah = (h < 0) ? -h : h, av = (v < 0) ? -v : v;
                            if (ah <= 45.0f && av <= 45.0f) {
                                g_parsed.h_deg = h; g_parsed.v_deg = v;
                                g_parsed.circles = circles;
                                g_parsed.updated = true; g_parsed.target_lost = false;
                                g_parsed.has_angle = true;  /* ★ NONE 帧绝不能覆盖此标志 */
                                g_parsed.last_packet_ms = sys_tick_ms; g_parsed.cnt_ok++;
                            } else g_parsed.cnt_bad++;
                        } else g_parsed.cnt_bad++;
                    } else g_parsed.cnt_bad++;
                }
                s_parse_state = PS_WAIT_LF;
            } else if (b == '\n') {
                /* LF without CR: 对 K230 虽非标准输出, 但部分终端/USB-UART
                 * 可能转换 \r\n→\n, 完整解析避免丢帧 */
                g_line_buf[s_parse_lidx] = '\0';
                if (s_parse_lidx == 4 && g_line_buf[0]=='N' && g_line_buf[1]=='O'
                    && g_line_buf[2]=='N' && g_line_buf[3]=='E') {
                    g_parsed.target_lost = true;
                    g_parsed.last_packet_ms = sys_tick_ms;
                    g_parsed.cnt_none++;
                } else if (!s_parse_overflow_line && s_parse_lidx > 0) {
                    /* 与 \r 分支相同的数据帧解析逻辑 */
                    char *c1 = NULL, *c2 = NULL;
                    for (uint8_t i = 0; i < s_parse_lidx; i++) {
                        if (g_line_buf[i] == ',') {
                            if (!c1)      { g_line_buf[i] = '\0'; c1 = &g_line_buf[i]; }
                            else if (!c2) { g_line_buf[i] = '\0'; c2 = &g_line_buf[i]; }
                            else break;
                        }
                    }
                    if (c1) {
                        char *fh = g_line_buf;
                        char *fv = c1 + 1;
                        int circles = 0;
                        if (c2) {
                            char *fn = c2 + 1;
                            if (!parse_int(fn, &circles)) { circles = 0; }
                        }
                        float h, v;
                        if (parse_float(fh, &h) && parse_float(fv, &v)) {
                            float ah = (h < 0) ? -h : h, av = (v < 0) ? -v : v;
                            if (ah <= 45.0f && av <= 45.0f) {
                                g_parsed.h_deg = h; g_parsed.v_deg = v;
                                g_parsed.circles = circles;
                                g_parsed.updated = true; g_parsed.target_lost = false;
                                g_parsed.has_angle = true;
                                g_parsed.last_packet_ms = sys_tick_ms; g_parsed.cnt_ok++;
                            } else g_parsed.cnt_bad++;
                        } else g_parsed.cnt_bad++;
                    } else g_parsed.cnt_bad++;
                }
                s_parse_state = PS_WAIT_DOLLAR; s_parse_lidx = 0; s_parse_overflow_line = false;
            } else {
                if (b == '$') {
                    s_parse_state = PS_MATCH_TARGET; s_parse_midx = 0; s_parse_lidx = 0; s_parse_overflow_line = false; g_parsed.cnt_bad++;
                } else if (!s_parse_overflow_line) {
                    if (s_parse_lidx < MAX_LINE_LEN - 1) g_line_buf[s_parse_lidx++] = (char)b;
                    else { s_parse_overflow_line = true; g_parsed.cnt_bad++; }
                }
            }
            break;

        case PS_WAIT_LF:
            if (b == '\n') { s_parse_state = PS_WAIT_DOLLAR; s_parse_lidx = 0; }
            else if (b == '$') { s_parse_midx = 0; s_parse_lidx = 0; s_parse_overflow_line = false; s_parse_state = PS_MATCH_TARGET; }
            else if (b != '\r') {
                s_parse_state = PS_WAIT_DOLLAR; s_parse_lidx = 0; g_parsed.cnt_bad++;
            }
            break;
        }
    }
}

/* ==========================================================================
 * 7. 追踪状态机
 * ========================================================================== */

typedef enum { ST_INIT, ST_SCAN, ST_TRACK } Stage_t;

static Stage_t  s_stage = ST_INIT;
static uint32_t s_stage_start = 0;
static int8_t   s_scan_dir = 1;
static float    s_ema_h = 0.0f, s_ema_v = 0.0f;
static bool     s_ema_ok = false;
static uint32_t s_none_start = 0;
static bool     s_in_none = false;
static float    s_target_pan = 0.0f, s_target_tilt = 0.0f;
static float    s_pan_ff = 0.0f;         /* 最近一次磁力计前馈量 */
static uint32_t s_stall_resets = 0;      /* UART 解析器自动恢复次数 */
static uint32_t s_scanner_stall_start = 0; /* SCAN 模式停滞计时起点 */
static uint8_t  s_settle_count = 0;      /* ★ v10: 近靶锁定连续确认计数 */

static void state_machine(void)
{
    uint32_t now = sys_tick_ms;

    switch (s_stage) {

    case ST_INIT:
        if (now - s_stage_start >= K230_BOOT_WAIT_MS) {
            s_stage = ST_SCAN; s_stage_start = now; s_scan_dir = 1;
            s_settle_count = 0;           /* ★ v10: 重置锁定计数 */
            g_parsed.updated = false; g_parsed.target_lost = false;
            g_parsed.has_angle = false;   /* ★ 防御: 清除 INIT 期间可能积累的残留标志 */
            g_parsed.last_packet_ms = 0;
        }
        break;

    case ST_SCAN:
        /*
         * ★ v8: UART 解析器停滞自动恢复 (看门狗)
         *
         * 场景: K230 启动期间 TX 引脚浮空/抖动 → 噪声字节进入环形缓冲区
         * → 解析器状态机可能长期处于 PS_WAIT_DOLLAR/PS_MATCH_TARGET 之间,
         *   所有字节被 `cnt_bad++` 丢弃 → g_parsed.last_packet_ms 永不更新。
         *
         * 检测: SCAN 模式下若 last_packet_ms 长时间不更新 (没有成功识别
         *       任何有效帧或 NONE 帧), 说明通信已中断 → 自动冲洗恢复。
         *       UART_STALL_RECOVERY_MS=0 时禁用 (仅依赖 $ 同步字符自愈)。
         *
         * ★ v8.1 修正: last_packet_ms==0 时 (从未收到任何数据), 不能用
         *   `now - s_scanner_stall_start` 判断 (之前每 tick 重置为 now → 超时永不触发)。
         *   正确逻辑:
         *     - last_packet_ms==0 且 s_scanner_stall_start==0 → 记录起始时间
         *     - last_packet_ms==0 且已超时 → 冲洗
         *     - last_packet_ms>0 且已超时 → 冲洗
         */
        if (UART_STALL_RECOVERY_MS > 0) {
            if (g_parsed.last_packet_ms == 0) {
                /* 从未收到任何帧: 从首次进入该分支开始计时 */
                if (s_scanner_stall_start == 0) {
                    s_scanner_stall_start = now;
                } else if (now - s_scanner_stall_start >= UART_STALL_RECOVERY_MS) {
                    uart_rx_flush();
                    s_stall_resets++;
                    s_scanner_stall_start = now;
                    g_parsed.last_packet_ms = now;  /* 防下一 tick 误判 */
                }
            } else if (now - g_parsed.last_packet_ms >= UART_STALL_RECOVERY_MS) {
                /* 曾收到过帧, 但现在停滞了 */
                uart_rx_flush();
                s_stall_resets++;
                s_scanner_stall_start = now;
                g_parsed.last_packet_ms = now;
            }
        }

        /* ★ 使用 has_angle 而非 updated && !target_lost:
         *    has_angle 仅由有效 $TARGET,h,v 帧置位, NONE 帧绝不覆盖,
         *    解决同批次 NONE 帧把 target_lost 覆盖而导致永远进不了 TRACK 的 bug */
        if (g_parsed.has_angle) {
            if (bad_float(g_parsed.h_deg) || bad_float(g_parsed.v_deg)) { g_parsed.updated = false; g_parsed.has_angle = false; break; }
            gimbal_stop();  /* 先停止扫描运动, 再下发追踪指令 */

            /*
             * ★ v7 修复: SCAN→TRACK 过渡时**立即下发首次追踪指令**。
             *   bug: 之前只初始化 EMA 但从未调用 gimbal_control_angle(),
             *        依赖下一帧到来才首次驱动电机 → 死循环无法追踪。
             *   fix: 用触发过渡的这一帧的角度, 立即计算目标并下发运动指令。
             *
             * ★ v7.1 修复: 仅当追踪指令成功下发后才过渡到 TRACK。
             *   若当前角度传感器返回 NaN → 无法计算目标 → 留在 SCAN 重试,
             *   避免进入 TRACK 后静坐不动直到超时退回的死循环。 */
            {
                float cp = gimbal_get_current_angle_pan();
                float ct = gimbal_get_current_angle_tilt();
                if (!bad_float(cp) && !bad_float(ct)) {
                    s_ema_h = g_parsed.h_deg; s_ema_v = g_parsed.v_deg; s_ema_ok = true;
                    s_in_none = false;

                    /* ★ v7.1: s_imu_first 必须在 imu_compute_feedforward 之前设置,
                     *   否则 TRACK→SCAN→TRACK 重入时用上次 SCAN 期间累积的过期
                     *   prev_yaw/prev_pan 计算前馈 → 产生虚假的角度跳变。 */
                    s_imu_first = true;

                    float tilt_ff = 0.0f;
                    s_pan_ff = imu_compute_feedforward(cp, ct, &tilt_ff);
                    float tp = cp + PAN_DIR_SIGN  * (s_ema_h + g_pan_offset)  * TRACK_GAIN + s_pan_ff;
                    float tt = ct + TILT_DIR_SIGN * (s_ema_v + g_tilt_offset) * TRACK_GAIN + tilt_ff;
                    if (!bad_float(tp) && !bad_float(tt)) {
                        s_target_pan = tp; s_target_tilt = tt;
                        gimbal_control_angle(tp, tt);
                        s_settle_count = 0;           /* ★ v10: 进入 TRACK 时重置锁定计数 */

                        /* ★ 仅追踪指令成功下发后才过渡到 TRACK */
                        s_stage = ST_TRACK; s_stage_start = now;
                        g_parsed.last_packet_ms = now;
                        g_parsed.updated = false;
                        g_parsed.has_angle = false;
                        g_parsed.target_lost = false;  /* ★ v7.1: 防残留 NONE 标志泄漏到 TRACK */
                        break;
                    }
                    /* tp/tt NaN: 极罕见 (IMU前馈异常), 留在 SCAN 等待下一帧 */
                    s_ema_ok = false;
                }
                /* cp/ct NaN: 传感器 I2C 暂时故障, 留在 SCAN, 等待恢复 */
            }

            /* 无法下发追踪指令 → 清除消费标志, 留在 SCAN, 下一轮继续扫描/重试 */
            g_parsed.updated = false;
            g_parsed.has_angle = false;
            g_parsed.target_lost = false;
            break;
        }
        if (!gimbal_is_moving()) {
            float cp = gimbal_get_current_angle_pan();
            if (cp >= SCAN_PAN_RANGE - SCAN_BOUNDARY_MARGIN) s_scan_dir = -1;
            else if (cp <= -SCAN_PAN_RANGE + SCAN_BOUNDARY_MARGIN) s_scan_dir = 1;
            float st = (s_scan_dir > 0) ? SCAN_PAN_RANGE : -SCAN_PAN_RANGE;
            /* SCAN_RPM=0 → 跳过扫描运动 (静默HOLD, 等靶子出现) */
            if (SCAN_RPM > 0) {
                gimbal_pan.rpm  = (uint16_t)SCAN_RPM;
                gimbal_tilt.rpm = (uint16_t)SCAN_RPM;
                gimbal_control_angle(st, SCAN_TILT_ANGLE);
                gimbal_pan.rpm  = TRACK_RPM;
                gimbal_tilt.rpm = TRACK_RPM;
            }
        }
        break;

    case ST_TRACK:
        /*
         * ★ v7 重构: 有效数据帧优先于 NONE 帧处理。
         *
         * bug: 之前 target_lost 分支在 updated 分支之前, 若同批次环形缓冲区
         *      中既有有效帧又有 NONE 帧, target_lost 先命中 → 清除 updated
         *      并 break → 有效帧的追踪数据被丢弃 → 追踪中断/抖回 SCAN。
         *
         * fix: updated 优先处理 (有效数据代表目标在视野中), target_lost 独立分支
         *      (仅在没有有效帧时计算 NONE 超时), 超时检测最后兜底。
         */
        if (g_parsed.updated) {
            s_in_none = false;

            /* EMA 初始化或更新 */
            if (!s_ema_ok) {
                if (bad_float(g_parsed.h_deg) || bad_float(g_parsed.v_deg)) { g_parsed.updated = false; break; }
                s_ema_h = g_parsed.h_deg; s_ema_v = g_parsed.v_deg; s_ema_ok = true;
            } else {
                /* ★ NaN 守卫 (防御纵深) */
                if (bad_float(g_parsed.h_deg) || bad_float(g_parsed.v_deg)) { g_parsed.updated = false; break; }

                /*
                 * ★ 突变保护: 先检查原始值 vs 旧EMA的跳变, 再决定是否更新EMA
                 *
                 * v6 修正: 之前 EMA 先更新再检查跳变 → 尖峰帧虽被拒绝但已污染 EMA。
                 *         现在改为: 先用旧 EMA 作基准检查 delta → 通过后才更新。
                 *         公式: |raw - old_ema| > ANGLE_JUMP_LIMIT → 拒绝, EMA不变。
                 */
                float dh = g_parsed.h_deg - s_ema_h;   /* raw vs OLD EMA (未更新!) */
                float dv = g_parsed.v_deg - s_ema_v;
                float ah = (dh < 0) ? -dh : dh;
                float av = (dv < 0) ? -dv : dv;
                if (ah > ANGLE_JUMP_LIMIT || av > ANGLE_JUMP_LIMIT) {
                    g_parsed.updated = false; break;    /* EMA 未被污染! */
                }

                /* 跳变检查通过 → 安全更新 EMA */
                s_ema_h = TRACK_EMA_ALPHA * g_parsed.h_deg + (1.0f - TRACK_EMA_ALPHA) * s_ema_h;
                s_ema_v = TRACK_EMA_ALPHA * g_parsed.v_deg + (1.0f - TRACK_EMA_ALPHA) * s_ema_v;
            }

            /* EMA NaN 守卫 (纵深防御 — 如果 EMA 因任何原因变为 NaN, 立即复位) */
            if (bad_float(s_ema_h) || bad_float(s_ema_v)) { s_ema_h = 0; s_ema_v = 0; s_ema_ok = false; g_parsed.updated = false; break; }

            /* 读取当前角度 + NaN 守卫 */
            float cp = gimbal_get_current_angle_pan();
            float ct = gimbal_get_current_angle_tilt();
            if (bad_float(cp) || bad_float(ct)) { g_parsed.updated = false; break; }

            /* IMU 双轴前馈: 抵消外部扰动 (PAN=yaw, TILT=pitch) */
            float tilt_ff = 0.0f;
            s_pan_ff = imu_compute_feedforward(cp, ct, &tilt_ff);

            /* 计算目标:
             *   视觉项: DIR_SIGN × (EMA + offset) × GAIN  (K230方向可反转)
             *   IMU项:  s_pan_ff / tilt_ff  已是物理坐标系, 不乘DIR_SIGN */
            float tp = cp + PAN_DIR_SIGN  * (s_ema_h + g_pan_offset)  * TRACK_GAIN + s_pan_ff;
            float tt = ct + TILT_DIR_SIGN * (s_ema_v + g_tilt_offset) * TRACK_GAIN + tilt_ff;
            if (bad_float(tp) || bad_float(tt)) { g_parsed.updated = false; break; }

            s_target_pan = tp; s_target_tilt = tt;

            /*
             * ★ v10: 近靶锁定 — 消除过冲振荡后的微抖
             *
             * 当两轴 EMA 误差都在阈值内、且电机已静止时,
             * 递增确认计数。连续 TRACK_SETTLE_CYCLES 个周期满足条件后,
             * 跳过 gimbal_control_angle() → 云台保持当前角度不动。
             *
             * EMA 持续更新: 靶子一旦移动 → |ema| 超阈值 → 立即清零
             * settle计数 → 恢复运动命令 → 重新追踪。
             *
             * 重要: 不用 break 跳出 case, 用 bool 标志跳过 move 调用,
             *       确保下面的 target_lost / 超时检测仍正常运行。
             */
            {
                bool do_move = true;
                float ah = (s_ema_h < 0) ? -s_ema_h : s_ema_h;
                float av = (s_ema_v < 0) ? -s_ema_v : s_ema_v;
                if (ah < TRACK_SETTLE_THRESHOLD && av < TRACK_SETTLE_THRESHOLD
                    && !gimbal_is_moving()) {
                    if (s_settle_count < TRACK_SETTLE_CYCLES) {
                        s_settle_count++;
                    }
                    if (s_settle_count >= TRACK_SETTLE_CYCLES) {
                        do_move = false;  /* 锁定: 不下发运动命令 */
                    }
                } else {
                    s_settle_count = 0;
                }
                if (do_move) {
                    gimbal_control_angle(tp, tt);
                }
            }
            g_parsed.updated = false;
        }

        /* ★ NONE 帧: 目标短暂丢失 (独立于 updated 分支) */
        if (g_parsed.target_lost) {
            if (!s_in_none) { s_none_start = now; s_in_none = true; }
            if (now - s_none_start >= TARGET_LOST_TIMEOUT_MS) {
                s_ema_ok = false; s_in_none = false; g_parsed.target_lost = false;
                s_settle_count = 0;       /* ★ v10.1: 退回 SCAN 时重置锁定 */
                g_parsed.has_angle = false;  /* ★ 退回 SCAN 时清除, 等新有效帧到来 */
                s_stage = ST_SCAN; s_stage_start = now; gimbal_stop();
                break;
            }
            g_parsed.updated = false;
            break;
        }

        /* ★ 无任何帧到达: 超时检测 (仅 updated==false && target_lost==false 时运行) */
        if (g_parsed.last_packet_ms > 0 &&
            now - g_parsed.last_packet_ms >= TARGET_LOST_TIMEOUT_MS) {
            s_ema_ok = false; s_in_none = false; g_parsed.target_lost = false;
            s_settle_count = 0;       /* ★ v10.1: 退回 SCAN 时重置锁定 */
            g_parsed.has_angle = false;  /* ★ 退回 SCAN 时清除 */
            s_stage = ST_SCAN; s_stage_start = now; gimbal_stop();
        }
        break;
    }
}

/* ==========================================================================
 * 8. 公开 API 实现
 * ========================================================================== */

void Vision_Init(void)
{
    /* 硬件 */
    gimbal_init();
    gimbal_reset_zero();
    gimbal_stall_monitor_init();
    gimbal_pan.rpm  = TRACK_RPM;
    gimbal_tilt.rpm = TRACK_RPM;

    /* UART — 尽早使能, 但后续会冲洗掉 IMU 校准期间积压的陈旧数据 */
    uart_init();

    /* 9-DOF IMU (MPU6050 + HMC5883L, 失败不影响追踪)
     * IMU_9DOF_Init 内部: MID_MPU6050_Init(配I2C1速率) + 自动鉴别磁力计 */
    s_imu_ok = IMU_9DOF_Init();
    if (s_imu_ok) {
        IMU_9DOF_CalibrateGyro(500);  /* 陀螺零偏校准 ~2.5s @200Hz, 需静止 */
    }
    s_imu_first = true;

    /*
     * ★ 冲洗 UART 环形缓冲区 + 复位解析器状态机
     *
     * 问题: IMU_9DOF_CalibrateGyro(500) 阻塞约 2.5s, 期间 K230 持续发送帧,
     *       2048 字节环形缓冲区可能溢出 → 数据丢失 → 帧残缺 → 解析器失步。
     *
     * 解决: 排空硬件 FIFO + 复位 buf 指针 + 复位解析器状态 + 清零 has_angle。
     *       冲洗后主循环启动时收到的第一帧就是"干净"的完整帧,
     *       保证 SCAN → TRACK 状态跳转不受初始化阶段陈旧数据干扰。
     */
    uart_rx_flush();

    /* 状态 */
    s_stage = ST_INIT;
    s_stage_start = sys_tick_ms;
    s_scan_dir = 1;
    s_ema_ok = false;
    s_in_none = false;
    s_ema_h = s_ema_v = 0.0f;
    s_scanner_stall_start = 0;  /* ★ v8: 停滞计时从 INIT 开始 */
    /* 注: g_parsed 已在 uart_rx_flush() 中复位, 此处不再 memset */
}

void Vision_Update(void)
{
    parse_packets();  /* 每轮消费 UART */

    /* 9-DOF IMU 更新 @200Hz (每 5ms) */
    static uint32_t last_imu_ms = 0;
    uint32_t now = sys_tick_ms;
    if ((now - last_imu_ms) >= 5) {
        last_imu_ms = now;
        if (s_imu_ok) IMU_9DOF_Update(&s_imu_data);
    }

    static uint32_t last_ms = 0;
    if ((now - last_ms) >= TRACK_LOOP_MS) {
        last_ms = now;
        state_machine();
        gimbal_stall_monitor_tick();
        if (gimbal_is_any_stalled()) {
            gimbal_stop();
            while (1) {
                /* 堵转故障 — 死循环等待人工复位/调试器干预 */
            }
        }
    }
    /* ★ 不在此处 __WFE() — 主线循环需尽可能快消费 UART 数据,
     *   否则 K230 帧可能累积导致环形缓冲区溢出或解析延迟。
     *   功率管理交给 IDLE 任务或 RTOS tickless, 不由裸机主循环负责。 */
}

void Vision_SetLaserOffset(float pan_deg, float tilt_deg)
{
    if (!bad_float(pan_deg))  g_pan_offset  = pan_deg;
    if (!bad_float(tilt_deg)) g_tilt_offset = tilt_deg;
}

VisionMode_t Vision_GetMode(void)
{
    switch (s_stage) {
        case ST_INIT:  return VISION_MODE_INIT;
        case ST_SCAN:  return VISION_MODE_SCANNING;
        case ST_TRACK: return VISION_MODE_TRACKING;
        default:       return VISION_MODE_INIT;
    }
}

void Vision_GetStats(VisionStats_t *stats)
{
    if (!stats) return;
    stats->h_deg       = g_parsed.h_deg;
    stats->v_deg       = g_parsed.v_deg;
    stats->target_pan  = s_target_pan;
    stats->target_tilt = s_target_tilt;
    stats->heading     = s_imu_ok ? s_imu_data.heading : 0.0f;
    stats->pan_ff      = s_pan_ff;
    stats->circles     = g_parsed.circles;
    stats->mode        = (int)Vision_GetMode();  /* ★ v8: 当前追踪模式 */
    stats->hmc_ok      = s_imu_ok;
    stats->packets_ok  = g_parsed.cnt_ok;
    stats->packets_none    = g_parsed.cnt_none;
    stats->packets_bad     = g_parsed.cnt_bad;
    stats->packets_overflow = g_parsed.cnt_overflow;
    stats->stall_resets    = s_stall_resets;     /* ★ v8: 自动恢复次数 */
    stats->total_bytes     = g_rx_total_bytes;   /* ISR 累计接收字节数 */
}

void Vision_EmergencyStop(void)
{
    gimbal_stop();
    s_stage = ST_INIT;
    s_stage_start = sys_tick_ms;  /* ★ 重置计时, 否则可能立即/错误跳转到SCAN */
    s_ema_ok = false;
    s_imu_first = true;  /* ★ 重设 IMU 参考, 避免用旧值计算差分 */
    s_scanner_stall_start = 0;   /* ★ v8: 重置停滞计时 */
    uart_rx_flush();      /* 冲洗缓冲区 + 复位解析器 + 清零帧标志 */
    /* 统计计数器(cn_ok/cnt_none等)保留, 用于诊断 */
}
