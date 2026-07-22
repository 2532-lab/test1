/**
 * @file    encoder.c
 * @brief   四路 AB 相编码器驱动实现 —— 硬件 QEI(C1) + 软件轮询(C2~C4)
 *
 * ======================== 软件编码器原理 ========================
 *
 *   AB 相正交编码器的输出波形（正转）：
 *
 *     A 相:  ───┐     ┌───┐     ┌───
 *              └─────┘   └─────┘
 *     B 相:  ──┐     ┌───┐     ┌───
 *              └─────┘   └─────┘
 *               ↑         ↑
 *           A下降沿    A上升沿
 *           B=1 → +1  B=0 → +1
 *
 *   双沿检测：A 相的上升沿和下降沿都触发计数。
 *   每个周期有 2 次跳变 → CPR = PPR × 2 × 减速比。
 *
 *   方向判断（以正转为例）：
 *     - A 下降沿(1→0) 时 B=1 → 正面方向 → +1
 *     - A 上升沿(0→1) 时 B=0 → 正面方向 → +1（减负得正）
 *
 *   反转时 B 相移相 90° 反相 → 数值相反。
 *
 * ======================== SysTick ISR 设计 ========================
 *
 *   SysTick_Handler 每秒触发 1000 次（1kHz），每次做：
 *     1. sys_tick_ms++（毫秒计数器）
 *     2. Encoder_Scan()（扫描 C2/C3/C4 的 A 相跳变）
 *
 *   ISR 执行时间：~20μs（三路扫描 + 换向判断），
 *   仅占 CPU 的 ~2%（32MHz 下 1000×20μs/1s）。
 *
 *   如果使用周期高于 1kHz：
 *     - RELOAD 减小 → 中断频率升高 → CPU 占用增加
 *     - 1kHz 对 660CPR 的编码器已足够（最大不失步 RPM ≈ 90）
 *
 * ======================== C3 方向修正 ========================
 *
 *   右后轮（C3）的 AB 相在 PCB 上物理接线与其他三路相反。
 *   PCB 已量产，不方便改板，在软件中翻转方向判断逻辑。
 *
 *   修正方法：在 Encoder_Scan 中对 sw==1（C3）使用
 *   与其他通道相反的方向判断条件。
 */

#include "encoder.h"
#include "ti_msp_dl_config.h"
#include "delay.h"
#include <string.h>

/* ===================================================================
 *  全局毫秒计数器
 *
 *  由 SysTick_Handler 每 1ms 递增。delay.h 中以 extern 声明，
 *  millis() 内联读取。实定义在 src/delay.c。
 * =================================================================== */

/* ===================================================================
 *  软件编码器常量
 * =================================================================== */

/** C2 在 EncoderID_t 数组中的起始索引 */
#define SW_FIRST_IDX    1   /* ENCODER_C2 */

/** 软件扫描的编码器数量（C2, C3, C4） */
#define SW_COUNT        3

/* ===================================================================
 *  软件编码器 GPIO 映射
 *
 *  优先使用 SysConfig 生成的宏（GPIO_ENCODER_C2_A_PORT 等）。
 *  若未使用 SysConfig（裸机工程），回退到硬编码的引脚常量。
 * =================================================================== */

#ifdef GPIO_ENCODER_C2_A_PORT
  /* SysConfig 已配置编码器 GPIO */
  #define ENC_C2_A_PORT   GPIO_ENCODER_C2_A_PORT
  #define ENC_C2_A_PIN    GPIO_ENCODER_C2_A_PIN
  #define ENC_C2_B_PORT   GPIO_ENCODER_C2_B_PORT
  #define ENC_C2_B_PIN    GPIO_ENCODER_C2_B_PIN
  #define ENC_C3_A_PORT   GPIO_ENCODER_C3_A_PORT
  #define ENC_C3_A_PIN    GPIO_ENCODER_C3_A_PIN
  #define ENC_C3_B_PORT   GPIO_ENCODER_C3_B_PORT
  #define ENC_C3_B_PIN    GPIO_ENCODER_C3_B_PIN
  #define ENC_C4_A_PORT   GPIO_ENCODER_C4_A_PORT
  #define ENC_C4_A_PIN    GPIO_ENCODER_C4_A_PIN
  #define ENC_C4_B_PORT   GPIO_ENCODER_C4_B_PORT
  #define ENC_C4_B_PIN    GPIO_ENCODER_C4_B_PIN
#else
  /* 回退：直接 GPIO 定义（裸机环境，无 SysConfig） */
  #define ENC_C2_A_PORT   GPIOB
  #define ENC_C2_A_PIN    DL_GPIO_PIN_22
  #define ENC_C2_B_PORT   GPIOB
  #define ENC_C2_B_PIN    DL_GPIO_PIN_16
  #define ENC_C3_A_PORT   GPIOB
  #define ENC_C3_A_PIN    DL_GPIO_PIN_20
  #define ENC_C3_B_PORT   GPIOA
  #define ENC_C3_B_PIN    DL_GPIO_PIN_22
  #define ENC_C4_A_PORT   GPIOA
  #define ENC_C4_A_PIN    DL_GPIO_PIN_15
  #define ENC_C4_B_PORT   GPIOA
  #define ENC_C4_B_PIN    DL_GPIO_PIN_14
#endif

/* ===================================================================
 *  软件编码器运行时状态
 *
 *  每路一个 EncoderSw_t 实例，记录 GPIO 端口/引脚、上一次
 *  A 相电平、累计脉冲数、本次净增量和上次快照（用于 rpm 计算）。
 *
 *  索引映射（数组下标）：
 *    sw_idx = EncoderID_t - SW_FIRST_IDX
 *    C2 → 0, C3 → 1, C4 → 2
 * =================================================================== */

typedef struct {
    GPIO_Regs *port_a;      /**< A 相 GPIO 端口（如 GPIOB） */
    uint32_t   pin_a;       /**< A 相引脚掩码（如 DL_GPIO_PIN_22） */
    GPIO_Regs *port_b;      /**< B 相 GPIO 端口 */
    uint32_t   pin_b;       /**< B 相引脚掩码 */
    uint8_t    last_a;      /**< 上一次 A 相电平：0=低, 1=高 */
    int32_t    count;       /**< 累计脉冲数（正=前进, 负=倒退） */
    int16_t    delta;       /**< 最近一次扫描的净增量 */
    int32_t    prev_count;  /**< 上次 GetSpeed 调用时的 count 快照 */
} EncoderSw_t;

static EncoderSw_t s_enc_sw[SW_COUNT];

/* ===================================================================
 *  C1 硬件编码器快照（用于 GetSpeed 差分）
 * =================================================================== */

/** 上次 Encoder_GetSpeed() 时 TIMG8.CTR 的计数器快照 */
static uint32_t s_hw_prev_count = 0;

/** 各路编码器上次 GetSpeed 调用的 millis() 时间戳（用于计算真实 dt） */
static uint32_t s_last_speed_ms[ENCODER_COUNT] = {0};

/* ===================================================================
 *  硬件编码器初始化（TIMG8 → C1）
 *
 *  SysConfig 已将 TIMG8 设为 Encoder Mode（PB21/PA30 = CCP0/CCP1）。
 *  这里只需清零计数器并启动。
 * =================================================================== */

static void Encoder_Init_HW(void)
{
#ifdef TIMG8_INST
    DL_TimerG_setTimerCount(TIMG8_INST, 0);
    DL_TimerG_startCounter(TIMG8_INST);
#endif
}

/* ===================================================================
 *  软件编码器扫描（ISR 内部调用）
 *
 *  对 C2/C3/C4 三路编码器，各做一次 A 相电平检测：
 *    1. 读 GPIO → 判断是否有跳变（cur != last_a）
 *    2. 有跳变 → 读 B 相电平确定方向
 *    3. 下降沿/上升沿都用 dir 修正（双沿计数）
 *    4. 更新 last_a、count、delta
 *
 *  C3 特殊处理：AB 相接反 → 方向判断取反。
 *
 *  注意：硬件编码器 C1 不需要软件扫描——TIMG8 自动计数。
 * =================================================================== */

void Encoder_Scan(void)
{
    for (int sw = 0; sw < SW_COUNT; sw++) {
        EncoderSw_t *e      = &s_enc_sw[sw];
        int32_t      prev   = e->count;

        /* 读取当前 A 相电平（高/低 → 1/0） */
        uint32_t raw = DL_GPIO_readPins(e->port_a, e->pin_a);
        uint8_t  cur = (raw != 0) ? 1 : 0;

        /* 检测跳变：cur != last_a → A 相电平发生了变化 */
        if (e->last_a != cur) {
            uint32_t b_raw = DL_GPIO_readPins(e->port_b, e->pin_b);
            int8_t   dir;

            /*
             * 方向判断：
             *
             * 标准（C2/C4）：
             *   正转（A 超前 B 90°）：
             *     A 下降沿(1→0)时 B=1 → dir=+1
             *     A 上升沿(0→1)时 B=0 → dir=+1（下方减法：减 -1 = +1）
             *   反转（B 超前 A 90°）→ 上述全部取反
             *
             * C3 (sw==1)：AB 相接反。不在边沿级判断，
             * 在 Encoder_GetCount 返回值整体取负。
             */
            if (0) {
                /* 保留：如需边沿级翻转可改 if(1)，当前用 GetCount 取负代替 */
                dir = (b_raw != 0) ? -1 : 1;
            } else {
                dir = (b_raw != 0) ? 1 : -1;
            }

            /*
             * 双沿计数：
             *   下降沿(1→0): count += dir
             *   上升沿(0→1): count -= dir
             *
             *   为什么上升沿是减法？
             *     正转时 dir=+1：下降沿 +1, 上升沿 -(-1)=+1 → 净+2/周期
             *     这正是"双沿"的含义——每个 AB 周期计两次。
             */
            if (cur == 0) {
                e->count += dir;       /* 下降沿: cur=0 → cur<last_a */
            } else {
                e->count -= dir;       /* 上升沿: cur=1 → cur>last_a */
            }
        }
        e->last_a = cur;

        /* 本次扫描净增量（用于 Encoder_GetDelta） */
        e->delta = (int16_t)(e->count - prev);
    }
}

/* ===================================================================
 *  SysTick 定时器初始化（1kHz，不占外设定时器）
 *
 *  Cortex-M0+ 的 SysTick 是 24-bit 递减计数器，独立于外设定时器。
 *  用做编码器扫描时钟不会浪费 TIMG/TIMA 资源。
 *
 *  计算公式：
 *    RELOAD = (CPU_FREQ / prescale / target_freq) - 1
 *           = (32MHz / 1 / 1kHz) - 1
 *           = 32000 - 1 = 31999
 *
 *  优先级：
 *    SysTick_IRQn  = 192 (0xC0) —— 最低，允许被任何中断打断
 *    UART0_INT_IRQn = 64  (0x40) —— 中等，蓝牙通信不可丢包
 *    GPIOB_IRQn     = 0   (0x00) —— 最高（默认值），MPU6050 数据就绪
 *
 *  Cortex-M0+ NVIC 只有 2 位优先级可用（bit7:6），
 *  有效值：0, 64, 128, 192（对应寄存器值 0x00, 0x40, 0x80, 0xC0）。
 * =================================================================== */

static void Encoder_ScanTimer_Init(void)
{
    SysTick->LOAD = 31999U;           /* 1kHz: 32000 - 1 */
    SysTick->VAL  = 0;                 /* 清零当前值，防止首次中断立即触发 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   /* 时钟源 = CPUCLK (32MHz) */
                  | SysTick_CTRL_TICKINT_Msk     /* 使能中断 */
                  | SysTick_CTRL_ENABLE_Msk;     /* 使能计数器 */

    NVIC_SetPriority(SysTick_IRQn,     192);
    NVIC_SetPriority(UART0_INT_IRQn,    64);
}

/* ===================================================================
 *  SysTick 中断服务程序
 *
 *  每 1ms 触发一次。两个任务：
 *    1. sys_tick_ms++  → 提供 millis() 时间戳
 *    2. Encoder_Scan() → 扫描 C2/C3/C4 编码器
 *
 *  注意：不要在此 ISR 中调用 printf 或阻塞函数。
 *        保持 ISR 短小（<100 行）是裸机编程的铁律。
 * =================================================================== */

void SysTick_Handler(void)
{
    sys_tick_ms++;
    Encoder_Scan();
}

/* ===================================================================
 *  公开接口：初始化
 * =================================================================== */

void Encoder_Init(void)
{
    /* C1: 硬件 QEI */
    Encoder_Init_HW();
    s_hw_prev_count = 0;

    /* 清零 GetSpeed 时间戳（强制首次调用返回 false，初始化基准） */
    for (int i = 0; i < ENCODER_COUNT; i++) {
        s_last_speed_ms[i] = 0;
    }

    /* C2/C3/C4: 初始化软件编码器状态 */
    memset(s_enc_sw, 0, sizeof(s_enc_sw));

    /* C2: 左前轮 A=PB22 B=PB16 */
    s_enc_sw[0].port_a = ENC_C2_A_PORT; s_enc_sw[0].pin_a = ENC_C2_A_PIN;
    s_enc_sw[0].port_b = ENC_C2_B_PORT; s_enc_sw[0].pin_b = ENC_C2_B_PIN;

    /* C3: 右后轮 A=PB20 B=PA22（AB 相接反） */
    s_enc_sw[1].port_a = ENC_C3_A_PORT; s_enc_sw[1].pin_a = ENC_C3_A_PIN;
    s_enc_sw[1].port_b = ENC_C3_B_PORT; s_enc_sw[1].pin_b = ENC_C3_B_PIN;

    /* C4: 左后轮 A=PA15 B=PA14 */
    s_enc_sw[2].port_a = ENC_C4_A_PORT; s_enc_sw[2].pin_a = ENC_C4_A_PIN;
    s_enc_sw[2].port_b = ENC_C4_B_PORT; s_enc_sw[2].pin_b = ENC_C4_B_PIN;

    /* 读取初始 A 相电平，避免首次扫描时误判为跳变 */
    for (int i = 0; i < SW_COUNT; i++) {
        uint32_t v = DL_GPIO_readPins(s_enc_sw[i].port_a,
                                      s_enc_sw[i].pin_a);
        s_enc_sw[i].last_a     = (v != 0) ? 1 : 0;
        s_enc_sw[i].prev_count = s_enc_sw[i].count;
    }

    /* 启动 SysTick 1kHz 扫描定时器 */
    Encoder_ScanTimer_Init();
}

/* ===================================================================
 *  公开接口：读取累计脉冲数
 * =================================================================== */

int32_t Encoder_GetCount(EncoderID_t id)
{
    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        return (int32_t)DL_TimerG_getTimerCount(TIMG8_INST);
#else
        return 0;
#endif
    }

    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return 0;

    /* C3(sw=1)：AB 相接反，整体取负修正方向 */
    int32_t val = s_enc_sw[sw].count;
    if (sw == 1) {
        val = -val;
    }
    return val;
}

/* ===================================================================
 *  公开接口：复位
 * =================================================================== */

void Encoder_Reset(EncoderID_t id)
{
    if (id >= ENCODER_COUNT) return;

    s_last_speed_ms[id] = 0;  /* 复位时间基准，下次 GetSpeed 返回 false */

    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        DL_TimerG_setTimerCount(TIMG8_INST, 0);
#endif
        s_hw_prev_count = 0;
        return;
    }

    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return;

    s_enc_sw[sw].count      = 0;
    s_enc_sw[sw].delta      = 0;
    s_enc_sw[sw].prev_count = 0;
}

void Encoder_ResetAll(void)
{
#ifdef TIMG8_INST
    DL_TimerG_setTimerCount(TIMG8_INST, 0);
#endif
    s_hw_prev_count = 0;

    for (int i = 0; i < SW_COUNT; i++) {
        s_enc_sw[i].count      = 0;
        s_enc_sw[i].delta      = 0;
        s_enc_sw[i].prev_count = 0;
    }
}

/* ===================================================================
 *  公开接口：测速（RPM）
 *
 *  公式：rpm = (pulse_diff / CPR) / (dt / 60)
 *         = pulse_diff × 60 / (CPR × dt)
 *
 *   例：dt=0.05s, diff=55 脉冲, CPR=660
 *       rpm = 55 × 60 / (660 × 0.05) = 100 RPM
 *
 *  C1（硬件 QEI）直接读 TIMG8.CTR 差分。
 *  C2~C4 从软件 count 差分，期间关 SysTick 中断保护。
 *
 *  为什么软件编码器需要关中断？
 *    Encoder_Scan（ISR 中）会在 read count 和 write prev_count
 *    之间修改 count，破坏差分的原子性。
 *    关中断（TICKINT=0）防止 ISR 插入——时间 < 1μs，可接受。
 * =================================================================== */

bool Encoder_GetSpeed(EncoderID_t id, float dt, float *rpm)
{
    (void)dt;  /* 不再使用调用者传入的固定 dt，改用 millis() 真实间隔 */
    if (rpm == NULL) return false;

    /*
     * ====== Fix: 用 millis() 计算真实 dt ======
     *
     * 旧版使用调用者传入的固定 dt（如 0.01），但实际调用频率因
     * 主循环抖动、中断抢占而波动（8~15ms 不等）→ 测速严重不准。
     *
     * 新版记录每次调用的 millis() 时间戳，自动计算真实时间间隔。
     * 首次调用（elapsed==0 或 >1s）返回 false，仅初始化时间基准。
     */
    uint32_t now     = millis();
    uint32_t elapsed = now - s_last_speed_ms[id];
    if (elapsed == 0 || elapsed > 1000) {
        s_last_speed_ms[id] = now;
        return false;
    }
    float real_dt = (float)elapsed * 0.001f;
    s_last_speed_ms[id] = now;

    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        int32_t cur  = (int32_t)DL_TimerG_getTimerCount(TIMG8_INST);
        int32_t diff = cur - (int32_t)s_hw_prev_count;
        s_hw_prev_count = (uint32_t)cur;
        *rpm = ((float)diff / (float)ENCODER_CPR_HW) / (real_dt / 60.0f);
        return true;
#else
        return false;
#endif
    }

    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return false;

    /* 临界区：关 SysTick 中断 → 读 count → 写 prev → 开中断 */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    int32_t cur  = s_enc_sw[sw].count;
    int32_t diff = cur - s_enc_sw[sw].prev_count;
    s_enc_sw[sw].prev_count = cur;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    *rpm = ((float)diff / (float)ENCODER_CPR) / (real_dt / 60.0f);
    return true;
}

/* ===================================================================
 *  公开接口：增量（最近一次扫描的净脉冲数）
 * =================================================================== */

int16_t Encoder_GetDelta(EncoderID_t id)
{
    if (id == ENCODER_C1) {
#ifdef TIMG8_INST
        int32_t cur  = (int32_t)DL_TimerG_getTimerCount(TIMG8_INST);
        return (int16_t)(cur - (int32_t)s_hw_prev_count);
#else
        return 0;
#endif
    }

    int sw = id - SW_FIRST_IDX;
    if (sw < 0 || sw >= SW_COUNT) return 0;
    return s_enc_sw[sw].delta;
}
