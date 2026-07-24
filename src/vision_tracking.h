/*
 * vision_tracking.h — 视觉追踪模块 API + 全部可调参数
 *
 * ============================================================================
 * === 快速上手 ===
 *
 *   #include "vision_tracking.h"
 *
 *   int main(void) {
 *       SYSCFG_DL_init();
 *       SysTick_Config(CPUCLK_FREQ / 1000U);
 *       Vision_Init();
 *       Vision_SetLaserOffset(0.0f, 0.0f);
 *       delay_ms(100);
 *       while (1) { Vision_Update(); }
 *   }
 *
 * ============================================================================
 * === 阅读顺序 ===
 *
 *   第一节 — 可调参数 (你需要改的都在这里)
 *   第二节 — API 函数 (你可以调用的)
 *   第三节 — 类型定义 (VisionMode_t / VisionStats_t)
 */

#ifndef VISION_TRACKING_H
#define VISION_TRACKING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 第一节: 可调参数 — 改这些数值即可, 无需看 .c 文件
 * ========================================================================== */

/* ── ① 追踪性能 ────────────────────────────────────────── */

/** 追踪主循环周期 (ms), 默认 20ms = 50Hz */
#define TRACK_LOOP_MS           20

/** EMA 平滑系数 (0~1), 小=更平滑但响应慢, 大=响应快但噪声多
 *
 * ★ v10 修复 (过冲振荡): 0.25→0.70
 * ★ v10.1 根治: 0.70→1.00 (直通, 不做 EMA)
 *
 * 控制理论分析:
 *   EMA(α<1) + P(GAIN) = 二阶系统, 判别式 = (2-GAIN·α-α)² - 4(1-α)
 *   代入 α=0.70, GAIN=0.35 → 判别式 = 1.113 - 1.20 = -0.087 < 0
 *   → 复数特征根 → 欠阻尼振荡, 收敛极慢 (|λ|=0.55/周期)
 *
 *   α=1.0 退化为一阶系统: C[n+1] = (1-GAIN)C[n] + GAIN·T
 *   特征根 = 1-GAIN ∈ (0,1), 单调收敛, 永不振荡。
 *
 * K230 已在图像端做 EMA(0.30) 平滑, MSPM0 不需要再做。
 * 跳变保护 (ANGLE_JUMP_LIMIT) 仍有效: 比较本帧 h_deg vs 上帧 h_deg。
 */
#define TRACK_EMA_ALPHA          1.00f

/** 视觉追踪增益 (0.5~1.0), <1 提供阻尼防止过冲振荡
 *
 * ★ v10: 0.5→0.35
 * ★ v10.1: 0.35→0.20 (配合 α=1.0, 一阶系统, 保守收敛)
 *
 *   一阶系统: C_next = (1-GAIN)×C + GAIN×T
 *   GAIN=0.20 → 5%收敛 ≈ 13 周期 (260ms), 完全无过冲
 *   GAIN=0.30 → 5%收敛 ≈ 8 周期 (160ms), 仍无过冲但余量小
 *
 *   K230 EMA(0.30) 引入 ~50ms 额外延迟 (在 MSPM0 外部),
 *   取 GAIN=0.20 提供充足相位余量, 补偿 K230 侧延迟。
 *   如果追踪太慢, 可以逐步调高到 0.30 (仍是一阶, 保证不振荡)。
 */
#define TRACK_GAIN               0.20f

/** 单帧帧间变化保护阈值 (°), 超过此值拒绝该帧 */
#define ANGLE_JUMP_LIMIT         20.0f

/** 追踪运动 RPM
 *
 * ★ v10: 10→180, ★ v10.1: 180→90
 *
 * RPM=90 理由:
 *   - 20ms 内转 ~10.8° (≈96步), 对 1°~5° 修正一周期内完成
 *   - 比 180 降低一半, 减少机械振动/共振 → 切断"电机振动→
 *     摄像头晃动→K230检测偏移→再调整"的恶性循环
 *   - 如果追踪响应偏慢, 可逐步调回 120→150→180
 */
#define TRACK_RPM                90

/** 目标丢失超时 (ms), 超时后从 TRACKING 切回 SCANNING */
#define TARGET_LOST_TIMEOUT_MS   2000

/** 近靶锁定阈值 (°)
 *
 * ★ v10 新增: 当 EMA 误差 (|s_ema_h| 和 |s_ema_v|) 同时小于此值、
 *   且电机静止、且连续 TRACK_SETTLE_CYCLES 个周期满足条件 → 暂停下发
 *   运动命令, 云台保持当前角度。EMA 持续更新, 一旦误差超阈值立即恢复追踪。
 *
 * 目的: 消除"近靶微抖" — 当靶子已基本居中, 噪声帧导致的单步修正会引起
 *       摄像头画面微振 → 正反馈 → 无法完全静止。
 *
 * 设太大 (如 2°) → 靶子明显偏离也不修正; 设太小 (如 0.05°) → 等于没锁。
 * 0.3° ≈ 2.7步, 对应 ~0.5像素 @FOCAL_PX=270, 人眼不可感知。
 */
#define TRACK_SETTLE_THRESHOLD    0.3f

/** 近靶锁定确认周期 — 连续满足条件 N 次才锁定 (防噪声误触发) */
#define TRACK_SETTLE_CYCLES       3

/* ── ② 扫描搜索 ────────────────────────────────────────── */

/** 扫描转速 RPM */
#define SCAN_RPM                 9

/** 水平扫描范围 (±°) */
#define SCAN_PAN_RANGE           45.0f

/** 扫描时 TILT 固定俯角 (°), 稍向下便于看到靶子 */
#define SCAN_TILT_ANGLE          10.0f

/** 扫描边界判定余量 (°), 防浮点抖动导致反复换向 */
#define SCAN_BOUNDARY_MARGIN     0.5f

/** K230 上电等待时间 (ms)
 *
 * ★ v7: 3000→8000ms. v8: 8000→15000ms.
 *       K230 运行 MicroPython + GC2093 摄像头初始化实测 5~15s,
 *       取上限 15s + IMU校准 2.5s = 17.5s 总等待, 确保 K230 就绪后才 SCAN。
 *       若 K230 启动期间 TX 引脚浮空/抖动 → 噪声字节进入解析器 → 解析器失步;
 *       延长 INIT 阶段可将噪声阻挡在 SCAN→TRACK 关键窗口之外。 */
#define K230_BOOT_WAIT_MS        5000

/** UART 解析器停滞自动恢复 (ms)
 *
 * 若 SCAN 模式下超过此时长未收到任何有效帧 (cnt_ok 不变),
 * 自动冲洗 UART 环形缓冲区 + 复位解析器状态机 (等效 Vision_EmergencyStop)。
 * 0 = 禁用自动恢复。 */
#define UART_STALL_RECOVERY_MS   30000

/* ── ②½ 方向符号 (运动方向反了改这里, 不改 .c) ────────── */

/** PAN 方向: +1.0 = 同向, -1.0 = 反向 */
#define PAN_DIR_SIGN             1.0f

/** TILT 方向: +1.0 = 同向, -1.0 = 反向 */
#define TILT_DIR_SIGN            -1.0f

/* ── ③ 9-DOF IMU 增稳 (MPU6050 + HMC5883L) ────────────── */

/**
 * PAN 轴前馈增益 (0~1.0). 用 yaw 变化率检测扰动.
 * 0.0 = 禁用, 0.6 = 抵消60% (默认), 1.0 = 全抵消
 */
#define KP_IMU_PAN               0.6f

/**
 * TILT 轴前馈增益 (0~1.0). 用 pitch 变化率检测扰动.
 * 默认 0.4 较保守 (俯仰惯性小, 噪声影响大)
 */
#define KP_IMU_TILT              0.4f

/* ==========================================================================
 * 第二节: 类型定义 (第三节 API 的参数/返回值类型, 必须在前)
 * ========================================================================== */

/** 追踪器工作模式 */
typedef enum {
    VISION_MODE_INIT = 0,     /**< 初始化等待 (K230 启动 ~3s) */
    VISION_MODE_SCANNING,     /**< 水平往复扫描搜索靶子 */
    VISION_MODE_TRACKING,     /**< 锁定追踪靶心 */
} VisionMode_t;

/** 追踪器统计快照 (只读, Vision_GetStats 填充) */
typedef struct {
    float    h_deg;           /**< 最新 K230 水平偏移角 (°) */
    float    v_deg;           /**< 最新 K230 垂直偏移角 (°) */
    float    target_pan;      /**< 当前 PAN 目标角度 (°) */
    float    target_tilt;     /**< 当前 TILT 目标角度 (°) */
    float    heading;         /**< HMC5883L 磁航向 (°), 0=无数据 */
    float    pan_ff;          /**< 磁力计 PAN 前馈量 (°) */
    int      circles;         /**< 同心圆数 (0=canmv协议无此字段) */
    int      mode;            /**< 当前追踪模式: 0=INIT, 1=SCANNING, 2=TRACKING */
    bool     hmc_ok;          /**< HMC5883L 是否就绪 */
    uint32_t packets_ok;      /**< 成功接收的有效帧数 */
    uint32_t packets_none;    /**< 收到的 NONE 帧数 */
    uint32_t packets_bad;     /**< 解析失败/拒绝的帧数 */
    uint32_t packets_overflow;/**< 环形缓冲区溢出次数 */
    uint32_t stall_resets;    /**< UART 解析器自动恢复次数 (>0说明发生过解析停滞) */
    uint32_t total_bytes;     /**< ISR 累计接收字节总数 (>0 确认 K230→MSPM0 物理连接正常) */
} VisionStats_t;

/* ==========================================================================
 * 第三节: API 函数 — 你可以在 main.c 或其他地方调用这些
 * ========================================================================== */

/**
 * @brief  初始化视觉追踪模块。
 *
 * 内部执行:
 *   - gimbal_init() + gimbal_reset_zero()
 *   - I2C1 总线配置 + MID_HMC5883L_Init()
 *   - UART2 RX 中断使能
 *   - 状态机进入 INIT, 等待 K230 启动
 *
 * @note   必须在 SYSCFG_DL_init() + SysTick_Config() 之后调用
 * @note   HMC5883L 初始化失败不影响追踪 (自动降级为纯视觉)
 */
void Vision_Init(void);

/**
 * @brief  主循环更新 — 在 while(1) 中调用, 越快越好。
 *
 * 内部自动节流:
 *   - UART 数据消费: 每次调用都执行 (非阻塞)
 *   - 状态机 + HMC5883L: 每 TRACK_LOOP_MS (20ms) 执行一次
 *   - 堵转检测: 随状态机周期执行
 */
void Vision_Update(void);

/**
 * @brief  设置激光-摄像头偏差补偿角度。
 *
 * 激光与摄像头虽同轴安装但存在物理装配偏差, 通过此函数补偿。
 * CCS 调试时在 Expressions 窗口直接调用即可实时生效。
 *
 * @param  pan_offset_deg   PAN 轴偏差 (°), 光斑偏右→正
 * @param  tilt_offset_deg  TILT 轴偏差 (°), 光斑偏下→正
 *
 * @note   offset 乘以 TRACK_GAIN 后生效, 修改 GAIN 需重新标定
 */
void Vision_SetLaserOffset(float pan_offset_deg, float tilt_offset_deg);

/**
 * @brief  获取当前追踪模式。
 * @return VISION_MODE_INIT / VISION_MODE_SCANNING / VISION_MODE_TRACKING
 */
VisionMode_t Vision_GetMode(void);

/**
 * @brief  获取追踪器统计快照 (调试/遥测用)。
 * @param  stats  输出: 不能为 NULL
 *
 * 可查看: 最新 K230 角度、目标角度、HMC5883L 航向、前馈量、
 *         磁力计状态、收包统计
 */
void Vision_GetStats(VisionStats_t *stats);

/**
 * @brief  紧急停止 — 立即停止双轴电机, 进入 INIT 状态。
 * @note   恢复需重新调用 Vision_Init()
 */
void Vision_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_TRACKING_H */
