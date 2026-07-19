/**
 * @file    test_motor_encoder.h
 * @brief   电机 + 编码器联合测试模块 —— 声明
 *
 * ======================== 测试目的 ========================
 *
 *   1. 验证四路 PWM → 实际转速的线性度（30%/50%/70% 占空比）
 *   2. 验证编码器方向（前进时 rpm 全为正）
 *   3. 验证 pivot 对称性（左转/右转 rpm 绝对值接近）
 *   4. 验证 L1306 桥 + 蓝牙透传链路完整性
 *
 * ======================== VOFA+ 通道定义 ========================
 *
 *   Ch0: C1 RPM（右前轮, TIMG8 硬件 QEI）
 *   Ch1: C2 RPM（左前轮, 软件扫描）
 *   Ch2: C3 RPM（右后轮, 软件扫描）
 *   Ch3: C4 RPM（左后轮, 软件扫描）
 *   Ch4: 测试阶段编号（0=停 1=30% 2=50% 3=70% 4=左转 5=右转）
 *   Ch5: 运行时间（秒）
 *
 * ======================== 测试序列 ========================
 *
 *   → STOP(2s) → FWD_30(3s) → FWD_50(3s) → FWD_70(3s) →
 *   → STOP(2s) → PIVOT_LEFT(3s) → PIVOT_RIGHT(3s) → 循环
 */

#ifndef TEST_MOTOR_ENCODER_H_
#define TEST_MOTOR_ENCODER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 遥测发送间隔（ms），50ms = 20Hz */
#define TEST_TELEM_INTERVAL_MS   50

/** JustFloat 通道数 */
#define TEST_TELEM_CHANNELS      6

/** 每个运动阶段持续时间（ms） */
#define TEST_PHASE_DURATION_MS   3000

/** 停止阶段持续时间（ms） */
#define TEST_STOP_DURATION_MS    2000

/** 测试阶段枚举 */
typedef enum {
    TEST_PHASE_STOP = 0,      /**< 全部停止 */
    TEST_PHASE_FWD_30,        /**< 前进 30% 占空比 */
    TEST_PHASE_FWD_50,        /**< 前进 50% 占空比 */
    TEST_PHASE_FWD_70,        /**< 前进 70% 占空比 */
    TEST_PHASE_PIVOT_LEFT,    /**< 原地左转（50% 占空比） */
    TEST_PHASE_PIVOT_RIGHT,   /**< 原地右转（50% 占空比） */
    TEST_PHASE_COUNT          /**< 阶段总数（循环边界） */
} TestPhase_t;

/**
 * @brief   初始化测试环境
 *
 * 调用：Motor_PWM_Init() + Encoder_Init() + Telemetry_Init()
 *
 * @note    必须在 SYSCFG_DL_init() 之后调用。
 *          Encoder_Init 会启用 SysTick 1kHz 中断。
 */
void Test_MotorEncoder_Init(void);

/**
 * @brief   运行测试序列 + 发送遥测（主循环每帧调用）
 *
 * 非阻塞：不包含 while 循环或 delay。每次调用检查时间戳，
 * 到达阶段切换时间时才更新电机状态。
 *
 * 编码器每帧都读（保持 prev_count 最新），
 * 但遥测只在 TEST_TELEM_INTERVAL_MS 到达时发送。
 */
void Test_MotorEncoder_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_MOTOR_ENCODER_H_ */
