/**
 * @file    test_motor_encoder.c
 * @brief   四轮电机 + 编码器联合测试 —— 自动遍历 PWM → RPM 验证
 *
 * ======================== 测试目的 ========================
 *
 *   1. 验证四路 PWM 到实际转速的线性度
 *   2. 验证编码器方向正确性（前进时 RPM 全为正）
 *   3. 验证 pivot 转向对称性
 *   4. 验证 L1306 桥 + 蓝牙 JustFloat 数据完整性
 *
 * ======================== 测试序列 ========================
 *
 *   STOP(2s)→30%(3s)→50%(3s)→70%(3s)→左转(3s)→右转(3s)→循环
 *
 * ======================== 依赖 ========================
 *
 *   motor.h    全部方向/PWM 操作通过 motor.c API
 *   encoder.h  编码器读数
 *   app_telemetry.h  JustFloat 遥测
 *   delay.h    millis() 时间戳
 */

#include "test_motor_encoder.h"
#include "app_telemetry.h"
#include "encoder.h"
#include "motor.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

static TestPhase_t   s_phase;
static uint32_t      s_phase_start_ms;
static uint32_t      s_test_start_ms;

/* ===================================================================
 *  方向/速度辅助（全部通过 motor.c API，不直接操作 GPIO）
 * =================================================================== */

static void m_all_stop(void)
{
    Motor_StopAll();
    Motor_SetAllForward();
}

/* dir>0 右转（右侧倒退+左侧前进），dir<0 左转（左侧倒退+右侧前进） */
static void m_pivot(int8_t dir, uint8_t duty)
{
    if (dir > 0) {
        Motor_SetAllForward();
        Motor_SetRightReverse();
    } else {
        Motor_SetAllForward();
        Motor_SetLeftReverse();
    }
    Motor_SetAllSpeed(duty);
}

/* ===================================================================
 *  初始化
 * =================================================================== */

void Test_MotorEncoder_Init(void)
{
    Motor_PWM_Init();
    Encoder_Init();
    Telemetry_Init(TEST_TELEM_CHANNELS, TEST_TELEM_INTERVAL_MS);

    s_phase          = TEST_PHASE_STOP;
    s_phase_start_ms = millis();
    s_test_start_ms  = s_phase_start_ms;

    m_all_stop();
}

/* ===================================================================
 *  测试序列运行
 * =================================================================== */

void Test_MotorEncoder_Run(void)
{
    uint32_t now      = millis();
    uint32_t elapsed  = now - s_phase_start_ms;
    uint32_t phase_ms = (s_phase == TEST_PHASE_STOP)
                        ? TEST_STOP_DURATION_MS
                        : TEST_PHASE_DURATION_MS;

    /* 阶段切换 */
    if (elapsed >= phase_ms) {
        s_phase = (TestPhase_t)((s_phase + 1) % TEST_PHASE_COUNT);
        s_phase_start_ms = now;

        switch (s_phase) {
        case TEST_PHASE_STOP:
            m_all_stop();
            break;
        case TEST_PHASE_FWD_30:
        case TEST_PHASE_FWD_50:
        case TEST_PHASE_FWD_70:
            Motor_SetAllSpeed(s_phase == TEST_PHASE_FWD_30 ? 30
                            : s_phase == TEST_PHASE_FWD_50 ? 50 : 70);
            break;
        case TEST_PHASE_PIVOT_LEFT:
            m_pivot(-1, 50);
            break;
        case TEST_PHASE_PIVOT_RIGHT:
            m_pivot(1, 50);
            break;
        default:
            break;
        }
    }

    /* 编码器数据 */
    float rpm[ENCODER_COUNT];
    for (int i = 0; i < ENCODER_COUNT; i++) {
        if (!Encoder_GetSpeed((EncoderID_t)i,
                              (float)TEST_TELEM_INTERVAL_MS * 0.001f,
                              &rpm[i])) {
            rpm[i] = 0.0f;
        }
    }

    /* 遥测发送 */
    float ch[TEST_TELEM_CHANNELS];
    ch[0] = rpm[ENCODER_C1];  /* 右前 */
    ch[1] = rpm[ENCODER_C2];  /* 左前 */
    ch[2] = rpm[ENCODER_C3];  /* 右后 */
    ch[3] = rpm[ENCODER_C4];  /* 左后 */
    ch[4] = (float)s_phase;
    ch[5] = (float)((now - s_test_start_ms) / 1000.0f);

    Telemetry_Send(ch, TEST_TELEM_CHANNELS);
}
