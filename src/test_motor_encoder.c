/**
 * @file    test_motor_encoder.c
 * @brief   电机 + 编码器联合测试 —— 修正电机宏定义后版本
 *
 * 测试目的：
 *   1. 验证四路电机 PWM → 实际转速的线性度
 *   2. 验证四路编码器方向正确性（前进时 RPM 全为正）
 *   3. 验证 pivot 转向时两侧轮子转速对称性
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
 *  方向控制辅助（宏定义已修正：函数名 = 物理轮子）
 * =================================================================== */

static void mdir_all_fwd(void)
{
    /* 右前轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT, GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT, GPIO_MOTOR_PIN_front_right_2_PIN);
    /* 右后轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_1_PORT,  GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_2_PORT,  GPIO_MOTOR_PIN_back_right_2_PIN);
    /* 左前轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_1_PORT,  GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_2_PORT,  GPIO_MOTOR_PIN_front_left_2_PIN);
    /* 左后轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_1_PORT,   GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_2_PORT,   GPIO_MOTOR_PIN_back_left_2_PIN);
}

/** 左侧两轮倒退（左转 pivot 时使用） */
static void mdir_left_rev(void)
{
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_1_PORT, GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_2_PORT, GPIO_MOTOR_PIN_front_left_2_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_1_PORT,  GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_2_PORT,  GPIO_MOTOR_PIN_back_left_2_PIN);
}

/** 右侧两轮倒退（右转 pivot 时使用） */
static void mdir_right_rev(void)
{
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT, GPIO_MOTOR_PIN_front_right_2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT, GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_1_PORT,  GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_2_PORT,  GPIO_MOTOR_PIN_back_right_2_PIN);
}

static void m_pwm_zero(void)
{
    Set_Motor_FrontLeft_Speed(0);
    Set_Motor_BackLeft_Speed(0);
    Set_Motor_FrontRight_Speed(0);
    Set_Motor_BackRight_Speed(0);
}

static void m_all_stop(void)
{
    m_pwm_zero();
    mdir_all_fwd();
}

static void m_all_fwd(uint8_t duty)
{
    mdir_all_fwd();
    Set_Motor_FrontLeft_Speed(duty);
    Set_Motor_BackLeft_Speed(duty);
    Set_Motor_FrontRight_Speed(duty);
    Set_Motor_BackRight_Speed(duty);
}

/** dir>0 右转(右侧倒退+左侧前进), dir<0 左转(左侧倒退+右侧前进) */
static void m_pivot(int8_t dir, uint8_t duty)
{
    if (dir > 0) {
        mdir_all_fwd();
        mdir_right_rev();
    } else {
        mdir_all_fwd();
        mdir_left_rev();
    }
    Set_Motor_FrontLeft_Speed(duty);
    Set_Motor_BackLeft_Speed(duty);
    Set_Motor_FrontRight_Speed(duty);
    Set_Motor_BackRight_Speed(duty);
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

    if (elapsed >= phase_ms) {
        s_phase = (TestPhase_t)((s_phase + 1) % TEST_PHASE_COUNT);
        s_phase_start_ms = now;

        switch (s_phase) {
        case TEST_PHASE_STOP:        m_all_stop();     break;
        case TEST_PHASE_FWD_30:      m_all_fwd(30);    break;
        case TEST_PHASE_FWD_50:      m_all_fwd(50);    break;
        case TEST_PHASE_FWD_70:      m_all_fwd(70);    break;
        case TEST_PHASE_PIVOT_LEFT:  m_pivot(-1, 50);  break;
        case TEST_PHASE_PIVOT_RIGHT: m_pivot(1, 50);   break;
        default: break;
        }
    }

    /* 读取编码器转速 */
    float rpm[ENCODER_COUNT];
    for (int i = 0; i < ENCODER_COUNT; i++) {
        if (!Encoder_GetSpeed((EncoderID_t)i,
                              (float)TEST_TELEM_INTERVAL_MS * 0.001f,
                              &rpm[i])) {
            rpm[i] = 0.0f;
        }
    }

    /* 发送 JustFloat 数据 */
    float ch[TEST_TELEM_CHANNELS];
    ch[0] = rpm[ENCODER_C1];
    ch[1] = rpm[ENCODER_C2];
    ch[2] = rpm[ENCODER_C3];
    ch[3] = rpm[ENCODER_C4];
    ch[4] = (float)s_phase;
    ch[5] = (float)((now - s_test_start_ms) / 1000.0f);

    Telemetry_Send(ch, TEST_TELEM_CHANNELS);
}
