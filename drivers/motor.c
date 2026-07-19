/**
 * @file    motor.c
 * @brief   四轮电机 PWM + 方向控制（宏定义已修正：函数名匹配物理轮子）
 *
 * == GPIO 方向引脚映射 ==
 *   PIN_front_right_1/2 → 物理前右轮 H 桥
 *   PIN_back_right_1/2  → 物理后右轮 H 桥
 *   PIN_front_left_1/2  → 物理前左轮 H 桥
 *   PIN_back_left_1/2   → 物理后左轮 H 桥
 *
 * == PWM → 物理轮子映射（已修正） ==
 *   TIMG12.CCP1 (PA25) → Set_Motor_FrontRight_Speed → 前右轮
 *   TIMG7.CCP1  (PA24) → Set_Motor_BackRight_Speed  → 后右轮
 *   TIMA1.CCP0  (PA28) → Set_Motor_BackLeft_Speed   → 后左轮
 *   TIMA1.CCP1  (PB1)  → Set_Motor_FrontLeft_Speed  → 前左轮
 *
 *   H 桥方向约定：Pin1=0 Pin2=1 → 前进 / Pin1=1 Pin2=0 → 倒退
 */

#include "motor.h"
#include "ti_msp_dl_config.h"

/* ============================================================================
 *  初始化
 * ============================================================================ */

void Motor_PWM_Init(void)
{
    /* ---- 方向 GPIO：全部设为前进 (Pin1=0, Pin2=1) ---- */
    /* 前右轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT, GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT, GPIO_MOTOR_PIN_front_right_2_PIN);
    /* 后右轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_1_PORT,  GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_2_PORT,  GPIO_MOTOR_PIN_back_right_2_PIN);
    /* 前左轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_1_PORT,  GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_2_PORT,  GPIO_MOTOR_PIN_front_left_2_PIN);
    /* 后左轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_1_PORT,   GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_2_PORT,   GPIO_MOTOR_PIN_back_left_2_PIN);

    /* ---- 使能 PWM 定时器 ---- */
    DL_TimerG_startCounter(PWM_front_right_INST);
    DL_TimerG_startCounter(PWM_back_right_INST);
    DL_TimerA_startCounter(PWM_left_INST);
}

/* ============================================================================
 *  PWM 速度设置（函数名 = 物理轮子）
 * ============================================================================ */

void Set_Motor_FrontRight_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerG_getLoadValue(PWM_front_right_INST);
    uint32_t compareValue = (period * duty) / 100;
    DL_TimerG_setCaptureCompareValue(PWM_front_right_INST, compareValue, DL_TIMER_CC_1_INDEX);
}

void Set_Motor_BackRight_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerG_getLoadValue(PWM_back_right_INST);
    uint32_t compareValue = (period * duty) / 100;
    DL_TimerG_setCaptureCompareValue(PWM_back_right_INST, compareValue, DL_TIMER_CC_1_INDEX);
}

/* 后左轮：TIMA1.CCP0 = PA28 */
void Set_Motor_BackLeft_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerA_getLoadValue(PWM_left_INST);
    uint32_t compareValue = (period * duty) / 100;
    DL_TimerA_setCaptureCompareValue(PWM_left_INST, compareValue, DL_TIMER_CC_0_INDEX);
}

/* 前左轮：TIMA1.CCP1 = PB1 */
void Set_Motor_FrontLeft_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerA_getLoadValue(PWM_left_INST);
    uint32_t compareValue = (period * duty) / 100;
    DL_TimerA_setCaptureCompareValue(PWM_left_INST, compareValue, DL_TIMER_CC_1_INDEX);
}

/* ============================================================================
 *  四轮综合控制
 * ============================================================================ */

void Motor_turn_up(uint8_t direction,
                   uint8_t FrontRight_Speed,
                   uint8_t BackRight_Speed,
                   uint8_t FrontLeft_Speed,
                   uint8_t BackLeft_Speed)
{
    switch (direction) {
    case 0: /* 前进——方向已在 Motor_PWM_Init 中设置 */
        Set_Motor_FrontRight_Speed(FrontRight_Speed);
        Set_Motor_BackRight_Speed(BackRight_Speed);
        Set_Motor_FrontLeft_Speed(FrontLeft_Speed);
        Set_Motor_BackLeft_Speed(BackLeft_Speed);
        break;

    case 1: /* 后退——翻转全部方向 */
        /* 前右轮倒退 */
        DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_1_PORT, GPIO_MOTOR_PIN_front_right_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_2_PORT, GPIO_MOTOR_PIN_front_right_2_PIN);
        /* 后右轮倒退 */
        DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_1_PORT,  GPIO_MOTOR_PIN_back_right_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_2_PORT,  GPIO_MOTOR_PIN_back_right_2_PIN);
        /* 前左轮倒退 */
        DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_1_PORT,  GPIO_MOTOR_PIN_front_left_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_2_PORT,  GPIO_MOTOR_PIN_front_left_2_PIN);
        /* 后左轮倒退 */
        DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_1_PORT,   GPIO_MOTOR_PIN_back_left_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_2_PORT,   GPIO_MOTOR_PIN_back_left_2_PIN);

        Set_Motor_FrontRight_Speed(FrontRight_Speed);
        Set_Motor_BackRight_Speed(BackRight_Speed);
        Set_Motor_FrontLeft_Speed(FrontLeft_Speed);
        Set_Motor_BackLeft_Speed(BackLeft_Speed);
        break;

    default: /* 无效方向 → 全部停止 */
        Set_Motor_FrontRight_Speed(0);
        Set_Motor_BackRight_Speed(0);
        Set_Motor_FrontLeft_Speed(0);
        Set_Motor_BackLeft_Speed(0);
        break;
    }
}
