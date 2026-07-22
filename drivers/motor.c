/**
 * @file    motor.c
 * @brief   四路直流电机 PWM + H 桥方向控制 —— 实现
 *
 * ======================== 整体设计 ========================
 *
 *   速度控制：
 *     - 右前/右后各用一路独立 TimerG（TIMG12/TIMG7）
 *     - 左前/左后共享一路 TimerA（TIMA1）的两个 CCP 通道
 *     - duty = 0~100 → compareValue = (period × duty) / 100
 *     - period = SysConfig 配置的 timerCount = 1600
 *
 *   方向控制（标准 IN1/IN2 接线，无交叉）：
 *     - Pin1=0, Pin2=1 → 前进
 *     - Pin1=1, Pin2=0 → 倒退
 *
 * ======================== 宏定义修正说明 ========================
 *
 *   ┌─────────────────────────────┬────────────┬──────────┬──────────┐
 *   │ 函数名                       │ 定时器      │ 引脚      │ 物理轮子  │
 *   ├─────────────────────────────┼────────────┼──────────┼──────────┤
 *   │ Set_Motor_FrontRight_Speed  │ TIMG12.CCP1│ PA25     │ 前右轮    │
 *   │ Set_Motor_BackRight_Speed   │ TIMG7.CCP1 │ PA24     │ 后右轮    │
 *   │ Set_Motor_BackLeft_Speed    │ TIMA1.CCP0 │ PA28     │ 后左轮    │
 *   │ Set_Motor_FrontLeft_Speed   │ TIMA1.CCP1 │ PB1      │ 前左轮    │
 *   └─────────────────────────────┴────────────┴──────────┴──────────┘
 */

#include "motor.h"
#include "ti_msp_dl_config.h"

/* ====================================================================
 *  初始化
 * ==================================================================== */

void Motor_PWM_Init(void)
{
    /* ---- 方向 GPIO：全部设为前进 IN1=0, IN2=1 ---- */
    Motor_SetAllForward();

    /* ---- 使能 PWM 定时器 ---- */
    DL_TimerG_startCounter(PWM_front_right_INST);  /* TIMG12 */
    DL_TimerG_startCounter(PWM_back_right_INST);   /* TIMG7  */
    DL_TimerA_startCounter(PWM_left_INST);         /* TIMA1  */
}

/* ====================================================================
 *  独立速度设置
 * ==================================================================== */

void Set_Motor_FrontRight_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerG_getLoadValue(PWM_front_right_INST);
    uint32_t cv = (period * duty) / 100;
    DL_TimerG_setCaptureCompareValue(PWM_front_right_INST, cv,
                                     DL_TIMER_CC_1_INDEX);
}

void Set_Motor_BackRight_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerG_getLoadValue(PWM_back_right_INST);
    uint32_t cv = (period * duty) / 100;
    DL_TimerG_setCaptureCompareValue(PWM_back_right_INST, cv,
                                     DL_TIMER_CC_1_INDEX);
}

void Set_Motor_BackLeft_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerA_getLoadValue(PWM_left_INST);
    uint32_t cv = (period * duty) / 100;
    DL_TimerA_setCaptureCompareValue(PWM_left_INST, cv,
                                     DL_TIMER_CC_0_INDEX);
}

void Set_Motor_FrontLeft_Speed(uint8_t duty)
{
    if (duty > 100) duty = 100;
    uint32_t period = DL_TimerA_getLoadValue(PWM_left_INST);
    uint32_t cv = (period * duty) / 100;
    DL_TimerA_setCaptureCompareValue(PWM_left_INST, cv,
                                     DL_TIMER_CC_1_INDEX);
}

/* ====================================================================
 *  四轮统一控制
 * ==================================================================== */

void Motor_turn_up(uint8_t direction,
                   uint8_t FrontRight_Speed,
                   uint8_t BackRight_Speed,
                   uint8_t FrontLeft_Speed,
                   uint8_t BackLeft_Speed)
{
    switch (direction) {
    case 0: /* 前进 */
        Motor_SetAllForward();
        break;
    case 1: /* 后退 */
        Motor_SetLeftReverse();
        Motor_SetRightReverse();
        break;
    default: /* 停止 */
        Motor_StopAll();
        return;
    }

    Set_Motor_FrontRight_Speed(FrontRight_Speed);
    Set_Motor_BackRight_Speed(BackRight_Speed);
    Set_Motor_FrontLeft_Speed(FrontLeft_Speed);
    Set_Motor_BackLeft_Speed(BackLeft_Speed);
}

/* ====================================================================
 *  方向控制（标准 IN1/IN2 接线，无交叉）
 *
 *  所有轮子统一使用：
 *    前进: IN1=0, IN2=1
 *    倒退: IN1=1, IN2=0
 * ==================================================================== */

void Motor_SetAllForward(void)
{
    /* 前右轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT,
                      GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT,
                      GPIO_MOTOR_PIN_front_right_2_PIN);
    /* 后右轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_1_PORT,
                      GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_2_PORT,
                      GPIO_MOTOR_PIN_back_right_2_PIN);
    /* 前左轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_1_PORT,
                      GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_2_PORT,
                      GPIO_MOTOR_PIN_front_left_2_PIN);
    /* 后左轮 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_1_PORT,
                      GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_2_PORT,
                      GPIO_MOTOR_PIN_back_left_2_PIN);
}

void Motor_SetLeftForward(void)
{
    /* 前左轮：IN1=0, IN2=1 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_1_PORT,
                      GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_2_PORT,
                      GPIO_MOTOR_PIN_front_left_2_PIN);
    /* 后左轮：IN1=0, IN2=1 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_1_PORT,
                      GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_2_PORT,
                      GPIO_MOTOR_PIN_back_left_2_PIN);
}

void Motor_SetRightForward(void)
{
    /* 前右轮：IN1=0, IN2=1 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_1_PORT,
                      GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_2_PORT,
                      GPIO_MOTOR_PIN_front_right_2_PIN);
    /* 后右轮：IN1=0, IN2=1 */
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_1_PORT,
                      GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_2_PORT,
                      GPIO_MOTOR_PIN_back_right_2_PIN);
}

void Motor_SetLeftReverse(void)
{
    /* 前左轮：IN1=1, IN2=0 */
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_left_1_PORT,
                      GPIO_MOTOR_PIN_front_left_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_left_2_PORT,
                      GPIO_MOTOR_PIN_front_left_2_PIN);
    /* 后左轮：IN1=1, IN2=0 */
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_left_1_PORT,
                      GPIO_MOTOR_PIN_back_left_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_left_2_PORT,
                      GPIO_MOTOR_PIN_back_left_2_PIN);
}

void Motor_SetRightReverse(void)
{
    /* 前右轮：IN1=1, IN2=0 */
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_front_right_1_PORT,
                      GPIO_MOTOR_PIN_front_right_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_front_right_2_PORT,
                      GPIO_MOTOR_PIN_front_right_2_PIN);
    /* 后右轮：IN1=1, IN2=0 */
    DL_GPIO_setPins  (GPIO_MOTOR_PIN_back_right_1_PORT,
                      GPIO_MOTOR_PIN_back_right_1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_back_right_2_PORT,
                      GPIO_MOTOR_PIN_back_right_2_PIN);
}

void Motor_StopAll(void)
{
    Set_Motor_FrontRight_Speed(0);
    Set_Motor_BackRight_Speed(0);
    Set_Motor_FrontLeft_Speed(0);
    Set_Motor_BackLeft_Speed(0);
    /* 不改变方向 GPIO，保留方向状态 */
}

void Motor_SetAllSpeed(uint8_t duty)
{
    Motor_SetAllForward();
    Set_Motor_FrontRight_Speed(duty);
    Set_Motor_BackRight_Speed(duty);
    Set_Motor_FrontLeft_Speed(duty);
    Set_Motor_BackLeft_Speed(duty);
}
