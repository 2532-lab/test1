/**
 * @file    motor.h
 * @brief   四路直流电机 PWM 速度控制 + H 桥方向控制 —— API 声明
 *
 * ======================== 硬件平台 ========================
 *   MCU:      MSPM0G3507 (Cortex-M0+, 32MHz SYSOSC)
 *   电机驱动:  TB6612FNG / L298N（双 H 桥）
 *   电机型号:  JGB37-520 直流减速电机（带 AB 相编码器）
 *
 * ======================== PWM 通道分配 ========================
 *
 *   ┌──────────┬────────────────┬──────────┬───────────────┐
 *   │ 物理轮子  │ SysConfig 名    │ 定时器    │ 引脚           │
 *   ├──────────┼────────────────┼──────────┼───────────────┤
 *   │ 前右轮    │ PWM_front_right │ TIMG12   │ CCP1 → PA25   │
 *   │ 后右轮    │ PWM_back_right  │ TIMG7    │ CCP1 → PA24   │
 *   │ 前左轮    │ PWM_left        │ TIMA1    │ CCP1 → PB1    │
 *   │ 后左轮    │ PWM_left        │ TIMA1    │ CCP0 → PA28   │
 *   └──────────┴────────────────┴──────────┴───────────────┘
 *
 *   SysConfig 已配置 PWM 为 Edge-Align Up 模式，timerCount=1600。
 *
 * ======================== H 桥方向控制 ========================
 *
 *   每个电机两个 GPIO 控制 H 桥芯片的 IN1/IN2：
 *   ┌──────┬──────┬──────────┐
 *   │ IN1  │ IN2  │ 电机状态  │
 *   ├──────┼──────┼──────────┤
 *   │  0   │  0   │ 刹车      │
 *   │  0   │  1   │ 前进      │
 *   │  1   │  0   │ 倒退      │
 *   │  1   │  1   │ 刹车      │
 *   └──────┴──────┴──────────┘
 *
 *   方向 GPIO 分配（全部为标准接线，IN1/IN2 无交叉）：
 *   ┌──────────────┬────────────┬────────────┐
 *   │ 轮子          │ IN1 (Pin1) │ IN2 (Pin2) │
 *   ├──────────────┼────────────┼────────────┤
 *   │ 前右 front_r  │ PA27       │ PA26       │
 *   │ 后右 back_r   │ PB4        │ PA8        │
 *   │ 前左 front_l  │ PA7        │ PA1        │
 *   │ 后左 back_l   │ PB6        │ PB19       │
 *   └──────────────┴────────────┴────────────┘
 *
 *   。
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* ====================================================================
 *  PWM 速度控制
 * ==================================================================== */

void Motor_PWM_Init(void);
void Set_Motor_FrontRight_Speed(uint8_t duty);
void Set_Motor_BackRight_Speed(uint8_t duty);
void Set_Motor_BackLeft_Speed(uint8_t duty);
void Set_Motor_FrontLeft_Speed(uint8_t duty);

/**
 * @brief   四轮统一控制 —— 同时设定方向和四轮独立速度
 * @param   direction  0=前进, 1=后退, 其他=停止(PWM=0,方向不变)
 */
void Motor_turn_up(uint8_t direction,
                   uint8_t FrontRight_Speed,
                   uint8_t BackRight_Speed,
                   uint8_t FrontLeft_Speed,
                   uint8_t BackLeft_Speed);

/* ====================================================================
 *  方向控制（供循迹 pivot / 测试等场景使用）
 *
 *  为什么需要这些函数？
 *    Motor_turn_up 只能"全体前进"或"全体后退"。
 *    PIVOT 原地旋转需要一侧前进、另一侧倒退，
 *    必须直接控制方向 GPIO 来实现差动方向。
 *
 *  用法示例（右转 pivot）：
 *    Motor_SetAllForward();        // 全部设为前进方向
 *    Motor_SetRightReverse();      // 右侧覆盖为倒退
 *    Motor_SetAllSpeed(65);        // 四轮同速
 *    → 右侧倒退 + 左侧前进 = 车体右转
 * ==================================================================== */

/**
 * @brief   全部四轮设为前进方向 (IN1=0, IN2=1)
 */
void Motor_SetAllForward(void);

/**
 * @brief   左侧两轮独立设为前进方向 (IN1=0, IN2=1)
 *          前左 + 后左，不影响右侧。用于速度闭环中正转输出。
 */
void Motor_SetLeftForward(void);

/**
 * @brief   右侧两轮独立设为前进方向 (IN1=0, IN2=1)
 *          前右 + 后右，不影响左侧。用于速度闭环中正转输出。
 */
void Motor_SetRightForward(void);

/**
 * @brief   左侧两轮设为倒退方向 (IN1=1, IN2=0)
 *          前左 + 后左，配合 Motor_SetAllForward() 后调用实现左转 pivot
 */
void Motor_SetLeftReverse(void);

/**
 * @brief   右侧两轮设为倒退方向 (IN1=1, IN2=0)
 *          前右 + 后右，配合 Motor_SetAllForward() 后调用实现右转 pivot
 */
void Motor_SetRightReverse(void);

/**
 * @brief   全部四轮 PWM 清零（不改变方向 GPIO）
 */
void Motor_StopAll(void);

/**
 * @brief   全部四轮设相同速度 + 前进方向
 *          等价于 Motor_SetAllForward() + 四路 Set_xxx_Speed(duty)
 */
void Motor_SetAllSpeed(uint8_t duty);

#endif /* __MOTOR_H */
