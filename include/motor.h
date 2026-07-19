#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/**
 * @brief 初始化并使能所有电机的 PWM 计数器
 * @note 必须在 SYSCFG_DL_init() 之后调用
 */
void Motor_PWM_Init(void);

/**
 * @brief 设置前右轮速度 (TIMG12.CCP1 → PA25)
 * @param duty 占空比百分比，范围：0 到 100
 */
void Set_Motor_FrontRight_Speed(uint8_t duty);

/**
 * @brief 设置后右轮速度 (TIMG7.CCP1 → PA24)
 * @param duty 占空比百分比，范围：0 到 100
 */
void Set_Motor_BackRight_Speed(uint8_t duty);

/**
 * @brief 设置后左轮速度 (TIMA1.CCP0 → PA28)
 * @param duty 占空比百分比，范围：0 到 100
 */
void Set_Motor_BackLeft_Speed(uint8_t duty);

/**
 * @brief 设置前左轮速度 (TIMA1.CCP1 → PB1)
 * @param duty 占空比百分比，范围：0 到 100
 */
void Set_Motor_FrontLeft_Speed(uint8_t duty);

/**
 * @brief 控制四轮电机的方向和速度
 * @param direction        方向：0 = 前进，1 = 后退，其他值 = 停止
 * @param FrontRight_Speed 前右轮占空比百分比，范围：0 到 100
 * @param BackRight_Speed  后右轮占空比百分比，范围：0 到 100
 * @param FrontLeft_Speed  前左轮占空比百分比，范围：0 到 100
 * @param BackLeft_Speed   后左轮占空比百分比，范围：0 到 100
 *
 * @note  宏定义已修正：函数参数名对应实际物理轮子
 */
void Motor_turn_up(uint8_t direction,
                   uint8_t FrontRight_Speed,
                   uint8_t BackRight_Speed,
                   uint8_t FrontLeft_Speed,
                   uint8_t BackLeft_Speed);

#endif
