
/**
 * @file    empty.c
 * @brief   双轴步进电机小车主程序
 *
 * 本文件是 MSPM0G3507 工程的主入口，负责：
 *   - 系统初始化（时钟、外设、SysTick）
 *   - 步进电机脉冲输出初始化
 *   - MPU6050 六轴传感器的初始化和校准
 *   - 主循环：以 200 Hz 频率读取 IMU 数据并更新小车控制
 *
 * 硬件平台：MSPM0G3507 (Arm Cortex-M0+)
 * 传感器：  MPU6050 (I2C 接口)
 * 执行器：  两个步进电机（双轴 XY 平台 / 两轮差速小车）
 */

/* ---- 系统头文件 ---- */
#include "ti_msp_dl_config.h"   /* TI MSPM0 DriverLib 自动生成的系统配置头文件 */

#include <stdbool.h>            /* bool 类型定义 */
#include <stdint.h>             /* uint32_t、int32_t 等定宽整数类型 */

/* ---- 应用层头文件 ---- */
#include "app_imu.h"            /* MPU6050 IMU 数据读取与姿态解算 */
#include "car_control.h"        /* 小车运动控制（速度/转角 → 电机脉冲） */

/* ---- 驱动层头文件 ---- */
#include "drivers/delay.h"          /* 微秒/毫秒级延时函数 */
#include "drivers/stepper_pulse.h"  /* 步进电机脉冲输出（TIMG 定时器 + DMA） */

/* ================================================================
 * 全局变量
 * ================================================================ */
/**
 * @brief 系统毫秒计数器
 *
 * 由 SysTick 中断每 1 ms 自增一次，millis() 读取它来实现非阻塞延时和定时。
 * volatile 确保编译器每次都从内存重新读取，不会被优化掉。
 */
volatile uint32_t sys_tick_ms = 0;

/* ================================================================
 * 中断服务函数
 * ================================================================ */

/**
 * @brief Cortex-M SysTick 定时器中断处理函数
 *
 * 每 1 ms 触发一次，仅递增 sys_tick_ms 全局计数器。
 * 配置点在 main() 中调用 SysTick_Config(CPUCLK_FREQ / 1000U)。
 */
void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/**
 * @brief 初始化 MPU6050 传感器
 *
 * 执行步骤：
 *   1. 调用 IMU_Init() 完成器件探测和寄存器配置。
 *   2. 陀螺仪零偏校准（采样 200 次取平均），期间小车需保持静止。
 *   3. 重置偏航角为 0。
 *
 * @return true  初始化成功
 * @return false IMU 未连接或初始化失败
 */
static bool init_imu(void)
{
    /* 尝试初始化 MPU6050，若失败（如 I2C 无应答）则直接返回 false */
    if (!IMU_Init()) return false;

    /* 陀螺仪零偏校准期间保持小车停止。 */
    /* 采集 200 个陀螺仪样本对其取平均，得到静态零偏值并写入寄存器 */
    IMU_CalibrateGyro(200);

    /* 将当前偏航角归零，作为后续角度控制的参考原点 */
    IMU_ResetYaw();
    return true;
}

/**
 * @brief 让左右车轮同时向前转指定圈数，然后自动停止。
 *
 * 这里使用 stepper_pulse_move() 的位置模式，而不是持续运行的速度模式：
 *   1. 根据目标步数输出 STEP 脉冲；
 *   2. 定时器中断在后台统计脉冲数；
 *   3. 达到目标步数后自动停止对应定时器。
 *
 * 当前 STEPPER_PULSES_PER_REV 为 3200，十圈需要 32000 个脉冲。
 * 该值必须与步进驱动器的实际细分设置保持一致。
 */
static void start_forward_ten_turns(void)
{
    const uint32_t total_steps =
        (uint32_t)CAR_FORWARD_TURNS * (uint32_t)STEPPER_PULSES_PER_REV;

    /*
     * 左右电机镜像安装，向前运动所需的 DIR 电平通常相反。
     * 下面两个方向宏已经在 car_control.h 中按当前接线配置好。
     */
    stepper_pulse_move(STEPPER_AXIS_LEFT,
                       CAR_LEFT_FORWARD_DIR,
                       CAR_FORWARD_RPM,
                       total_steps);
    stepper_pulse_move(STEPPER_AXIS_RIGHT,
                       CAR_RIGHT_FORWARD_DIR,
                       CAR_FORWARD_RPM,
                       total_steps);
}

/* ================================================================
 * 主函数
 * ================================================================ */

/**
 * @brief 程序入口
 *
 * 主循环以固定的 200 Hz（每 5 ms）运行：
 *   - 读取 MPU6050 数据（加速度 + 陀螺仪）
 *   - 姿态解算（互补滤波 / DMP，在 app_imu 中实现）
 *   - 更新小车控制输出（car_control 中速度/转角 → 电机脉冲频率）
 *
 * 如果 IMU 断开连接，主循环继续尝试每 1 秒重连一次，
 * 断开期间不驱动电机（Car_Update(NULL, false)），保证安全。
 */
int main(void)
{
    IMU_Data_t imu = {0};   /* IMU 数据结构体，存放加速度/角速度/姿态角 */
    bool imu_ready;         /* IMU 是否就绪的标志 */
    uint32_t next_update;   /* 下一次执行控制更新的时间戳 */
    uint32_t next_retry;    /* 下一次 IMU 重连尝试的时间戳 */

    /* ---- 系统初始化 ---- */
    SYSCFG_DL_init();                           /* 初始化时钟树、GPIO、I2C 等外设 */
    SysTick_Config(CPUCLK_FREQ / 1000U);       /* 配置 SysTick 每 1 ms 产生一次中断 */

    /* ---- 外设初始化 ---- */
    stepper_pulse_init();   /* 初始化两个步进电机的 TIMG 定时器和 DMA 通道 */
    Car_Init();             /* 初始化小车控制参数（默认速度 0，停止状态） */

    /* ---- IMU 初始化 ---- */
    /* 默认速度指令为停止；下面通过位置模式单独启动十圈前进行驶。 */
    imu_ready = init_imu();         /* 尝试初始化 IMU，成功则 imu_ready = true */

    /* MPU6050 初始化成功后，只启动一次十圈前进任务。 */
    if (imu_ready) {
        start_forward_ten_turns();
    }

    next_update = millis();         /* 记录当前毫秒数作为第一次更新的基准 */
    next_retry = next_update + 1000U; /* 若初始化失败，1 秒后开始重试 */

    /* ---- 主循环 ---- */
    while(1){
        const uint32_t now = millis();    /* 获取当前系统毫秒时间戳 */

        /* -- 控制更新（200 Hz，即每 5 ms 执行一次） -- */
        if ((int32_t)(now - next_update) >= 0) {
            next_update += 5U;    /* 将下次更新时间设为 5 ms 之后 */

            if (imu_ready) {
                /* IMU 正常：读取传感器数据并传给控制模块 */
                const bool sample_ok = IMU_Update(&imu);    /* 触发 I2C 读取 + 姿态解算 */
                Car_Update(&imu, sample_ok);                /* 根据姿态更新电机输出 */
            } else {
                /* IMU 不可用：传入 NULL，控制模块应停止所有电机输出 */
                Car_Update(NULL, false);
            }
        }

        /* -- IMU 自动重连（每 1 秒尝试一次） -- */
        /* 允许断开连接的 MPU6050 自动恢复，但恢复期间不驱动车轮。 */
        if (!imu_ready && (int32_t)(now - next_retry) >= 0) {
            next_retry += 1000U;        /* 将下次重试时间设为 1 秒之后 */
            imu_ready = init_imu();     /* 重新尝试初始化 IMU */
        }
    }
}
