/**
 * @file    hal_mpu6050.c
 * @brief   硬件抽象层实现 — I2C1 通信与 INT 中断
 *
 * 基于 MSPM0G3507 DriverLib 的 I2C 控制器模式实现。
 * 读写操作采用两段式传输：先写寄存器地址（TX），
 * 等待完成后发起读数据（RX），之间由 I2C STOP/START 分隔。
 *
 * SysConfig 与 HAL 的职责划分：
 *   SysConfig（SYSCFG_DL_I2C_1_init）→ 时钟源、模拟毛刺滤波、引脚 IOMUX
 *   本层（HAL_MPU6050_Init）         → 波特率、时钟延展、数字毛刺滤波、控制器使能
 */

#include "hal_mpu6050.h"

/* ============================================================================
 *  内部变量
 * ============================================================================ */

/** INT 引脚中断回调函数指针（静态分配，单例）
 *  ★ volatile：ISR (GROUP1_IRQHandler) 读取此变量，
 *    主循环 (HAL_MPU6050_SetIntCallback) 写入此变量 */
static volatile HAL_MPU_IntCallback_t s_int_callback = NULL;

/* ============================================================================
 *  内部辅助函数
 * ============================================================================ */

/**
 * @brief 等待 I2C 控制器进入 IDLE 状态
 * @param inst   I2C 实例指针
 * @param tout   最大等待循环次数
 * @return true = 在超时前进入 IDLE, false = 超时
 */
static bool I2C_WaitIdle(I2C_Regs *inst, uint32_t tout)
{
    while (tout--) {
        if (DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_IDLE) {
            return true;
        }
    }
    DL_I2C_resetControllerTransfer(inst);
    return false;
}

/**
 * @brief 等待 I2C 控制器完成当前传输（BUSY 清除）
 * @param inst   I2C 实例指针
 * @param tout   最大等待循环次数
 * @return true = 成功完成, false = 超时
 */
static bool I2C_WaitComplete(I2C_Regs *inst, uint32_t tout)
{
    while (tout--) {
        uint32_t st = DL_I2C_getControllerStatus(inst);
        if (st & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_resetControllerTransfer(inst);
            return false;
        }
        if (!(st & DL_I2C_CONTROLLER_STATUS_BUSY)) {
            return true;
        }
    }
    DL_I2C_resetControllerTransfer(inst);
    return false;
}

/* ============================================================================
 *  初始化
 * ============================================================================ */

void HAL_MPU6050_Init(void)
{
    /* ---- I2C1 控制器初始化 ----
     *
     * SysConfig 的 SYSCFG_DL_I2C_1_init() 已配置：
     *   - I2C 时钟源（BUSCLK）
     *   - 引脚 IOMUX 复用（PA16=I2C1_SDA, PA17=I2C1_SCL）
     * 本层在此之上使能控制器并设定通信速率。
     */

    /* 设定 I2C 时钟速率
     *   SCL = f_I2CCLK / (2 * (TPR + 4))
     *   f_I2CCLK = 32 MHz (BUSCLK, 分频 1)
     *   TPR = 36 → SCL ≈ 32M / (2 * 40) = 400 kHz (Fast Mode)
     */
    DL_I2C_setTimerPeriod(MPU_I2C_INST, 36);

    /* 使能时钟延展，MPU6050 在部分寄存器访问时需要。 */
    DL_I2C_enableControllerClockStretching(MPU_I2C_INST);

    /* 使能 I2C 控制器 */
    DL_I2C_enableController(MPU_I2C_INST);

    delay_cycles(160);  /* 等待控制器稳定 */

    /* ---- 数字毛刺滤波器 ----
     *
     * 在 PWM 电机噪声环境中，4 × BUSCLK 周期（≈125 ns @ 32 MHz）
     * 的毛刺滤波器可有效抑制电机换相尖峰，同时不误触 400 kHz
     * I2C 信号边沿（位周期 2500 ns）。
     *
     * 注意：SysConfig 不生成此配置。模拟毛刺滤波器（50 ns 脉宽）
     * 由 SYSCFG_DL_I2C_1_init() 自动配置，本层不重复处理。
     */
    DL_I2C_setDigitalGlitchFilterPulseWidth(MPU_I2C_INST,
        DL_I2C_DIGITAL_GLITCH_FILTER_WIDTH_CLOCKS_4);


    /* ---- INT 引脚初始化 ----
     *
     * PB24 IOMUX 和上拉：由 SysConfig (GPIO_MPU_INT) 管理
     * 中断极性 + NVIC：HAL 层手动控制
     */
    /* 配置中断极性：上升沿触发（INT: LOW→HIGH = 数据就绪） */
    DL_GPIO_setUpperPinsPolarity(MPU_INT_PORT, DL_GPIO_PIN_24_EDGE_RISE);

    /* 清除可能的遗留中断标志 */
    DL_GPIO_clearInterruptStatus(MPU_INT_PORT, MPU_INT_PIN);

    /* 使能 GPIOB 中断（NVIC 层级） */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/* ============================================================================
 *  寄存器写
 * ============================================================================ */

bool HAL_MPU6050_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };

    /* 清空 TX FIFO */
    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);

    /* 填充 TX FIFO */
    DL_I2C_fillControllerTXFIFO(MPU_I2C_INST, buf, 2);

    /* 启动传输：START + addr(W) + reg + data + STOP */
    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    return I2C_WaitComplete(MPU_I2C_INST, MPU_I2C_TIMEOUT);
}

/* ============================================================================
 *  寄存器读（单字节）
 * ============================================================================ */

bool HAL_MPU6050_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return HAL_MPU6050_ReadMulti(addr, reg, data, 1);
}

/* ============================================================================
 *  寄存器读（多字节）
 * ============================================================================ */

bool HAL_MPU6050_ReadMulti(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0) return false;

    /* ---- 阶段 1：写入寄存器地址（TX） ---- */

    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    DL_I2C_transmitControllerData(MPU_I2C_INST, reg);

    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!I2C_WaitComplete(MPU_I2C_INST, MPU_I2C_TIMEOUT / 2)) {
        return false;
    }

    /* 等待 IDLE */
    if (!I2C_WaitIdle(MPU_I2C_INST, MPU_I2C_TIMEOUT / 2)) {
        return false;
    }


    /* ---- 阶段 2：读取数据（RX） ---- */

    DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);

    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    /* 从 RX FIFO 中读取数据 */
    uint16_t received = 0;
    uint32_t timeout  = MPU_I2C_TIMEOUT;

    while (received < len) {
        uint32_t st = DL_I2C_getControllerStatus(MPU_I2C_INST);

        if (st & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_resetControllerTransfer(MPU_I2C_INST);
            return false;
        }

        if (!DL_I2C_isControllerRXFIFOEmpty(MPU_I2C_INST)) {
            buf[received++] = DL_I2C_receiveControllerData(MPU_I2C_INST);
            timeout = MPU_I2C_TIMEOUT; /* 重置超时（仍在接收） */
        } else {
            if (--timeout == 0) {
                DL_I2C_resetControllerTransfer(MPU_I2C_INST);
                return false;
            }
        }
    }

    /* 等待传输完全结束 */
    return I2C_WaitIdle(MPU_I2C_INST, MPU_I2C_TIMEOUT / 2);
}

/* ============================================================================
 *  中断回调注册
 * ============================================================================ */

void HAL_MPU6050_SetIntCallback(HAL_MPU_IntCallback_t cb)
{
    s_int_callback = cb;

    if (cb != NULL) {
        /* 清除标志 + 使能引脚中断 */
        DL_GPIO_clearInterruptStatus(MPU_INT_PORT, MPU_INT_PIN);
        DL_GPIO_enableInterrupt(MPU_INT_PORT, MPU_INT_PIN);
    } else {
        /* 取消注册时关闭中断 */
        DL_GPIO_disableInterrupt(MPU_INT_PORT, MPU_INT_PIN);
    }
}

/* ============================================================================
 *  GPIOB 中断处理程序（弱符号，用户可覆盖）
 * ============================================================================ */

SYSCONFIG_WEAK void GROUP1_IRQHandler(void)
{
    if (DL_GPIO_getEnabledInterruptStatus(MPU_INT_PORT, MPU_INT_PIN)) {
        DL_GPIO_clearInterruptStatus(MPU_INT_PORT, MPU_INT_PIN);

        if (s_int_callback != NULL) {
            s_int_callback();
        }
    }
}
