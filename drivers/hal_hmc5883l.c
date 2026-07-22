/**
 * @file    hal_hmc5883l.c
 * @brief   硬件抽象层实现 — I2C1 通信与 DRDY 引脚
 *
 * 与 MPU6050 共享 I2C1 总线，I2C 读写逻辑镜像 hal_mpu6050.c 的轮询模式。
 * 不重新初始化 I2C1——时钟速率(TPR=36→400kHz)、时钟延展、毛刺滤波器已在
 * HAL_MPU6050_Init() 中设置。
 */

#include "hal_hmc5883l.h"

/* ============================================================================
 *  内部辅助函数（镜像 hal_mpu6050.c 同名函数）
 * ============================================================================ */

/** 等待 I2C 控制器进入 IDLE 状态 */
static bool I2C_WaitIdle(uint32_t tout)
{
    while (tout--) {
        if (DL_I2C_getControllerStatus(HMC_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) {
            return true;
        }
    }
    DL_I2C_resetControllerTransfer(HMC_I2C_INST);
    return false;
}

/** 等待 I2C 控制器完成当前传输 */
static bool I2C_WaitComplete(uint32_t tout)
{
    while (tout--) {
        uint32_t st = DL_I2C_getControllerStatus(HMC_I2C_INST);
        if (st & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_resetControllerTransfer(HMC_I2C_INST);
            return false;
        }
        if (!(st & DL_I2C_CONTROLLER_STATUS_BUSY)) {
            return true;
        }
    }
    DL_I2C_resetControllerTransfer(HMC_I2C_INST);
    return false;
}

/* ============================================================================
 *  初始化
 * ============================================================================ */

void HAL_HMC5883L_Init(void)
{
    /*
     * 注意：不重新初始化 I2C1。SYSCFG_DL_init() + HAL_MPU6050_Init()
     * 已将 I2C1 配置为：
     *   - TPR=36 → SCL=400kHz Fast Mode
     *   - 时钟延展已使能
     *   - 数字毛刺滤波器 4 个 BUSCLK 周期
     *   - 引脚 IOMUX：PA16=SDA, PA17=SCL
     */

    /* ---- DRDY 引脚（PA13）确认为数字输入 ----
     * DRDY 是 HMC5883L 的推挽输出，MSPM0 复位后默认即为高阻输入。
     * 显式调用 disableOutput 防止 PA13 被其他代码误配为输出。 */
    DL_GPIO_disableOutput(HMC_DRDY_PORT, HMC_DRDY_PIN);
}

/* ============================================================================
 *  寄存器写
 * ============================================================================ */

bool HAL_HMC5883L_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };

    DL_I2C_flushControllerTXFIFO(HMC_I2C_INST);
    DL_I2C_fillControllerTXFIFO(HMC_I2C_INST, buf, 2);

    DL_I2C_startControllerTransfer(HMC_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    return I2C_WaitComplete(HMC_I2C_TIMEOUT);
}

/* ============================================================================
 *  寄存器读（单字节）
 * ============================================================================ */

bool HAL_HMC5883L_ReadReg(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return HAL_HMC5883L_ReadMulti(addr, reg, data, 1);
}

/* ============================================================================
 *  寄存器读（多字节）
 * ============================================================================ */

bool HAL_HMC5883L_ReadMulti(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0) return false;

    /* ---- 阶段 1：写入寄存器地址（TX） ---- */

    DL_I2C_flushControllerTXFIFO(HMC_I2C_INST);
    DL_I2C_transmitControllerData(HMC_I2C_INST, reg);

    DL_I2C_startControllerTransfer(HMC_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    if (!I2C_WaitComplete(HMC_I2C_TIMEOUT / 2)) {
        return false;
    }

    if (!I2C_WaitIdle(HMC_I2C_TIMEOUT / 2)) {
        return false;
    }

    /* ---- 阶段 2：读取数据（RX） ---- */

    DL_I2C_flushControllerRXFIFO(HMC_I2C_INST);

    DL_I2C_startControllerTransfer(HMC_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    uint16_t received = 0;
    uint32_t timeout  = HMC_I2C_TIMEOUT;

    while (received < len) {
        uint32_t st = DL_I2C_getControllerStatus(HMC_I2C_INST);

        if (st & DL_I2C_CONTROLLER_STATUS_ERROR) {
            DL_I2C_resetControllerTransfer(HMC_I2C_INST);
            return false;
        }

        if (!DL_I2C_isControllerRXFIFOEmpty(HMC_I2C_INST)) {
            buf[received++] = DL_I2C_receiveControllerData(HMC_I2C_INST);
            timeout = HMC_I2C_TIMEOUT;
        } else {
            if (--timeout == 0) {
                DL_I2C_resetControllerTransfer(HMC_I2C_INST);
                return false;
            }
        }
    }

    return I2C_WaitIdle(HMC_I2C_TIMEOUT / 2);
}

/* ============================================================================
 *  DRDY 引脚轮询
 * ============================================================================ */

bool HAL_HMC5883L_IsDataReady(void)
{
    return DL_GPIO_readPins(HMC_DRDY_PORT, HMC_DRDY_PIN) != 0;
}
