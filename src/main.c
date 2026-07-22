/**
 * @file    main.c
 * @brief   灰度循迹测试
 *
 * 传感器：五路 TCRT5000 灰度 → 位置偏差 → P 控制 → 四轮差速 PWM
 * 编码器：四路 AB 相（C1 硬件 QEI + C2/C3/C4 软件 1kHz 扫描）
 * 电机：  TB6612FNG 双 H 桥，四路 PWM
 * 调试：  UART_0 → L1306 → 蓝牙 → PC 串口助手
 */

#include "ti_msp_dl_config.h"
#include "line_follower.h"
#include "motor.h"
#include "encoder.h"
#include "bluetooth_uart.h"
#include "delay.h"
#include <string.h>

/* ===================================================================
 *  格式化输出循迹状态到 PC
 * =================================================================== */

static void print_status(const LineFollower_t *lf, float rpm_l, float rpm_r)
{
    char buf[80];
    int  len = 0;

    /* 位置 */
    const char *pos_str = "POS=";
    while (*pos_str) buf[len++] = *pos_str++;

    int16_t p = lf->position;
    if (p < 0) { buf[len++] = '-'; p = -p; }
    if (p >= 1000) buf[len++] = '0' + (p / 1000) % 10;
    buf[len++] = '0' + (p / 100) % 10;
    buf[len++] = '0' + (p / 10) % 10;
    buf[len++] = '0' + (p % 10);

    buf[len++] = ' ';
    buf[len++] = 'L';
    buf[len++] = 'D';
    buf[len++] = '=';
    buf[len++] = '0' + (lf->left_duty / 10) % 10;
    buf[len++] = '0' + (lf->left_duty % 10);

    buf[len++] = ' ';
    buf[len++] = 'R';
    buf[len++] = 'D';
    buf[len++] = '=';
    buf[len++] = '0' + (lf->right_duty / 10) % 10;
    buf[len++] = '0' + (lf->right_duty % 10);

    /* 丢线状态 */
    if (lf->line_lost) {
        buf[len++] = ' ';
        buf[len++] = 'L';
        buf[len++] = 'O';
        buf[len++] = 'S';
        buf[len++] = 'T';
    }

    /* 直角弯 */
    if (lf->turn_state == LF_TURN_PIVOT) {
        buf[len++] = ' ';
        buf[len++] = 'P';
        buf[len++] = 'I';
        buf[len++] = 'V';
    }

    buf[len++] = '\r';
    buf[len++] = '\n';
    buf[len] = '\0';

    BluetoothUART_SendString(buf);
}

/* ===================================================================
 *  主函数
 * =================================================================== */

int main(void)
{
    /* 1. 外设初始化 */
    SYSCFG_DL_init();

    /* 2. 编码器初始化（含 SysTick 1kHz） */
    Encoder_Init();

    /* 3. 电机 PWM 初始化 */
    Motor_PWM_Init();
    Motor_SetAllForward();

    /* 4. 蓝牙调试串口 */
    BluetoothUART_Init();

    BluetoothUART_SendString(
        "=== Line Follower Test ===\r\n"
        "POS  LD RD STATUS\r\n"
        "=======================\r\n\r\n");

    /* 5. 循迹器初始化 */
    LineFollower_t lf;
    LineFollower_Init(&lf, NULL);  /* 使用默认参数 */

    /* 6. 主循环 */
    while (1) {

  
  
        
      
        /* 获取编码器 RPM（C1=右前, C2=左前） */
        float rpm_r = 0, rpm_l = 0;
     #if 0  
        Encoder_GetSpeed(ENCODER_C1, 0.01f, &rpm_r);
        Encoder_GetSpeed(ENCODER_C2, 0.01f, &rpm_l);
#endif
        /* 执行一次循迹更新 */
        LineFollower_Update(&lf, rpm_l, rpm_r);
#if 0
        /* 每秒打印一次状态 */
        static uint32_t last_ms = 0;
        if (millis() - last_ms > 500) {
            print_status(&lf, rpm_l, rpm_r);
            last_ms = millis();
        }
#endif
        delay_ms(2);  /* ~500Hz 控制频率 */
    }
}

