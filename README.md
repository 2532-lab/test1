# Test1 — MSPM0G3507 视觉云台追踪项目

基于 TI MSPM0G3507 (Cortex-M0+) 的双轴步进电机云台控制系统，
接收 K230 (立创·庐山派) 视觉模块的角度数据实现靶子自动追踪。

## 硬件平台

| 项目 | 说明 |
|------|------|
| MCU | MSPM0G3507 (ARM Cortex-M0+, 80MHz) |
| 开发板 | LP-MSPM0G3507 LaunchPad |
| 视觉模块 | K230 庐山派 + GC2093 摄像头 |
| 传感器 | MPU6050 六轴 + HMC5883L/QMC5883L 磁力计 |
| 执行器 | 二轴步进电机 (16细分, 3200 PPR) |
| 调试器 | XDS-110 (板载) |
| IDE | TI Code Composer Studio Theia |

## 工程结构

```
test1/
├── src/            # 应用层 (vision_tracking, gimbal_stabilizer, IMU)
├── drivers/        # 驱动层 (stepper_pulse, gimbal_control, I2C HAL)
├── include/        # 公共头文件
├── docs/           # 详细文档 (vision_tracking_guide.md)
├── openmv/         # 视觉模块参考代码
├── targetConfigs/  # CCS 调试配置文件
└── Debug/          # 编译输出（不入库）
```

## 视觉协议

K230 通过 UART (115200 8N1) 发送文本协议:
- 检测到靶子: `$TARGET,<水平角>,<垂直角>,<同心圆数>\r\n`
- 未检测到: `$TARGET,NONE\r\n`

详见 `target_protocol_doc[1].txt` (K230 端接口文档) 和
`docs/vision_tracking_guide.md` (MCU 端开发文档)。

## 关键特性

- K230 预解算角度 (arctan 透视投影模型)，MCU 直接使用
- PID 闭环增稳 + 9-DOF IMU 姿态融合
- 自动模式切换 (TRACK ↔ HOLD) + 超时保护 (500ms)
- 非阻塞 ISR 步数计数，50Hz 主循环
- 安全保护: 硬限幅、增量突变保护、同心圆数过滤

## 快速开始

1. 用 CCS Theia 打开本目录作为工作区工程
2. 在 SysConfig 中配置 UART (名称: cloud_uart_vision, 115200 8N1)
3. 编译: `Project → Build Project`
4. 接线: K230 GPIO11(TX) → MSPM0 RX; 步进电机驱动器; MPU6050
5. 上电: 驱动器 → MSPM0 → K230
6. 调试: `Run → Debug`（通过板载 XDS-110）
