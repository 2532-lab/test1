# Test1 — 视觉-云台坐标对接工程文档 (K230 文本协议 v18)

## 目录

1. [工程概述](#1-工程概述)
2. [硬件平台与引脚](#2-硬件平台与引脚)
3. [工程架构](#3-工程架构)
4. [K230 视觉协议 (Target Protocol v18)](#4-k230-视觉协议)
5. [坐标系变换](#5-坐标系变换)
6. [工作模式详解](#6-工作模式详解)
7. [核心函数 API 参考](#7-核心函数-api-参考)
8. [快速启动指南](#8-快速启动指南)
9. [调参指南](#9-调参指南)
10. [故障排查](#10-故障排查)

---

## 1. 工程概述

本工程实现**K230 视觉模块与云台的坐标对接**——基于 **Camera-on-Gimbal（眼在手）** 机械结构。

### 系统拓扑

```
[K230 庐山派]                    [MSPM0G3507]
     │                                │
  GC2093 摄像头                      UART RX
  红色同心圆检测                       │
  靶心定位 (像素→角度 arctan)    解析 $TARGET 文本协议
  EMA 时间滤波                        │
  UART2 115200 输出              PID 闭环增稳
     │                                │
     └──── $TARGET,h,v,n\r\n ────→ 步进电机云台跟踪
```

### 机械结构

```
         ┌──────────────────────┐
         │  摄像头 + 激光笔       │  ← 固定在 TILT 支架最上层
         │  (并排同轴安装)         │     两者随 TILT 电机同步俯仰
         └──────────┬───────────┘
                    │         TILT 电机 — 控制垂直俯仰扫描
         ┌──────────┴───────────┐
         │     PAN 电机          │  ← 下层基座 — 控制水平旋转
         └──────────────────────┘
```

**关键特性**:
- 摄像头固定在云台上 (eye-in-hand) → K230 发送的偏角可直接作为云台修正量
- 激光笔与摄像头同轴 → 光斑=光轴指向, 可直接标定零点
- IMU (MPU6050 + 磁力计) 固定在 TILT 支架上 → 感知云台末端真实姿态
- ★ K230 已完成角度解算 (arctan 透视投影模型)，MCU 无需像素→角度换算

### 核心特性

| 特性 | 说明 |
|:---|:---|
| ★ K230 文本协议 | `$TARGET,<水平角>,<垂直角>,<同心圆数>\r\n` 纯 ASCII，简单可靠 |
| 自动模式切换 | K230 数据到达→TRACK / NONE→HOLD / 超时 500ms→HOLD |
| 9-DOF 姿态融合 | 互补滤波 Roll/Pitch + 磁力计修正 Yaw (无漂移) |
| PID 闭环增稳 | 位置式 PID + 积分分离 + 抗饱和 + 死区 + 增量突变保护 |
| 非阻塞架构 | ISR 自动步数计数, 主循环 50Hz 追踪, CPU 空闲时休眠 |
| 磁力计自动鉴别 | 先探 HMC5883L (0x1E), 失败则探 QMC5883L (0x0D) |
| 安全保护 | 超时自动 HOLD、模式延迟切换、机械硬限幅、增量突变保护、同心圆数过滤 |

---

## 2. 硬件平台与引脚

### MCU

| 项目 | 说明 |
|:---|:---|
| 型号 | MSPM0G3507 (ARM Cortex-M0+, 80MHz) |
| 开发板 | LP-MSPM0G3507 LaunchPad |
| IDE | TI Code Composer Studio Theia |
| SDK | MSPM0 SDK |
| 配置工具 | SysConfig |

### 引脚分配

| 外设 | 引脚 | 功能 |
|:---|:---|:---|
| TIMA0 CCP2 | PB0 | TILT 俯仰轴 PWM 脉冲 |
| TILT DIR | PB13 | TILT 俯仰轴方向 |
| TIMG6 CCP0 | PB2 | PAN 水平轴 PWM 脉冲 |
| PAN DIR | PB3 | PAN 水平轴方向 |
| I2C1 SCL | PA17 | MPU6050 + 磁力计 时钟 |
| I2C1 SDA | PA16 | MPU6050 + 磁力计 数据 |
| UART RX | (SysConfig配置) | 视觉上位机数据接收 |
| MPU6050 INT | PB24 | IMU 数据就绪中断 |

### 步进电机参数

| 参数 | 值 |
|:---|:---|
| 细分 | 16 微步 |
| 每转脉冲数 (PPR) | 3200 |
| 单步角度 | 0.1125° |
| 最大 RPM | 3000 |
| 最小 RPM | 30 |

---

## 3. 工程架构

### 目录结构

```
test1/
├── src/                        # 应用层
│   ├── vision_tracking.h       # 视觉追踪模块头文件
│   ├── vision_tracking.c       # 视觉追踪模块实现 (★ 核心新增)
│   ├── vision_tracking_main.c  # 主程序 (★ 入口)
│   ├── gimbal_stabilizer.h     # 增稳控制器头文件
│   ├── gimbal_stabilizer.c     # 增稳控制器实现
│   ├── app_imu.c               # 6-DOF IMU (仅 Roll/Pitch)
│   ├── app_imu_9dof.c          # 9-DOF IMU (含磁力计, ★ 新增)
│   ├── app_telemetry.c         # VOFA+ 遥测模块
│   └── tracking_main.c         # 旧 CanMV 像素追踪 (保留)
├── drivers/                    # 驱动层
│   ├── stepper_pulse.h/c       # 步进脉冲 + 步数计数 ISR
│   ├── gimbal_control.h/c      # 云台双模式控制 (角度+速度)
│   ├── delay.h/c               # 毫秒/微秒延时 + millis()
│   ├── hal_mpu6050.h/c         # MPU6050 I2C HAL
│   ├── mid_mpu6050.h/c         # MPU6050 寄存器协议
│   ├── hal_hmc5883l.h/c        # 磁力计 I2C HAL (★ 新增)
│   ├── mid_hmc5883l.h/c        # HMC5883L 寄存器协议 (★ 新增)
│   └── mid_qmc5883l.h/c        # QMC5883L 寄存器协议 (★ 新增)
├── include/                    # 公共头文件
├── docs/                       # 文档
└── openmv/                     # OpenMV 视觉代码
```

### 分层架构

```
┌─────────────────────────────────────────────────┐
│  应用层 (src/)                                    │
│  vision_tracking.c  ← 视觉坐标桥接                │
│  gimbal_stabilizer.c ← PID闭环增稳               │
│  app_imu_9dof.c ← 9轴姿态解算                     │
├─────────────────────────────────────────────────┤
│  中间层 (drivers/mid_*.c)                        │
│  mid_mpu6050.c ← MPU6050 寄存器协议               │
│  mid_hmc5883l.c ← HMC5883L 寄存器协议             │
│  mid_qmc5883l.c ← QMC5883L 寄存器协议             │
├─────────────────────────────────────────────────┤
│  硬件抽象层 (drivers/hal_*.c)                    │
│  hal_mpu6050.c ← I2C1 通信 + INT 中断             │
│  hal_hmc5883l.c ← I2C1 通信 + DRDY 引脚          │
├─────────────────────────────────────────────────┤
│  底层驱动 (drivers/)                              │
│  stepper_pulse.c ← PWM + ISR 步数计数             │
│  gimbal_control.c ← 双模式云台控制                │
├─────────────────────────────────────────────────┤
│  MSPM0G3507 硬件                                 │
│  TIMA0 / TIMG6 / I2C1 / UART / GPIO              │
└─────────────────────────────────────────────────┘
```

### 数据流

```
[K230 庐山派] ──UART 115200──▶ [UART RX ISR]
  arctan 角度解算                    │
  EMA 时间滤波                 ring buffer (512B)
  $TARGET,h,v,n\r\n                  │
                           parse_vision_packets()
                           (文本行解析状态机)
                                    │
                          g_vision_tracker.raw_pan/tilt
                          (= 角度 × 100, 如 4.72°→472)
                                    │
                        vision_coord_transform()
                        (方向符号 + 偏移 + 硬限幅)
                                    │
                       Stabilizer_SetTarget()
                           (EMA 平滑目标)
                                    │
            ┌───────────────────────┤
            │  IMU_9DOF_Update()    │  ← 200Hz
            │  (互补滤波+磁力计)     │
            │         │             │
            │   IMU_9DOF_Data_t     │
            │         │             │
            └──▶ Stabilizer_Update()│  ← 50Hz
                       │
                 PID_Compute() × 2轴
                 (P + I分离 + D测量值微分)
                       │
                 gimbal_control_angle()
                 (非阻塞角度定位)
                       │
                 stepper_pulse_move()
                 (ISR 后台步数计数)
                       │
             [TIMA0 PWM] [TIMG6 PWM]
             [TILT 电机] [PAN 电机]
```

### ISR 关系图

| ISR | 触发源 | 频率 | 职责 |
|:---|:---|:---|:---|
| `SysTick_Handler` | 1ms 定时 | 1000 Hz | `sys_tick_ms++` 全局时钟 |
| `TIMA0_IRQHandler` | PWM 零事件 | = 脉冲频率 | TILT 步数计数 + 位置模式自动停止 |
| `TIMG6_IRQHandler` | PWM 零事件 | = 脉冲频率 | PAN 步数计数 + 位置模式自动停止 |
| `cloud_uart_vision_IRQHandler` | UART RX | 不定 | 写入 ring buffer (K230 $TARGET 文本协议) |

### 关键共享变量

| 变量 | ISR 访问 | 主循环访问 | volatile |
|:---|:---|:---|:---|
| `sys_tick_ms` | SysTick(写) | 多处(读) | ✅ |
| `uart_rx_buf[256]` | UART ISR(写) | parse(读) | ✅ |
| `uart_rx_head` | UART ISR(写) | parse(读) | ✅ |
| `uart_rx_tail` | 无 | parse(写) | ✅ |
| `g_position_steps[2]` | TIMA0/TIMG6(写) | get(读) | ✅ |
| `g_busy[2]` | ISR(写false) | 主循环(写true+读) | ✅ |
| `g_steps_remaining[2]` | ISR(递减) | 主循环(写初始值) | ✅ |

---

## 4. K230 视觉协议 (Target Protocol v18)

### 协议概述

K230 (立创·庐山派) 通过 GC2093 摄像头实时检测靶子上的红色同心圆，
解算出靶心相对画面中心的水平角和垂直角，通过 UART2 发送文本协议给 MSPM0G3507。

**核心优势**: K230 已完成像素→角度解算 (arctan 透视投影模型)，
MCU 收到的直接就是角度值，无需测距，无需换算。

### 帧格式 (ASCII 文本)

**检测到靶子时** (每帧发送):
```
$TARGET,<水平角>,<垂直角>,<同心圆数>\r\n
```

**未检测到靶子时** (每帧发送):
```
$TARGET,NONE\r\n
```

### 字段说明

| 字段 | 格式 | 含义 |
|:---|:---|:---|
| `$TARGET` | 固定 | 帧头，标识这是靶子定位数据 |
| 水平角 | ±xx.xx (°) | 靶心偏离画面中心的水平角度 |
| | | **+ = 偏右**，云台应向右转 |
| | | **- = 偏左**，云台应向左转 |
| 垂直角 | ±xx.xx (°) | 靶心偏离画面中心的垂直角度 |
| | | **+ = 偏下**，云台应向下转 |
| | | **- = 偏上**，云台应向上转 |
| 同心圆数 | 0~5 | 检测到的红色同心圆数量 |
| | | ≥2 才发送有效数据 |
| | | 数值越大检测越可靠 |
| `NONE` | 固定 | 表示当前帧未检测到靶子 |
| `\r\n` | 固定 | 帧尾（回车换行） |

### 示例

```
$TARGET,4.72,-1.35,4\r\n
       ^^^^  ^^^^ ^
       |      |    └─ 同心圆数=4 (高可靠)
       |      └────── 垂直角=-1.35° (靶心偏上)
       └───────────── 水平角=4.72° (靶心偏右)
```

```
$TARGET,NONE\r\n    → 此帧未检测到靶子
```

### MCU 端解析行为

| 接收数据 | MCU 动作 |
|:---|:---|
| `$TARGET,h,v,n` 且 n≥2 | 切换到 TRACK_ANGLE，驱动云台追踪 (h→PAN目标, v→TILT目标) |
| `$TARGET,h,v,n` 且 n<2 | 丢弃此帧 (MCU端额外安全检查) |
| `$TARGET,NONE` | 切换到 HOLD，锁定当前位置增稳 |
| 连续 500ms 无数据 | 自动切换到 HOLD (超时保护) |

### K230 端处理流程 (MCU 开发者了解即可)

```
[摄像头帧 640×480]
     ↓
[LAB 颜色空间红色阈值提取]  ← (12,100,-47,14,-1,58)
     ↓
[圆形度筛选 + 外接圆提取]   ← roundness > 0.2
     ↓
[同心圆聚类 + 半径比例验证] ← 圆心距离 < 25px, 比例 1:2:3:4:5
     ↓
[面积加权靶心定位]           ← r² 加权 (大圆抗干扰更强)
     ↓
[EMA 时间滤波 + 稳定帧验证]  ← 连续3帧有效才输出
     ↓
[角度解算: arctan(像素偏移/焦距)]  ← 焦距 539.7 (像素单位)
     ↓
[UART2 输出: $TARGET,h_deg,v_deg,n\r\n]
```

### K230 角度计算公式

```
水平角 = arctan((Tx - 320) / 539.7)
垂直角 = arctan((Ty - 240) / 539.7)

其中:
  (Tx, Ty)  = 靶心像素坐标
  (320,240) = 画面中心
  539.7     = GC2093 焦距 (像素单位, 已标定)
```

**为什么用 arctan？**
- 镜头存在透视变形，画面边缘的像素密度高于中心
- arctan 能补偿非线性，边缘角度精度更高
- 标定焦距 539.7 保证角度值的物理准确性

**距离不变性**:
- 靶子在 50cm 处和 200cm 处，只要在画面同一位置，输出角度相同
- 这是本方案的核心优势：无需测距

### 角度定义 (以摄像头画面为参考系)

```
        垂直角 = 负（偏上）
              ↑
水平角=负    │    水平角=正
（偏左）←────┼────→（偏右）
              │
        垂直角 = 正（偏下）
```

| 角度值 | 靶心位置 | 云台应执行 |
|:---|:---|:---|
| 水平角 = +5.0 | 画面右半边 | 云台向右转 5.0° |
| 水平角 = -3.0 | 画面左半边 | 云台向左转 3.0° |
| 垂直角 = +2.0 | 画面下半边 | 云台向下转 2.0° |
| 垂直角 = -4.0 | 画面上半边 | 云台向上转 4.0° |
| 两个角都 ≈ 0 | 画面正中心 | 云台锁定不动 |

角度范围：水平约 ±30°，垂直约 ±23°
（由 GC2093 摄像头 FOV ≈ 62°×46° 和 640×480 分辨率决定）

### 硬件连接

```
K230 (庐山派)              MSPM0G3507
─────────────────────────────────────
GPIO11 (UART2_TX)   →    UART_RX
GPIO12 (UART2_RX)   →    UART_TX（可选）
GND                  ↔    GND（必须共地!）

波特率: 115200, 8N1, 无流控
```

### 视觉端 Python 参考代码

```python
"""
K230 靶子检测 → UART 发送示例
硬件: 立创·庐山派K230, GC2093摄像头
"""

import serial

# 打开串口
uart = serial.Serial('/dev/ttyS2', 115200, timeout=0.01)

# 发送角度数据 (在你的检测循环中调用)
def send_target(h_deg, v_deg, circles):
    \"\"\"
    发送靶子角度
    h_deg: 水平角(°), 正=偏右
    v_deg: 垂直角(°), 正=偏下
    circles: 同心圆数量(2~5)
    \"\"\"
    frame = f"$TARGET,{h_deg:.2f},{v_deg:.2f},{circles}\r\n"
    uart.write(frame.encode())

# 发送未检测到
def send_none():
    uart.write(b"$TARGET,NONE\r\n")

# 示例: 靶子在画面右侧 4.72°, 上方 1.35°, 4个同心圆
send_target(4.72, -1.35, 4)

# 示例: 靶子丢失
send_none()
```

> **协议核心就一句话**: 文本行 `$TARGET,h_deg,v_deg,n\r\n`，K230 发送预解算的角度值，MCU 直接使用。

---

## 5. 坐标系变换

### Camera-on-Gimbal 坐标映射

由于摄像头固定在云台上，图像坐标系与云台坐标系直接耦合：

```
图像 X 轴 (水平像素)  ←→  PAN 轴 (水平偏航)
图像 Y 轴 (垂直像素)  ←→  TILT 轴 (垂直俯仰)
```

### 变换公式

```
gimbal_pan  = vision_pan  × PAN_DIR_SIGN  + PAN_OFFSET
gimbal_tilt = vision_tilt × TILT_DIR_SIGN + TILT_OFFSET
```

### 参数说明

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `PAN_DIR_SIGN` | +1.0 | 视觉水平方向与云台PAN方向的关系: +1=同向, -1=反向 |
| `TILT_DIR_SIGN` | +1.0 | 视觉垂直方向与云台TILT方向的关系 |
| `PAN_OFFSET` | 0.0° | 视觉坐标系零点与云台PAN机械零点的角度差 |
| `TILT_OFFSET` | 0.0° | 视觉坐标系零点与云台TILT机械零点的角度差 |

### 像素→角度 换算 (视觉伺服模式)

基于小孔成像模型:

```
angle_error(°) = pixel_error(px) × (HFOV_deg / image_width_px)

例如: HFOV=60°, image_width=320px
  → 1px ≈ 0.1875°
  → 目标偏右 40px ≈ 需要 PAN 右转 7.5°
```

设置相机参数:
```c
VisionTracking_SetCameraParams(60.0f, 320, 240);  // HFOV=60°, QVGA
```

换算函数:
```c
float pan_deg, tilt_deg;
VisionTracking_PixelToAngle(40, -15, &pan_deg, &tilt_deg);
// pan_deg ≈ 7.5°, tilt_deg ≈ -2.8°
```

### 激光笔标定 (确定 OFFSET)

```
标定流程:
  1. 上电后手动控制云台, 使激光光斑对准标定板十字中心
  2. 调用 VisionTracking_LaserCalibrate()
  3. 系统自动计算: PAN_OFFSET = -当前PAN角度
                    TILT_OFFSET = -当前TILT角度
  4. 此后视觉发送绝对角度 0° = 云台回到标定点位置
```

### 方向符号确定方法

```
测试方法:
  1. 发送小角度 (如 PAN=+5°) 观察云台运动方向
  2. 若云台向右转但摄像头看到目标向左移动 → 正确 (同向)
  3. 若云台向左转 → PAN_DIR_SIGN 需要取反

摄像头排线方向常见影响:
  - 排线朝上/朝下安装 → 图像可能旋转 180°
  - 此时 PAN_DIR_SIGN 和 TILT_DIR_SIGN 都需要取反
```

### 运行时设置

```c
// 方向确认后设置
VisionTracking_SetTransform(+1.0f, +1.0f, 0.0f, 0.0f);

// 或使用激光标定
VisionTracking_LaserCalibrate();  // 自动设置 OFFSET
```

---

## 6. 工作模式详解

### K230 协议下的自动模式切换

K230 文本协议不包含显式的 mode 字段。MCU 根据接收数据自动切换：

```
                  ┌─────────────────────────────┐
                  │      MCU 上电 → IDLE          │
                  └─────────────┬───────────────┘
                                │
              ┌─────────────────┼──────────────────┐
              │                 │                  │
    收到 $TARGET,h,v,n    收到 $TARGET,NONE    超时 500ms
    且 n ≥ 2                                  无数据
              │                 │                  │
              ▼                 ▼                  ▼
      ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
      │ TRACK_ANGLE  │   │ HOLD_CURRENT│   │ HOLD_CURRENT│
      │ 追踪目标角度  │   │ 锁定当前位置│   │ 锁定当前位置│
      └──────┬──────┘   └─────────────┘   └─────────────┘
             │
             │ 再次收到 $TARGET,NONE
             │ 或超时 500ms
             ▼
      ┌─────────────┐
      │ HOLD_CURRENT│
      └─────────────┘
```

### 各模式行为

| 模式 | 触发条件 | 电机行为 | IMU反馈 |
|:---|:---|:---|:---|
| IDLE | 上电默认 / 手动调用 | 停止, 自由 | 无 |
| TRACK_ANGLE | 收到 `$TARGET,h,v,n` (n≥2) | K230 角度 → PID 追踪 | PID增稳跟踪 |
| HOLD_CURRENT | 收到 `$TARGET,NONE` 或超时 | 锁定当前位置 | PID增稳抵抗扰动 |
| TRACK_OFFSET | 保留 (当前不使用) | — | — |
| TRACK_PIXEL | 保留 (旧 CanMV 兼容) | — | — |

### 模式切换规则

| 切换 | 延迟 | 说明 |
|:---|:---|:---|
| 任何模式 → HOLD | 立即 | $TARGET,NONE 或超时 500ms → 立即锁定 |
| HOLD → TRACK_ANGLE | 立即 | 收到有效角度 → 立即恢复追踪 |
| 任何模式 → IDLE | 300ms | 仅手动 (VisionTracking_SetMode)，防误触发 |
| IDLE → 其他 | 300ms | 从 IDLE 恢复需延迟确认 |

---

## 7. 核心函数 API 参考

### 7.1 vision_tracking 模块

#### `VisionTracking_Init()`

```c
bool VisionTracking_Init(void);
```

**职责**: 初始化视觉追踪模块全部子系统。

**内部流程**:
1. 清零追踪器状态 (坐标变换参数设为默认值)
2. `gimbal_init()` + `gimbal_reset_zero()` — 云台控制初始化
3. `IMU_9DOF_Init()` — 9-DOF IMU 初始化 (自动鉴别磁力计)
4. `IMU_9DOF_CalibrateGyro(500)` — 陀螺仪零偏校准 (~2.5s)
5. `IMU_9DOF_SetFilter(0.98, 0.96, 0.005)` — 设置滤波器参数
6. `IMU_9DOF_SetDeclination(-5.0)` — 设置磁偏角
7. `Stabilizer_Init()` — 增稳控制器初始化
8. `vision_uart_init()` — UART + NVIC 中断使能

**返回值**: `true` = 初始化成功, `false` = IMU 初始化失败

**调用时机**: `main()` 中 `SYSCFG_DL_init()` 之后调用一次

---

#### `VisionTracking_Update()`

```c
void VisionTracking_Update(void);
```

**职责**: ★ 核心追踪循环 — 消费UART数据 + 更新IMU + 驱动增稳。

**内部流程**:
1. `parse_vision_packets()` — 非阻塞消费 UART ring buffer, 解析协议帧 (每循环)
2. `IMU_9DOF_Update()` — 按 200Hz 间隔读取传感器, 姿态解算
3. `process_mode_switch()` — 处理模式切换延迟确认逻辑
4. 超时检测 — 500ms 无数据 → 自动 HOLD
5. `vision_coord_transform()` — 根据当前模式将原始值转换为云台目标角度
6. 增量突变保护 — 单次变化 >30° → 拒绝, 切换 HOLD
7. `Stabilizer_SetTarget()` — 下发 EMA 平滑目标
8. `Stabilizer_Update()` — PID 闭环增稳控制

**调用频率**: 50Hz (每 20ms), 在主循环中调用

---

#### `VisionTracking_SetTransform()`

```c
void VisionTracking_SetTransform(float pan_sign, float tilt_sign,
                                  float pan_offset, float tilt_offset);
```

**职责**: 设置视觉→云台坐标变换参数。

**参数**:
- `pan_sign`: PAN 轴方向符号, 仅接受 +1.0 或 -1.0
- `tilt_sign`: TILT 轴方向符号, 仅接受 +1.0 或 -1.0
- `pan_offset`: PAN 轴角度偏移 (度), 范围 ±180°
- `tilt_offset`: TILT 轴角度偏移 (度), 范围 ±180°

**NaN/Inf 守卫**: 自动拒绝非法浮点值

---

#### `VisionTracking_SetMode()`

```c
void VisionTracking_SetMode(VisionMode_t mode);
```

**职责**: 手动切换到指定工作模式 (绕过 UART 协议, 调试用)。

**参数**:
- `mode`: VISION_MODE_IDLE / TRACK_ANGLE / OFFSET / PIXEL / WORLD / HOLD_CURRENT

---

#### `VisionTracking_SetTargetDirect()`

```c
void VisionTracking_SetTargetDirect(float pan_deg, float tilt_deg);
```

**职责**: 手动设置目标角度 (绕过协议和坐标变换, 调试用)。

直接调用 `Stabilizer_SetTargetInstant()`, 云台立即开始朝向目标运动。

**参数**:
- `pan_deg`: PAN 目标角度 (度), 范围 ±170°
- `tilt_deg`: TILT 目标角度 (度), 范围 ±90°

---

#### `VisionTracking_EmergencyStop()`

```c
void VisionTracking_EmergencyStop(void);
```

**职责**: 紧急停止 — 切断所有电机输出, 切换到 IDLE 模式。

---

#### `VisionTracking_GetState()`

```c
const VisionTracker_t *VisionTracking_GetState(void);
```

**职责**: 获取追踪器完整状态快照 (用于 OLED/TFT 调试显示)。

---

#### `VisionTracking_CalibrateGyro()`

```c
void VisionTracking_CalibrateGyro(uint16_t samples);
```

**职责**: 在线执行陀螺仪零偏校准 (需保持云台静止)。

**参数**: `samples` — 采样点数, 建议 500~1000

---

#### `VisionTracking_CalibrateMag()`

```c
void VisionTracking_CalibrateMag(uint16_t samples, uint32_t delay_each_ms);
```

**职责**: 在线执行磁力计硬铁/软铁校准 (需缓慢旋转云台覆盖全方向)。

**参数**:
- `samples`: 采样点数, 建议 ≥1000
- `delay_each_ms`: 每次采样间隔 (ms)

---

#### `VisionTracking_SetCameraParams()`

```c
void VisionTracking_SetCameraParams(float hfov_deg,
                                     uint16_t img_width, uint16_t img_height);
```

**职责**: 设置相机内参, 用于像素→角度换算。

**参数**:
- `hfov_deg`: 相机水平视场角 (度), 例如 OV7725=60°, 广角=120°
- `img_width`: 图像水平像素
- `img_height`: 图像垂直像素

---

#### `VisionTracking_PixelToAngle()`

```c
void VisionTracking_PixelToAngle(int16_t pixel_x, int16_t pixel_y,
                                  float *pan_out, float *tilt_out);
```

**职责**: 像素偏差 → 云台角度修正 (Camera-on-Gimbal 视觉伺服核心)。

**换算**: `angle = pixel × (HFOV / width)` (小孔成像, 方形像素假设)

**参数**:
- `pixel_x/y`: 目标像素相对于图像中心的偏差
- `pan_out/tilt_out`: 输出 PAN/TILT 角度修正 (度)

---

#### `VisionTracking_LaserCalibrate()`

```c
void VisionTracking_LaserCalibrate(void);
```

**职责**: 激光笔标定 — 记录当前光斑位置的云台角度, 自动设置坐标偏移。

**变换公式**: `gimbal_angle = vision_angle × sign + offset`
**标定逻辑**: vision=0 时 gimbal 应停在标定点 → `offset = +current_position`

**使用**: 手动对准标定点后调用一次, 后续视觉发送 0° 目标即可回到标定点。

---

### 7.2 gimbal_stabilizer 模块

#### `Stabilizer_Init()`

```c
void Stabilizer_Init(void);
```

**职责**: 初始化增稳控制器, 将两轴 PID 设为默认参数, 模式设为 IDLE。

**调用时机**: `gimbal_init()` 和 `IMU_Init()` 之后

---

#### `Stabilizer_Update()`

```c
bool Stabilizer_Update(const IMU_Data_t *imu);
```

**职责**: ★ 增稳控制核心 — 将 IMU 数据转换为云台角度指令。

**内部流程**:
- HOLD 模式: 锁定目标角度, PID 闭环抵抗外部扰动
- TRACK 模式: EMA 平滑目标 → PID 计算 → 死区检查 → 限幅 → `gimbal_control_angle()`
- IDLE 模式: 无操作, 直接返回

**参数**: `imu` — IMU_Update() 输出的姿态数据

**返回值**: `true` = 正常执行, `false` = 数据无效或 IDLE 模式

**调用频率**: 100~200Hz (5~10ms)

---

#### `Stabilizer_SetMode()`

```c
void Stabilizer_SetMode(StabilizeMode_t mode);
```

**职责**: 切换增稳工作模式。

**参数**:
- `STABILIZE_HOLD` — 锁定当前位置为增稳目标
- `STABILIZE_TRACK` — 等待 `Stabilizer_SetTarget()` 下发追踪目标
- `STABILIZE_IDLE` — 停止所有电机

---

#### `Stabilizer_SetTarget()`

```c
void Stabilizer_SetTarget(float pan_deg, float tilt_deg);
```

**职责**: 设置追踪目标角度 (TRACK 模式下使用, 内置 EMA 平滑)。

**参数**: `pan_deg`, `tilt_deg` — 目标角度 (度)

---

#### `Stabilizer_SetTargetInstant()`

```c
void Stabilizer_SetTargetInstant(float pan_deg, float tilt_deg);
```

**职责**: 立即跳转到目标角度 (绕过 EMA 平滑)。

---

#### `Stabilizer_SetPID()`

```c
void Stabilizer_SetPID(StabilizeAxis_t axis, float Kp, float Ki, float Kd);
```

**职责**: 在线调整指定轴的 PID 增益。

**参数**: `axis` = STAB_AXIS_PAN (水平) 或 STAB_AXIS_TILT (俯仰)

**调优顺序**: Kp → Kd → Ki (每次只改一个参数)

| 现象 | 原因 | 调整 |
|:---|:---|:---|
| 缓慢到达目标, 不振荡 | Kp 太小 | ↑ Kp × 1.5 |
| 在目标附近快速摆动 | Kp 太大 | ↓ Kp × 0.7 |
| 到达后过冲再回来 | Kd 太小 | ↑ Kd × 1.5 |
| 到达前就明显减速 | Kd 太大 | ↓ Kd × 0.7 |
| 长期有小角度静差 | Ki 太小 | ↑ Ki × 2.0 |
| 低频振荡 (>1s) | Ki 太大 | ↓ Ki × 0.5 |

---

### 7.3 IMU 9-DOF 模块

#### `IMU_9DOF_Init()`

```c
bool IMU_9DOF_Init(void);
```

**职责**: 初始化 9-DOF 系统, 自动鉴别磁力计芯片 (先 HMC5883L 0x1E, 后 QMC5883L 0x0D)。

**返回值**: `true` = 所有传感器初始化成功, `false` = MPU6050 失败

---

#### `IMU_9DOF_Update()`

```c
bool IMU_9DOF_Update(IMU_9DOF_Data_t *data);
```

**职责**: ★ 9-DOF 数据更新 — 读取 MPU6050 + 磁力计, 互补滤波姿态解算。

**融合流程**:
1. 读 MPU6050 (14字节 burst read) → 加速度 + 角速度
2. 读磁力计 (DRDY 预检查) → 磁场强度 μT
3. 加速度计 → Roll_accel / Pitch_accel
4. 互补滤波: `roll = α×(roll+gyro_x×dt) + (1-α)×roll_accel`
5. 磁力计倾角补偿 → 航向角 heading
6. Yaw 互补滤波: `yaw += gyro_z×dt + yaw_err×(1-α_yaw)`

**输出**: `IMU_9DOF_Data_t` — 包含 roll/pitch/yaw/heading + 全部传感器原始值

---

#### `IMU_9DOF_SetFilter()`

```c
void IMU_9DOF_SetFilter(float alpha, float alpha_yaw, float dt);
```

**职责**: 设置滤波器参数。

**参数**:
- `alpha`: Roll/Pitch 互补系数 (0.90~0.995, 默认 0.98)
- `alpha_yaw`: Yaw 互补系数 (0.90~0.995, 默认 0.96)
- `dt`: 滤波周期 (秒, 默认 0.005 = 200Hz)

---

#### `IMU_9DOF_SetDeclination()`

```c
void IMU_9DOF_SetDeclination(float declination_deg);
```

**职责**: 设置磁偏角 (用于真北修正)。

**中国主要城市磁偏角参考** (2025年近似值):

| 城市 | 磁偏角 | 城市 | 磁偏角 |
|:---|:---|:---|:---|
| 北京 | -7° | 上海 | -5° |
| 广州 | -2° | 成都 | -2° |
| 哈尔滨 | -11° | 乌鲁木齐 | +3° |

---

### 7.4 gimbal_control 模块

#### `gimbal_control_angle()`

```c
void gimbal_control_angle(float pan_deg, float tilt_deg);
```

**职责**: ★ 绝对角度位置控制 — 非阻塞, ISR 自动停止。

**内部流程**:
1. 读取当前位置 → 计算角度差 → 死区检查 (0.12°)
2. 角度 → 步数: `steps = |delta| × 3200 / 360`
3. `stepper_pulse_move()` 下发运动, 立即返回
4. ISR 在后台计数, 到达目标步数后自动停止

**主循环通过 `gimbal_is_moving()` 轮询完成状态**

---

#### `gimbal_track_velocity()`

```c
void gimbal_track_velocity(int16_t error_x, int16_t error_y, uint32_t now_ms);
```

**职责**: ★ 速度追踪 — 像素误差 → EMA滤波 → P控制 → RPM → 连续调速。

**内部流程**:
1. 超时检测 (>200ms 无数据 → error=0)
2. 死区过滤 (3px)
3. 非对称 EMA 滤波 (加速 α=0.15 / 减速 α=0.30)
4. P 控制器: `target_rpm = Kp × |ema|`
5. RPM 变化率限制 (200 RPM/周期, 防丢步)
6. 原始误差定方向 (无 EMA 滞后)

**调用频率**: 50Hz (每 20ms)

---

### 7.5 stepper_pulse 模块

#### `stepper_pulse_move()`

```c
void stepper_pulse_move(uint8_t axis, uint8_t dir, uint16_t rpm, uint32_t steps);
```

**职责**: 位置模式 — 有限步数运动, 非阻塞, ISR 自动停止。

---

#### `stepper_pulse_set_speed()`

```c
void stepper_pulse_set_speed(uint8_t axis, uint8_t dir, uint16_t rpm);
```

**职责**: 速度模式 — 连续调速, 同方向不中断脉冲, 丝滑过渡。

---

### 7.6 默认 PID 参数

| 参数 | PAN (水平) | TILT (俯仰) |
|:---|:---|:---|
| Kp (比例增益) | 1.2 | 1.2 |
| Ki (积分增益) | 0.15 | 0.15 |
| Kd (微分增益) | 0.35 | 0.35 |
| out_max (输出限幅) | 3.0° | 3.0° |
| i_max (积分限幅) | 2.0° | 2.0° |
| i_zone (积分分离区) | 5.0° | 5.0° |
| deadband (死区) | 0.15° | 0.15° |

---

## 8. 快速启动指南

### 8.1 SysConfig 配置

在 CCS Theia 中打开 `.syscfg` 文件, 添加以下外设:

1. **UART** (名称: `cloud_uart_vision`):
   - 波特率: 115200, 8N1
   - RX 引脚: 接视觉上位机 TX
   - RX 中断: 使能

2. **Timer** (名称: `cloud_1_TILT` / `cloud_2_PAN`)
   - 已存在于现有 SysConfig

3. **I2C** (名称: `I2C_1`)
   - 已存在于现有 SysConfig

### 8.2 编译与烧录

```
1. CCS Theia → Project → Build Project
2. Run → Debug (通过板载 XDS-110)
```

### 8.3 接线

```
K230 GPIO11 (UART2 TX) ──── MSPM0 UART RX (SysConfig 配置的 GPIO)
K230 GND                  ──── MSPM0 GND (必须共地!)
步进电机驱动器:
  PUL+ → PB0 (TILT) / PB2 (PAN)
  DIR+ → PB13 (TILT) / PB3 (PAN)
  PUL- → GND
  DIR- → GND
MPU6050:
  SDA → PA16
  SCL → PA17
  VCC → 3.3V
  GND → GND
磁力计 (GY-87模块内置):
  通过 MPU6050 AUX I2C 连接, 无需额外接线
```

### 8.4 上电顺序

```
1. 步进电机驱动器上电
2. MSPM0 上电 → 自动初始化 (~3s 含校准)
3. 视觉上位机上电 → 开始发送坐标
4. 观察云台响应
```

### 8.5 最小验证程序 (Python)

```python
import serial
import struct
import time

# 连接 MSPM0 UART
ser = serial.Serial('COM3', 115200, timeout=0.1)

def send_target(mode, pan_deg, tilt_deg):
    pan  = int(pan_deg * 100)
    tilt = int(tilt_deg * 100)
    payload = struct.pack('>Bhh4B', mode, pan, tilt, 0, 0, 0, 0)
    # CRC16-CCITT
    crc = 0xFFFF
    for b in payload:
        crc = (crc << 8) ^ CRC16_TABLE[((crc >> 8) ^ b) & 0xFF]
    frame = b'\x5A\xA5' + payload + struct.pack('>H', crc) + b'\x0D\x0A'
    ser.write(frame)

# 测试: 让云台指向 PAN=45°, TILT=-20°
send_target(1, 45.0, -20.0)
time.sleep(2)

# 锁定当前位置
send_target(5, 0, 0)
time.sleep(1)

# 停止
send_target(0, 0, 0)
ser.close()
```

---

## 9. 调参指南

### 9.1 首次测试建议

```c
// 低速测试
gimbal_pan.rpm  = 60;   // PAN 轴 60 RPM
gimbal_tilt.rpm = 60;   // TILT 轴 60 RPM

// 增大死区 (480p 摄像头建议)
// 在 gimbal_control.h 中:
// #define GIMBAL_DEADBAND_PAN/TILT 8
```

### 9.2 调参顺序

1. **确认方向**: 发送小角度 (±5°) 验证 PAN/TILT 方向正确
2. **调整 DIR_SIGN**: 若方向反了, 用 `VisionTracking_SetTransform(-1, 1, 0, 0)`
3. **调整 PID**: 按 Kp→Kd→Ki 顺序, 每次只改一个参数
4. **逐步提高 RPM**: 确认不丢步后逐步提高转速

### 9.3 常用配置

```c
// 高响应追踪 (牺牲平滑度)
Stabilizer_SetPID(STAB_AXIS_PAN,  2.0f, 0.0f, 0.5f);
Stabilizer_SetPID(STAB_AXIS_TILT, 2.0f, 0.0f, 0.5f);

// 高精度增稳 (牺牲响应速度)
Stabilizer_SetPID(STAB_AXIS_PAN,  0.8f, 0.2f, 0.5f);
Stabilizer_SetPID(STAB_AXIS_TILT, 0.8f, 0.2f, 0.5f);

// 更改 IMU 滤波器带宽
IMU_9DOF_SetFilter(0.95f, 0.92f, 0.005f);  // 更快响应
IMU_9DOF_SetFilter(0.99f, 0.98f, 0.005f);  // 更平滑
```

---

## 10. 故障排查

| 现象 | 可能原因 | 排查步骤 |
|:---|:---|:---|
| 初始化卡死 | IMU/磁力计 I2C 通信失败 | 检查 PA16/PA17 接线, 确认 MPU6050 供电 3.3V |
| 云台不响应 K230 | 接线错误/波特率不匹配/K230 未运行 | **①** 用串口助手监听 K230 TX脚, 确认输出 `$TARGET,...\r\n`; **②** 确认 115200 8N1; **③** 确认 K230 GPIO11(TX) → MSPM0 RX; **④** 共地! |
| K230 发了数据但 MCU 没反应 | 协议解析失败 | 查看 `g_vision_tracker.packets_bad_frame` 计数器; 确认 K230 发送格式严格匹配 `$TARGET,±xx.xx,±xx.xx,n\r\n` |
| 云台方向反了 | DIR_SIGN 符号错误 | 调用 `VisionTracking_SetTransform(-1, 1, 0, 0)` |
| 云台在目标附近来回摆动 | PID Kp 太大 | 降低 Kp (×0.7) |
| 云台跟不上快速移动目标 | RPM 太低或 Kp 太小 | 提高 RPM 到 600+, 增大 Kp 和 EMA_ALPHA |
| 长时间运行后 yaw 漂移 | 磁力计未校准或不在线 | 调用 `VisionTracking_GetMagChip()` 确认磁力计类型 |
| 步进电机丢步 (异响) | RPM 太高 / 加速度太大 | 降低 MAX_RPM 到 1000, 降低 RPM_SLEW_RATE |
| 堵转故障 | 机械卡住 / 负载过重 | 检查机械结构, 调用 `gimbal_clear_stall_fault()` |
| 靶子在画面中但输出 NONE | K230 LAB红色阈值不合适/光照变化 | 调整 K230 端 LAB 阈值参数; 检查光照是否均匀 |
| 角度跳动大 | K230 EMA 系数太小/摄像头松动 | K230 端增大 EMA_ALPHA; 检查摄像头固定 |

---

## 11. 可调参数完整参考 (★ 比赛现场调参必读)

> **取值说明**: `[min, max]` 为安全范围; `默认` 为出厂推荐值;
> `步进` 为建议每次调整量; `⚠` 标记表示超范围可能导致硬件损坏。

---

### 11.1 步进电机脉冲层 (stepper_pulse.h)

| 参数 | 默认值 | 取值范围 | 步进 | 说明 |
|:---|:---|:---|:---|:---|
| `STEPPER_PULSE_MAX_RPM` | 3000 | [500, 5000] ⚠ | 500 | **硬上限**。超过此值力矩急剧下降，极易堵转丢步。高负载场景建议 ≤1500 |
| `STEPPER_PULSE_MIN_RPM` | 30 | [10, 100] | 10 | **硬下限**。低于此值步进电机会明显抖动(低频共振) |
| `STEPPER_PULSE_DEFAULT_RPM` | 1200 | [60, 2000] | 200 | 角度模式/位置模式默认转速(首次测试建议60) |
| `STEPPER_PULSES_PER_REV` | 3200 | 不可改 | - | 200 全步 × 16 细分 = 3200 PPR (由驱动器拨码决定) |
| `STEPPER_RPM_STOP` | 0 | 固定 | - | 停止标记值 |

**调参建议**: 高负载(激光笔+摄像头+支架)建议 MAX_RPM≤1500, DEFAULT_RPM≤600。

---

### 11.2 云台控制层 — 速度追踪 (gimbal_control.h)

| 参数 | 默认值 | 取值范围 | 步进 | 说明 |
|:---|:---|:---|:---|:---|
| `GIMBAL_TRACK_KP_PAN` | 30.0 | [5.0, 200.0] | 10.0 | **P 增益 (RPM/像素)**。30 表示 1px 误差 → 30 RPM。摄像头分辨率越高此值应越小 |
| `GIMBAL_TRACK_KP_TILT` | 30.0 | [5.0, 200.0] | 10.0 | TILT 轴 P 增益，同上 |
| `GIMBAL_DEADBAND_PAN` | 3 | [0, 20] | 1 | **像素死区**。误差小于此值电机不动作。QVGA(320×240)建议 3~5, 720p 建议 5~10 |
| `GIMBAL_DEADBAND_TILT` | 3 | [0, 20] | 1 | TILT 轴像素死区 |
| `GIMBAL_EMA_ALPHA` | 0.15 | [0.05, 0.50] | 0.05 | **EMA 平滑系数**(加速方向)。越小越平滑但滞后越大。50Hz 更新率下 0.15≈截止频率 1.3Hz |
| `GIMBAL_EMA_ALPHA_DECAY` | 0.30 | [0.10, 0.60] | 0.05 | **EMA 平滑系数**(减速方向)。比 ALPHA 大 → 目标消失时快速降速, 防止过冲 |
| `GIMBAL_RPM_SLEW_RATE` | 200 | [50, 500] | 50 | **RPM 变化率限制** (RPM/20ms周期)。200 表示每秒最多加速 10000 RPM。防丢步 |
| `GIMBAL_TRACK_MAX_RPM` | 2000 | [300, 3000] ⚠ | 200 | **追踪模式最大 RPM**。超过此值钳位 |
| `GIMBAL_TRACK_MIN_RPM` | 80 | [30, 200] | 20 | **追踪模式最小 RPM**。低于此值直接停止(低速抖动) |
| `GIMBAL_LOST_TIMEOUT_MS` | 200 | [50, 1000] | 50 | **目标丢失超时**。超过此时长无新UART数据 → 自动停止追踪 |

**QVGA(320×240) 典型配置**:
```
低速安全:  KP=15, DEADBAND=5, MAX_RPM=600,  SLEW=100
快速响应:  KP=50, DEADBAND=2, MAX_RPM=1500, SLEW=300
```

---

### 11.3 云台控制层 — 角度位置 (gimbal_control.h)

| 参数 | 默认值 | 取值范围 | 步进 | 说明 |
|:---|:---|:---|:---|:---|
| `GIMBAL_ANGLE_DEFAULT_RPM` | 300 | [30, 2000] | 100 | 角度定位模式默认转速 |
| `GIMBAL_PAN_ANGLE_MAX` | 170.0° | [30, 180] ⚠ | - | **PAN 机械限位**。超过可能导致线缆缠绕或机械碰撞 |
| `GIMBAL_TILT_ANGLE_MAX` | 90.0° | [20, 100] ⚠ | - | **TILT 机械限位**。必须小于物理可动范围 |
| `GIMBAL_ANGLE_DEADBAND` | 0.12° | [0.05, 0.5] | 0.05 | **角度死区**。略大于单步精度 0.1125°, 防亚步抖动 |

---

### 11.4 增稳控制器 PID (gimbal_stabilizer.h)

**PAN 轴 (水平偏航) 默认参数**:

| 参数 | 默认值 | 取值范围 | 步进 | 说明 |
|:---|:---|:---|:---|:---|
| `STAB_PAN_KP_DEFAULT` | 1.2 | [0.1, 10.0] | 0.2 | **比例增益**(度/度)。1.2 表示 1° 误差 → 1.2° 输出补偿 |
| `STAB_PAN_KI_DEFAULT` | 0.15 | [0.0, 2.0] | 0.05 | **积分增益**(1/秒)。消除稳态静差, 过大会低频振荡 |
| `STAB_PAN_KD_DEFAULT` | 0.35 | [0.0, 2.0] | 0.1 | **微分增益**(秒)。抑制过冲, 对测量值微分避免 derivative kick |
| `STAB_PAN_OUT_MAX_DEFAULT` | 3.0° | [0.5, 20.0] | 1.0 | **PID 输出限幅**。单次最大角度补偿量, 防过冲丢步 |
| `STAB_PAN_I_MAX_DEFAULT` | 2.0° | [0.1, 10.0] | 0.5 | **积分项限幅**。防积分饱和 (windup) |
| `STAB_PAN_I_ZONE_DEFAULT` | 5.0° | [1.0, 30.0] | 1.0 | **积分分离区**。\|error\|>此值时 Ki=0, 防大误差下积分累积 |
| `STAB_PAN_DEADBAND_DEFAULT` | 0.15° | [0.05, 2.0] | 0.05 | **PID 死区**。\|error\|<此值时输出 0 + 清零积分。略大于步距(0.1125°) |

**TILT 轴 (垂直俯仰) 参数**: 与 PAN 相同前缀, 默认值一致。TILT 轴受重力影响更大, 通常需要稍大的 Kp/Ki。

**增稳模式参数**:

| 参数 | 默认值 | 取值范围 | 步进 | 说明 |
|:---|:---|:---|:---|:---|
| `STAB_TRACK_EMA_DEFAULT` | 0.30 | [0.05, 0.80] | 0.05 | **追踪模式 EMA 平滑系数**。视觉目标经此系数平滑后送入 PID。越大响应越快 |
| `STAB_HOLD_UPDATE_MS` | 5 | [2, 20] | 1 | HOLD 模式 PID 更新间隔 (ms) = 200Hz |
| `STAB_TRACK_UPDATE_MS` | 10 | [5, 50] | 5 | TRACK 模式 PID 更新间隔 (ms) = 100Hz |

**安全限幅**:

| 参数 | 默认值 | 取值范围 | 说明 |
|:---|:---|:---|:---|
| `STAB_PAN_ANGLE_HARD_LIMIT` | 170.0° | [30, 180] ⚠ | **PAN 绝对角度硬限幅**。超过此值软件拒绝, 防机械碰撞 |
| `STAB_TILT_ANGLE_HARD_LIMIT` | 90.0° | [20, 100] ⚠ | **TILT 绝对角度硬限幅** |
| `STAB_DELTA_HARD_LIMIT` | 10.0° | [2.0, 30.0] | **单次增量硬限幅**。超过此值的角度跳变被视为数据异常, 拒绝执行 |

**PID 调优速查**:

| 现象 | 原因 | 调整 |
|:---|:---|:---|
| 缓慢到达目标, 不振荡 | Kp 太小 | ↑ Kp × 1.5 |
| 在目标附近快速来回摆动(>5Hz) | Kp 太大 | ↓ Kp × 0.7 |
| 到达后过冲再回来 | Kd 太小 | ↑ Kd × 1.5 |
| 到达前就明显减速(爬行) | Kd 太大 | ↓ Kd × 0.7 |
| 长期有小角度静差(>2s) | Ki 太小 | ↑ Ki × 2.0 |
| 低频振荡(周期>1s) | Ki 太大 | ↓ Ki × 0.5 |
| 电机高频啸叫 | deadband 太小 | ↑ deadband × 2.0 |
| 目标附近"抽搐" | deadband 太大 | ↓ deadband × 0.5 |

---

### 11.5 视觉追踪层 (vision_tracking.h)

**协议与循环参数**:

| 参数 | 默认值 | 取值范围 | 步进 | 说明 |
|:---|:---|:---|:---|:---|
| `VISION_TRACK_LOOP_MS` | 20 | [5, 50] | 5 | 追踪主循环周期 (ms) = 50Hz |
| `VISION_IMU_UPDATE_MS` | 5 | [2, 20] | 1 | IMU 更新周期 (ms) = 200Hz。必须与互补滤波器 dt 一致 |
| `VISION_LOST_TIMEOUT_MS` | 500 | [100, 2000] | 100 | **视觉目标丢失超时**。超过此时长无有效帧 → 自动 HOLD |
| `VISION_MODE_SWITCH_DELAY_MS` | 300 | [100, 1000] | 100 | **模式切换延迟**。IDLE 请求需持续此时长才生效, 防 UART 噪声误触发 |
| `VISION_ANGLE_SCALE` | 100.0 | 固定 | - | 协议传输缩放因子: 传输值 = 角度×100 (精度 0.01°) |

**坐标变换参数** (运行时通过 API 设置):

| 参数 | 默认值 | 取值范围 | API | 说明 |
|:---|:---|:---|:---|:---|
| `pan_dir_sign` | +1.0 | {-1.0, +1.0} | `VisionTracking_SetTransform` | PAN 方向符号: +1=视觉右→云台右, -1=反向 |
| `tilt_dir_sign` | +1.0 | {-1.0, +1.0} | 同上 | TILT 方向符号 |
| `pan_offset` | 0.0° | [-180, +180] | 同上 | PAN 零点偏移 |
| `tilt_offset` | 0.0° | [-180, +180] | 同上 | TILT 零点偏移 |

**安全限幅**:

| 参数 | 默认值 | 取值范围 | 说明 |
|:---|:---|:---|:---|
| `VISION_PAN_OUTPUT_LIMIT` | 170.0° | [30, 180] ⚠ | 坐标变换后 PAN 输出硬限幅 |
| `VISION_TILT_OUTPUT_LIMIT` | 90.0° | [20, 100] ⚠ | 坐标变换后 TILT 输出硬限幅 |

**相机内参** (运行时通过 `VisionTracking_SetCameraParams` 设置):

| 参数 | 默认值 | 取值范围 | 说明 |
|:---|:---|:---|:---|
| `s_cam_hfov_deg` | 60.0° | [10, 180] | 相机水平视场角。OV7725≈60°, 广角镜头≈120°, 长焦≈25° |
| `s_cam_img_width` | 320 | [1, 2048] | 图像水平像素。QVGA=320, VGA=640, 720p=1280 |
| `s_cam_img_height` | 240 | [1, 2048] | 图像垂直像素。QVGA=240, VGA=480, 720p=720 |

**像素→角度换算验证** (基于小孔成像):

| HFOV | 图像宽度 | deg/pixel | 10px 对应角度 | 50px 对应角度 |
|:---|:---|:---|:---|:---|
| 60° | 320 (QVGA) | 0.1875° | 1.88° | 9.38° |
| 60° | 640 (VGA) | 0.0938° | 0.94° | 4.69° |
| 120° | 320 | 0.3750° | 3.75° | 18.75° |
| 25° | 320 | 0.0781° | 0.78° | 3.91° |

> **注意**: 像素→角度换算是理想小孔模型。实际镜头有畸变(尤其是广角), 边缘区域误差可能 ×1.5~2 倍。精度要求高的场景建议视觉端预解算角度。

---

### 11.6 IMU 姿态解算 (app_imu_9dof)

| 参数 | 默认值 | 取值范围 | 步进 | API | 说明 |
|:---|:---|:---|:---|:---|:---|
| `alpha` (Roll/Pitch) | 0.98 | [0.90, 0.995] | 0.01 | `IMU_9DOF_SetFilter` | **互补滤波系数**。时间常数 τ=α·dt/(1-α)。0.98@5ms→τ≈0.245s |
| `alpha_yaw` (Yaw) | 0.96 | [0.90, 0.995] | 0.01 | 同上 | Yaw 互补滤波系数。磁力计噪声大, 建议比 alpha 略低 |
| `dt` | 0.005s | [0.001, 0.05] | 0.001 | 同上 | 滤波周期(秒)。必须与实际调用频率一致 (200Hz=0.005s) |
| `declination` | -5.0° | [-30, +30] | 0.5 | `IMU_9DOF_SetDeclination` | **磁偏角**(°)。东偏为正。中国: 北京-7°, 上海-5°, 广州-2°, 成都-2°, 哈尔滨-11°, 乌鲁木齐+3° |

**IMU 滤波器时间常数参考** (dt=0.005s):

| alpha | τ (秒) | 特性 |
|:---|:---|:---|
| 0.99 | 0.495s | 极其平滑, 响应很慢 → 静态增稳 |
| 0.98 | 0.245s | **默认**, 平衡平滑与响应 |
| 0.95 | 0.095s | 响应快, 噪声稍多 → 动态追踪 |
| 0.90 | 0.045s | 响应极快, 噪声明显 → 仅用于快速机动 |

---

### 11.7 磁力计配置 (mid_hmc5883l.h / mid_qmc5883l.h)

**HMC5883L (正品 Honeywell, I2C 地址 0x1E)**:

| 参数 | 默认值 | 可选值 | 说明 |
|:---|:---|:---|:---|
| `avg` (采样平均) | HMC5883L_AVG_8 | 1/2/4/8 | 8次平均=最低噪声, 1次=最快更新 |
| `rate` (数据率) | HMC5883L_RATE_75HZ | 0.75/1.5/3/7.5/15/30/75 Hz | 最高 75Hz |
| `gain` (增益) | HMC5883L_GAIN_1090 | ±0.88~±8.1 Ga | 1090 LSB/Gauss=±1.3Ga, 适合大多数地区 |

**QMC5883L (国产替代, I2C 地址 0x0D)**:

| 参数 | 默认值 | 可选值 | 说明 |
|:---|:---|:---|:---|
| `osr` (过采样) | QMC5883L_OSR_512 | 64/128/256/512 | 512次=最高精度, 64次=最快 |
| `range` (量程) | QMC5883L_RANGE_8G | ±2G / ±8G | ±8G 适合大部分应用 |
| `odr` (数据率) | QMC5883L_ODR_200HZ | 10/50/100/200 Hz | 200Hz 匹配 IMU 更新率 |

---

### 11.8 参数快速参考卡 (比赛现场速查)

```
┌─ 首次上电安全配置 ─────────────────────────────────────┐
│ gimbal_pan.rpm  = 60;     gimbal_tilt.rpm = 60;        │
│ GIMBAL_TRACK_MAX_RPM = 600                             │
│ GIMBAL_DEADBAND_PAN/TILT = 8                           │
│ STAB_PAN_KP = 0.8;  STAB_TILT_KP = 0.8                │
│ 确认方向后用激光标定: VisionTracking_LaserCalibrate()   │
└────────────────────────────────────────────────────────┘

┌─ 中速追踪配置 ─────────────────────────────────────────┐
│ gimbal_pan.rpm  = 300;    gimbal_tilt.rpm = 300;       │
│ GIMBAL_TRACK_MAX_RPM = 1200                            │
│ GIMBAL_TRACK_KP_PAN/TILT = 30                          │
│ GIMBAL_DEADBAND_PAN/TILT = 3                           │
│ STAB_PAN_KP = 1.2;  STAB_TILT_KP = 1.2                │
└────────────────────────────────────────────────────────┘

┌─ 快速响应配置 (轻载) ──────────────────────────────────┐
│ gimbal_pan.rpm  = 600;    gimbal_tilt.rpm = 600;       │
│ GIMBAL_TRACK_MAX_RPM = 2000                            │
│ GIMBAL_TRACK_KP_PAN/TILT = 60                          │
│ GIMBAL_DEADBAND_PAN/TILT = 2                           │
│ GIMBAL_EMA_ALPHA = 0.25                                │
│ STAB_PAN_KP = 2.0;  STAB_TILT_KP = 2.0                │
│ IMU_9DOF_SetFilter(0.95, 0.92, 0.005)                  │
└────────────────────────────────────────────────────────┘```

