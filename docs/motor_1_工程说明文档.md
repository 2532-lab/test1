# motor_1 工程说明文档

> **MCU:** MSPM0G3507 (Cortex-M0+) | **IDE:** Code Composer Studio (CCS) + SysConfig
> **SDK:** mspm0_sdk@2.11.00.07 | **SysConfig:** 1.26.2
> **主频:** 32 MHz BUSCLK | **编译器:** TI Arm Clang
> **最后更新:** 2026-07-27

---

## 目录

1. [工程概述](#1-工程概述)
2. [硬件架构](#2-硬件架构)
3. [软件架构](#3-软件架构)
4. [文件结构](#4-文件结构)
5. [核心参数与调校指南](#5-核心参数与调校指南)
6. [API 参考](#6-api-参考)
7. [典型使用流程](#7-典型使用流程)
8. [调试与故障排查](#8-调试与故障排查)
9. [引脚映射表](#9-引脚映射表)
10. [SysConfig 配置指南](#10-sysconfig-配置指南)
11. [附录](#11-附录)

---

## 1. 工程概述

`motor_1` 是一个基于 **MSPM0G3507** 的两轮差速步进电机小车控制系统。核心功能包括：

- **双轴步进电机 PWM 脉冲控制** — 硬件 PWM + ISR 步数计数，支持**速度模式**和**位置模式**（含梯形速度曲线）
- **MPU6050 IMU 姿态解算** — 三层架构（HAL → 中间层 → 应用层），互补滤波融合，含非重力加速度拒收保护
- **差速转向控制** — 左右轮独立调速 + PD 航向保持算法 + 故障锁存机制
- **安全保护** — 倾斜检测急停、I2C 连续故障自动重初始化、紧急停止锁存
- **上电自检** — IMU 初始化成功后自动前进 5 圈（位置模式，梯形加减速），验证电机接线

### 关键特性

| 特性 | 参数 |
|------|------|
| 电机控制精度 | 0.1125°/步 (200 全步 × 16 细分 = 3200 PPR) |
| 转速范围 | 9 ~ 3000 RPM（物理）；≤1200 RPM（业务限幅） |
| IMU 更新率 | 200 Hz (每 5ms) |
| 互补滤波 α | 0.98 (可调 0.90~0.995) |
| I2C 速率 | 400 kHz (Fast Mode) |
| 倾斜保护阈值 | ±35° (可调) |
| 位置模式梯形速度曲线 | 起步 20 RPM → 加速到目标 → 减速到 20 RPM |

---

## 2. 硬件架构

### 2.1 系统框图

```
┌──────────────────────────────────────────────────┐
│                  MSPM0G3507 (32MHz)               │
│                                                   │
│  ┌─────────┐   ┌─────────┐   ┌──────────────┐   │
│  │  TIMA0   │   │  TIMG6   │   │    I2C1      │   │
│  │(RIGHT轮) │   │(LEFT轮)  │   │  (MPU6050)   │   │
│  │ CCP2→PB0 │   │ CCP0→PB2 │   │ SDA=PA16     │   │
│  │ DIR→PB13 │   │ DIR→PB3  │   │ SCL=PA17     │   │
│  └────┬─────┘   └────┬─────┘   └──────┬───────┘   │
│       │               │                │           │
│  ┌────┴─────┐   ┌─────┴────┐  ┌───────┴───────┐   │
│  │ 步进驱动 │   │ 步进驱动 │  │   MPU6050     │   │
│  │ (RIGHT)  │   │ (LEFT)   │  │  6-axis IMU   │   │
│  └────┬─────┘   └─────┬────┘  │  INT→PB24     │   │
│       │               │       └───────────────┘   │
│  ┌────┴───────────────┴────┐                       │
│  │      两轮差速小车底盘    │                       │
│  └─────────────────────────┘                       │
└──────────────────────────────────────────────────┘
```

### 2.2 电机与驱动器

- **电机:** 两相步进电机 (1.8° 步距角)
- **驱动器:** DM542 或兼容驱动器 (16 细分, 共阳接法)
- **控制信号:** PULSE+DIR (共阳 — 公共端接 +5V, 脉冲端/方向端接 MSPM0 GPIO)

> ⚠️ **重要:** 驱动器细分 (DIP 开关) 必须与代码中 `STEPPER_PULSES_PER_REV` (3200) 保持一致。如果修改细分，必须同步修改此值。

### 2.3 传感器

- **型号:** MPU6050 (InvenSense/TDK)
- **通信:** I2C (400kHz Fast Mode)
- **地址:** 0x68 (AD0 = GND)
- **INT 引脚:** PB24 (上升沿触发, 数据就绪)
- **量程:** 陀螺仪 ±1000°/s, 加速度 ±4g
- **DLPF:** 44Hz, 采样率 200Hz (SMPLRT_DIV=4)

---

## 3. 软件架构

### 3.1 三层 IMU 架构

代码遵循严格的三层分层架构，每层职责明确：

```
┌───────────────────────────────────────────┐
│  应用层 (src/app_imu.c/h)                  │
│  - 互补滤波器 (Roll/Pitch/Yaw)             │
│  - 非重力加速度拒收 (碰撞/急加速保护)      │
│  - 校准 (陀螺仪零偏 / 加速度零偏)          │
│  - 姿态数据输出 (IMU_Data_t)               │
│  - I2C 连续故障自动重初始化                │
│  - 中断回调管理                            │
├───────────────────────────────────────────┤
│  中间层 (drivers/mid_mpu6050.c/h)          │
│  - MPU6050 寄存器映射 (完整寄存器宏)       │
│  - 初始化序列 (复位→唤醒→时钟源→量程→DLPF)│
│  - 原始数据读取 (14 字节 burst read)       │
│  - 量程/滤波器/采样率动态配置              │
│  - 自检 (SELF_TEST 寄存器)                 │
├───────────────────────────────────────────┤
│  硬件抽象层 (drivers/hal_mpu6050.c/h)      │
│  - I2C1 控制器 (TX/RX, 超时, FIFO)        │
│  - INT 引脚 (PB24 GPIO 中断)               │
│  - 数字毛刺滤波器 (4 BUSCLK = 125ns)       │
│  - 弱符号 GROUP1_IRQHandler                │
│  - I2C 等待/超时辅助函数                   │
└───────────────────────────────────────────┘
         ↓ 依赖
┌───────────────────────────────────────────┐
│  SysConfig 自动生成 (ti_msp_dl_config)     │
│  - 时钟树 (BUSCLK = 32MHz)                 │
│  - 引脚 IOMUX (I2C, PWM, GPIO)             │
│  - 定时器初始化 (TIMA0, TIMG6)             │
│  - 模拟毛刺滤波器 (50ns)                   │
└───────────────────────────────────────────┘
```

### 3.2 小车控制架构

```
┌────────────────────────────────────────────┐
│  src/car_control.c/h                        │
│  - 差速指令 → 左右轮 RPM 分解               │
│  - PD 航向保持 (yaw correction)             │
│  - 倾斜检测 → 紧急停止                      │
│  - 故障锁存与管理 (Car_ClearFault 解锁)     │
│  - 转速限幅 (clamp_command)                 │
│  - 死区过滤 (command_magnitude)             │
├────────────────────────────────────────────┤
│  drivers/stepper_pulse.c/h                  │
│  - 速度模式 (连续 PWM, 同向调速不中断)      │
│  - 位置模式 (ISR 计数 + 梯形速度曲线)       │
│  - RPM → 定时器周期换算 (核心公式)          │
│  - 方向反转安全序列 (CCP 冻结→换向→重启)    │
│  - 紧急停止 (禁用计数→CCP冻结→双轴清零)     │
│  - 幽灵脉冲防护 (busy守卫 + CCP冻结序列)    │
└────────────────────────────────────────────┘
```

### 3.3 主循环调度 (src/empty.c)

```
main():
  ├─ SYSCFG_DL_init()             // SysConfig 初始化 (时钟/定时器/I2C/GPIO)
  ├─ SysTick_Config(32M/1000)    // 1ms 系统时钟
  ├─ stepper_pulse_init()         // DIR 引脚 + ISR 使能
  ├─ Car_Init()                   // 小车状态初始化 (停止, 航向保持开启)
  ├─ init_imu()                   // IMU_Init + 陀螺校准(200次采样)
  │   └─ 成功 → start_forward_ten_turns()
  │       └─ stepper_pulse_move() × 2  // 左右轮各前进 5 圈 @ 50 RPM
  │
  └─ for(;;) 主循环:
       ├─ 每 5ms (200Hz):
       │    ├─ stepper_pulse_update() // 位置模式梯形速度曲线更新
       │    ├─ IMU_Update(&imu)       // I2C 读取 + 互补滤波
       │    └─ Car_Update(&imu)       // 航向修正 + 差速控制
       │
       └─ 每 1s (IMU 故障时):
            └─ init_imu()             // 自动重试初始化
```

---

## 4. 文件结构

```
motor_1/
├── src/                          # ★ 应用层源码
│   ├── empty.c                   #   主程序入口 (main + 调度循环 + 上电自检)
│   ├── car_control.h             #   小车差速控制头文件 + 核心参数定义
│   ├── car_control.c             #   差速控制 + PD 航向保持 + 故障管理
│   ├── app_imu.h                 #   IMU 姿态解算头文件 + 滤波器数据结构
│   └── app_imu.c                 #   互补滤波 + 校准 + I2C 故障恢复
│
├── drivers/                      # ★ 驱动层源码
│   ├── stepper_pulse.h           #   步进电机脉冲控制头文件 + 运动参数定义
│   ├── stepper_pulse.c           #   PWM 控制 + ISR + 梯形速度曲线 + RPM 换算
│   ├── delay.h                   #   毫秒/微秒延时头文件 (millis 内联)
│   ├── delay.c                   #   SysTick 忙等 + delay_cycles 实现
│   ├── hal_mpu6050.h             #   I2C + INT 硬件抽象层头文件
│   ├── hal_mpu6050.c             #   I2C1 TX/RX + GPIO 中断 + 毛刺滤波
│   ├── mid_mpu6050.h             #   MPU6050 完整寄存器映射 + 协议头文件
│   └── mid_mpu6050.c             #   初始化序列 + 原始数据 + 自检
│
├── Debug/                        # SysConfig 自动生成 (请勿手动修改)
│   ├── ti_msp_dl_config.h        #   外设宏定义 (定时器/I2C/GPIO/时钟)
│   ├── ti_msp_dl_config.c        #   外设初始化实现 (SYSCFG_DL_init)
│   └── motor_1.out               #   编译产物 (ELF)
│
├── empty.syscfg                  # SysConfig 配置文件
├── targetConfigs/
│   └── MSPM0G3507.ccxml          # 调试器目标配置
├── .ccsproject / .cproject       # CCS 工程配置
├── README.md                     # 英文 README
├── README.html                   # 英文 README (HTML)
└── motor_1_工程说明文档.md        # 本文件
```

---

## 5. 核心参数与调校指南

> ⚠️ **调试时必须阅读本节！** 以下参数直接影响电机行为、姿态精度和系统稳定性。

### 5.1 电机运动参数 (`drivers/stepper_pulse.h`)

| 参数 | 默认值 | 范围 | 作用 | 调校建议 |
|------|--------|------|------|----------|
| `STEPPER_PULSE_MAX_RPM` | 3000 | [100, 5000] | 物理最高转速，保护电机不堵转 | 堵转时**调小**；力矩充裕时**调大** |
| `STEPPER_PULSE_MIN_RPM` | 9 | [1, 100] | 物理最低转速，受16位定时器限制 | **不要改小！** 低速请用大细分或位置模式 |
| `STEPPER_PULSE_DEFAULT_RPM` | 1200 | [9, 3000] | 默认转速参考值 | 中速参考值，一般无需修改 |
| `STEPPER_PULSES_PER_REV` | 3200 | — | 每转脉冲数 (200全步×16细分) | **必须与驱动器DIP开关一致！** |
| `STEPPER_STEP_DEG` | 0.1125° | — | 单步角度 = 360°/PPR | 只读，由 PPR 决定 |
| `STEPPER_RPM_STOP` | 0 | — | 停止转速常量 | 固定值，传入 set_speed 即停止 |

#### RPM → 定时器周期换算（核心公式）

```
period = TIMER_CLK × 60 ÷ (RPM × PPR)

示例 (32MHz BUSCLK, 3200 PPR):
  RPM=3000  →  period ≈ 200     (高速, 力矩小, 易堵转)
  RPM=1200  →  period ≈ 500     (中速, 默认工作点)
  RPM=50    →  period ≈ 12000   (低速, 位置模式巡航)
  RPM=20    →  period ≈ 30000   (起步速度, 梯形曲线起始)
  RPM=9     →  period ≈ 66667   → 超过 65535, 被 PERIOD_MAX 截断!

定时器周期约束:
  PERIOD_MIN = 10:   防止周期过短 → ISR 风暴 (CPU 100%)
  PERIOD_MAX = 65535: 16 位定时器硬件上限
```

### 5.2 小车控制参数 (`src/car_control.h`)

| 参数 | 默认值 | 范围 | 作用 | 调校建议 |
|------|--------|------|------|----------|
| `CAR_MAX_RPM` | 1200 | [100, 3000] | 小车最高行驶转速 (业务限幅) | 不要超过 `STEPPER_PULSE_MAX_RPM` |
| `CAR_MAX_TILT_DEG` | 35.0° | [10.0, 60.0] | 最大允许倾斜角，超限触发急停 | 平地跑可调小；坡道路面调大 |
| `CAR_HEADING_KP` | 6.0 | [0.5, 50.0] | 航向保持比例系数 | 跑偏时**调大**；左右摆动时**调小** |
| `CAR_HEADING_KD` | 0.25 | [0.0, 5.0] | 航向保持微分系数 (阻尼) | 过冲振荡时**调大**；回正太慢时**调小** |
| `CAR_HEADING_CORRECTION_MAX` | 180 | [10, 500] | 航向修正 RPM 上限 | 修正量不够时调大；影响直行平稳性时调小 |
| `CAR_LEFT_FORWARD_DIR` | `STEPPER_DIR_CW` | CW/CCW | 左轮前进方向 (当前接线: CW=前进) | 若实车倒退，**交换**左右这两个定义 |
| `CAR_RIGHT_FORWARD_DIR` | `STEPPER_DIR_CCW` | CW/CCW | 右轮前进方向 (当前接线: CCW=前进) | 同上 |

#### 位置模式运动参数 (`src/car_control.h`)

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `CAR_FORWARD_TURNS` | 5 | 上电后自动前进圈数（用于验证接线和功能） |
| `CAR_FORWARD_RPM` | 50 | 位置模式目标转速 |
| `CAR_PROFILE_START_RPM` | 20 | 梯形速度曲线起步/结束速度 |
| `CAR_PROFILE_RAMP_STEPS` | 1600 | 加速段和减速段各占步数 (≈0.5圈) |

#### 梯形速度曲线说明

```
转速
  │
  ├────────────────────────────  目标 RPM (如 50)
  │              ╱‾‾‾‾‾‾‾‾‾╲
  │             ╱            ╲
  │            ╱              ╲
  ├───────────╱                ╲────  起步 RPM (20)
  │
  └──────┬──────┬──────────┬──────┬──→ 步数
       加速段   匀速段     减速段
    (1600步)             (1600步)

若总步数不足两个完整斜坡 → 自动切换为三角速度曲线 (无匀速段)
```

#### PD 航向修正公式

```
correction = -(KP × yaw_error + KD × gyro_z)

yaw_error = 当前偏航角 - 目标偏航角 (归一化到 ±180°)
gyro_z    = Z 轴角速度 (°/s), 正值表示正在右转
负号:     若 yaw_error 为正 (车头偏右), 需要左转 (负修正量)

> **调参口诀:** KP 调"硬"→ 快速回正；KD 调"软"→ 防止来回摆。
```

### 5.3 IMU 滤波器参数 (`src/app_imu.h` / `src/app_imu.c`)

| 参数 | 默认值 | 范围 | 作用 | 调校建议 |
|------|--------|------|------|----------|
| `IMU_DEFAULT_ALPHA` (α) | 0.98 | [0.90, 0.995] | 互补滤波系数 | 高速响应→调小 (0.96)；高稳定性→调大 (0.995) |
| `IMU_DEFAULT_DT` (dt) | 0.005s | [0.001, 0.1] | 滤波周期 (秒) | **必须与主循环 IMU_Update 调用间隔一致！** |
| `IMU_I2C_FAIL_RESET` | 20 | — | 连续I2C失败次数后自动重初始化 | 约 100ms @ 200Hz (20×5ms) |

#### 互补滤波公式

```
angle[n] = α × (angle[n-1] + gyro × dt) + (1-α) × accel_angle

α → 1 (0.995): 信任陀螺仪 → 响应快, 但漂移大
α → 0 (0.90):  信任加速度计 → 稳定, 但噪声大, 对振动敏感

★ 非重力加速度拒收:
  当 |accel_norm - 1.0g| ≥ 0.3g 时 (碰撞、急加速、急转弯),
  跳过加速度计修正, 仅用陀螺仪积分。
  防止外部加速度污染姿态估计。
```

#### 推荐参数组合

| 场景 | alpha | dt | 说明 |
|------|-------|-----|------|
| 高速追踪 | 0.96 | 0.005 | 适合快速转向, 紧跟运动 |
| 平衡模式 | 0.98 | 0.005 | **默认**, 适合一般行驶 |
| 静态保持 | 0.995 | 0.005 | 角度更稳定, 但响应变慢 |

### 5.4 定时器与时钟参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `CPUCLK_FREQ` | 32,000,000 | CPU 主频 (Hz), SysConfig 配置 |
| `TIMER_CLK_HZ` | 32,000,000 | 定时器时钟 (BUSCLK), 与 CPUCLK 相同 |
| `PERIOD_MIN` | 10 | 定时器周期最小值 (防止 ISR 风暴) |
| `PERIOD_MAX` | 65535 | 定时器周期最大值 (16 位硬件限制) |
| SysTick 周期 | 1ms | `SysTick_Config(CPUCLK_FREQ/1000U)` |

### 5.5 I2C 参数 (`drivers/hal_mpu6050.h`)

| 参数 | 值 | 说明 |
|------|-----|------|
| `MPU_I2C_TIMEOUT` | 200,000 | I2C 操作超时循环次数 |
| `MPU6050_ADDR_7BIT` | 0x68 | MPU6050 7-bit I2C 地址 (AD0=GND) |
| I2C TPR | 36 | 定时周期寄存器值 → SCL ≈ 400kHz |
| 数字毛刺滤波 | 4 BUSCLK | ≈ 125ns @ 32MHz, 抑制电机噪声 |
| 模拟毛刺滤波 | 50ns | SysConfig 自动配置 |

---

## 6. API 参考

### 6.1 小车控制 (`src/car_control.h`)

```c
// 初始化小车 (停止状态, 航向保持开启, 故障清除)
void Car_Init(void);

// 设置差速指令
//   linear_rpm: 前进速度 (正=前进, 负=后退)
//   turn_rpm:   差速量 (正=左转, 负=右转)
//   返回 false 表示故障锁存中 (需先 Car_ClearFault)
bool Car_SetVelocity(int16_t linear_rpm, int16_t turn_rpm);

// 停止 (不锁存故障, 后续可立即恢复运动)
void Car_Stop(void);

// 紧急停止 (★ 锁存故障, 需 Car_ClearFault() 解除)
// 触发条件: IMU 无效 | roll/pitch > 35° | s_fault 已置位
void Car_EmergencyStop(void);

// 清除锁存故障 (调用后小车仍处于停止状态)
void Car_ClearFault(void);

// 开启/关闭航向保持
//   enable=true:  直行时自动 PD 修正偏航
//   enable=false: 纯差速控制, 不修正偏航
void Car_SetHeadingHold(bool enable);

// ★ 核心控制更新 (每个控制周期调用, 当前 200Hz)
//   imu:       IMU 数据指针 (可为 NULL, 表示 IMU 未就绪)
//   imu_valid: IMU 数据是否有效 (I2C 读取成功)
//   返回 false = 触发了紧急停止
//   内部流程: 安全检查 → PD 航向修正 → 差速分解 → 硬件输出
bool Car_Update(const IMU_Data_t *imu, bool imu_valid);

// 读取当前状态 (只读, 不修改内部状态)
void Car_GetStatus(CarStatus_t *status);

// CarStatus_t 字段:
//   linear_rpm    — 当前线性速度指令
//   turn_rpm      — 当前差速转向指令
//   heading_hold  — 航向保持是否开启
//   fault         — 是否处于故障锁存状态
```

### 6.2 步进电机 (`drivers/stepper_pulse.h`)

```c
/* ── 轴选择器 ── */
#define STEPPER_AXIS_LEFT   0   // 同 STEPPER_AXIS_PAN
#define STEPPER_AXIS_RIGHT  1   // 同 STEPPER_AXIS_TILT
#define STEPPER_AXIS_PAN    0   // LEFT 轮 (TIMG6)
#define STEPPER_AXIS_TILT   1   // RIGHT 轮 (TIMA0)

/* ── 方向定义 ── */
#define STEPPER_DIR_CW      0   // 顺时针, DIR=LOW
#define STEPPER_DIR_CCW     1   // 逆时针, DIR=HIGH

/* ── 初始化 ── */
// 必须在 SYSCFG_DL_init() 之后调用一次
// 配置 DIR 引脚 + 使能 TIMA0/TIMG6 零事件中断 + NVIC + 清零所有状态
void stepper_pulse_init(void);

/* ── 速度模式 (连续行驶) ── */
// 持续 PWM 输出, 永不自动停止
// 同方向调速: 直接修改周期 → 脉冲不中断, 丝滑过渡
// 方向反转:  CCP 冻结 → 停止 → 换向 → 重启 (安全序列)
// rpm==0:     调用 stepper_pulse_stop()
void stepper_pulse_set_speed(uint8_t axis, uint8_t dir, uint16_t rpm);

/* ── 位置模式 (按步数行驶, 含梯形速度曲线) ── */
// 下发目标步数 → 启动 PWM → ISR 计数 → 到达后自动停止
// 非阻塞 — 函数立即返回, 主循环轮询 stepper_pulse_is_busy()
// 梯形速度曲线: 加速段(CAR_PROFILE_RAMP_STEPS步) → 匀速段 → 减速段
// 短行程自动切换为三角速度曲线
void stepper_pulse_move(uint8_t axis, uint8_t dir, uint16_t rpm, uint32_t steps);

// ★ 位置模式梯形速度曲线更新 — 必须在主循环中周期性调用
// 内部根据已完成步数和剩余步数动态调整 PWM 周期
void stepper_pulse_update(void);

/* ── 停止 ── */
// 停止指定轴 (CCP 安全序列: 停计数器→CCP冻结→busy=false→清中断)
void stepper_pulse_stop(uint8_t axis);

// 双轴紧急停止 (禁用ISR计数→CCP冻结双轴→停计数器→清中断→批量清零)
void stepper_pulse_emergency_stop(void);

/* ── 状态查询 ── */
bool     stepper_pulse_is_busy(uint8_t axis);       // 运动中?
uint16_t stepper_pulse_get_current_rpm(uint8_t axis); // 当前转速
uint8_t  stepper_pulse_get_current_dir(uint8_t axis); // 当前方向
int32_t  stepper_pulse_get_position_steps(uint8_t axis); // 累计步数 (带符号)
float    stepper_pulse_get_position_deg(uint8_t axis);   // 累计角度 (度)
void     stepper_pulse_set_position_zero(uint8_t axis);  // 标记逻辑零点
```

### 6.3 IMU (`src/app_imu.h`)

```c
/* ── 数据结构 ── */
typedef struct {
    float roll, pitch, yaw;   // 姿态角 (度)
    float accel_x, accel_y, accel_z;  // 加速度 (g), 已扣零偏
    float gyro_x, gyro_y, gyro_z;     // 角速度 (°/s), 已扣零偏
    float temperature;                // 芯片温度 (°C)
} IMU_Data_t;

/* ── 初始化 ── */
// 内部调用 MID_MPU6050_Init() → HAL_MPU6050_Init()
// 默认配置: 陀螺±1000°/s, 加速±4g, DLPF 44Hz, 采样率 200Hz
// 返回 false = 器件未检测到或配置失败
bool IMU_Init(void);

/* ── 校准 (传感器必须静止!) ── */
void IMU_CalibrateGyro(uint16_t samples);    // 建议 200~1000 次
void IMU_CalibrateAccel(uint16_t samples);   // 建议 100~500 次 (水平放置)

/* ── 核心更新 (200Hz, ★ 每 5ms 调用) ── */
// 流程: I2C读取14字节 → LSB转物理单位 → 扣零偏 →
//      加速度计算Roll/Pitch → 互补滤波融合 → 角度归一化 → 填充输出
// 返回 false: I2C读取失败或模块未初始化
// 连续失败20次 → 自动重初始化 IMU
bool IMU_Update(IMU_Data_t *data);

// 获取缓存数据 (不触发 I2C 读取, 零开销)
void IMU_GetData(IMU_Data_t *data);

/* ── 滤波器配置 ── */
// alpha: [0.90, 0.995]  dt: [0.001, 0.1]
void IMU_SetFilter(float alpha, float dt);

/* ── 辅助 ── */
void IMU_ResetYaw(void);                  // 偏航角归零
bool IMU_IsReady(void);                   // 模块已初始化?
const IMU_Calib_t *IMU_GetCalibration(void); // 获取校准数据 (只读)

/* ── 中断支持 (低功耗场景, 可选) ── */
void IMU_SetDataReadyCallback(void (*cb)(void));
bool IMU_EnableInterrupt(bool enable);
```

### 6.4 延时 (`drivers/delay.h`)

```c
// ★ 非阻塞时间戳 (主循环推荐, 内联函数)
// 返回自 SysTick 启动后的毫秒数 (uint32_t, 溢出周期 ≈ 49.7 天)
// 溢出安全用法: if ((uint32_t)(now - last) >= INTERVAL) { ... }
static inline uint32_t millis(void);

// 阻塞延时 (★ 仅初始化阶段使用, 主循环禁止!)
void delay_ms(uint32_t ms);   // 基于 SysTick 忙等
void delay_us(uint32_t us);   // 基于 delay_cycles

// 系统毫秒计数器 (需在 main.c 中定义)
extern volatile uint32_t sys_tick_ms;
```

---

## 7. 典型使用流程

### 7.1 完整初始化序列

```c
int main(void)
{
    // 1. SysConfig 外设初始化 (时钟, IOMUX, 定时器, I2C)
    SYSCFG_DL_init();

    // 2. 启动系统时钟 (1ms SysTick)
    SysTick_Config(CPUCLK_FREQ / 1000U);

    // 3. 步进电机初始化 (DIR 引脚 + ISR)
    stepper_pulse_init();

    // 4. 小车控制初始化
    Car_Init();

    // 5. IMU 初始化 + 陀螺仪校准
    IMU_Data_t imu = {0};
    bool imu_ready = IMU_Init();
    if (imu_ready) {
        IMU_CalibrateGyro(200);   // 静止采样 200 次, 约 1s
        IMU_ResetYaw();           // 偏航角归零

        // 可选: 启动上电自检 (前进5圈验证接线)
        stepper_pulse_move(STEPPER_AXIS_LEFT,
            CAR_LEFT_FORWARD_DIR, CAR_FORWARD_RPM,
            CAR_FORWARD_TURNS * STEPPER_PULSES_PER_REV);
        stepper_pulse_move(STEPPER_AXIS_RIGHT,
            CAR_RIGHT_FORWARD_DIR, CAR_FORWARD_RPM,
            CAR_FORWARD_TURNS * STEPPER_PULSES_PER_REV);
    }

    // 6. 主循环
    uint32_t next_update = millis();
    uint32_t next_retry  = next_update + 1000U;
    while (1) {
        uint32_t now = millis();

        // 每 5ms (200Hz) 控制更新
        if ((int32_t)(now - next_update) >= 0) {
            next_update += 5U;

            stepper_pulse_update();      // 梯形速度曲线更新
            if (imu_ready) {
                imu_ready = IMU_Update(&imu);
                Car_Update(&imu, imu_ready);
            } else {
                Car_Update(NULL, false);  // 安全停车
            }
        }

        // 每 1s IMU 自动重连
        if (!imu_ready && (int32_t)(now - next_retry) >= 0) {
            next_retry += 1000U;
            imu_ready = IMU_Init() && IMU_CalibrateGyro(200), IMU_ResetYaw(), true;
        }
    }
}
```

### 7.2 差速行驶控制示例

```c
// 前进 800 RPM, 直行, 航向自动保持
Car_SetVelocity(800, 0);

// 前进中左转 (左轮快于右轮)
Car_SetVelocity(600, 200);  // 左轮 800 RPM, 右轮 400 RPM → 左转

// 原地右转 (差速转向)
Car_SetVelocity(0, -300);   // 左轮 -300, 右轮 +300 → 原地右转

// 后退
Car_SetVelocity(-400, 0);

// 停止 (可立即恢复)
Car_Stop();

// 紧急停止 (锁存, 需 Car_ClearFault 解锁)
Car_EmergencyStop();
```

### 7.3 按步数行驶示例 (位置模式)

```c
// 左轮前进 3200 步 (= 1 圈) @ 200 RPM, 梯形加减速
stepper_pulse_move(STEPPER_AXIS_LEFT, STEPPER_DIR_CW, 200, 3200);

// 非阻塞轮询等待完成
while (stepper_pulse_is_busy(STEPPER_AXIS_LEFT)) {
    // 主循环继续执行其他任务
    // 梯形速度曲线由 stepper_pulse_update() 自动维护
}

// 读取当前位置
int32_t steps = stepper_pulse_get_position_steps(STEPPER_AXIS_LEFT);
float angle   = stepper_pulse_get_position_deg(STEPPER_AXIS_LEFT); // 度

// 标记当前为零点
stepper_pulse_set_position_zero(STEPPER_AXIS_LEFT);
```

---

## 8. 调试与故障排查

### 8.1 电机不转

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 电机完全不动 | 驱动器未上电 / 使能脚悬空 | 检查驱动器电源和 ENA 引脚 |
| 电机啸叫但不转 (堵转) | RPM 太高, 力矩不足 | **降低** `STEPPER_PULSE_MAX_RPM` 或 `CAR_MAX_RPM` |
| 电机抖动 | 线序错误 (A+/A-/B+/B- 接错) | 检查电机四线接线 |
| 脉冲有输出但电机不动 | DIR 信号未切换 / 共阳接法错误 | 用示波器检查 PULSE 和 DIR 脚波形 |
| 上电后前进方向不对 | 方向宏配置与接线不一致 | 交换 `CAR_LEFT_FORWARD_DIR` 和 `CAR_RIGHT_FORWARD_DIR` |

### 8.2 方向错误

| 现象 | 解决方法 |
|------|----------|
| 前进变后退 | 交换 `CAR_LEFT_FORWARD_DIR` 和 `CAR_RIGHT_FORWARD_DIR` |
| 左转变右转 | 交换两个 `FORWARD_DIR` 宏或检查驱动器 DIR 极性 |
| 仅一个轮方向正确 | 单独修改该轮的 `FORWARD_DIR` 宏 |

> **当前接线配置:** 左轮前进=STEPPER_DIR_CW, 右轮前进=STEPPER_DIR_CCW (镜像安装)

### 8.3 角度漂移 / IMU 异常

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| Roll/Pitch 严重漂移 | 未校准或校准时机身未静止 | 重新调用 `IMU_CalibrateGyro(500)` |
| Yaw 快速漂移 | 正常现象 — 无磁力计, 纯积分必漂 | 使用 `IMU_ResetYaw()` 定期归零或启用航向保持 |
| IMU_Init 返回 false | MPU6050 未连接 / 地址错误 | 检查 I2C 线连接, 确认 AD0=GND (地址=0x68) |
| I2C 读取频繁失败 | 电机 EMI 干扰 I2C 总线 | 缩短 I2C 线长, 增加屏蔽, 确保毛刺滤波器已使能 |
| 连续 I2C 失败后自动重启 | `IMU_I2C_FAIL_RESET=20` 触发 | 正常行为: 20帧(100ms)连续失败 → 自动重初始化 |

### 8.4 航向保持异常

| 现象 | 调校方法 |
|------|----------|
| 小车直行严重跑偏 | **增大** `CAR_HEADING_KP` (逐步 +2) |
| 小车左右摆动前进 | **增大** `CAR_HEADING_KD` (逐步 +0.1) 或 **减小** `CAR_HEADING_KP` |
| 航向修正太慢/无力 | **增大** `CAR_HEADING_CORRECTION_MAX` |
| 正常加减速误触发急停 | **增大** `CAR_MAX_TILT_DEG` |

### 8.5 位置模式异常

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 步数与实际不符 | `STEPPER_PULSES_PER_REV` 与驱动器细分不一致 | 核对驱动器 DIP 开关, 同步修改 PPR |
| 高速丢步 | RPM 过高 + 惯性负载过大 | 降低 RPM 或增大 `CAR_PROFILE_RAMP_STEPS` |
| 梯形曲线不起作用 | 未在主循环中调用 `stepper_pulse_update()` | 确保每个控制周期都调用 |
| 速度曲线不对称 | 总步数 < 2×`CAR_PROFILE_RAMP_STEPS` | 正常: 短行程自动切换为三角速度曲线 |

### 8.6 幽灵脉冲 (位置计数异常)

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 停止后位置仍在跳变 | g_busy 守卫失效或中断残留 | 检查 ISR 停止序列: CCP冻结→busy=false→stopCounter |
| 高速时角度误差 1.7°+ | 停止后定时器产生额外零事件被计入 | 确保 g_busy 在 stopCounter 之前设为 false |

### 8.7 编译/下载问题

| 现象 | 解决方法 |
|------|----------|
| `ti_msp_dl_config.h` not found | 确保已在 CCS 中打开工程并通过 SysConfig 生成配置文件 |
| delay_ms 调试时卡死 | CCS → 调试属性 → 取消 "Pause SysTick while debugging" |
| 编译警告 `SYSCONFIG_WEAK` | 正常, 弱符号 GROUP1_IRQHandler 的预期行为 |

---

## 9. 引脚映射表

### 9.1 步进电机

| 信号 | TILT 轴 (RIGHT 轮) | PAN 轴 (LEFT 轮) |
|------|--------------------|--------------------|
| 定时器 | TIMA0 | TIMG6 |
| 定时器类型 | TimerA (通用) | TimerG (通用) |
| PULSE 引脚 | PB0 (CCP2) | PB2 (CCP0) |
| DIR 引脚 | PB13 | PB3 |
| DIR IOMUX | PINCM30 | PINCM16 |
| IRQ 处理函数 | `TIMA0_IRQHandler` | `TIMG6_IRQHandler` |
| SysConfig 名称 | `cloud_1_TILT` | `cloud_2_PAN` |

### 9.2 MPU6050

| 信号 | MSPM0G3507 引脚 | IOMUX | 功能 |
|------|-----------------|-------|------|
| SDA | PA16 | PINCM38 | I2C1_SDA |
| SCL | PA17 | PINCM39 | I2C1_SCL |
| INT | PB24 | PINCM52 | GPIO 输入 (数据就绪, 上升沿) |
| DRDY | PA13 | — | GPIO 输入 (备用, 未使用) |
| VCC | 3.3V | — | 电源 |
| GND | GND | — | 电源地 |

### 9.3 调试与系统

| 信号 | 引脚 | 说明 |
|------|------|------|
| SWCLK | PA20 | 调试时钟 (J101 15:16 ON) |
| SWDIO | PA19 | 调试数据 (J101 13:14 ON) |
| 3.3V | — | MSPM0G3507 核心电源 + MPU6050 |
| 5V | — | 步进电机驱动器光耦侧供电 |
| GND | — | 所有模块共地 |

---

## 10. SysConfig 配置指南

如果需要重建或验证 SysConfig 配置，请按以下步骤操作：

### 10.1 定时器配置

**cloud_1_TILT (TIMA0) — RIGHT 轮:**
- 模式: PWM Edge-Align Up
- 时钟源: BUSCLK (32MHz), 分频 /1
- CCP2 输出: PB0
- 零事件中断: 使能 (用于步数计数 ISR)

**cloud_2_PAN (TIMG6) — LEFT 轮:**
- 模式: PWM Edge-Align Up
- 时钟源: BUSCLK (32MHz), 分频 /1
- CCP0 输出: PB2
- 零事件中断: 使能 (用于步数计数 ISR)

### 10.2 I2C 配置

**I2C_1:**
- SDA: PA16, SCL: PA17
- 模式: Controller (Master)
- 速率: Fast Mode (400kHz, TPR=36)
- 模拟毛刺滤波: 50ns (SysConfig 自动)
- 时钟延展: 使能 (HAL 层手动配置)

### 10.3 GPIO 配置

**GPIO_MPU_INT:**
- PIN_INT (PB24): 数字输入, 无上下拉, 无迟滞
- DRDY (PA13): 数字输入, 无上下拉, 无迟滞 (备用, 未使用)

---

## 11. 附录

### 附录 A: 速度模式 vs 位置模式

| 特性 | 速度模式 | 位置模式 |
|------|----------|----------|
| API | `stepper_pulse_set_speed()` | `stepper_pulse_move()` |
| 停止方式 | 主循环显式调用 stop | ISR 自动停止 |
| 步数控制 | 无 (无限脉冲) | 精确到步 |
| 梯形速度曲线 | 无 | 有 (加速→匀速→减速) |
| 主循环维护 | 调速时调用 set_speed | 需周期调用 `stepper_pulse_update()` |
| 适用场景 | 连续行驶、差速转向 | 按步数行驶、定位、上电自检 |
| 调速 | 同方向丝滑, 反向先停后启 | 运动中由 update 自动调速 |
| 阻塞性 | 非阻塞 (函数立即返回) | 非阻塞 (轮询 is_busy) |

### 附录 B: 电机方向定义 (当前接线)

```
从电机输出轴方向看:

  CW  (顺时针, STEPPER_DIR_CW =0): DIR 引脚 = LOW  → 电机正转
  CCW (逆时针, STEPPER_DIR_CCW=1): DIR 引脚 = HIGH → 电机反转

小车前进方向 (左右镜像安装, 当前接线):
  左轮前进 = CW  (顺时针, CAR_LEFT_FORWARD_DIR  = STEPPER_DIR_CW)
  右轮前进 = CCW (逆时针, CAR_RIGHT_FORWARD_DIR = STEPPER_DIR_CCW)

★ 若实车运动方向相反, 交换 car_control.h 中的两个 FORWARD_DIR 宏即可。
```

### 附录 C: I2C 故障自动恢复流程

```
IMU_Update() 调用
  ├─ MID_MPU6050_ReadRaw() 成功
  │   └─ s_i2c_fail_count = 0   // 复位计数器
  │   └─ 正常姿态解算
  │
  └─ MID_MPU6050_ReadRaw() 失败
      └─ s_i2c_fail_count++
      └─ if s_i2c_fail_count >= 20:
          ├─ s_initialized = false
          ├─ IMU_Init()           // 重新初始化整个三层架构
          └─ s_i2c_fail_count = 0
      └─ return false             // 本次更新失败, data 不变
```

### 附录 D: 幽灵脉冲防护机制

```
零事件 ISR 中的防护:
  if (g_step_count_enabled && g_busy[axis]) {
      // ★ g_busy 守卫: 定时器 stopCounter 是异步的,
      //   停止后可能仍有 1~N 个零事件, g_busy=false 阻止计入
      更新位置 / 递减步数
  }

停止序列 (CCP 冻结安全协议):
  速度模式 stop:
    1. DL_TimerX_stopCounter()       // 先停计数器
    2. setCaptureCompareValue(MAX)   // CCP 冻结 (输出锁死高电平)
    3. g_busy = false                // 拒收幽灵中断
    4. clearInterruptStatus()        // 清残留标志

  紧急停止:
    1. g_step_count_enabled = false  // ★ 先禁用全局 ISR 计数
    2. CCP 冻结双轴 (CCR=MAX)
    3. stopCounter 双轴
    4. clearIRQ 双轴
    5. 批量清零所有状态
    6. g_step_count_enabled = true   // 恢复 ISR
```

### 附录 E: 差速控制公式

```
差速分解:
  left_rpm  = linear_rpm + turn_rpm    (clamp 到 ±CAR_MAX_RPM)
  right_rpm = linear_rpm - turn_rpm    (clamp 到 ±CAR_MAX_RPM)

物理含义:
  turn_rpm = 0               → 直行, 左右轮同速
  turn_rpm > 0               → 左轮快于右轮 → 车右转
  turn_rpm = linear_rpm      → 右轮 = 0, 左轮 = 2×linear → 原地左转
  linear_rpm = 0, turn_rpm   → 原地旋转 (差速转向, 中心在两轮连线中点)

死区过滤:
  |command| < STEPPER_PULSE_MIN_RPM (9) → 输出 0 (停止)
  原因: 低于 9 RPM 定时器周期超过 16 位上限, 无法产生有效脉冲
```

---

*文档基于 motor_1 工程源码 (2026-07-27) 完整审阅生成。如有疑问请参考源码中各 .c/.h 文件的详细注释块。*
