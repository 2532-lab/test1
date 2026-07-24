# Test1 双轴云台激光追踪 — 工程说明与调参手册

> MSPM0G3507 + K230 庐山派 + MPU6050 + HMC5883L | 2026-07-22

---

## 1. 系统架构

### 1.1 硬件拓扑

```
┌──────────────────────────────────────┐
│  GC2093 摄像头 + 激光笔 (同轴固定)     │ ← TILT 电机旋转轴
│  MPU6050 + HMC5883L (GY-87)          │ ← 姿态感知
└──────────────┬───────────────────────┘
               │ TILT 电机: TIMA0 CCP2=PB0, DIR=PB13
┌──────────────┴───────────────────────┐
│  PAN 电机: TIMG6 CCP0=PB2, DIR=PB3   │
└──────────────────────────────────────┘
         UART2 (PB17_TX / PB18_RX) ←→ K230 115200 8N1
         I2C1  (PA16_SDA / PA17_SCL) ←→ MPU6050 + HMC5883L
```

| 组件 | 参数 |
|:---|:---|
| MCU | MSPM0G3507, Cortex-M0+, 32MHz |
| 步进驱动 | 16 细分, 3200 PPR, 0.1125°/步 |
| 视觉 | K230 庐山派 + GC2093, 320×240 RGB565, ~50fps |
| IMU | MPU6050 (6轴) + HMC5883L (3轴磁力计) |
| 协议 | `$TARGET,h_deg,v_deg\r\n` (2字段) / `$TARGET,NONE\r\n` |

### 1.2 状态机

```
上电 ──→ INIT (K230 等待)
            │
            ▼
       ┌ SCANNING ──────────────┐
       │ PAN ±45° 往复扫描       │
       │ TILT 固定 10° 下俯      │
       └────┬───────────────────┘
            │ 收到 $TARGET,h,v → EMA 初始化 + 立即下发首次追踪
            ▼
       ┌ TRACKING ──────────────┐
       │ EMA 平滑 + IMU 前馈    │
       │ 每 20ms 计算目标位置    │
       └────┬───────────────────┘
            │ 超时 2000ms 无数据 / 连续 NONE
            └──→ SCANNING
```

### 1.3 追踪公式

```
PAN:  tp = cp + PAN_DIR_SIGN × (ema_h + offset_pan) × TRACK_GAIN + pan_ff
TILT: tt = ct + TILT_DIR_SIGN × (ema_v + offset_tilt) × TRACK_GAIN + tilt_ff

其中:
  pan_ff  = -(yaw_delta - pan_delta) × KP_IMU_PAN       ← IMU 扰动前馈(PAN)
  tilt_ff = +(pitch_delta + tilt_delta) × KP_IMU_TILT    ← IMU 扰动前馈(TILT)
```

### 1.4 文件结构

```
test1/
├── src/
│   ├── vision_tracking.h    ← ★ 全部可调参数 (改这里!)
│   ├── vision_tracking.c    ← ★ 主逻辑: UART ISR, 协议解析, 状态机, EMA, IMU前馈
│   ├── empty.c              ← main() 入口
│   └── app_imu_9dof.c/h     ← 9-DOF 姿态融合
├── drivers/
│   ├── gimbal_control.c/h   ← 云台角度控制 (gimbal_control_angle)
│   ├── stepper_pulse.c/h    ← 步进 PWM + ISR 步数计数
│   ├── hal_mpu6050.c/h      ← MPU6050 I2C 底层
│   ├── mid_mpu6050.c/h      ← MPU6050 中间层
│   ├── hal_hmc5883l.c/h     ← HMC5883L I2C 底层
│   ├── mid_hmc5883l.c/h     ← HMC5883L 中间层
│   └── delay.c/h            ← SysTick 毫秒时钟
├── canmv/                   ← K230 MicroPython 程序
└── docs/                    ← 本文档
```

---

## 2. 完整参数清单

> **约定**: 所有 `#define` 参数在 `src/vision_tracking.h`，特殊情况会标注文件路径。

### 2.1 追踪性能 (`vision_tracking.h` §①)

| 参数 | 当前值 | 含义 |
|:---|:---|:---|
| `TRACK_LOOP_MS` | 20 | 追踪主循环周期 (ms), 50Hz |
| `TRACK_EMA_ALPHA` | 0.25 | EMA 平滑系数 (0~1). 小=更平滑响应慢, 大=响应快噪声多 |
| `TRACK_GAIN` | 0.5 | 视觉追踪增益. <1 提供阻尼防过冲; 每个周期加 `EMA × GAIN` 到目标角 |
| `ANGLE_JUMP_LIMIT` | 20.0 | 单帧角度突变保护 (°). 超过此值的帧被丢弃, EMA 不更新 |
| `TRACK_RPM` | 30 | 追踪时电机转速. 30RPM = 每秒 3.6° (每 20ms 周期可修正 3.6°) |
| `TARGET_LOST_TIMEOUT_MS` | 2000 | 连续无数据超时 (ms). 超时后 TRACK→SCAN |

### 2.2 扫描搜索 (`vision_tracking.h` §②)

| 参数 | 当前值 | 含义 |
|:---|:---|:---|
| `SCAN_RPM` | 9 | 扫描转速. 设为 0 则跳过扫描运动 (静默等待靶子) |
| `SCAN_PAN_RANGE` | 45.0 | 水平扫描范围 (±°) |
| `SCAN_TILT_ANGLE` | 10.0 | 扫描时 TILT 固定角 (°). 正值=向下, 稍向下便于看到靶子 |
| `SCAN_BOUNDARY_MARGIN` | 0.5 | 扫描边界余量 (°). 防浮点抖动导致反复换向 |
| `K230_BOOT_WAIT_MS` | 5000 | INIT 阶段等待 K230 启动的时间 (ms) |
| `UART_STALL_RECOVERY_MS` | 30000 | UART 解析停滞自动恢复 (ms). 0=禁用 |

### 2.3 方向符号 (`vision_tracking.h` §②½)

| 参数 | 当前值 | 含义 |
|:---|:---|:---|
| `PAN_DIR_SIGN` | +1.0 | PAN 方向: +1=同向, -1=反向. 靶子在右→PAN 向右转 |
| `TILT_DIR_SIGN` | -1.0 | TILT 方向: +1=同向, -1=反向. 靶子在上→TILT 向上转 |

> ⚠️ **重要**: 如果追踪时某轴运动方向与预期相反 (靶子在上方电机却向下转), 优先检查此参数。两个轴的 DIR 引脚极性相同 (CCW→HIGH→position++), 所以通常两个符号应一致。

### 2.4 IMU 增稳 (`vision_tracking.h` §③)

| 参数 | 当前值 | 含义 |
|:---|:---|:---|
| `KP_IMU_PAN` | 0.6 | PAN 轴 IMU 前馈增益 (0~1). 抵消 yaw 方向外部扰动 |
| `KP_IMU_TILT` | 0.4 | TILT 轴 IMU 前馈增益 (0~1). 抵消 pitch 方向外部扰动 (较保守) |

> IMU 坐标系: pitch 正=相机抬头, TILT 电机角度正=相机低头。两者符号相反，`tilt_ff` 公式已处理此差异。

### 2.5 电机驱动 (`drivers/gimbal_control.h` + `drivers/stepper_pulse.h`)

| 参数 | 当前值 | 位置 | 含义 |
|:---|:---|:---|:---|
| `GIMBAL_PAN_ANGLE_MAX` | 170.0° | gimbal_control.h | PAN 硬限幅 |
| `GIMBAL_TILT_ANGLE_MAX` | 90.0° | gimbal_control.h | TILT 硬限幅 |
| `GIMBAL_ANGLE_DEADBAND` | 0.12° | gimbal_control.h | 角度死区 (>单步0.1125°，防亚步抖动) |
| `GIMBAL_ANGLE_DEFAULT_RPM` | 300 | gimbal_control.h | 角度模式 RPM 回退值 (rpm=0 时触发) |
| `STEPPER_PULSE_MAX_RPM` | 3000 | stepper_pulse.h | 最大转速. 超过可能失步/堵转 |
| `STEPPER_PULSE_MIN_RPM` | 9 | stepper_pulse.h | 最低转速 |
| `STEPPER_PULSES_PER_REV` | 3200 | stepper_pulse.h | 每转脉冲数 (200×16细分) |

---

## 3. 场景化调参指南

### 场景 A — 追踪方向反了

**现象**: 靶子在某方向，电机往反方向跑，越跑越远，最终冲向限位。

| 检查项 | 参数/位置 | 操作 |
|:---|:---|:---|
| 垂直方向反 | `vision_tracking.h`: `TILT_DIR_SIGN` | 翻转: +1⇄-1 |
| 水平方向反 | `vision_tracking.h`: `PAN_DIR_SIGN` | 翻转: +1⇄-1 |

> 验证方法: 靶子放在摄像头中心**上方**→电机应**向上**转 (TILT角度减小); 靶子在**右方**→PAN应**向右**转 (PAN角度增大)。

---

### 场景 B — 追踪过冲/振荡

**现象**: 电机追到目标后继续冲过去，然后在目标两侧来回摆。

| 优先级 | 参数 | 调整方向 | 原理 |
|:---|:---|:---|:---|
| ① | `TRACK_GAIN` | **降低**: 0.5 → 0.3~0.4 | 减小每周期修正量 = 减小冲量 |
| ② | `TRACK_EMA_ALPHA` | **降低**: 0.25 → 0.10~0.15 | 更平滑 → 响应更慢但更稳 |
| ③ | `TRACK_RPM` | **降低**: 30 → 15~20 | 电机慢一点, 给视觉反馈留时间 |

---

### 场景 C — 追踪响应慢/滞后

**现象**: 靶子移动后，电机慢半拍才跟上，光斑总落在靶子后面。

| 优先级 | 参数 | 调整方向 | 原理 |
|:---|:---|:---|:---|
| ① | `TRACK_GAIN` | **提高**: 0.5 → 0.7~0.85 | 增大每周期修正量 |
| ② | `TRACK_EMA_ALPHA` | **提高**: 0.25 → 0.35~0.45 | 减少平滑 → 更快响应变化 |
| ③ | `TRACK_RPM` | **提高**: 30 → 60~120 | 电机更快到达目标位置 |
| ④ | `TRACK_LOOP_MS` | **减小**: 20 → 10~15 | 提高控制频率 |

---

### 场景 D — 扫描时漏靶

**现象**: 靶子在视野内但云台扫描时没发现，或扫过去才看到但已错过。

| 优先级 | 参数 | 调整方向 | 原理 |
|:---|:---|:---|:---|
| ① | `SCAN_RPM` | **降低**: 9 → 5~7 | 扫慢一点, 给 K230 更多帧来检测 |
| ② | `SCAN_PAN_RANGE` | **增大**: 45° → 60°~90° | 覆盖更宽视野 |
| ③ | `SCAN_TILT_ANGLE` | **调整**: 10° → 5°~20° | 匹配靶子实际高度 |
| ④ | `ANGLE_JUMP_LIMIT` | **增大**: 20° → 30°~40° | 放宽突变保护 (靶子边缘出现时可能跳变大) |

---

### 场景 E — 目标频繁丢失/光标抖动

**现象**: 追踪中频繁切换到 SCAN，或光斑在靶心附近抖动。

| 优先级 | 参数 | 调整方向 | 原理 |
|:---|:---|:---|:---|
| ① | `TARGET_LOST_TIMEOUT_MS` | **增大**: 2000 → 3000~5000 | 容忍更长的短暂丢失 |
| ② | `TRACK_EMA_ALPHA` | **降低**: 0.25 → 0.10~0.15 | 更平滑 = 更少抖动 |
| ③ | `ANGLE_JUMP_LIMIT` | **减小**: 20° → 8°~12° | 更严格过滤噪声帧 |
| ④ | `GIMBAL_ANGLE_DEADBAND` | **增大**: 0.12° → 0.15°~0.20° | 增大死区抑制微小振动 |

---

### 场景 F — 电机堵转/失步

**现象**: 电机发出异响但不转，或转动角度明显不对。

| 优先级 | 参数 | 调整方向 | 原理 |
|:---|:---|:---|:---|
| ① | `TRACK_RPM` / `SCAN_RPM` | **降低**: 30→15, 9→5 | 降低转速=增大扭矩 |
| ② | `STEPPER_PULSE_MAX_RPM` | **降低**: 3000 → 1500~2000 | 硬限制最高转速 |
| ③ | 机械 | 检查负载/驱动电流 | 减小云台负载或增大驱动器电流 |

---

### 场景 G — 激光光斑与靶心有固定偏差

**现象**: 追踪稳定但光斑始终偏某个方向 (不在靶心正中)。

| 优先级 | 方法 | 操作 |
|:---|:---|:---|
| ① | CCS Expressions 在线调 | `Vision_SetLaserOffset(pan_deg, tilt_deg)` 实时生效 |
| ② | 回写到代码 | 调好后把值写入 `main.c` 中 `Vision_SetLaserOffset(x, y)` |

> 光斑偏右 → `pan_offset` 正; 光斑偏左 → `pan_offset` 负。
> 光斑偏下 → `tilt_offset` 正; 光斑偏上 → `tilt_offset` 负。
> ★ offset 乘以 `TRACK_GAIN` 后生效, 修改 GAIN 需重新标定 offset。

---

### 场景 H — 外部扰动致光斑抖动 (风吹/碰到底座)

**现象**: 追踪本身没问题，但轻微碰触云台底座或风吹后光斑偏移。

| 优先级 | 参数 | 调整方向 | 原理 |
|:---|:---|:---|:---|
| ① | `KP_IMU_PAN` | **增大**: 0.6 → 0.8~1.0 | 更强的 yaw 扰动前馈 |
| ② | `KP_IMU_TILT` | **增大**: 0.4 → 0.6~0.8 | 更强的 pitch 扰动前馈 |
| ③ | IMU 过强自激 | **减小**: KP→0.2~0.3 | 前馈太强反而不稳 |
| ④ | 完全禁用 IMU | **全设 0** | 降级为纯视觉追踪 |

---

### 场景 I — K230 通信异常

**现象**: `g_uart_isr_rx_count=0` (ISR 从未触发) 或 `g_parsed.cnt_ok=0` (收到字节但无有效帧)。

**诊断链** (逐层排查，值为 0 的那层就是问题所在):

| 层级 | CCS 查看变量 | 为 0 说明... |
|:---|:---|:---|
| ① 物理层 | `g_rx_total_bytes` | TX/RX 线接反、波特率不匹配、K230 未发数据 |
| ② ISR 层 | `g_uart_isr_rx_count` | ISR 函数名与向量表不匹配 (见 `.claude.md` §3.1) |
| ③ 协议层 | `g_parsed.cnt_ok` | 波特率错或 K230 协议格式不匹配 |
| ④ 状态机 | `g_parsed.has_angle` | K230 持续发 NONE, 未检测到靶子 |

修复后调用 `Vision_EmergencyStop()` 或在 CCS Expressions 中设 `s_stage=ST_INIT` 重新初始化。

---

### 场景 J — K230 启动时序问题

**现象**: 上电后云台在 SCAN 阶段卡住不收帧，`g_parsed.cnt_bad` 持续增长。

| 操作 | 参数 | 调整 |
|:---|:---|:---|
| 延长等待 | `K230_BOOT_WAIT_MS` | 5000 → 10000~15000 (K230 MicroPython 启动需要时间) |
| 自动恢复 | `UART_STALL_RECOVERY_MS` | 30000 → 10000 (更快触发自动冲洗) |

---

### 场景 K — 仅 PAN 或仅 TILT 不工作

**现象**: 一个轴完全不动，另一个正常。

| 检查项 | 位置 |
|:---|:---|
| 堵转标志 | CCS 查看 `gimbal_pan.fault` / `gimbal_tilt.fault` |
| 角度限幅 | 是否目标角已到 ±170°/±90° 限位 |
| DIR 引脚 | PB3(PAN) / PB13(TILT) 接线 |
| PULSE 引脚 | PB2(PAN) / PB0(TILT) 接线 |
| 定时器 ISR | 检查 `g_position_steps[0]` / `[1]` 是否变化 |

---

## 4. 安全机制

| 机制 | 位置 | 说明 |
|:---|:---|:---|
| 角度硬限幅 | `gimbal_control.c` | PAN ±170°, TILT ±90°, clampf 含 NaN 守卫 |
| 角度突变保护 | `vision_tracking.c` | 帧间变化 > ANGLE_JUMP_LIMIT → 拒绝, EMA 不变 |
| NaN/Inf 多层守卫 | `vision_tracking.c` | EMA 入口、追踪公式入口、clampf 三层防御 |
| 目标丢失超时 | `vision_tracking.c` | 超时 → TRACK→SCAN |
| 环形缓冲区溢出 | `vision_tracking.c` ISR | 满则丢弃, 同步统计 g_rx_overflow |
| 堵转检测 | `gimbal_control.c` | 占位, 可扩展编码器/电流检测 |
| IMU 降级 | `vision_tracking.c` | 初始化失败 → 纯视觉追踪, 不宕机 |

---

## 5. 调试技巧

### 5.1 CCS Expressions 关键变量

| 变量 | 类型 | 含义 |
|:---|:---|:---|
| `sys_tick_ms` | uint32_t | 系统毫秒计数器 (持续递增=正常) |
| `g_uart_isr_rx_count` | uint32_t | UART ISR 触发次数 (>0=ISR工作) |
| `g_rx_total_bytes` | uint32_t | 累计接收字节数 (>0=物理连接OK) |
| `g_parsed` | struct | 协议解析结果 (cnt_ok / cnt_none / cnt_bad / has_angle / h_deg / v_deg) |
| `s_stage` | enum | 当前状态: 0=INIT, 1=SCAN, 2=TRACK |
| `s_ema_h` / `s_ema_v` | float | EMA 平滑后的水平/垂直偏差 |
| `s_target_pan` / `s_target_tilt` | float | 最近一次下发的目标角度 |
| `gimbal_pan.angle` / `gimbal_tilt.angle` | float | 软件估算的当前角度 |
| `g_position_steps[0]` / `[1]` | int32_t | 硬件步数计数器 (PAN=0, TILT=1) |
| `s_pan_ff` | float | 最近一次 PAN 轴 IMU 前馈量 |

### 5.2 在线调参

- **激光偏差**: CCS Expressions 中直接调用 `Vision_SetLaserOffset(x, y)`, 实时生效
- **参数宏**: 修改 `vision_tracking.h` 中的 `#define`, 重新编译烧录
- **临时改 static 变量**: 把 `static` 去掉 → 全局变量 → CCS Expressions 直接读写

### 5.3 烧录后快速验证

1. 上电 → 激光点亮
2. INIT → 观察 `sys_tick_ms` 递增
3. SCAN → PAN 往复扫描, TILT 停在 ~10°
4. 靶子放摄像头前方 → `g_parsed.has_angle` 变为 true → TRACK
5. 移动靶子 → 电机跟踪
6. 撤走靶子 → 2s 后切回 SCAN
7. 轻推云台底座 → 磁力计前馈对抗扰动

---

> 版本: v9 (TILT_DIR_SIGN修复 + IMU前馈tilt_ff修复) | MCU: MSPM0G3507 | 电机: 16细分 3200PPR
