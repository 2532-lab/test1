/*
 * car_control.c — motor_1 两轮小车差速控制器 实现
 *
 * === 本模块职责 ===
 *
 *   1. 将上层下发的 (linear_rpm, turn_rpm) 差速指令分解为
 *      左右轮各自的目标转速，通过 stepper_pulse 驱动硬件。
 *   2. 直行时利用 IMU 偏航角 + 陀螺仪角速度实现 PD 航向修正，
 *      使小车在路面不平或轮径差异时仍能保持直线行驶。
 *   3. 车身倾斜检测——当 roll/pitch 超限时紧急停止，
 *      防止翻倒后车轮空转损坏电机。
 *   4. 故障锁存机制——紧急停止后锁定，需显式清除才能恢复。
 *
 * === 数据流 ===
 *
 *   上层指令                    IMU 数据
 *      │                           │
 *      ▼                           ▼
 *   Car_SetVelocity()         IMU_Update()
 *      │                           │
 *      ▼                           ▼
 *   Car_Update() ◄─────────── IMU_Data_t
 *      │
 *      ├─ 倾斜检测 → Car_EmergencyStop()
 *      ├─ 航向 PD 修正 (仅直行时)
 *      └─ 差速分解 → 左右轮 RPM
 *                       │
 *                       ▼
 *              stepper_pulse_set_speed()
 *
 * === 坐标系约定 ===
 *
 *   linear_rpm > 0 → 前进（车头方向）
 *   turn_rpm   > 0 → 左转（左轮快于右轮，以车头为参考）
 *   yaw 增大       → 车头右转（绕 Z 轴，右手定则）
 *
 *   左右轮电气方向由 car_control.h 中的宏定义：
 *     CAR_LEFT_FORWARD_DIR  / CAR_RIGHT_FORWARD_DIR
 *   若实车运动方向相反，交换这两个宏即可。
 */

#include "car_control.h"

#include <math.h>

/* ==========================================================================
 * 内部状态变量 — 模块级静态变量，外部通过 API 访问
 * ========================================================================== */

static int16_t s_linear_rpm;          /* 当前目标前进速度     */
static int16_t s_turn_rpm;            /* 当前目标差速转向量   */
static bool    s_heading_hold;        /* 航向保持使能标志     */
static bool    s_heading_valid;       /* 目标航向是否已记录   */
static bool    s_fault;               /* 故障锁存标志         */
static float   s_heading_target_deg;  /* 直行目标偏航角 (度)  */

/* ==========================================================================
 * clamp_command — 转速限幅
 * ==========================================================================
 *
 * 将输入值限制在 [-CAR_MAX_RPM, +CAR_MAX_RPM] 范围内。
 *
 * 为什么需要: 上层可能传入任意 int32_t 范围的指令（如传感器噪声、
 * 通信错误），必须在进入差速计算前限幅，防止溢出导致左右轮分配异常。
 *
 * 参数:
 *   value — 待限幅的转速值 (int32_t, 支持中间计算不溢出)
 * 返回:
 *   限幅后的 int16_t 值
 *
 * 注意: 输入用 int32_t 是为了容纳 (linear_rpm + turn_rpm) 的中间和，
 *       防止两个 int16_t 相加溢出。限幅后再转为 int16_t 输出。
 */
static int16_t clamp_command(int32_t value)
{
    if (value > CAR_MAX_RPM)  return CAR_MAX_RPM;
    if (value < -CAR_MAX_RPM) return -CAR_MAX_RPM;
    return (int16_t)value;
}

/* ==========================================================================
 * command_magnitude — 提取转速幅值并施加死区
 * ==========================================================================
 *
 * 将带符号的转速指令转换为无符号的幅值，同时完成两项保护：
 *
 *   1. 死区过滤: 幅值小于 STEPPER_PULSE_MIN_RPM 时返回 0。
 *      原因: 9 RPM 是 16 位定时器的硬件下限，过低的转速无法
 *      产生有效的 PWM 脉冲。强行传入会导致周期溢出 → 行为不确定。
 *
 *   2. 上限限幅: 幅值大于 CAR_MAX_RPM 时截断。
 *
 * 参数:
 *   command — 带符号的转速指令 (正=前进方向, 负=后退方向)
 * 返回:
 *   无符号的幅值 RPM (0 表示停止)
 *
 * 典型调用链:
 *   Car_SetVelocity() → apply_wheel_commands() → command_magnitude()
 */
static int16_t command_magnitude(int16_t command)
{
    int32_t magnitude = command;

    if (magnitude < 0) magnitude = -magnitude;
    if (magnitude < STEPPER_PULSE_MIN_RPM) return 0;   /* 死区: 太慢发不出脉冲 */
    if (magnitude > CAR_MAX_RPM) magnitude = CAR_MAX_RPM;
    return (int16_t)magnitude;
}

/* ==========================================================================
 * reverse_dir — 方向取反
 * ==========================================================================
 *
 * 工具函数: 将 CW 换为 CCW，将 CCW 换为 CW。
 * 用于根据"前进方向"宏推导出对应的"后退方向"。
 *
 * 例: 左轮前进 = CCW → 左轮后退 = reverse_dir(CCW) = CW
 */
static uint8_t reverse_dir(uint8_t dir)
{
    return (dir == STEPPER_DIR_CCW) ? STEPPER_DIR_CW : STEPPER_DIR_CCW;
}

/* ==========================================================================
 * apply_wheel_speed — 将单个车轮的指令下发到硬件
 * ==========================================================================
 *
 * 这是业务层到物理层的最后一道转换。输入带符号的转速指令，
 * 解析出"方向 + 幅值"后调用 stepper_pulse_set_speed()。
 *
 * 符号含义由 CAR_LEFT_FORWARD_DIR / CAR_RIGHT_FORWARD_DIR 定义：
 *   command >= 0 → 前进方向 → DIR = forward_dir
 *   command <  0 → 后退方向 → DIR = reverse_dir(forward_dir)
 *
 * 参数:
 *   axis    — STEPPER_AXIS_LEFT (0) 或 STEPPER_AXIS_RIGHT (1)
 *   command — 带符号的转速指令 (正=前进, 负=后退)
 *
 * 注意: command_magnitude() 内部处理了死区和限幅，
 *       幅值为 0 时 stepper_pulse_set_speed 内部会调用 stop。
 */
static void apply_wheel_speed(uint8_t axis, int16_t command)
{
    const bool forward = command >= 0;             /* 正值为前进方向 */
    const int16_t magnitude = command_magnitude(command);
    uint8_t forward_dir;
    uint8_t dir;

    /* 根据轴选择对应的前进方向 */
    if (axis == STEPPER_AXIS_LEFT) {
        forward_dir = CAR_LEFT_FORWARD_DIR;
    } else {
        forward_dir = CAR_RIGHT_FORWARD_DIR;
    }

    /* 前进则用前进方向，后退则用反向 */
    dir = forward ? forward_dir : reverse_dir(forward_dir);
    stepper_pulse_set_speed(axis, dir, (uint16_t)magnitude);
}

/* ==========================================================================
 * apply_wheel_commands — 差速分解：将 (线性速度, 转向量) 拆为左右轮转速
 * ==========================================================================
 *
 * 这是差速驱动的核心公式：
 *
 *   left_rpm  = linear_rpm + turn_rpm    (左轮 = 基准速度 + 差分量)
 *   right_rpm = linear_rpm - turn_rpm    (右轮 = 基准速度 - 差分量)
 *
 * 物理含义:
 *   turn_rpm = 0               → 直行, 左右轮同速
 *   turn_rpm > 0               → 左轮快于右轮 → 车右转
 *   turn_rpm = linear_rpm      → 右轮 = 0, 左轮 = 2×linear → 原地左转
 *   linear_rpm = 0, turn_rpm   → 原地旋转 (差速转向)
 *
 * 每个分量在分解后单独通过 clamp_command() 限幅，
 * 保证左右轮各自不会超过 CAR_MAX_RPM。
 */
static void apply_wheel_commands(int16_t linear_rpm, int16_t turn_rpm)
{
    /* 正的 turn_rpm 表示左轮快于右轮 → 车右转 (差速原理) */
    int16_t left_rpm  = clamp_command((int32_t)linear_rpm + turn_rpm);
    int16_t right_rpm = clamp_command((int32_t)linear_rpm - turn_rpm);

    apply_wheel_speed(STEPPER_AXIS_LEFT,  left_rpm);
    apply_wheel_speed(STEPPER_AXIS_RIGHT, right_rpm);
}

/* ==========================================================================
 * wrap_angle_error — 角度误差归一化到 [-180°, +180°]
 * ==========================================================================
 *
 * 偏航角在 0° ~ 360° 范围内循环。直接做差会得到错误结果。
 * 例如: 当前 350°, 目标 10° → 直接差 = 340°, 实际应左转 20°。
 *
 * 本函数将任意角度值折合到 ±180° 区间:
 *   angle >  180° → angle -= 360°
 *   angle < -180° → angle += 360°
 *
 * 参数:
 *   angle_deg — 待归一化的角度误差 (度)
 * 返回:
 *   归一化后的角度误差, 范围 [-180°, +180°]
 */
static float wrap_angle_error(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/* ==========================================================================
 * Car_Init — 初始化小车控制模块
 * ==========================================================================
 *
 * 调用时机: main() 中, stepper_pulse_init() 之后, 进入主循环之前。
 *
 * 操作:
 *   1. 将所有运动指令清零（停止状态）
 *   2. 默认开启航向保持（直行时自动修正偏航）
 *   3. 航向目标无效（等待首次直行指令时采样）
 *   4. 清除故障锁存
 *   5. 调用 stepper_pulse_stop 确保两轴硬件处于停止状态
 *
 * 注意: 此函数不启动电机，仅设置初始状态。
 *       需要上层调用 Car_SetVelocity() 后小车才会运动。
 */
void Car_Init(void)
{
    s_linear_rpm = 0;
    s_turn_rpm = 0;
    s_heading_hold = true;          /* 默认开启航向保持             */
    s_heading_valid = false;        /* 目标航向待下次直行时记录     */
    s_heading_target_deg = 0.0f;    /* 初始值, 首次直行时被覆盖     */
    s_fault = false;

    stepper_pulse_stop(STEPPER_AXIS_LEFT);
    stepper_pulse_stop(STEPPER_AXIS_RIGHT);
}

/* ==========================================================================
 * Car_SetVelocity — 设置差速驱动指令
 * ==========================================================================
 *
 * 这是小车运动的主要控制接口。上层定时调用此函数下发目标速度。
 *
 * 参数:
 *   linear_rpm — 前进速度 (正=前进, 负=后退, 0=停止)
 *   turn_rpm   — 差速量 (正=左转, 负=右转, 0=直行)
 *
 * 返回:
 *   true  = 指令已接受
 *   false = 故障锁存中, 指令被拒绝 (需先调用 Car_ClearFault())
 *
 * 行为细节:
 *   - turn_rpm ≠ 0 时: 标记 s_heading_valid = false,
 *     表示"正在转向, 不记录航向目标"。直行恢复时重新采样。
 *   - 两个参数均为 0 时: 直接调用 Car_Stop(), 等效于"松油门停车"。
 *
 * ★ 故障锁存机制:
 *   一旦 Car_EmergencyStop() 被触发 (倾斜/IMU 异常),
 *   s_fault = true → 此后所有 Car_SetVelocity() 调用均返回 false,
 *   只有调用 Car_ClearFault() 才能解锁。这确保故障原因被确认前
 *   不会意外恢复运动。
 */
bool Car_SetVelocity(int16_t linear_rpm, int16_t turn_rpm)
{
    if (s_fault) return false;  /* 故障锁存: 拒绝一切运动指令 */

    s_linear_rpm = clamp_command(linear_rpm);
    s_turn_rpm   = clamp_command(turn_rpm);

    /* 目标航向将在下一次有效的直行 IMU 更新中记录。
     * 因为当前是转向指令 → 航向正在变化 → 不适合记录。 */
    s_heading_valid = false;

    /* 全零指令 = 停车 (不锁存故障, 允许后续重新运动) */
    if (s_linear_rpm == 0 && s_turn_rpm == 0) {
        Car_Stop();
    }

    return true;
}

/* ==========================================================================
 * Car_Stop — 停止小车 (软停止, 不锁存故障)
 * ==========================================================================
 *
 * 与 Car_EmergencyStop() 的区别:
 *   - 本函数不设置 s_fault → 之后可以直接 Car_SetVelocity() 恢复运动
 *   - 用于正常的停车操作 (如到达目的地、用户手动停止)
 *
 * 操作:
 *   1. 清零运动指令
 *   2. 航向目标无效 (下次运动时重新采样)
 *   3. 硬件层双轴停止
 */
void Car_Stop(void)
{
    s_linear_rpm = 0;
    s_turn_rpm = 0;
    s_heading_valid = false;

    stepper_pulse_stop(STEPPER_AXIS_LEFT);
    stepper_pulse_stop(STEPPER_AXIS_RIGHT);
}

/* ==========================================================================
 * Car_EmergencyStop — 紧急停止 (硬停止, 锁存故障)
 * ==========================================================================
 *
 * 触发条件 (在 Car_Update 中自动检测):
 *   1. IMU 数据无效 (I2C 故障、传感器离线)
 *   2. 车身倾斜超过 CAR_MAX_TILT_DEG (翻倒/受阻)
 *
 * 与 Car_Stop() 的关键区别:
 *   - s_fault = true → 锁存故障
 *   - 调用硬件层 emergengy_stop (停 ISR 步数计数 → 双轴 CCP 冻结)
 *   - 此后 Car_SetVelocity() 全部返回 false
 *   - 必须调用 Car_ClearFault() 才能解锁
 *
 * 设计意图:
 *   紧急停止意味着发生了不安全事件 (翻车、传感器故障),
 *   不能自动恢复, 必须人工确认后清除。
 */
void Car_EmergencyStop(void)
{
    s_linear_rpm = 0;
    s_turn_rpm = 0;
    s_heading_valid = false;
    s_fault = true;                     /* ★ 锁存故障 */

    stepper_pulse_emergency_stop();     /* 硬件层双轴急停 (含 ISR 保护) */
}

/* ==========================================================================
 * Car_ClearFault — 清除故障锁存
 * ==========================================================================
 *
 * 调用后:
 *   - s_fault = false → 恢复接受运动指令
 *   - 小车仍处于停止状态 (调用 Car_Stop 确保)
 *
 * 典型使用:
 *   if (Car_GetStatus().fault) {
 *       // 人工确认安全后
 *       Car_ClearFault();
 *   }
 */
void Car_ClearFault(void)
{
    Car_Stop();       /* 确保清除故障时小车处于停止状态 */
    s_fault = false;  /* 解锁 */
}

/* ==========================================================================
 * Car_SetHeadingHold — 开启或关闭航向保持
 * ==========================================================================
 *
 * enable = true:  直行时自动根据偏航误差修正左右轮转速差
 * enable = false: 纯差速控制, 不修正偏航
 *
 * 切换时 s_heading_valid = false, 下次直行时重新采样目标航向。
 */
void Car_SetHeadingHold(bool enable)
{
    s_heading_hold = enable;
    s_heading_valid = false;  /* 状态切换后重新采样目标航向 */
}

/* ==========================================================================
 * Car_Update — ★ 核心控制函数, 每个控制周期调用一次
 * ==========================================================================
 *
 * 本函数是控制系统的"心脏", 在每个控制周期 (当前为 5ms / 200Hz)
 * 执行以下步骤:
 *
 *   步骤1: 若当前指令为零 (停车), 直接返回 true (无需控制)
 *
 *   步骤2: 安全检查
 *     - s_fault = true        → 故障锁存中, 触发急停
 *     - imu == NULL           → IMU 未初始化, 触发急停
 *     - imu_valid = false     → I2C 读失败, 触发急停
 *     - |roll| > MAX_TILT     → 车身侧翻, 触发急停
 *     - |pitch| > MAX_TILT    → 车身前后倾倒, 触发急停
 *
 *   步骤3: PD 航向修正 (仅 s_heading_hold=true 且 turn_rpm=0 时)
 *     - 首次直行: 采样当前 yaw 作为目标航向
 *     - 计算偏航误差: yaw_error = wrap_angle_error(yaw - target)
 *     - PD 修正量: correction = -(KP * yaw_error + KD * gyro_z)
 *     - 修正量限制在 ±CAR_HEADING_CORRECTION_MAX 内
 *
 *   步骤4: 差速控制
 *     - turn_command = s_turn_rpm + correction
 *     - apply_wheel_commands(s_linear_rpm, turn_command)
 *
 * 参数:
 *   imu       — IMU 姿态数据指针 (可为 NULL, 表示 IMU 未就绪)
 *   imu_valid — IMU 数据是否有效 (I2C 读取成功)
 *
 * 返回:
 *   true  = 控制正常执行
 *   false = 触发了紧急停止 (倾斜/故障)
 *
 * 调用约定:
 *   本函数在 IMU_Update() 之后调用, 频率严格匹配 200Hz。
 *   不要在 ISR 中调用 — 内部会调用 stepper_pulse API,
 *   stepper_pulse API 本身非 ISR 安全 (会写定时器寄存器)。
 */
bool Car_Update(const IMU_Data_t *imu, bool imu_valid)
{
    int16_t correction = 0;
    int16_t turn_command;

    /* ── 步骤1: 停止状态直接返回, 不做任何计算 ── */
    if (s_linear_rpm == 0 && s_turn_rpm == 0) {
        return true;
    }

    /* ── 步骤2: 安全检查 → 任一条件触发则急停 ── */

    /* 故障锁存或 IMU 异常 → 急停 */
    if (s_fault || !imu_valid || imu == NULL) {
        if (!s_fault) Car_EmergencyStop();   /* 防止重复急停 */
        return false;
    }

    /* 车身明显倾斜 → 判定为倾倒或受阻 → 急停
     * 阈值 CAR_MAX_TILT_DEG 取决于路面情况, 见 car_control.h */
    if (fabsf(imu->roll)  > CAR_MAX_TILT_DEG ||
        fabsf(imu->pitch) > CAR_MAX_TILT_DEG) {
        Car_EmergencyStop();
        return false;
    }

    /* ── 步骤3: PD 航向修正 ── */
    /* 条件: 航向保持开启 且 当前为直行指令 (turn_rpm==0) */
    if (s_heading_hold && s_turn_rpm == 0) {

        /* 首次直行或刚恢复直行: 采样当前偏航角作为目标 */
        if (!s_heading_valid) {
            s_heading_target_deg = imu->yaw;
            s_heading_valid = true;
        }

        /* 计算偏航误差 (归一化到 ±180°) */
        const float yaw_error =
            wrap_angle_error(imu->yaw - s_heading_target_deg);

        /* PD 控制律:
         *   P 项 = KP × yaw_error       → 比例: 角度偏多少就补多少
         *   D 项 = KD × gyro_z          → 微分: 正在转多快就刹车
         *   负号: 若 yaw_error 为正 (车头偏右), 需要左转 (负修正)
         *
         *   gyro_z > 0 表示正在右转 → D 项产生左转力矩 (阻尼)
         */
        const float correction_rpm =
            -(CAR_HEADING_KP * yaw_error + CAR_HEADING_KD * imu->gyro_z);

        /* 修正量限幅: 防止大角度偏航导致一侧轮骤停
         * 限幅后修正量不会超过 CAR_HEADING_CORRECTION_MAX */
        if (correction_rpm > CAR_HEADING_CORRECTION_MAX) {
            correction = CAR_HEADING_CORRECTION_MAX;
        } else if (correction_rpm < -CAR_HEADING_CORRECTION_MAX) {
            correction = -CAR_HEADING_CORRECTION_MAX;
        } else {
            correction = (int16_t)correction_rpm;
        }
    }

    /* ── 步骤4: 差速控制 ── */
    /* 最终转向指令 = 上层转向指令 + PD 修正量
     * clamp_command 确保最终值在安全范围内 */
    turn_command = clamp_command((int32_t)s_turn_rpm + correction);
    apply_wheel_commands(s_linear_rpm, turn_command);

    return true;
}

/* ==========================================================================
 * Car_GetStatus — 读取当前小车状态
 * ==========================================================================
 *
 * 只读接口, 不修改任何内部状态。适合 UI 显示、日志记录、上位机查询。
 *
 * 参数:
 *   status — [out] 指向 CarStatus_t 的指针 (传 NULL 安全, 直接返回)
 *
 * CarStatus_t 字段:
 *   linear_rpm   — 当前线性速度指令
 *   turn_rpm     — 当前差速转向指令
 *   heading_hold — 航向保持是否开启
 *   fault        — 是否处于故障锁存状态
 */
void Car_GetStatus(CarStatus_t *status)
{
    if (status == NULL) return;

    status->linear_rpm   = s_linear_rpm;
    status->turn_rpm     = s_turn_rpm;
    status->heading_hold = s_heading_hold;
    status->fault        = s_fault;
}
