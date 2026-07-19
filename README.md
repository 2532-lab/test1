# E - MSPM0G3507 嵌入式项目

基于 TI MSPM0G3507 (Cortex-M0+) 的嵌入式控制项目，属于 TI 电子竞赛系列工程。

## 硬件平台

| 项目 | 说明 |
|------|------|
| MCU | MSPM0G3507 (ARM Cortex-M0+, 80MHz) |
| 开发板 | LP-MSPM0G3507 LaunchPad |
| 调试器 | XDS-110 (板载) |
| IDE | TI Code Composer Studio Theia |

## 工程结构

```
E/
├── src/            # 应用层源代码
├── drivers/        # 驱动层（HAL 硬件抽象 + MID 中间层）
├── include/        # 公共头文件
├── docs/           # 项目文档
├── openmv/         # OpenMV 视觉模块代码
├── targetConfigs/  # CCS 调试配置文件
└── Debug/          # 编译输出（不入库）
```

## 架构分层

| 层级 | 目录 | 职责 |
|------|------|------|
| 应用层 | `src/` | 业务逻辑、控制算法、任务调度 |
| 中间层 | `drivers/mid_*.c` | 传感器协议封装、数据处理 |
| 硬件抽象层 | `drivers/hal_*.c` | 寄存器级外设操作 |
| 公共接口 | `include/` | 模块间通信的类型定义和函数声明 |

## 开发环境

- **IDE**: TI Code Composer Studio Theia
- **编译器**: TI ARM Clang Compiler (ti-cgt-armllvm)
- **SDK**: MSPM0 SDK
- **配置工具**: SysConfig

## 引脚分配

| 外设 | 引脚 | 功能 |
|------|------|------|
| DEBUGSS | PA20 | SWCLK |
| DEBUGSS | PA19 | SWDIO |

## 快速开始

1. 用 CCS Theia 打开本目录作为工作区工程
2. 编译：`Project → Build Project`
3. 调试：`Run → Debug`（通过板载 XDS-110）

## 分支说明

| 分支 | 用途 |
|------|------|
| `main` | 稳定主线 |
| `feat/*` | 功能开发分支 |
| `test/*` | 实验性验证代码 |

## 相关项目

- [Gimbal](https://github.com/2532-lab/TI-Series-Projects-for-Electronic-Competition) — 云台主控项目
