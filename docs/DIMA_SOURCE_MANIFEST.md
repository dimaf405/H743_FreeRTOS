# Dima 上游源码与许可证清单

- 日期：2026-07-31
- 文档状态：阶段 3 SBUS、RCUpdate 与 ManualControl 已适配并通过目标构建/签名校验；目标板接收和遥控行为待人工验收
- 许可证决策：`PENDING`

## 1. 管理规则

本清单只记录上游版本、原始路径、本地映射和修改摘要。上游文件保留原始版权头与来源文字。

许可证状态只使用：

- `PENDING`：当前尚未收敛。
- `DEFERRED`：延后到产品发布阶段处理。

许可证事项不在当前阶段展开，也不阻塞内部移植、构建和实车调试。

## 2. PX4 正式移植基线

| 字段 | 内容 |
|---|---|
| 用途 | Parameter、ModuleParams、uORB API/消息契约、WorkQueue 接口、SBUS、RCUpdate、ManualControl、Commander Rover 子集、RoverDifferential、执行器链和 EKF2 |
| 正式目标版本 | PX4 v1.17.0 |
| 正式 commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 当前状态 | 阶段 1～3 基础链已按 v1.17.0 接口和行为适配；阶段 3 已通过 Windows 目标构建与签名校验 |
| 许可证状态 | `PENDING`；逐文件保留原始许可证 |
| 本地目录规则 | 产品目录使用 Dima；上游符号和许可证文字保持原样 |

正式导入前必须验证 tag/commit，并将每个导入文件登记到“文件级映射”表。

## 3. PX4 本地预研快照

| 字段 | 内容 |
|---|---|
| 本地分支 | `release/1.16` |
| 本地 commit（短） | `75f9a32a12` |
| 用途 | 阶段 0 架构预研、模块依赖和适配边界分析 |
| 限制 | 不能称为 v1.17.0，不能代替正式基线验证 |
| 导入状态 | 阶段 0 不从该快照导入生产源码 |

如后续需要记录完整 commit，应从本地仓库重新读取并补充，不根据短 commit 推测。

## 4. ArduPilot Rover 行为参考基线

| 字段 | 内容 |
|---|---|
| 参考 commit | `3f2e4763accb` |
| 用途 | Arming、Failsafe、RC 行为、轮速、差速车导航、倒车和 PivotTurn 行为参考 |
| 直接代码来源 | 当前仅作行为参考；直接代码处理状态为 `DEFERRED` |
| EKF3 状态 | 不采用；此前 EKF3 计划已由 EKF2 最终选择取代 |
| 许可证状态 | `PENDING` |

ArduPilot 当前仅用于功能需求、状态机和验收行为参考；其他处理状态为 `DEFERRED`。

## 5. 当前本地目录归属

已退役的顶层 `App/` 已归并到 `Dima/`，不再作为当前源码根。此次调整是本地所有权和路径变更，不改变对应上游来源、许可证或 API 身份。

| 职责 | 当前本地位置 | 状态 |
|---|---|---|
| 启动壳、C ABI 入口、appMainTask | `Dima/application/` | RELOCATED / TARGET VERIFY PASS |
| BootHealth、HelloWorld | `Dima/modules/boot_health/`、`Dima/modules/hello_world/` | RELOCATED / TARGET VERIFY PASS |
| USB Console、MCUboot 应用适配 | `Dima/adapters/` | RELOCATED / TARGET VERIFY PASS |
| 生命周期 | `Dima/middleware/lifecycle/` | RELOCATED / TARGET VERIFY PASS |
| C/C++ Runtime、no-heap、平台时间 | `Dima/platform/freertos/libc/`、`Dima/platform/freertos/platform_time.*` | RELOCATED / TARGET VERIFY PASS |
| Motor、Rover Control | `Dima/lib/motor/`、`Dima/lib/rover_control/` | RELOCATED / TARGET VERIFY PASS |

`Core/`、`Boards/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立边界。目录边界已收敛；2026-07-30 Windows 本地目标构建与签名验证通过，目标运行仍待板测。

## 6. 计划中的上游模块映射

| 子系统 | 上游来源 | 计划本地位置 | 阶段 | 当前状态 |
|---|---|---|---:|---|
| 时间、WorkQueue、uORB、Logging 兼容接口 | PX4 v1.17.0 | `Dima/platform/freertos/`、`Dima/middleware/` | 1 | ADAPTED |
| Parameter、ModuleParams | PX4 v1.17.0 | `Dima/middleware/parameters/` | 2 | ADAPTED / TARGET VERIFY PASS |
| SBUS、SbusRc、RCUpdate、ManualControl | PX4 v1.17.0 | `Dima/lib/rc/`、`Dima/modules/rc/`、`Dima/platform/freertos/` | 3 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| Commander Rover 子集 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/safety/` | 4 | PLANNED |
| RoverDifferential 与执行器链 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/rover/` | 5 | PLANNED |
| EKF2 与 Estimator 支撑库 | PX4 v1.17.0 | `Dima/modules/estimator/ekf2/`、`Dima/lib/estimator/` | 7 | PLANNED |
| Position、Waypoint、Reverse、PivotTurn | PX4 v1.17.0；APM 行为参考 | `Dima/modules/rover/` | 9 | PLANNED |

## 7. 文件级映射

阶段 2 的唯一 PX4 Parameter 来源基线为 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。

| 上游原始路径/功能 | 本地映射 | 适配方式 | 状态 |
|---|---|---|---|
| `src/lib/parameters/parameters.cpp`、Parameter Layer/Core、AtomicTransaction | `Dima/middleware/parameters/` | 保留 `param_*`、稀疏 Layer、事务及参数更新语义；FreeRTOS 锁与存储回调适配 | ADAPTED |
| `platforms/common/include/px4_platform_common/param.h`、`param_macros.h`、`module_params.h` | `Dima/middleware/parameters/` | 保留 `px4::Param<T>`、`ModuleParams` 和参数宏兼容接口 | ADAPTED |
| `Tools/px4params/process_params.py` 相关 parser、scanner、XML/JSON 输出逻辑 | `tools/parameters/` | 直接复用官方 parser 数据模型；标准库 renderer 等价生成 `px4_parameters.hpp`，不依赖 Jinja2 | ADAPTED |
| `src/lib/tinybson/tinybson.h/.cpp` | `Dima/lib/tinybson/` | 保留上游 BSD 头；删除 fd、POSIX 和动态扩容路径，仅保留固定 Buffer 编解码 | ADAPTED |
| `src/lib/parameters/flashparams/` | `Dima/middleware/parameters/flashparams/` | 改为 Parameter enumerator/visitor 与 TinyBSON Buffer 之间的适配，不直接访问文件系统 | ADAPTED |
| PX4 Parameter Autosave | `Dima/middleware/parameters/` | 300 ms 合并、保存间隔至少 2 s、失败最多重试 3 次；保存工作进入 LP/service 路径 | ADAPTED |
| PX4 参数命令行为与 USB 接入需求 | `Dima/adapters/usb_console/`、Parameter Service | CDC ISR 仅写固定 1024-byte SPSC Ring并立即恢复接收；瞬时重挂接失败由 LP 服务重试；任务侧解析和执行 | ADAPTED |
| PX4 flashparams/flashfs 思路 | `Dima/platform/freertos/parameter_flash.cpp` | Bank 2 最后一个 128 KiB 扇区的单扇区追加 Journal；CRC、Sequence、最终 Commit Marker、ECC 安全读和参数区受限 BusFault 恢复；与 MCUboot 共用全局 Flash 递归互斥；空间满返回 ENOSPC且不自动擦除 | ADAPTED |
| Rover Parameter 定义 | `Dima/middleware/parameters/definitions/` | 导入 24 项 `RO_*`/`RD_*` 参数，名称、默认值、单位和元数据保持 PX4 来源 | ADAPTED |
| `platforms/common/include/px4_platform_common/log.h`、`platforms/common/px4_log.cpp` | `Dima/middleware/logging/` | 保留 PX4 日志宏和默认输出格式，固定 Ring + LP USB flush | ADAPTED |
| `src/lib/rc/sbus.h`、`src/lib/rc/sbus.cpp` | `Dima/lib/rc/sbus.hpp`、`Dima/lib/rc/sbus.cpp` | 保留 25-byte 帧、16 路 11-bit 通道、数字 17/18、4 ms 重同步、Failsafe/Frame-Lost 与 PX4 数值映射；移除 POSIX 串口和 SBUS 输出 | ADAPTED |
| `src/drivers/rc/sbus_rc/SbusRc.hpp`、`SbusRc.cpp` | `Dima/modules/rc/SbusRc.*` | 保留 WorkItem 接收、锁定、重试和 `input_rc` 发布流程；串口抽象映射到 FreeRTOS/HAL 后端 | ADAPTED |
| PX4 串口配置与板级 RC 输入行为 | `Dima/platform/freertos/sbus_uart_backend.*` | 适配 STM32H743 UART RXINV、DMA1 Stream2、DMAMUX、多 UART 参数选择、D2 Cache 维护和 FromISR 唤醒 | DIMA BACKEND |
| `src/modules/rc_update/rc_update.h`、`rc_update.cpp` | `Dima/modules/rc/RCUpdate.*` | 保留 18 通道校准、功能映射、开关离散化、失联与 `parameter_update` 语义；裁剪 MAVLink RC 参数映射及非 Rover 功能 | ADAPTED |
| `src/modules/manual_control/ManualControl.hpp`、`ManualControl.cpp` | `Dima/modules/rc/ManualControl.*` | 保留 RC setpoint 和开关边沿 Action Request；阶段 3 只要求差速 Rover 的 throttle/yaw 有效，未接 Commander | ADAPTED |
| `msg/InputRc.msg`、`RcChannels.msg`、`ManualControlSetpoint.msg`、`ManualControlSwitches.msg`、`ActionRequest.msg` | `Dima/messages/` | 保留 PX4 字段、枚举和 Topic 契约；`action_request` Queue Depth 为 8 | ADAPTED |
| PX4 RC 参数定义与生成元数据 | `Dima/middleware/parameters/definitions/rc_params.c` | 导入 18 通道 MIN/TRIM/MAX/REV/DZ、核心映射、失联和板级 SBUS 端口参数；阶段 3 后总参数数为 135 | ADAPTED |

许可证状态仅记录为 `PENDING`；延后处理项记录为 `DEFERRED`。该状态不阻塞当前内部移植、编译和板级调试工作。

## 8. 许可证状态

- PX4 v1.17.0 来源文件：`PENDING`。
- 最终产品分发与 Notice 收敛：`DEFERRED`。
- 当前阶段仅保持来源、commit、本地映射和原始版权头可追踪；许可证事项延后收敛，不阻塞技术实现。
