# Dima 上游源码与许可证清单

- 日期：2026-08-04
- 文档状态：阶段 1～4 已迁入强制分层的公共 capability、FreeRTOS 后端和 STM32H7 后端；正式构建证据与目标板待验项见本次交付
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
| 当前状态 | 阶段 1～4 基础链已按 v1.17.0 接口和行为适配；阶段 4 已通过目标构建、签名和镜像校验，板测待完成 |
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
| USB Console 协议适配 | `Dima/adapters/usb_console/` | RELOCATED / PLATFORM ISOLATED |
| 生命周期 | `Dima/middleware/lifecycle/` | RELOCATED / TARGET VERIFY PASS |
| 公共 capability 与时间契约 | `Dima/platform/api/` | DIMA CONTRACT |
| FreeRTOS Task/同步/Heap 后端与 C/C++ Runtime | `Dima/platform/freertos/` | DIMA BACKEND / PLATFORM ISOLATED |
| STM32H7 时钟、MPU/cache、DMA、Flash、USB、SBUS 与中断后端 | `Dima/platform/stm32h7/` | DIMA BACKEND / PLATFORM ISOLATED / BOARD PENDING |
| H743 capability 组合根 | `Boards/H743/Src/platform_composition.cpp` | DIMA COMPOSITION |
| Rover 产品装配、专属控制与导航 | `Dima/rover/`、`Dima/rover/control/`、`Dima/rover/navigation/` | UNIQUE PRODUCT ROOT / CONTROL+NAV PLANNED |
| Motor、Rover Control | `Dima/lib/motor/`、`Dima/lib/rover_control/` | RELOCATED / TARGET VERIFY PASS |
| Commander Rover 安全子集 | `Dima/modules/safety/` | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| Commander 状态消息 | `Dima/messages/` | ADAPTED / TARGET VERIFY PASS |

`Core/`、`Boards/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立边界。目录边界已收敛；modules 下不再保留或重新引入重复 Rover 子目录。表中的 `TARGET VERIFY PASS` 是对应历史批次的构建状态；2026-08-05 生命周期收敛增量目前只有源码/静态门禁证据，Windows 原生 clean build、最终 ELF 和目标运行仍待验证。

## 6. 计划中的上游模块映射

| 子系统 | 上游来源 | 计划本地位置 | 阶段 | 当前状态 |
|---|---|---|---:|---|
| 时间、WorkQueue、uORB、Logging 兼容接口 | PX4 v1.17.0 | `Dima/platform/api/`、`Dima/platform/stm32h7/Clock.cpp`、`Dima/middleware/` | 1 | ADAPTED / PLATFORM ISOLATED |
| Parameter、ModuleParams | PX4 v1.17.0 | `Dima/middleware/parameters/` | 2 | ADAPTED / TARGET VERIFY PASS |
| SBUS、SbusRc、RCUpdate、ManualControl | PX4 v1.17.0 | `Dima/lib/rc/`、`Dima/modules/rc/`、`Dima/platform/stm32h7/SbusUart.cpp` | 3 | ADAPTED / PLATFORM ISOLATED / BOARD PENDING |
| Commander Rover 子集 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/safety/` | 4 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| RoverDifferential 与执行器链 | PX4 v1.17.0；APM 行为参考 | `Dima/rover/control/` | 5 | PLANNED |
| EKF2 与 Estimator 支撑库 | PX4 v1.17.0 | `Dima/modules/estimator/ekf2/`、`Dima/lib/estimator/` | 7 | PLANNED |
| Position、Waypoint、Reverse、PivotTurn | PX4 v1.17.0；APM 行为参考 | `Dima/rover/navigation/` | 9 | PLANNED |

## 7. 文件级映射

阶段 2～4 的唯一 PX4 直接代码来源基线为 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；ArduPilot commit `3f2e4763accb` 在阶段 4 仅作 Rover 安全行为参考，不直接导入代码。

| 上游原始路径/功能 | 本地映射 | 适配方式 | 状态 |
|---|---|---|---|
| `src/lib/parameters/parameters.cpp`、Parameter Layer/Core、AtomicTransaction | `Dima/middleware/parameters/` | 保留 `param_*`、稀疏 Layer、事务及参数更新语义；锁、执行上下文和内存只通过公共 capability | ADAPTED / PLATFORM ISOLATED |
| `platforms/common/include/px4_platform_common/param.h`、`param_macros.h`、`module_params.h` | `Dima/middleware/parameters/` | 保留 `px4::Param<T>`、`ModuleParams` 和参数宏兼容接口 | ADAPTED |
| `Tools/px4params/process_params.py` 相关 parser、scanner、XML/JSON 输出逻辑 | `tools/parameters/` | 直接复用官方 parser 数据模型；标准库 renderer 等价生成 `px4_parameters.hpp`，不依赖 Jinja2 | ADAPTED |
| `src/lib/tinybson/tinybson.h/.cpp` | `Dima/lib/tinybson/` | 保留上游 BSD 头；删除 fd、POSIX 和动态扩容路径，仅保留固定 Buffer 编解码 | ADAPTED |
| `src/lib/parameters/flashparams/` | `Dima/middleware/parameters/flashparams/` | 改为 Parameter enumerator/visitor 与 TinyBSON Buffer 之间的适配，不直接访问文件系统 | ADAPTED |
| PX4 Parameter Autosave 与 Runtime cache | `Dima/middleware/parameters/` | 300 ms 合并、保存间隔至少 2 s、失败最多重试 3 次；`Param<T>` 构造无 Core 副作用，每次 start bind，每次 shutdown 清 ready/used/unsaved/value cache、callback 和动态 Layer | ADAPTED / RUNTIME LIFECYCLE |
| PX4 参数命令行为与 USB 接入需求 | `Dima/adapters/usb_console/`、Parameter Service | CDC ISR 仅写固定 1024-byte SPSC Ring并立即恢复接收；瞬时重挂接失败由 LP 服务重试；任务侧解析和执行 | ADAPTED |
| PX4 flashparams/flashfs 思路 | `Dima/middleware/parameters/ParameterJournal.*`、`Dima/platform/stm32h7/FlashDevice.cpp` | 平台无关 Journal 与 raw Flash device 分离；保持 Bank 2 最后 128 KiB、v1 字节格式、CRC/Sequence/Commit Marker 和 ENOSPC；ECC 安全读、实际修改范围 cache 一致性及通用 Flash BusFault hook 归 MCU 后端 | ADAPTED / PLATFORM ISOLATED |
| Rover Parameter 定义 | `Dima/middleware/parameters/definitions/` | 导入 24 项 `RO_*`/`RD_*` 参数，名称、默认值、单位和元数据保持 PX4 来源 | ADAPTED |
| `platforms/common/include/px4_platform_common/log.h`、`platforms/common/px4_log.cpp` | `Dima/middleware/logging/` | 保留 PX4 日志宏和默认输出格式，固定 Ring + LP USB flush | ADAPTED |
| `src/lib/rc/sbus.h`、`src/lib/rc/sbus.cpp` | `Dima/lib/rc/sbus.hpp`、`Dima/lib/rc/sbus.cpp` | 保留 25-byte 帧、16 路 11-bit 通道、数字 17/18、4 ms 重同步、Failsafe/Frame-Lost 与 PX4 数值映射；移除 POSIX 串口和 SBUS 输出 | ADAPTED |
| `src/drivers/rc/sbus_rc/SbusRc.hpp`、`SbusRc.cpp` | `Dima/modules/rc/SbusRc.*` | 保留 WorkItem 接收、锁定、重试和 `input_rc` 发布流程；仅依赖公共 SbusInput 与 ISR-safe callback | ADAPTED / PLATFORM ISOLATED |
| PX4 串口配置与板级 RC 输入行为 | `Dima/platform/stm32h7/SbusUart.cpp`、`DmaMemory.cpp` | 适配 STM32H743 UART RXINV、DMA1 Stream2、DMAMUX 和多 UART 参数选择；64-byte non-cacheable DMA Buffer 在 ISR 复制到 256 项 CPU-only SPSC Ring，overflow/UART/restart 同时推进接收 epoch 并清 parser | DIMA BACKEND / OWNERSHIP GATED |
| `src/modules/rc_update/rc_update.h`、`rc_update.cpp` | `Dima/modules/rc/RCUpdate.*` | 保留 18 通道校准、功能映射、开关离散化、失联与 `parameter_update` 语义；裁剪 MAVLink RC 参数映射及非 Rover 功能 | ADAPTED |
| `src/modules/manual_control/ManualControl.hpp`、`ManualControl.cpp` | `Dima/modules/rc/ManualControl.*` | 保留 RC setpoint 和开关边沿 Action Request；差速 Rover 的 throttle/yaw 保持中心双向语义，阶段 4 由 Commander 消费动作 | ADAPTED |
| `src/modules/commander/Commander.hpp`、`Commander.cpp`、`ModeUtil/control_mode.*` | `Dima/modules/safety/Commander.*` | 保留 Action Request、Arming/Kill/Termination、状态发布顺序和 Manual/Termination 控制模式语义；改为 `wq:hp_default` WorkItem，裁剪飞行器模式、任务、执行器和其他 Commander 服务 | ADAPTED |
| `msg/InputRc.msg`、`RcChannels.msg`、`ManualControlSetpoint.msg`、`ManualControlSwitches.msg`、`ActionRequest.msg` | `Dima/messages/` | 保留 PX4 字段、枚举和 Topic 契约；`action_request` Queue Depth 为 8 | ADAPTED |
| `msg/versioned/VehicleStatus.msg`、`msg/versioned/VehicleControlMode.msg`、`msg/ActuatorArmed.msg` | `Dima/messages/` | 完整保留三个公开消息的字段、枚举和版本号；本地三个 Topic 均为单深度 | ADAPTED |
| PX4 RC/Commander 参数定义与生成元数据 | `Dima/middleware/parameters/definitions/rc_params.c`、`commander_params.c` | 保留 RC 失联参数；增加 Dima Rover 公开参数 `COM_ARM_STICK_DZ`，避免复用单位为 m/s 的 `RO_SPEED_TH`；阶段 4 后总参数数为 136 | ADAPTED / DIMA PARAMETER |
| Dima Rover 生命周期与 Parameter Autosave 写门控 | `Dima/rover/ApplicationContext.*`、`Dima/modules/parameters/ParameterService.*`、`Dima/middleware/uorb/`、`Dima/middleware/work_queue/`、`Dima/platform/api/Platform.*` | ApplicationContext 只注入 capability；Parameter、Commander 与 BootControl 共用 Armed/Flash coordinator；Runtime shutdown 失效 Parameter cache、推进 uORB epoch、drain WorkQueue 并释放 Console，只有 Termination 跨 Runtime 保留 | DIMA INTEGRATION / SOURCE GATE PASS / TARGET REVERIFY PENDING |
| MCUboot image confirmation 与 Recovery | `Dima/platform/stm32h7/BootControl.cpp`、`flash_bank1.c` | 非阻塞 transaction 保留 DEFERRED；Bank 1 program 在 DTCM 执行并统一调用 cache helper | DIMA BACKEND |
| Fault 跨复位诊断持久化 | `Boards/H743/Src/boot_diagnostics.c`、`boot_diagnostics_store.c`、`Bootloader/Src/main.c` | Application Fault/Panic 只写 non-cacheable D3 record、执行 barrier 并复位；MCUboot 冷启动独占诊断 Flash store 和 Recovery，Application ELF 禁止链接 store 符号 | DIMA SAFETY / ELF GATED |

许可证状态仅记录为 `PENDING`；延后处理项记录为 `DEFERRED`。该状态不阻塞当前内部移植、编译和板级调试工作。

## 8. 许可证状态

- PX4 v1.17.0 来源文件：`PENDING`。
- 最终产品分发与 Notice 收敛：`DEFERRED`。
- 当前阶段仅保持来源、commit、本地映射和原始版权头可追踪；许可证事项延后收敛，不阻塞技术实现。
