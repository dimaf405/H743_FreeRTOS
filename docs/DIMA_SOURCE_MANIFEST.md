# Dima 上游源码与许可证清单

- 日期：2026-08-19
- 文档状态：阶段 1～6 已接通强制分层的平台、生命周期、两轴 Manual 差速控制、六路安全 PWM 链和 MAVLink v2.0 协议处理；Windows 原生目标构建已通过，实板电气、时序和车辆行为仍待验收
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
| 用途 | Parameter、ModuleParams、uORB API/消息契约、WorkQueue 接口、SBUS、RCUpdate、ManualControl RC 子集、Commander Rover 子集、RoverDifferential、执行器链、MAVLink v2.0 协议处理和后续 EKF2 |
| 正式目标版本 | PX4 v1.17.0 |
| 正式 commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 当前状态 | 阶段 1～4 基础链已按 v1.17.0 接口和行为适配；阶段 5 已接通 Manual 两轴请求、RoverDifferential、MotorOutput 和六路 PWM，源码/目标构建通过，板级波形验证待完成 |
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
| BootHealth、MAVLink STATUSTEXT 日志 | `Dima/modules/boot_health/`、`Dima/modules/logging/` | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| USB Console 协议适配 | `Dima/adapters/usb_console/` | RELOCATED / PLATFORM ISOLATED |
| 生命周期 | `Dima/middleware/lifecycle/` | RELOCATED / TARGET VERIFY PASS |
| 公共 capability 与时间契约 | `Dima/platform/api/` | DIMA CONTRACT |
| FreeRTOS Task/同步/Heap 后端与 C/C++ Runtime | `Dima/platform/freertos/` | DIMA BACKEND / PLATFORM ISOLATED |
| STM32H7 时钟、MPU/cache、DMA、Flash、USB、SBUS、中断与六路 PWM 后端 | `Dima/platform/stm32h7/` | DIMA BACKEND / PLATFORM ISOLATED / TARGET VERIFY PASS / BOARD PENDING |
| MAVLink v2.0 协议处理 | `Dima/modules/mavlink/` | ADAPTED / TARGET VERIFY PASS |
| H743 capability 组合根 | `Boards/H743/Src/platform_composition.cpp` | DIMA COMPOSITION |
| Rover 产品装配、模式、专属控制与导航 | `Dima/rover/`、`Dima/rover/modes/`、`Dima/rover/control/` | UNIQUE PRODUCT ROOT / MANUAL OUTPUT CHAIN ADAPTED / NAV INTERFACE RESERVED |
| Rover 纯控制算法 | `Dima/lib/rover/` | 当前只保留已由生产链调用的 `DifferentialDrive`；阶段 8 闭环控制器尚未导入 |
| Commander Rover 安全子集 | `Dima/modules/safety/` | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| Commander 状态消息 | `Dima/messages/` | ADAPTED / TARGET VERIFY PASS |

`Core/`、`Boards/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立边界。2026-08-21 当前源码通过 `git diff --check` 和覆盖 269 个首方源码文件的架构门禁，生成合同为 203 项固件参数/203 项公开 Parameter Metadata 和 24 条 MAVLink 消息；六路共 18 个 `MIN/CENT/MAX` 字段的生成范围均为 500～2500 us，默认仍为 1000/1500/2000 us。General 声明 type 1 Parameter 与 type 5 Actuator，生成六路 PWM Actuator Metadata，但不生成 `SERVO_OUTPUT_RAW`，也不处理 `MAV_CMD_ACTUATOR_TEST`。Windows 原生 clean `dima_rover` 完整通过 `[235/235]`：Application `text=244780/data=13420/bss=359536`、未签名 BIN 258240 bytes、本次签名样本 image digest `205ce460f59fdee2ba7a6a1599973205d0efa1a7b4c9d7ecace67a604e9584dc`，向量 `0x08040400`，Application/MCUboot 未解析符号为空；Signed BIN 259415 bytes 与 Factory 容器长度仍受 DER 签名影响，不作为确定性合同。当前 QGC Actuators 页面可用由用户实测确认；QGC Radio 页面、参数编辑不复位、六路物理 PWM 与 500/1500/2500 us 波形、USB 长连接、SD 物理拔插/重插、真实 IWDG 时限、掉电窗口、参数恢复及车辆行为仍为 `BOARD PENDING`。

## 6. 计划中的上游模块映射

| 子系统 | 上游来源 | 计划本地位置 | 阶段 | 当前状态 |
|---|---|---|---:|---|
| 时间、WorkQueue、uORB、Logging 兼容接口 | PX4 v1.17.0 | `Dima/platform/api/`、`Dima/platform/stm32h7/system/Clock.cpp`、`Dima/middleware/` | 1 | ADAPTED / PLATFORM ISOLATED |
| Parameter、ModuleParams | PX4 v1.17.0 | `Dima/middleware/parameters/` | 2 | ADAPTED / TARGET VERIFY PASS |
| SBUS、SbusRc、RCUpdate、ManualControl RC 子集 | PX4 v1.17.0 | `Dima/lib/rc/`、`Dima/modules/rc/`、`Dima/platform/stm32h7/serial/SbusUart.cpp` | 3 | ADAPTED / PLATFORM ISOLATED / BOARD PENDING |
| Commander Rover 子集 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/safety/` | 4 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| RoverDifferential 与执行器链 | PX4 v1.17.0；APM 行为参考 | `Dima/rover/control/`、`Dima/lib/rover/`、`Dima/modules/motor/` | 5 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| MAVLink v2.0 协议处理（RX/TX、心跳、命令、Classic + Ext 参数、5 Hz 原始 RC、空任务、时间同步） | PX4 v1.17.0；mavlink/mavlink commit `33af200d` | `Dima/modules/mavlink/`、`build/generated/mavlink/` | 6 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| EKF2 与 Estimator 支撑库 | PX4 v1.17.0 | 尚未创建；正式导入后分别落入运行模块与纯算法所有者目录 | 7 | PLANNED / NO SOURCE PLACEHOLDER |
| Position、Waypoint、Reverse、PivotTurn | PX4 v1.17.0；APM 行为参考 | `Dima/rover/navigation/` | 9 | PLANNED |

## 7. 文件级映射

阶段 2～5 的唯一 PX4 直接代码来源基线为 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；ArduPilot commit `3f2e4763accb` 仅作 Rover 安全、倒车和油门输出行为参考，不直接导入代码。

| 上游原始路径/功能 | 本地映射 | 适配方式 | 状态 |
|---|---|---|---|
| `src/lib/parameters/parameters.cpp`、Parameter Layer/Core、AtomicTransaction | `Dima/middleware/parameters/` | 保留 `param_*`、稀疏 Layer、事务及参数更新语义；锁、执行上下文和内存只通过公共 capability | ADAPTED / PLATFORM ISOLATED |
| `platforms/common/include/px4_platform_common/param.h`、`param_macros.h`、`module_params.h` | `Dima/middleware/parameters/` | 保留 `px4::Param<T>`、`ModuleParams` 和参数宏兼容接口 | ADAPTED |
| `Tools/px4params/process_params.py` 相关 parser、scanner、XML/JSON 输出逻辑 | `tools/parameters/` | 直接复用官方 parser 数据模型并只扫描 Make 显式源文件；标准库 renderer 等价生成 `px4_parameters.hpp`，Header/类型表/`param_info`/JSON 按当前参数名全局排序，不保留旧 handle 或 stable-tail；不依赖 Jinja2 | ADAPTED |
| `src/lib/tinybson/tinybson.h/.cpp` | `Dima/lib/tinybson/` | 保留上游 BSD 头；删除 fd、POSIX 和动态扩容路径，仅保留固定 Buffer 编解码 | ADAPTED |
| `src/lib/parameters/flashparams/` | `Dima/middleware/parameters/flashparams/` | 改为 Parameter enumerator/visitor 与 TinyBSON Buffer 之间的适配，不直接访问文件系统 | ADAPTED |
| PX4 Parameter Autosave 与 Runtime cache | `Dima/middleware/parameters/` | 300 ms 合并、保存间隔至少 2 s、失败最多重试 3 次；`Param<T>` 构造无 Core 副作用，每次 start bind，每次 shutdown 清 ready/used/unsaved/value cache、callback 和动态 Layer | ADAPTED / RUNTIME LIFECYCLE |
| PX4 参数协议与 USB 接入 | `Dima/modules/mavlink/`、Parameter Service | MavlinkService 独占 CDC RX/TX；ParameterService 只负责 Core、FlashFS/FileStorage、Autosave 和 Flash，Classic/Ext 参数请求通过 uORB/Parameter API 处理；每次 LIST 前标记真实 RC、16 项公开 SERIAL/QGC 参数并在参数锁内冻结 used 句柄，按 index 补读与快照内 ACK 复用本轮 count/index；FlashFS 忽略 11 项固定 Summary 合同及已禁用 mode mapping 的旧值，串口参数正常持久化 | ADAPTED / SINGLE OWNER |
| PX4 flashparams/flashfs 思路 | `Dima/middleware/parameters/flashfs.*`、`FileStorage.*`、`Dima/platform/api/ParameterFileStore.hpp`、`Dima/platform/freertos/storage/`、`Boards/H743/Src/fatfs_diskio.c` | SD/FatFs 在唯一固件配置中强制编译。FlashFS 是主存储，SD 是 generation 排序的镜像和恢复源；同 generation 还比较 payload CRC，发现分裂时以既有 Flash 优先规则重建 SD。运行期写入由 `appMain + BootHealth` 批准维护票据，Flash 按 32-byte、FatFs 按 512-byte 分步推进，存储层不得直接喂 IWDG；ENOSPC 仅暂停 Autosave，插卡后恢复一次受控保存。无 card-detect GPIO 时每 3 秒软件探测，拔卡后 Flash 参数读写继续 | ADAPTED / PLATFORM ISOLATED / TARGET VERIFY PASS / BOARD PENDING |
| Rover Parameter 定义 | `Dima/middleware/parameters/definitions/` | 导入 24 项 `RO_*`/`RD_*` 参数，名称、默认值、单位和元数据保持 PX4 来源 | ADAPTED |
| `platforms/common/include/px4_platform_common/log.h`、`platforms/common/px4_log.cpp` | `Dima/middleware/logging/`、`Dima/modules/logging/` | 保留 PX4 日志宏和 SourcePolicy；uORB 初始化后注册 non-blocking structured sink，普通/RAW 日志写入深度 8 的 `mavlink_log`，MavlinkService 有界转为 STATUSTEXT；Event Ring 继续独立保存安全事件 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| `src/lib/rc/sbus.h`、`src/lib/rc/sbus.cpp` | `Dima/lib/rc/sbus.hpp`、`Dima/lib/rc/sbus.cpp` | 保留 25-byte 帧、16 路 11-bit 通道、数字 17/18、4 ms 重同步、Failsafe/Frame-Lost 与 PX4 数值映射；移除 POSIX 串口和 SBUS 输出 | ADAPTED |
| `src/drivers/rc/sbus_rc/SbusRc.hpp`、`SbusRc.cpp`；ArduPilot `AP_RCProtocol::requires_3_frames()` 行为 | `Dima/modules/rc/SbusRc.*` | 保留 WorkItem 接收、锁定、重试和 `input_rc` 发布流程；无强 CRC 的 SBUS 冷启动/Failsafe/UART 恢复后要求连续 3 个健康帧，锁定前仍发布原始通道但标记 lost；协议选择自动决定电气状态，任务上下文输出锁定/失联/恢复/Failsafe 和单次后端故障日志；ArduPilot 仅作行为参考，未复制 GPL 实现 | ADAPTED / PLATFORM ISOLATED |
| PX4 串口生成与板级 RC 输入行为 | `Boards/H743/serial_ports.json`、`tools/serial/generate_config.py`、`Dima/modules/serial/SerialConfig.*`、`Dima/platform/stm32h7/serial/SbusUart.cpp`、`SbusUartHal.cpp` | 板级清单锁定最新版 VCU-H7 原理图与 `H743_FreeRTOS.ioc`，单源生成 SERIAL0=USB，并让 SERIAL1..8 直接对应 `USART1/USART2/USART3/UART4/UART5/USART6/UART7/UART8`。八组 `SERIALx_BAUD/FUNCTION` 使用当前产品默认值，Function 只允许 Disabled/RC Input 且单 owner；SerialConfig 应用普通 8N1，SBUS 接管时切换 100000/8E2/RXINV/RX-only，释放后恢复。固件不包含旧板级键、迁移表或迁移版本参数 | DIMA BACKEND / OWNERSHIP GATED / TARGET VERIFY PASS / BOARD PENDING |
| `src/modules/rc_update/rc_update.h`、`rc_update.cpp` | `Dima/modules/rc/RCUpdate.*` | 保留 18 通道校准、Flaps/Aux/主控制功能映射、开关离散化、失联与 `parameter_update` 语义；协议锁定后再要求连续健康 100 ms 才解除控制 lost，frame-lost 只计数；差速 Rover 默认 Throttle/Yaw=通道 1/2，Roll/Pitch 默认未映射且不进入车辆输出；`COM_RC_IN_MODE` 非 0 时 fail-closed；裁剪 `PARAM_MAP_RC` 任意参数调节及非 Rover 功能 | ADAPTED |
| `src/modules/manual_control/ManualControl.hpp`、`ManualControl.cpp`；ArduPilot RC switch debounce 行为 | `Dima/modules/rc/RcManualInput.*` | 保留 RC setpoint 和二段开关边沿 Action Request；Arm/Kill 必须至少两份严格前进的一致样本并稳定 200 ms，启动、RC 恢复及映射/阈值变化后首个稳定状态只建立无动作基线；本地名称明确其只拥有 RC 来源转换，ArduPilot 仅作行为参考 | ADAPTED |
| `src/modules/commander/Commander.hpp`、`Commander.cpp`、`ModeUtil/control_mode.*`；ArduPilot Rover 左右电机 pre-arm 行为 | `Dima/modules/safety/Commander.*` | 保留 Action Request、Arming/Kill/Termination、QGC RC calibration、状态发布顺序和 Manual/Termination 语义；通过 `actuator_output_status` 要求新鲜/严格前进、后端 ready、至少一右一左映射及 `DISARMED_NEUTRAL`，Armed 输出 Fault/Stale 强制 Disarm；允许新鲜完整 hard-off 先清执行器 cause，避免恢复自锁；Kill 固定为 Disarm，Unkill 不自动 Arm；ArduPilot 仅作行为参考 | ADAPTED |
| `msg/InputRc.msg`、`RcChannels.msg`、`ManualControlSetpoint.msg`、`ManualControlSwitches.msg`、`ActionRequest.msg` | `Dima/messages/` | 保留 PX4 字段、枚举和 Topic 契约；`action_request` Queue Depth 为 8 | ADAPTED |
| `msg/versioned/VehicleStatus.msg`、`msg/versioned/VehicleControlMode.msg`、`msg/ActuatorArmed.msg` | `Dima/messages/` | 完整保留三个公开消息的字段、枚举和版本号；本地三个 Topic 均为单深度 | ADAPTED |
| `msg/versioned/ActuatorMotors.msg` | `Dima/messages/actuator_motors.*` | 完整保留 version 0、12 路 control、reversible flags 和采样时间；Topic 单深度，阶段 5 仅使用前两路 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| Dima Rover 两轴请求与输出状态 | `Dima/messages/rover_motion_request.*`、`actuator_output_status.*` | 两个 Topic 深度均为 8；前者隔离 Manual/未来 Navigation 与混控，后者显式暴露 configured/right/left mask 以及 `HARD_SAFE_OFF / DISARMED_NEUTRAL / ACTIVE / RETRY / FAULT` 六路后端状态 | DIMA CONTRACT / TARGET VERIFY PASS / BOARD PENDING |
| `src/modules/rover_differential/RoverDifferential.cpp`、`DifferentialDriveModes/DifferentialManualMode/`、`DifferentialActControl/` | `Dima/rover/modes/ManualMode.*`、`Dima/rover/control/RoverDifferential.*`、`Dima/lib/rover/DifferentialDrive.*` | 保留 Manual 两轴、100 Hz rate-control WorkQueue 和 `actuator_motors` 边界；反向不对称在混控前将两侧可行域限制为 `[-1/MOT_THR_ASYM, 1]`，先保留转向/油门优先级再作反向补偿，避免倒车两侧同时饱和丢失转向；其余适配保持固定存储、Commander 三 Topic 门控及 ARMED 参数延后 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| PX4 actuator function/output 行为边界；ArduPilot `MOT_SAFE_DISARM=0` 中立行为参考 | `Dima/modules/motor/MotorOutput.*`、`Dima/platform/api/ActuatorPwmLimits.h`、`ActuatorPwm.hpp`、`Dima/platform/stm32h7/io/ActuatorPwm.cpp`、`Boards/H743/Src/motor_pwm.c` | C/C++ 共用普通 PWM 产品包络固定为 500～2500 us，板级后端同时执行防御性校验；本地收敛为独立生命周期的 MotorRight/MotorLeft 到 S1～S6 固定存储映射。健康普通 Disarmed 仅向已配置通道输出 `CENT`，Kill/Termination/Failsafe/故障进入物理 hard-off；分离 ACTIVE inhibit 与 hard-safe inhibit，任一负向安全 Topic 先到即可 fail-closed；不导入通用 Mixer/FunctionMotors/MixingOutput，APM GPL 代码未复制 | DIMA ADAPTATION / TARGET VERIFY PASS / BOARD PENDING |
| PX4 `src/lib/rover_control`、`src/lib/pid`、`src/lib/slew_rate` | 尚未创建；阶段 8 按届时状态估计和运行接口重新导入 | 不保留无消费者的预实现控制器；导入时仍须保持 Speed、Yaw Rate、Heading、Stop 控制核的纯算法边界 | PLANNED / NO SOURCE PLACEHOLDER |
| ArduPilot `libraries/AR_Motors/AP_MotorsUGV.cpp`、Rover pre-arm 与 watchdog 行为 | `Dima/lib/rover/DifferentialDrive.*`、`Dima/modules/safety/Commander.*`、IWDG 健康合同 | 仅参考倒车车头方向、饱和优先级、slew、静摩擦补偿、反向不对称、左右映射 pre-arm、换向延时和主循环停滞复位行为；未复制 GPL 源码 | BEHAVIOR REFERENCE ONLY |
| 旧 Dima `speed_to_pwm` 固定六路转换 | 已移除 | 自建仓以来没有生产调用，历史唯一消费者为已删除的 Host Test；当前六路参数化转换与 safe-off 所有权统一由 `Dima/modules/motor/MotorOutput.*` 承担 | RETIRED / SUPERSEDED |
| Stage 5 Rover 控制与油门保护参数 | `Dima/middleware/parameters/definitions/rover_actuator_params.c` | 新增请求超时、倒车转向、混控优先级、最小/最大输出、slew、换向延时、expo、反向不对称及解锁 ramp；运行期仅在新鲜 DISARMED 快照后整体应用 | DIMA PARAMETER / TARGET VERIFY PASS |
| 六路 PWM 映射参数 | `Dima/middleware/parameters/definitions/rover_actuator_params.c` | S1～S6 各提供 `FUNC/MIN/CENT/MAX/REV`；范围 500～2500 us、默认 1000/1500/2000 us，默认 Function 为 Disabled，功能只允许 MotorRight/MotorLeft。完整 DISARMED 快照中逐通道校验；`MIN/MAX` 反序只交换运行时端点，其他非法通道禁用而不停止运行时；普通 Disarmed 在其余有效通道输出 `CENT`，只有至少一右一左时 `drive_available=true` 并允许 ACTIVE | DIMA PARAMETER / TARGET VERIFY PASS / BOARD PENDING |
| PX4 RC/Commander/QGC 参数定义与生成元数据 | `Dima/middleware/parameters/definitions/`、`Boards/H743/serial_ports.json` | 保留真实消费的 RC、固定 MAV sysid、RC/GCS loss；移除未实现的六档模式与 RTL 参数，`RC_MAP_FLTMODE` 仅作为固定 Disabled 的 QGC 合同字段；`qgc_compat_params.c` 提供 11 项关键 Summary 合同，板级生成器提供 16 项公开 SERIAL 参数。不保留旧 handle、stable-tail、旧键或内部迁移版本，当前 203 项参数按名称排序 | ADAPTED / DIMA PARAMETER / TARGET VERIFY PASS / BOARD PENDING |
| Dima Rover 生命周期、BootHealth 与 Parameter Autosave 写门控 | `Dima/rover/ApplicationContext.*`、`Dima/modules/boot_health/BootHealthService.*`、`Dima/modules/parameters/ParameterService.*`、`Dima/middleware/maintenance/`、`Dima/middleware/uorb/`、`Dima/middleware/work_queue/`、`Dima/platform/api/{Services,Execution,Flash,Memory,Synchronization,TaskRuntime}.*` | ApplicationContext 只注入 capability；BootHealth 依据安全/输出 Topic 进展推进 generation，不跨队列读取普通模块状态。维护票据还要求 Disarmed、neutral/hard-safe、`appMain` reload 确认、单调存储进度和长期 Arm interlock；shutdown 在释放资源前确认六路物理 hard-off | DIMA INTEGRATION / SOURCE AND TARGET GATE PASS / BOARD PENDING |
| STM32H7 IWDG 与 MCUboot 跨复位衔接 | `Dima/platform/api/Boot.hpp`、`Dima/platform/stm32h7/system/Watchdog.cpp`、`Dima/application/app_main.cpp`、`Bootloader/Inc/boot_watchdog.h`、`Bootloader/Src/boot_watchdog.c`、MCUboot feed hook | 应用约 2048 ms、100 ms 检查，appMain 为唯一应用 feed owner；冷启动按 STM32 HAL 顺序先 start IWDG/LSI、再写配置并等待 SR 同步，跨复位已运行时允许重复 start key；MCUboot 对已运行 watchdog 临时扩展到约 32 s但不主动启动 inactive IWDG，Recovery/校验/swap/Flash/USB 长循环统一 feed；DBG halt 冻结，复位原始原因跨应用桥接保留 | DIMA SAFETY / TARGET VERIFY PASS / BOARD PENDING |
| MCUboot image confirmation 与 Recovery | `Dima/platform/stm32h7/system/BootControl.cpp`、`flash/flash_bank1.c` | 非阻塞 transaction 保留 DEFERRED；Bank 1 program 在 DTCM 执行并统一调用 cache helper | DIMA BACKEND |
| `src/modules/mavlink/mavlink_main.cpp`、`streams/RC_CHANNELS.hpp` | `Dima/modules/mavlink/MavlinkService.hpp/cpp`、`MavlinkChannelState.cpp`、`MavlinkSystemMessages.cpp`、`MavlinkRcStream.cpp` | Service 主文件持有 RX/TX 主循环、USB 传输独占、ACK/日志和延迟 reboot；ChannelState 使全部分源编码器共享单一 parser buffer 与 RX/TX sequence；System 文件持有版本与 Component Metadata 发现回复；RC 文件直接订阅原始 `input_rc` 并以 5 Hz 发送有效通道。TX 优先级在 RC 后每轮最多处理一帧三文件只读 Metadata FTP，pending 未解决时不发送更低优先级参数/日志；Runtime 复位全部协议状态，USB 物理下降沿丢弃旧 RX 并复位 parser/channel 与 FTP session | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| `src/modules/mavlink/mavlink_receiver.cpp`（命令处理子集） | `Dima/modules/mavlink/MavlinkCommands.hpp/.cpp` | COMMAND_LONG/COMMAND_INT 接收、target 过滤、source system/component 保留、`vehicle_command` 发布及 `MAV_CMD_REQUEST_MESSAGE`；Commander ACK 定向回命令 source；裁剪流间隔控制和 Autotune | ADAPTED |
| `src/modules/mavlink/mavlink_mission.cpp`（空任务语义） | `Dima/modules/mavlink/MavlinkMission.hpp/.cpp` | `MISSION_REQUEST_LIST` 返回 count 0，`MISSION_CLEAR_ALL` 返回 ACCEPTED；不建立任务存储或上传链路 | ADAPTED / EMPTY MISSION |
| `src/modules/mavlink/mavlink_parameters.cpp` | `Dima/modules/mavlink/MavlinkParameters.hpp/.cpp`、`MavlinkParameterExt.cpp` | 主文件处理 Classic LIST/READ/SET 与 PARAM_VALUE，Ext 文件处理 EXT_REQUEST_READ 与 PARAM_EXT_VALUE；每次 LIST 前公开真实 RC、16 项 SERIAL 参数和 11 项固定 QGC 合同，并从不可变 used 句柄快照发送整轮 count/index。Function 写入只允许 Disabled/RC Input，选择新 RC owner 时在单个参数事务内禁用旧 owner并回传受影响参数；当前目录没有旧板级键或内部迁移参数 | ADAPTED |
| `src/modules/mavlink/mavlink_timesync.cpp` | `Dima/modules/mavlink/MavlinkTimesync.hpp/.cpp`、`Dima/lib/timesync/Timesync.hpp/.cpp` | TIMESYNC 处理——远端回传、本地喂入收敛滤波器；省略 SYSTEM_TIME 时钟设置 | ADAPTED |
| PX4 `src/modules/mavlink/mavlink_ftp.*` 与 Component Metadata | `Dima/modules/mavlink/MavlinkMetadataFtp.hpp/.cpp`、`tools/mavlink/generate_parameter_metadata.py`、`build/generated/component_metadata/` | 397 现代发现与 395 deprecated 回退共用 URI/CRC；General 声明 type 1 Parameter 与 type 5 Actuator Metadata。FTP 只允许 General/Parameter/Actuator 三个 Flash 虚拟文件和 Open/Burst/Read/Reset/Terminate，不含目录、写入或 Event Metadata；Actuator Metadata 开放六路 PWM 分配和参数编辑，但 MotorRight/MotorLeft 排除执行器测试，固件不实现 `MAV_CMD_ACTUATOR_TEST` | ADAPTED / READ-ONLY / TARGET VERIFY PASS / BOARD PENDING |
| `src/lib/mavlink/`（mavlink commit `33af200d`，pymavlink submodule `fcaa2c7d`） | `tools/mavlink/` 输入与 `build/generated/mavlink/` 生成物 | 仅跟踪 common/standard/minimal XML、lock 和生成脚本；clean build 确定性生成 24 条消息，包含 FILE_TRANSFER_PROTOCOL、COMPONENT_METADATA 和 COMPONENT_INFORMATION，禁止 COMPONENT_INFORMATION_BASIC 与 `SERVO_OUTPUT_RAW` | ADAPTED / TRIMMED / REPRODUCIBLE PIPELINE |
| PX4 `src/modules/mavlink/` 心跳与身份语义 | `Dima/modules/mavlink/MavlinkIdentity.hpp/.cpp`、`HeartbeatPacer.hpp/.cpp` | USB ready 边沿立即发送 HEARTBEAT/AUTOPILOT_VERSION，之后 1 Hz；Manual `custom_mode=0x00010000`，Disarmed/Armed base mode=65/193，内部 Termination 保持真实编码；capability 声明 PARAM_FLOAT、BYTEWISE、FTP、COMMAND_INT、MAVLINK2 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| Fault/Reset 跨复位诊断持久化 | `Boards/H743/Src/boot_diagnostics.c`、`boot_diagnostics_store.c`、`Bootloader/Src/main.c` | Application Fault/Panic 只写 non-cacheable D3 record、执行 barrier 并复位；原始 RCC reset flags（含 IWDG）在 MCUboot 应用桥接软件复位与同次 hot handoff 中保留；冷上电/全片擦除/ROM DFU 后无有效 D3 头时由 MCUboot 先建立最小 v2 bridge 记录，避免首启永久停在 Recovery；MCUboot 冷启动独占诊断 Flash store 和 Recovery，Application ELF 禁止链接 store 符号 | DIMA SAFETY / ELF GATED |

## 7.1 STM32Cube 中间件导入

| 字段 | 内容 |
|---|---|
| 来源 | STM32Cube Firmware Package for STM32H7 `STM32Cube_FW_H7_V1.10.0` |
| 用途 | FatFs FAT 文件系统中间件，用于 SD 卡文件存储 |
| 本地目录 | 核心与配置位于 `Middlewares/Third_Party/FatFs/src/`；文件后端位于 `Dima/platform/freertos/storage/`；H743 disk port 位于 `Boards/H743/Src/fatfs_diskio.c` |
| 许可证状态 | `PENDING`；FatFs 原始 BSD 条款，ST 包装部分 BSD-3-Clause |

| 上游原始路径 | 本地映射 | 适配方式 | 状态 |
|---|---|---|---|
| `Middlewares/Third_Party/FatFs/src/ff.c` | 同路径 | 原样复制，未修改 | UNMODIFIED |
| `Middlewares/Third_Party/FatFs/src/ff.h` | 同路径 | 原样复制 | UNMODIFIED |
| `Middlewares/Third_Party/FatFs/src/integer.h` | 同路径 | 原样复制 | UNMODIFIED |
| `Middlewares/Third_Party/FatFs/src/ffconf_template.h` | `ffconf.h` | 定制配置：`_FS_REENTRANT=0`（唯一客户端由 FileStorage 后端互斥量串行化）、`_USE_LFN=0`、`_CODE_PAGE=1`（ASCII 8.3）、`_USE_MKFS=0`、`_USE_FASTSEEK=0`、`_FS_LOCK=0`、`_VOLUMES=1`、`_FS_NORTC=1` | ADAPTED |
| `Middlewares/Third_Party/FatFs/src/diskio.c` | `Boards/H743/Src/fatfs_diskio.c` | 重写为直接调用 HAL_SD（`hsd1`），不使用 `ff_gen_drv` 抽象层；增加 32 字节对齐 scratch buffer 位于 `.dima_dma` 区 | REWRITTEN / BOARD OWNED |
| `Middlewares/Third_Party/FatFs/src/diskio.h` | 同路径 | 重写，移除 `ff_gen_drv` 依赖 | REWRITTEN |

2026-08-19 使用 Windows 原生 GNU Make 4.4.1 与项目缓存的 Arm GCC 10.3.1 执行 `make clean` 后 `make -j4 NO_COLOR=1 dima_rover`，完整通过 `[212/212]`。Application 为 `text=233772/data=12284/bss=356176`、未签名 BIN `246096` bytes，MCUboot 为 `text=47712/data=380/bss=10192`、BIN `48100` bytes，向量 `0x08040400`。Application/MCUboot 未解析符号均为空，ELF 已确认 SBUS/Commander/MotorOutput/IWDG 健康链及 MCUboot watchdog prepare/feed 符号实际链接。同一 image digest `601c65353ffebce20cea8f040d975000e3c4caa2b27aaa15dd409529740e26ce` 的两次有效 ECDSA P-256 重签分别生成 `247271/247270`-byte Signed BIN 与 `710861/710859`-byte Factory HEX；imgtool 的 DER 签名长度可变，因此这两个容器的文件长度和文件 SHA-256 不属于确定性构建合同。SBUS 电气、PWM 波形、电调握手、IWDG 实际时限和车辆行为仍为 `BOARD PENDING`。

同日实板 ROM DFU 首启排查又修复 D3 bridge 冷启动循环依赖和 IWDG start-before-sync 顺序。故障记录为 v2 sequence 1：`ERROR_HANDLER`、`APPLICATION_RUNNING`、`stacked_r0=2048`，且 `CFSR/HFSR/ABFSR=0`。修复后 Windows 原生 clean build 通过 `[212/212]`：Application 仍为 `233772/12284/356176` bytes、BIN `246096` bytes，MCUboot 为 `47864/380/10192` bytes、BIN `48252` bytes，image digest `835947050b2df9062be4289bcb5b876abe0dee99b46792051827e79f123eeec3`，Signed BIN `247271` bytes、Factory HEX `711217` bytes；烧写后实板稳定枚举为 Application，并由只读 preflight 返回 `Dima Rover MAVLink version=0x00010000 board=1`。Signed/Factory 长度仍遵循上述 DER 非确定性边界。

同日完成目录与源码职责整理后，再次使用 Windows 原生 GNU Make 4.4.1 与项目缓存的 Arm GCC 10.3.1 执行 `make NO_COLOR=1 clean` 和 `make -j4 NO_COLOR=1 dima_rover`，完整通过 `[230/230]`，架构检查覆盖 261 个首方源码文件。Application 为 `text=220092/data=12284/bss=356056`、未签名 BIN `232416` bytes，ELF/BIN SHA-256 分别为 `3305882171aa18ec61286d08f9acd9b406e0d043809ac77450e2a67649a97a20` / `92ce5c84c6a6b12ad61d7c0a80ca83497c4f4fe1fea9e3b5284099ddf76392fb`；本次签名样本的 image digest 为 `c661dc5566ae50d13ca549d526934b0400684963daace1b2ca525f4ab0fefad2`，Signed BIN 为 `233591` bytes（SHA-256 `7f18d0ec86a9d549788b53f615d3ef98d73d3304732b75a3e150f6a63f3b6ca2`），Factory HEX 为 `678293` bytes（SHA-256 `c249358338195ac304829ca3d5d9a54db70a442508009fba3c7f473246a8b6bb`）。MCUboot 为 `text=47864/data=380/bss=10192`、BIN `48252` bytes，ELF/BIN SHA-256 分别为 `7a89d92a443ee581f7e26a32c673b944b1a481ebdc1d7653b183c2c33b34b1a6` / `379562b2bf473f23b73c5924f1491235f0df4ab9bdd894a50a69485f4ce51499`。应用向量仍为 `0x08040400`，19 项 init-array 白名单、两份未解析符号检查、Signed/Factory 布局、watchdog prepare/feed、安全 PWM、SBUS 与 Commander 链均通过；Signed/Factory 的长度和文件哈希仍只是本次 DER 签名样本，不作为确定性合同。

2026-08-21 当前生成合同为 203 项固件参数和 203 项公开 Parameter Metadata，按当前参数名排序，不保留旧 handle、stable-tail、旧键或内部迁移版本。General/Parameter/Actuator XZ 分别为 `192/4904/420` bytes；最新正式生成 stamp 的 General/Parameter/Actuator CRC32 为 `0xc49e389e/0x6f14d0f7/0xaf58d846`，General Metadata 类型严格为 `1,5`。派生文件的 size/SHA-256 不再手工复制到本文；唯一可机读事实源是 `build/generated/component_metadata/.generated.json`，生成器从同一组内存输出写入全部七个派生文件及其 size/SHA-256 stamp，正式架构目标会重新校验输入 SHA、七份 size/SHA、XZ round-trip 与三项 CRC。Actuator Metadata 是经本地结构门禁验证的 Dima output-only 适配：保留六路 `outputs_v1`，故意使用空 `mixer_v1.config`，且禁止 MotorRight/MotorLeft actuator testing。当前 QGC Actuators 页面可用由用户实测确认；实际输出、SD 拔插、IWDG 时限和车辆行为仍分别保留板级验收边界。

随后将 `Dima/platform/stm32h7` 的扁平实现整理为 `system/memory/flash/serial/io` 五个职责目录，根部只保留硬件工厂声明和总 README；原先无人包含的 `flash_fault.h` 下沉到 `flash/`，并改为 Core 弱钩子与 C++ 强实现共同包含的唯一 C ABI 声明。Windows 原生 clean `make -j4 NO_COLOR=1 dima_rover` 再次完整通过 `[230/230]`，架构检查覆盖 261 个首方源码文件；Application/MCUboot 的 text/data/bss、向量、19 项 init array、未解析符号和安全链结果均不变。与搬移前封存产物相比，两份 loadable BIN、ELF program headers 和已定义符号表逐项一致：Application BIN SHA-256 仍为 `92ce5c84c6a6b12ad61d7c0a80ca83497c4f4fe1fea9e3b5284099ddf76392fb`，MCUboot BIN 仍为 `379562b2bf473f23b73c5924f1491235f0df4ab9bdd894a50a69485f4ce51499`，image digest 仍为 `c661dc5566ae50d13ca549d526934b0400684963daace1b2ca525f4ab0fefad2`。ELF 整体哈希仅因 DWARF 源路径变化更新为 Application `c8eddd965266834765c90c286fe2b0ccdda03caf6ae7e993f2275ad3f43660de`、MCUboot `9e2d372ab51e58a1a50b47034a9d16d91da50fc6fcb287a50088777c035185e6`；串口、参数与 Component Metadata 三棵生成树仍逐字节一致。Windows `make intellisense` 也已生成 191 个源码的 211 条新路径编译命令。

许可证状态仅记录为 `PENDING`；延后处理项记录为 `DEFERRED`。该状态不阻塞当前内部移植、编译和板级调试工作。

## 7.2 nanoprintf 格式化实现

| 字段 | 内容 |
|---|---|
| 来源 | `https://github.com/charlesnicholson/nanoprintf` |
| 正式版本 | `v0.8.0` |
| 正式 commit | `115c916031ef51b73c5c373852fa550eaf134d49` |
| 本地目录 | `Middlewares/Third_Party/nanoprintf/` |
| 用途 | Application 日志与 MAVLink `PARAM_EXT_VALUE` 的固定容量格式化，替代 full newlib printf 转换链 |
| 适配边界 | 上游 header/LICENSE 原样保留；项目配置和 `format_to`/`vformat_to` wrapper 位于 `Dima/lib/format/`；USB stdout/setvbuf 能力保持不变 |
| 许可证状态 | `PENDING`；上游声明双许可 Unlicense/0BSD，本阶段不作为构建阻断项 |

固定 header SHA-256 为
`c7e445450ce496e61f15c1cbdaa5a93e741f8ac4718f4ab6bcfedae1dc81c47c`。

## 8. 许可证状态

- PX4 v1.17.0 来源文件：`PENDING`。
- 最终产品分发与 Notice 收敛：`DEFERRED`。
- 当前阶段仅保持来源、commit、本地映射和原始版权头可追踪；许可证事项延后收敛，不阻塞技术实现。
