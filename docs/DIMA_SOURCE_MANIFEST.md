# Dima 上游源码与许可证清单

- 日期：2026-08-31
- 文档状态：阶段 1～6 已接通；N1 单实例 PX4 v1.17.0 EKF2 源码、权威参数/uORB/MAVLink 合同和运行适配已导入，Wind/Terrain 状态与无效 GSF TAS 参数已从唯一权威入口移除，并通过 Windows fresh build、verify、Parameter Metadata、ELF 与架构验收；未新增 SHA-256 行为或验收基线
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
| 用途 | Parameter Core、`dima::Param<T>` 公开适配、uORB API/消息契约、WorkQueue 接口、SBUS、RCUpdate、ManualControl RC 子集、Commander Rover 子集、RoverDifferential、执行器链、MAVLink v2.0 协议处理和 N1 EKF2 |
| 正式目标版本 | PX4 v1.17.0 |
| 正式 commit | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 当前状态 | 阶段 1～6 已按 v1.17.0 接口和行为适配；N1 单实例 EKF2、GNSS/Mag/Gravity fusion、bias/GSF/output 与四条导航 MAVLink 流已接入并通过 Windows fresh acceptance |
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
| MAVLink v2.0 协议处理 | `Dima/modules/mavlink/` | ADAPTED / N1 NAV STREAMS / WINDOWS BUILD VERIFIED |
| PX4 EKF2 单实例模块与支撑库 | `Dima/modules/ekf2/`、`Dima/lib/{ekf2,matrix,mathlib,geo,lat_lon_alt,world_magnetic_model}/` | ADAPTED / N1 PHYSICAL CLOSURE / WINDOWS BUILD VERIFIED |
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
| Parameter Core、`dima::Param<T>` | PX4 v1.17.0 | `Dima/middleware/parameters/` | 2 | ADAPTED / TARGET VERIFY PASS |
| SBUS、SbusRc、RCUpdate、ManualControl RC 子集 | PX4 v1.17.0 | `Dima/lib/protocols/sbus/`、`Dima/drivers/rc/sbus/`、`Dima/modules/rc/`、`Dima/platform/stm32h7/serial/` | 3 | ADAPTED / PLATFORM ISOLATED / BOARD PENDING |
| Commander Rover 子集 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/safety/` | 4 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| RoverDifferential 与执行器链 | PX4 v1.17.0；APM 行为参考 | `Dima/rover/control/`、`Dima/lib/rover/`、`Dima/modules/motor/` | 5 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| MAVLink v2.0 协议处理（RX/TX、心跳、命令、Classic + Ext 参数、5 Hz 原始 RC、空任务、时间同步） | PX4 v1.17.0；mavlink/mavlink commit `33af200d` | `Dima/modules/mavlink/`、`build/generated/mavlink/` | 6 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| EKF2 与 Estimator 支撑库 | PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | `Dima/modules/ekf2/`、`Dima/lib/{ekf2,matrix,mathlib,geo,lat_lon_alt,world_magnetic_model}/` | 7 | ADAPTED / SINGLE INSTANCE / WINDOWS BUILD VERIFIED |
| Position、Waypoint、Reverse、PivotTurn | PX4 v1.17.0；APM 行为参考 | `Dima/rover/navigation/` | 9 | PLANNED |

## 7. 文件级映射

阶段 2～5 的唯一 PX4 直接代码来源基线为 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；ArduPilot commit `3f2e4763accb` 仅作 Rover 安全、倒车和油门输出行为参考，不直接导入代码。

### 7.0 生成工具来源闭包

| 生成域 | 固定来源 | 本地原件与机读闭包 | 生成边界 |
|---|---|---|---|
| Parameter YAML | PX4 commit `1f6b6f61f8f42eaab0269c16a442cb580f954d7c` | `tools/upstream/parameter_yaml_20260827/`、`SOURCE_MANIFEST.json` | 原始 validate/module generator/process/header 脚本与 schema/template 逐文件 SHA-256；产品只保留 YAML 权威输入 |
| uORB | PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | `tools/upstream/uorb_v1_17/`、`SOURCE_MANIFEST.json` | 原始脚本/helper/EmPy 模板与 PX4 同名参考 schema 逐文件 SHA-256；Topic ID/hash/JSON/registry 全部派生 |
| MAVLink wire | definitions commit `33af200d25ec6f0925b49b1ba82bbf1294ea5f72`；pymavlink 2.4.47 commit `fcaa2c7d25e3169dc66155929c338487941555e9` | `tools/mavlink/message_definitions/`、`mavlink.lock.json` | 固定 XML SHA-256、pymavlink archive/tree SHA-256；`dima.xml` 直接进入原始 mavgen，运行 YAML 不参与 wire |
| DroneCAN DSDL | PX4/DSDL commit `993be80a62ec957c01fb41115b83663959a49f46`；dronecan_dsdlc commit `431170fa4bfe2212b516b8f33bdc796267907f1c` | `Middlewares/Third_Party/dronecan_dsdl/`、`SOURCE.md`；薄编排位于 `tools/dronecan/generate_contract.py` | 受版本控制的 DSDL 目录是 wire 唯一输入，工具自动发现完整文件闭包并生成 codec/描述符；不保留 DroneCAN JSON、消息清单或参数特例，产品参数独立来自 `module_dronecan.yaml` |

`tools/generation/source_manifest.py` 与架构门禁动态扫描文件集合，不在 Make/Python/文档中复制上游文件或消息名称列表。来源目录增删或任一文件 hash 漂移都会在正式生成前失败。

| 上游原始路径/功能 | 本地映射 | 适配方式 | 状态 |
|---|---|---|---|
| `src/lib/parameters/parameters.cpp`、Parameter Layer/Core、AtomicTransaction | `Dima/middleware/parameters/` | 保留 `param_*`、稀疏 Layer、事务及参数更新语义；锁、执行上下文和内存只通过公共 capability | ADAPTED / PLATFORM ISOLATED |
| `platforms/common/include/px4_platform_common/param.h` | `Dima/middleware/parameters/` | 保留显式 bind/update 语义，对产品公开为 `dima::Param<T>` 与 `dima::params` | ADAPTED / DIMA PUBLIC API |
| `Tools/validate_yaml.py`、`Tools/module_config/generate_params.py`、`src/lib/parameters/{px_process_params.py,px_generate_params.py,px4params/,templates/}` | `tools/upstream/parameter_yaml_20260827/`；薄编排位于 `tools/parameters/generate_parameters.py` | 原始工具依次生成中间 `module_params.c`、XML/JSON 和原始暂存头；Dima 只机械安装 `dima_parameters.hpp` 并从官方产物派生运行时合同，不保留本地 parser/renderer | UNMODIFIED UPSTREAM / THIN ORCHESTRATION |
| `Tools/msg/px_generate_uorb_topic_files.py`、helper 与 `templates/uorb/*.em` | `tools/upstream/uorb_v1_17/`；薄编排位于 `tools/uorb/generate_messages.py` | 原始工具从 PascalCase PX4 `.msg` 生成 Topic 头/源、ID、hash、JSON 和 `uORBTopics`；旧 `.hpp` 路径只由 `ORB_DECLARE` 动态派生 include-only 转发头 | UNMODIFIED UPSTREAM / THIN ORCHESTRATION |
| `src/lib/tinybson/tinybson.h/.cpp` | `Dima/lib/tinybson/` | 保留上游 BSD 头；删除 fd、POSIX 和动态扩容路径，仅保留固定 Buffer 编解码 | ADAPTED |
| `src/lib/parameters/flashparams/` | `Dima/middleware/parameters/flashparams/` | 改为 Parameter enumerator/visitor 与 TinyBSON Buffer 之间的适配，不直接访问文件系统 | ADAPTED |
| PX4 Parameter Autosave 与 Runtime cache | `Dima/middleware/parameters/` | 300 ms 合并、保存间隔至少 2 s、失败最多重试 3 次；ParameterService/Autosave 位于独立 `wq:storage`，不得以 SD 同步 I/O 阻塞 MAVLink；`Param<T>` 构造无 Core 副作用，每次 start bind，每次 shutdown 清 ready/used/unsaved/value cache、callback 和动态 Layer | ADAPTED / RUNTIME LIFECYCLE |
| PX4 参数协议与 USB 接入 | `Dima/modules/mavlink/`、Parameter Service | MavlinkService 独占 CDC RX/TX；ParameterService 只负责 Core、FlashFS/FileStorage、Autosave 和 Flash。Classic/Ext 按官方连续 handle 遍历完整参数目录，LIST、按 index 补读、READ/SET 与 ACK 复用同一 count/index，不维护 QGC/public 参数名单；旧快照中的退役名称跳过，其余有效配置继续恢复 | ADAPTED / SINGLE OFFICIAL CATALOG |
| PX4 flashparams/flashfs 行为参考 | `Dima/middleware/parameters/flashfs.*`、`FileStorage.*`、`Dima/platform/api/ParameterFileStore.hpp`、`Dima/platform/freertos/storage/`、`Boards/H743/Src/fatfs_diskio.c` | 不复制依赖 NuttX SDIO/MMCSD/VFS 的 PX4 驱动，只保留“有限等待、独立 I/O 执行域、失败降级、Flash 主存储/SD 备份”合同。SD/FatFs 在唯一固件配置中强制编译；FlashFS 是主存储，SD 是 generation 排序的镜像和恢复源，同 generation 还比较 payload CRC。运行期写入由 `appMain + BootHealth` 批准维护票据，Flash 按 32-byte、FatFs 按 512-byte 分步推进；无 card-detect GPIO 时由 `wq:storage` 每 3 秒探测，挂载后稳定 500 ms，介质错误使旧挂载失效。HAL 软件轮询以 500 ms 为单个等待点上限；拔卡后 Flash 参数和 MAVLink 继续 | ADAPTED / PLATFORM ISOLATED / TARGET VERIFY PASS / BOARD PENDING |
| Rover Parameter 定义 | `Dima/middleware/parameters/definitions/` | 导入 24 项 `RO_*`/`RD_*` 参数，名称、默认值、单位和元数据保持 PX4 来源 | ADAPTED |
| `platforms/common/include/px4_platform_common/log.h`、`platforms/common/px4_log.cpp` | `Dima/middleware/logging/`、`Dima/modules/logging/` | 保留 PX4 日志宏和 SourcePolicy；uORB 初始化后注册 non-blocking structured sink，普通/RAW 日志写入深度 8 的 `mavlink_log`，MavlinkService 有界转为 STATUSTEXT；RAW 只绕过普通等级过滤，不绕过实时格式化禁令，因此 QGC 校准事务固定运行于非实时 `wq:lp_default`；Event Ring 继续独立保存安全事件 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| `src/lib/rc/sbus.h`、`src/lib/rc/sbus.cpp` | `Dima/lib/protocols/sbus/SbusProtocol.*` | 保留 25-byte 帧、16 路 11-bit 通道、数字 17/18、4 ms 重同步、Failsafe/Frame-Lost 与 PX4 数值映射；移除 POSIX 串口和 SBUS 输出 | ADAPTED |
| `src/drivers/rc/sbus_rc/SbusRc.hpp`、`SbusRc.cpp`；ArduPilot `AP_RCProtocol::requires_3_frames()` 行为 | `Dima/drivers/rc/sbus/SbusRc.*` | 保留 WorkItem 接收、锁定、重试和 `input_rc` 发布流程；无强 CRC 的 SBUS 冷启动/Failsafe/UART 恢复后要求连续 3 个健康帧，锁定前仍发布原始通道但标记 lost；驱动显式申请 100000 8E2/RXINV/RX-only 通用串口 capability，任务上下文输出锁定/失联/恢复/Failsafe 和单次后端故障日志；ArduPilot 仅作行为参考，未复制 GPL 实现 | ADAPTED / PLATFORM ISOLATED |
| PX4 串口参数与板级 SBUS/GPS 输入行为 | `Dima/middleware/parameters/definitions/module_serial.yaml`、`Dima/modules/serial/SerialConfig.*`、`Dima/platform/stm32h7/serial/` | 标准 `module_serial.yaml` 是 SERIAL 参数唯一源，既有包装器只调用锁定上游 validator/generator，不含串口特化，也不生成串口专用合同。SerialConfig 从 `dima::parameter_catalog` 按命名规则发现生成参数；STM32 句柄、DMA、IRQ 与 GPIO token 留在板级实现。SERIAL5 因无 UART5 留空，Disabled/SBUS/GPS 均按单 owner fail-closed | DIMA BACKEND / TARGET VERIFY PASS / BOARD PENDING |
| `src/modules/rc_update/rc_update.h`、`rc_update.cpp` | `Dima/modules/rc/RCUpdate.*` | 保留 18 通道校准、主控制与 Arm/Kill 功能映射、开关离散化、失联与 `parameter_update` 语义；退役无消费者的 Flaps/Aux 参数及后端 uORB 字段；协议锁定后再要求连续健康 100 ms 才解除控制 lost，frame-lost 只计数；差速 Rover 默认 Throttle/Yaw=通道 1/2，Roll/Pitch 默认未映射且不进入车辆输出；`COM_RC_IN_MODE` 非 0 时 fail-closed；裁剪 `PARAM_MAP_RC` 任意参数调节及非 Rover 功能 | ADAPTED |
| `src/modules/manual_control/ManualControl.hpp`、`ManualControl.cpp`；ArduPilot RC switch debounce 行为 | `Dima/modules/rc/RcManualInput.*` | 保留 RC setpoint 和二段开关边沿 Action Request；Arm/Kill 必须至少两份严格前进的一致样本并稳定 200 ms，启动、RC 恢复及映射/阈值变化后首个稳定状态只建立无动作基线；本地名称明确其只拥有 RC 来源转换，ArduPilot 仅作行为参考 | ADAPTED |
| `src/modules/commander/Commander.hpp`、`Commander.cpp`、`ModeUtil/control_mode.*`；ArduPilot Rover 左右电机 pre-arm 行为 | `Dima/modules/safety/Commander.*` | 保留 Action Request、Arming/Kill/Termination、QGC RC calibration、状态发布顺序和 Manual/Termination 语义；通过 `actuator_output_status` 要求新鲜/严格前进、后端 ready、至少一右一左映射及 `DISARMED_NEUTRAL`，Armed 输出 Fault/Stale 强制 Disarm；允许新鲜完整 hard-off 先清执行器 cause，避免恢复自锁；Kill 固定为 Disarm，Unkill 不自动 Arm；ArduPilot 仅作行为参考 | ADAPTED |
| `msg/InputRc.msg`、`RcChannels.msg`、`ManualControlSetpoint.msg`、`ManualControlSwitches.msg`、`ActionRequest.msg` | `Dima/messages/schemas/` | 保留 PX4 字段、枚举和 Topic 契约；头、metadata、Topic 定义与 catalog 由工具直接生成，队列深度来自 schema 的原生 `ORB_QUEUE_LENGTH` | ADAPTED |
| `msg/versioned/VehicleStatus.msg`、`VehicleControlMode.msg`、`ActuatorArmed.msg` | `Dima/messages/schemas/` | 完整保留三个公开消息的字段、枚举和版本号；本地三个 Topic 均为单深度，派生 C++ 合同不进入源码树 | ADAPTED |
| `msg/versioned/ActuatorMotors.msg` | `Dima/messages/schemas/ActuatorMotors.msg` | 完整保留 version 0、12 路 control、reversible flags 和采样时间；Topic 单深度，阶段 5 仅使用前两路 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| `src/modules/ekf2/EKF2.*`、`src/modules/ekf2/EKF/` 与所需 `src/lib/{matrix,mathlib,geo,lat_lon_alt,world_magnetic_model}` | `Dima/modules/ekf2/`、`Dima/lib/{ekf2,matrix,mathlib,geo,lat_lon_alt,world_magnetic_model}/` | 固定 PX4 v1.17.0 commit 的 N1 单实例物理闭包；Wrapper 只适配 Dima 生命周期、参数和 uORB，Core 保留上游版权头。排除 Selector、多实例、Wind/Airspeed/Baro/Flow/Range/Terrain fusion/EV/Sideslip/Drag/Aux/Wheel | ADAPTED / SINGLE INSTANCE / WINDOWS BUILD VERIFIED |
| `src/modules/ekf2/EKF/python/ekf_derivation/` | `Dima/lib/ekf2/EKF/python/ekf_derivation/` | 只保留一份 `derivation.py` 和一份 generated 目录；脚本直接不定义 Wind/Terrain 状态且不注册 Wind 公式函数，13 个公式头只由该脚本生成，不手改、不维护其他状态闭包产物 | ADAPTED SYMFORCE GENERATION / WIND AND TERRAIN REMOVED |
| EKF2 `module.yaml`/`params_*.yaml` 与 Estimator/vehicle `.msg` | `Dima/middleware/parameters/definitions/module_ekf2*.yaml`、`module_sensor_params.yaml`、`Dima/messages/schemas/` | 参数名称、类型、默认值、reboot_required、Topic alias/ID/hash/registry 均由既有正式 Parameter/uORB 工具派生；没有手写派生目录、消息列表或 `EKF2_EN` | AUTHORITATIVE YAML/MSG / GENERATED CONTRACT |
| PX4 VehicleIMU in-flight calibration 与 EKF2 sensor bias | `Dima/modules/sensors/imu/VehicleImu.*`、`Dima/modules/ekf2/Ekf2Bias.cpp` | 单实例稳定 bias 按 PX4 方差/连续时间条件发布；VehicleImu 以 `R^T*bias` 合并传感器 offset。realtime sensors/estimator 只发布固定请求快照，`wq:lp_default` 分别提交完整 IMU 参数组和 `EKF2_MAG_DECL`，失败限速且物理持久化沿用 autosave | ADAPTED / NON-REALTIME PARAM COMMIT / NO DIRECT FLASH |
| Dima Rover 两轴请求与输出状态 | `Dima/messages/schemas/RoverMotionRequest.msg`、`ActuatorOutputStatus.msg` | 两个 Topic 深度均为 8；前者隔离 Manual/未来 Navigation 与混控，后者显式暴露 configured/right/left mask 以及 `HARD_SAFE_OFF / DISARMED_NEUTRAL / ACTIVE / RETRY / FAULT` 六路后端状态 | DIMA CONTRACT / TARGET VERIFY PASS / BOARD PENDING |
| `src/modules/rover_differential/RoverDifferential.cpp`、`DifferentialDriveModes/DifferentialManualMode/`、`DifferentialActControl/` | `Dima/rover/modes/ManualMode.*`、`Dima/rover/control/RoverDifferential.*`、`Dima/lib/rover/DifferentialDrive.*` | 保留 Manual 两轴、100 Hz rate-control WorkQueue 和 `actuator_motors` 边界；反向不对称在混控前将两侧可行域限制为 `[-1/MOT_THR_ASYM, 1]`，先保留转向/油门优先级再作反向补偿，避免倒车两侧同时饱和丢失转向；其余适配保持固定存储、Commander 三 Topic 门控及 ARMED 参数延后 | ADAPTED / TARGET VERIFY PASS / BOARD PENDING |
| PX4 actuator function/output 行为边界；ArduPilot `MOT_SAFE_DISARM=0` 中立行为参考 | `Dima/modules/motor/MotorOutput.*`、`Dima/platform/api/ActuatorPwmLimits.h`、`ActuatorPwm.hpp`、`Dima/platform/stm32h7/pwm/ActuatorPwm.cpp`、`Boards/H743/Src/motor_pwm.c` | C/C++ 共用普通 PWM 产品包络固定为 500～2500 us，板级后端同时执行防御性校验；本地收敛为独立生命周期的 MotorRight/MotorLeft 到 S1～S6 固定存储映射。健康普通 Disarmed 仅向已配置通道输出 `CENT`，Kill/Termination/Failsafe/故障进入物理 hard-off；分离 ACTIVE inhibit 与 hard-safe inhibit，任一负向安全 Topic 先到即可 fail-closed；不导入通用 Mixer/FunctionMotors/MixingOutput，APM GPL 代码未复制 | DIMA ADAPTATION / TARGET VERIFY PASS / BOARD PENDING |
| PX4 `src/lib/rover_control`、`src/lib/pid`、`src/lib/slew_rate` | 尚未创建；阶段 8 按届时状态估计和运行接口重新导入 | 不保留无消费者的预实现控制器；导入时仍须保持 Speed、Yaw Rate、Heading、Stop 控制核的纯算法边界 | PLANNED / NO SOURCE PLACEHOLDER |
| ArduPilot `libraries/AR_Motors/AP_MotorsUGV.cpp`、Rover pre-arm 与 watchdog 行为 | `Dima/lib/rover/DifferentialDrive.*`、`Dima/modules/safety/Commander.*`、IWDG 健康合同 | 仅参考倒车车头方向、饱和优先级、slew、静摩擦补偿、反向不对称、左右映射 pre-arm、换向延时和主循环停滞复位行为；未复制 GPL 源码 | BEHAVIOR REFERENCE ONLY |
| 旧 Dima `speed_to_pwm` 固定六路转换 | 已移除 | 自建仓以来没有生产调用，历史唯一消费者为已删除的 Host Test；当前六路参数化转换与 safe-off 所有权统一由 `Dima/modules/motor/MotorOutput.*` 承担 | RETIRED / SUPERSEDED |
| Stage 5 Rover 控制与油门保护参数 | `Dima/middleware/parameters/definitions/module_rover_actuator_params.yaml` | 请求超时、倒车转向、混控优先级、最小/最大输出、slew、换向延时、expo、反向不对称及解锁 ramp 只由 PX4 YAML 定义；运行期仅在新鲜 DISARMED 快照后整体应用 | DIMA PARAMETER YAML / TARGET VERIFY PASS |
| 六路 PWM 映射参数 | `Dima/middleware/parameters/definitions/module_rover_actuator_params.yaml` | S1～S6 的 `FUNC/MIN/CENT/MAX/REV` 名称、范围和默认值由同一 YAML→官方 XML/JSON/Header 链产生；完整 DISARMED 快照中逐通道校验，只有至少一右一左时允许 ACTIVE | DIMA PARAMETER YAML / TARGET VERIFY PASS / BOARD PENDING |
| PX4 RC/Commander/QGC、传感器、GPS、串口与 DroneCAN 参数及 Metadata | `Dima/middleware/parameters/definitions/module_*.yaml` | 所有产品参数直接由受版本控制的 YAML 统一汇入锁定上游正式链；固件目录、Metadata 与协议索引只读生成物，不存在串口/DroneCAN JSON 或专用参数生成器。五个已删除的可选校准参数不通过别名/虚拟参数恢复，当前缺失状态保持用户确认的可校准基线 | ADAPTED / SINGLE YAML SOURCE / BOARD PENDING |
| Dima Rover 生命周期、BootHealth 与 Parameter Autosave 写门控 | `Dima/rover/ApplicationContext.*`、`Dima/modules/boot_health/BootHealthService.*`、`Dima/modules/parameters/ParameterService.*`、`Dima/middleware/maintenance/`、`Dima/middleware/uORB/`、`Dima/middleware/work_queue/`、`Dima/platform/api/{Services,Execution,Flash,Memory,Synchronization,TaskRuntime}.*` | ApplicationContext 只注入 capability；BootHealth 依据安全/输出 Topic 进展推进 generation，不跨队列读取普通模块状态。维护票据还要求 Disarmed、neutral/hard-safe、`appMain` reload 确认、单调存储进度和长期 Arm interlock；shutdown 在释放资源前确认六路物理 hard-off | DIMA INTEGRATION / SOURCE AND TARGET GATE PASS / BOARD PENDING |
| STM32H7 IWDG 与 MCUboot 跨复位衔接 | `Dima/platform/api/Boot.hpp`、`Dima/platform/stm32h7/system/Watchdog.cpp`、`Dima/application/app_main.cpp`、`Bootloader/Inc/boot_watchdog.h`、`Bootloader/Src/boot_watchdog.c`、MCUboot feed hook | 应用约 2048 ms、100 ms 检查，appMain 为唯一应用 feed owner；冷启动按 STM32 HAL 顺序先 start IWDG/LSI、再写配置并等待 SR 同步，跨复位已运行时允许重复 start key；MCUboot 对已运行 watchdog 临时扩展到约 32 s但不主动启动 inactive IWDG，Recovery/校验/swap/Flash/USB 长循环统一 feed；DBG halt 冻结，复位原始原因跨应用桥接保留 | DIMA SAFETY / TARGET VERIFY PASS / BOARD PENDING |
| MCUboot image confirmation 与 Recovery | `Dima/platform/stm32h7/system/BootControl.cpp`、`flash/flash_bank1.c` | 非阻塞 transaction 保留 DEFERRED；Bank 1 program 在 DTCM 执行并统一调用 cache helper | DIMA BACKEND |
| `src/modules/mavlink/mavlink_main.cpp`、`streams/RC_CHANNELS.hpp`、传感器与 Estimator streams | `Dima/modules/mavlink/MavlinkService.hpp/cpp`、`MavlinkChannelState.cpp`、`MavlinkSystemMessages.cpp`、`MavlinkRcStream.cpp`、`MavlinkSensorStreams.cpp` | Service 主文件持有 RX/TX 主循环、USB 传输独占、ACK/日志和延迟 reboot；Sensor 文件发送 50 Hz `HIGHRES_IMU`、25 Hz `SCALED_IMU`、5 Hz 原始 `GPS_RAW_INT`、1 Hz `SYS_STATUS`，并按 PX4 字段映射发送 50 Hz `ATTITUDE`、30 Hz `LOCAL_POSITION_NED`、10 Hz `GLOBAL_POSITION_INT`、5 Hz `ESTIMATOR_STATUS`。原始 GPS 可见性与 EKF checks health 分离；周期流支持单次请求和 `SET/GET_MESSAGE_INTERVAL`。TX/Metadata/USB session 边界保持原实现 | ADAPTED / N1 NAV STREAMS / WINDOWS BUILD VERIFIED / BOARD PENDING |
| `src/modules/mavlink/mavlink_receiver.cpp`（命令处理子集） | `Dima/modules/mavlink/MavlinkCommands.hpp/.cpp` | COMMAND_LONG/COMMAND_INT 接收、target 过滤、source system/component 保留、`vehicle_command` 发布及 `MAV_CMD_REQUEST_MESSAGE`；Commander ACK 定向回命令 source；保留 PX4 `MAV_CMD_SET_MESSAGE_INTERVAL`、`MAV_CMD_GET_MESSAGE_INTERVAL` 和 `MESSAGE_INTERVAL` 回复，裁剪 Autotune | ADAPTED |
| `src/modules/mavlink/mavlink_mission.cpp`（空任务语义） | `Dima/modules/mavlink/MavlinkMission.hpp/.cpp` | `MISSION_REQUEST_LIST` 返回 count 0，`MISSION_CLEAR_ALL` 返回 ACCEPTED；不建立任务存储或上传链路 | ADAPTED / EMPTY MISSION |
| `src/modules/mavlink/mavlink_parameters.cpp` | `Dima/modules/mavlink/MavlinkParameters.hpp/.cpp`、`MavlinkParameterExt.cpp` | Classic/Ext 直接遍历官方完整 handle 目录并发送统一 count/index；SERIAL Function 写入只允许 Disabled/SBUS/GPS，选择新 owner 时在单个参数事务内禁用旧 owner并回传受影响参数。公开分组来自同一官方生成源，不维护第二份参数名单 | ADAPTED |
| `src/modules/mavlink/mavlink_timesync.cpp` | `Dima/modules/mavlink/MavlinkTimesync.hpp/.cpp`、`Dima/lib/timesync/Timesync.hpp/.cpp` | TIMESYNC 处理——远端回传、本地喂入收敛滤波器；省略 SYSTEM_TIME 时钟设置 | ADAPTED |
| PX4 `src/modules/mavlink/mavlink_ftp.*` 与 Component Metadata | `Dima/modules/mavlink/MavlinkMetadataFtp.hpp/.cpp`、`tools/mavlink/generate_parameter_metadata.py`、`build/generated/component_metadata/` | 397 现代发现与 395 deprecated 回退共用 URI/CRC；General 声明 type 1 Parameter 与 type 5 Actuator Metadata。FTP 只允许 General/Parameter/Actuator 三个 Flash 虚拟文件和 Open/Burst/Read/Reset/Terminate，不含目录、写入或 Event Metadata；Actuator Metadata 开放六路 PWM 分配和参数编辑，但 MotorRight/MotorLeft 排除执行器测试，固件不实现 `MAV_CMD_ACTUATOR_TEST` | ADAPTED / READ-ONLY / TARGET VERIFY PASS / BOARD PENDING |
| `src/lib/mavlink/`（mavlink commit `33af200d`，pymavlink submodule `fcaa2c7d`） | `tools/mavlink/message_definitions/dima.xml`、`mavlink_runtime.yaml`、`build/generated/mavlink/` | `dima.xml` 只 include 固定 common，Make 直接执行原始 mavgen 生成当前 230-message wire 闭包；独立 YAML 生成实际 14 outbound/11 inbound 路由合同并引用 `MAVLINK_MSG_ID_*`，不参与 ID/CRC/payload/codec | UNMODIFIED MAVGEN / NATIVE DIALECT / REPRODUCIBLE PIPELINE |
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
| `Middlewares/Third_Party/FatFs/src/diskio.c` | `Boards/H743/Src/fatfs_diskio.c` | 重写为直接调用 HAL_SD（`hsd1`），不使用 `ff_gen_drv` 抽象层；增加 32 字节对齐 scratch buffer 位于 `.dima_dma` 区；块读写和 ready 等待统一使用 500 ms 产品上限 | REWRITTEN / BOARD OWNED |
| `Middlewares/Third_Party/FatFs/src/diskio.h` | 同路径 | 重写，移除 `ff_gen_drv` 依赖 | REWRITTEN |

2026-08-19 使用 Windows 原生 GNU Make 4.4.1 与项目缓存的 Arm GCC 10.3.1 执行 `make clean` 后 `make -j4 NO_COLOR=1 dima_rover`，完整通过 `[212/212]`。Application 为 `text=233772/data=12284/bss=356176`、未签名 BIN `246096` bytes，MCUboot 为 `text=47712/data=380/bss=10192`、BIN `48100` bytes，向量 `0x08040400`。Application/MCUboot 未解析符号均为空，ELF 已确认 SBUS/Commander/MotorOutput/IWDG 健康链及 MCUboot watchdog prepare/feed 符号实际链接。同一 image digest `601c65353ffebce20cea8f040d975000e3c4caa2b27aaa15dd409529740e26ce` 的两次有效 ECDSA P-256 重签分别生成 `247271/247270`-byte Signed BIN 与 `710861/710859`-byte Factory HEX；imgtool 的 DER 签名长度可变，因此这两个容器的文件长度和文件 SHA-256 不属于确定性构建合同。SBUS 电气、PWM 波形、电调握手、IWDG 实际时限和车辆行为仍为 `BOARD PENDING`。

同日实板 ROM DFU 首启排查又修复 D3 bridge 冷启动循环依赖和 IWDG start-before-sync 顺序。故障记录为 v2 sequence 1：`ERROR_HANDLER`、`APPLICATION_RUNNING`、`stacked_r0=2048`，且 `CFSR/HFSR/ABFSR=0`。修复后 Windows 原生 clean build 通过 `[212/212]`：Application 仍为 `233772/12284/356176` bytes、BIN `246096` bytes，MCUboot 为 `47864/380/10192` bytes、BIN `48252` bytes，image digest `835947050b2df9062be4289bcb5b876abe0dee99b46792051827e79f123eeec3`，Signed BIN `247271` bytes、Factory HEX `711217` bytes；烧写后实板稳定枚举为 Application，并由只读 preflight 返回 `Dima Rover MAVLink version=0x00010000 board=1`。Signed/Factory 长度仍遵循上述 DER 非确定性边界。

同日完成目录与源码职责整理后，再次使用 Windows 原生 GNU Make 4.4.1 与项目缓存的 Arm GCC 10.3.1 执行 `make NO_COLOR=1 clean` 和 `make -j4 NO_COLOR=1 dima_rover`，完整通过 `[230/230]`，架构检查覆盖 261 个首方源码文件。Application 为 `text=220092/data=12284/bss=356056`、未签名 BIN `232416` bytes，ELF/BIN SHA-256 分别为 `3305882171aa18ec61286d08f9acd9b406e0d043809ac77450e2a67649a97a20` / `92ce5c84c6a6b12ad61d7c0a80ca83497c4f4fe1fea9e3b5284099ddf76392fb`；本次签名样本的 image digest 为 `c661dc5566ae50d13ca549d526934b0400684963daace1b2ca525f4ab0fefad2`，Signed BIN 为 `233591` bytes（SHA-256 `7f18d0ec86a9d549788b53f615d3ef98d73d3304732b75a3e150f6a63f3b6ca2`），Factory HEX 为 `678293` bytes（SHA-256 `c249358338195ac304829ca3d5d9a54db70a442508009fba3c7f473246a8b6bb`）。MCUboot 为 `text=47864/data=380/bss=10192`、BIN `48252` bytes，ELF/BIN SHA-256 分别为 `7a89d92a443ee581f7e26a32c673b944b1a481ebdc1d7653b183c2c33b34b1a6` / `379562b2bf473f23b73c5924f1491235f0df4ab9bdd894a50a69485f4ce51499`。应用向量仍为 `0x08040400`，19 项 init-array 白名单、两份未解析符号检查、Signed/Factory 布局、watchdog prepare/feed、安全 PWM、SBUS 与 Commander 链均通过；Signed/Factory 的长度和文件哈希仍只是本次 DER 签名样本，不作为确定性合同。

2026-08-21 当前生成合同为 203 项固件参数和 203 项公开 Parameter Metadata，按当前参数名排序，不保留旧 handle、stable-tail、旧键或内部迁移版本。General/Parameter/Actuator XZ 分别为 `192/4904/420` bytes；最新正式生成 stamp 的 General/Parameter/Actuator CRC32 为 `0xc49e389e/0x6f14d0f7/0xaf58d846`，General Metadata 类型严格为 `1,5`。派生文件的 size/SHA-256 不再手工复制到本文；唯一可机读事实源是 `build/generated/component_metadata/.generated.json`，生成器从同一组内存输出写入全部七个派生文件及其 size/SHA-256 stamp，正式架构目标会重新校验输入 SHA、七份 size/SHA、XZ round-trip 与三项 CRC。Actuator Metadata 是经本地结构门禁验证的 Dima output-only 适配：保留六路 `outputs_v1`，故意使用空 `mixer_v1.config`，且禁止 MotorRight/MotorLeft actuator testing。当前 QGC Actuators 页面可用由用户实测确认；实际输出、SD 拔插、IWDG 时限和车辆行为仍分别保留板级验收边界。

随后将 `Dima/platform/stm32h7` 的扁平实现整理为 `system/memory/flash/serial/io` 五个职责目录，根部只保留硬件工厂声明和总 README；原先无人包含的 `flash_fault.h` 下沉到 `flash/`，并改为 Core 弱钩子与 C++ 强实现共同包含的唯一 C ABI 声明。Windows 原生 clean `make -j4 NO_COLOR=1 dima_rover` 再次完整通过 `[230/230]`，架构检查覆盖 261 个首方源码文件；Application/MCUboot 的 text/data/bss、向量、19 项 init array、未解析符号和安全链结果均不变。与搬移前封存产物相比，两份 loadable BIN、ELF program headers 和已定义符号表逐项一致：Application BIN SHA-256 仍为 `92ce5c84c6a6b12ad61d7c0a80ca83497c4f4fe1fea9e3b5284099ddf76392fb`，MCUboot BIN 仍为 `379562b2bf473f23b73c5924f1491235f0df4ab9bdd894a50a69485f4ce51499`，image digest 仍为 `c661dc5566ae50d13ca549d526934b0400684963daace1b2ca525f4ab0fefad2`。ELF 整体哈希仅因 DWARF 源路径变化更新为 Application `c8eddd965266834765c90c286fe2b0ccdda03caf6ae7e993f2275ad3f43660de`、MCUboot `9e2d372ab51e58a1a50b47034a9d16d91da50fc6fcb287a50088777c035185e6`；串口、参数与 Component Metadata 三棵生成树仍逐字节一致。Windows `make intellisense` 也已生成 191 个源码的 211 条新路径编译命令。

2026-08-24 完成 IMU、DroneCAN 磁力计、UM982 GPS、PX4/QGC 校准和实时 MAVLink 可观测链后，Windows 原生 clean `make -j4 NO_COLOR=1 dima_rover` 完整通过 `[265/265]`：架构门禁覆盖 324 个首方源码文件，确定性生成 227 项固件参数/公开 Parameter Metadata 和 27 条 MAVLink 消息，Component Metadata XZ 为 General/Parameter/Actuator `196/5928/420` bytes。Application 为 `text=252900/data=2844/bss=399552`、总计 655296 bytes，Flash 占用 `252700/785408` bytes，DTCM 占用 `84356/131072` bytes；本次签名样本 image digest 为 `2fcdcc1df7bb4beca9723bd0d715a39c66f60eab9b7ee144ce78f9beae39bc84`，Signed BIN 256960 bytes，Factory HEX 734396 bytes。应用向量仍为 `0x08040400`，Application/MCUboot 未解析符号均为空，ELF 已确认 `SensorCalibration`、`send_highres_imu`、`send_gps_raw_int` 与 `send_system_status` 实际链接；固定内存校准算法测试和 UM982 `NO_FIX` 解析回归测试通过。QGC 实际校准向导、三条实时消息的板端值、ICM42688P WHOAMI/SPI/DMA/中断、DroneCAN RM3100 接线/方向、GPS UART/RTK fix/双天线 yaw、参数重启持久化、校准残差和车辆行为仍为 `BOARD PENDING`。

2026-08-24 本轮修复 STM32H7 SPI45 时钟查询、ICM42688P 探测诊断、PX4 参数分组、阶段可实现的 IMU/磁力计运行参数，并补齐 GPS/DroneCAN 原子 Disarmed/Arm 重配置互锁后，使用 Windows 原生 GNU Make 4.4.1 与项目缓存 Arm GCC 10.3.1 执行 `make NO_COLOR=1 clean` 和 `make -j4 NO_COLOR=1 dima_rover`，fresh build 完整通过 `[265/265]`。架构门禁覆盖 325 个首方源码文件；确定性生成 230 项固件参数/公开 Parameter Metadata、27 条 MAVLink 消息，General/Parameter/Actuator XZ 为 `192/6392/420` bytes。Application 为 `text=256172/data=2860/bss=399808`、总计 `658840` bytes，未签名 BIN `259072` bytes（SHA-256 `ec77ef1add2cfaf5fbe8ce1f77fe563071e584a326b1c73e0ccb6d9bc6179ec0`）；MCUboot 为 `text=47808/data=380/bss=10192`、BIN `48196` bytes（SHA-256 `9fc674c68c539ad34d143290832863229683749cbc0cf83cc8d5dd497bce39ea`）。本次签名样本 image digest 为 `0546e3304b826f21993a28e9e2bb5aa49227f4e7bc549ec4b7968ba327f1f60e`，Signed BIN `260247` bytes（SHA-256 `09775a77d72c8de9133322ce568cfb2fef641ea8d714771ce2e85806b01e8281`），Factory HEX `742309` bytes（SHA-256 `9242ebab62fe93e147ac168776b13a5eb7670bcfeb43ba6584648044146531cd`）。应用向量为 `0x08040400`，init-array 为 4 个许可项，两份 `nm -u` 均为 0 条；ELF 已确认 `ICM42688P::probe`、`HAL_SPI_TxRxCpltCallback`、`DroneCanMag2::handle_magnetic_field`、`Um982Gps::publish_if_ready`、`SensorCalibration`、`send_highres_imu` 和 `send_gps_raw_int` 实际链接。SPI/IMU/磁力计运行算法、固定内存校准算法及 UM982 协议三组 Windows 主机回归均以 `-Werror` 通过。该段仅证明静态与构建结果；新镜像上的 QGC 分组/校准、ICM42688P WHOAMI/SPI/DMA/中断、RM3100 CAN 收发/方向、UM982 UART/RTK/yaw、实时 MAVLink 数值、参数持久化和车辆行为仍为 `BOARD PENDING`。

本次验收未执行 commit、push、upload 或烧录。

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

## 9. 2026-08-25 PX4/QGC IMU、磁力计与校准链验收

本节是原生生成链重构前的历史构建记录，其中消息数量和旧参数发现策略不代表当前架构合同；当前合同以 7.0 节和本文末尾最新验收为准。

本轮以 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 和 QGroundControl commit `4a2c0358115a16bafe290af259c29e5b6cb4e26c` 为固定对照，复核并收口 QGC Gyro/Accelerometer/Compass 校准、`HIGHRES_IMU`、`SCALED_IMU`、原始 `sensor_mag` 与参数发现链。PX4 USB 单实例合同保持为：50 Hz `HIGHRES_IMU` 从 `vehicle_imu + vehicle_magnetometer` 发送 SI/Gauss 数据，25 Hz `SCALED_IMU` 从 `vehicle_imu + vehicle_imu_status + sensor_mag` 发送 mG/mrad/s/milliGauss/cdegC 数据；两条流都由新 Topic 驱动，磁场 freshness 只进入 `SYS_STATUS` health，不清零或阻断最近合法值。固定 PX4 流表没有注册 `RAW_IMU`，产品也只有实例 0，因此不生成 `RAW_IMU` 或伪造 `SCALED_IMU2/3`。Topic 复制完成后统一重新采样 health 时钟，避免高优先级 IMU、磁力计或 GPS 发布恰好发生在复制期间时被短暂误判为未来时间戳。

QGC 校准命令固定为 gyro `param1=1`、mag `param2=1`、accel `param5=1`、取消 `param1..7=0`；长事务使用 PX4 v2 `[cal] calibration started/progress/orientation/side done/done/failed/cancelled` STATUSTEXT。`SensorCalibration` 运行于非实时 `wq:lp_default`，全部 `[cal]` 协议文本走 RAW 日志路径，拒绝启动时也发送 terminal failure token，避免 QGC 等待悬挂。参数流在冻结 LIST 快照前标记 `CAL_*`、`SENS_*` 和固定 QGC 参数；生成结果包含 QGC Sensors 页面无条件读取的十个 Fact 以及 Airframe prerequisite `SYS_AUTOSTART=50000`。磁力计 diagonal scale 的 Metadata 与前端范围统一为 PX4 的 `0.1..3.0`。

Windows 原生 `E:\freertos\H743_FreeRTOS` clean acceptance 依次通过前置架构门禁、`make NO_COLOR=1 clean`、`make -j4 NO_COLOR=1 dima_rover`、构建后架构门禁和 `make NO_COLOR=1 parameter-metadata-verify`：`[258/258]`，架构检查 `PASS (296 first-party source files)`，生成 233 个无重复参数、27 个 uORB 合同和 29 条 MAVLink 消息（Dima 25、standard 2、minimal 2）。Application `text/data/bss=278428/12316/395520`，总计 `686264` bytes；Flash `278228/785408`（35.4%）、DTCM `89796/131072`（68.5%）、SRAM `318240/884736`（36.0%）。未签名 BIN `290784` bytes，SHA-256 `b9c1b8f1cd14bba9538866d87367bd813a66dbefea231a6c362fa947a13e437f`；本次签名样本 image digest `12ab982a6b66cb38b527f8963f9a54e3eef5a020aa510198aa1fe16ba4315fd5`，Signed BIN `291959` bytes，SHA-256 `2a480ddcd163cd535be18c48321fdf26f4a04bf2d5e7e480a01b19b58c40850d`；Factory HEX `818633` bytes，SHA-256 `561663b3bc4cbf2dcf3a66b1471b88df1f202d9f396372167c8f76398b192347`；MCUboot BIN `48196` bytes，SHA-256 `9fc674c68c539ad34d143290832863229683749cbc0cf83cc8d5dd497bce39ea`。Application/MCUboot `nm -u` 均为 0，ELF 确认 `SensorCalibration`、`VehicleImu::Run`、`VehicleMagnetometer::Run`、`DroneCanMag2::handle_magnetic_field`、`send_highres_imu` 和 `send_scaled_imu` 已链接；`git diff --check` 为 0，构建结束无残留进程。

本轮没有新增、修改或扩展任何测试框架、测试文件、runner、harness、fixture、mock 或 test-only API，也没有执行 commit、push、upload 或烧录。结论边界为 `SOURCE/STATIC/WINDOWS BUILD VERIFIED`；QGC 三个校准按钮与六面 UI 实际推进、板端 `[cal]` STATUSTEXT、`HIGHRES_IMU/SCALED_IMU/SYS_STATUS` 频率和值、ICM42688P SPI/DMA/IRQ、DroneCAN RM3100 方向与采样、校准拟合残差、参数断电持久化、USB 长连接与重连仍为 `BOARD/QGC PENDING`。

## 10. 2026-08-26 D2 参数布局与 RC 参数裁剪验收

本节同样保留为重构前历史证据；旧的裁剪消息数量和生成方式已经退役。

Windows 原生 `E:\freertos\H743_FreeRTOS` 依次执行 `make NO_COLOR=1 clean` 和 `make -j4 NO_COLOR=1 dima_rover`，clean build 完整通过 `[264/264]`；共享工作树随后出现并发 UM982/维护协调器输入变化，又执行 `make -j4 NO_COLOR=1 verify` 并通过 `[15/15]`，架构检查保持 `PASS (296 first-party source files)`。生成链实际报告 221 个参数、28 个 uORB 合同和 29 条 MAVLink 消息；参数总数不再作为需要人工同步的固定门禁，解析完整性、名称唯一性、固件/Metadata 顺序一致性及 RC 连续通道合同仍由工具验证。RC1～RC18 共 90 个校准条目完整保留，所有可变 RC 映射范围均为 `0..18`；Flaps/Aux 参数及其后端字段已退役，Arm/Kill 安全链保留。

最终 `verify` 样本的 Application `text/data/bss=285904/12316/400736`，总计 `698956` bytes；Flash `285704/785408`（36.4%）、DTCM `1248/131072`（1.0%）、SRAM `412004/884736`（46.6%），其中 D1 SRAM `311296/524288`（59.4%）、D2 data `93764/262144`（35.8%）、DMA `6720/32768`（20.5%）、D3 diag `224/65536`（0.3%）。Signed BIN 为 `299436` bytes，Factory HEX 为 `836722` bytes，向量仍为 `0x08040400`；Application/MCUboot 未解析符号均为空。五个无后端的可选校准参数在该版本中保持删除；用户随后确认当前缺少它们仍可正常校准，因此“缺少 Fact 会禁用校准页面”不再作为当前结论。该历史构建没有执行新的 QGC/实板回归，因此本段不对 QGC 参数缓存刷新、实板参数迁移或 RC7～RC18 实际接收作验证结论。

## 11. 2026-08-27 PX4 原生生成链重构验收

Windows 原生 `E:\freertos\H743_FreeRTOS` 从 `make NO_COLOR=1 clean` 开始，依次完成 Parameter、uORB、MAVLink 独立生成与确定性复验、`make -j4 NO_COLOR=1 dima_rover`、`verify`、`parameter-metadata-verify` 和 `check-architecture`。Parameter 两次生成的 8 个正式文件逐字节一致，聚合 SHA-256 为 `2010308f1861953ea31ac799e94e609a2d9fdf753ba46408f4d533a1b055f416`；uORB 与 MAVLink 的 verify 入口均在临时目录重新执行原始上游工具后逐文件比较通过。

本轮生成 221 个参数、27 个产品 uORB schema，其中 23 个 PX4 同名 schema 与固定 v1.17 快照逐字节一致；`dima.xml + common.xml` 由原始 mavgen 生成 230 条 wire 消息，运行策略派生 10 个 outbound 与 11 个 inbound 路由。Component Metadata 生成输出为 General/Parameter/Actuator `196/6448/420` bytes。静态闭包确认五个已删除参数在权威 YAML、参数生成头、Metadata 和协议目录中的命中数为 0，源码树无 `PARAM_DEFINE_*`、本地参数/uORB parser/renderer、手写 MAVLink ID/CRC/codec/registry；`tests/` tracked 与 untracked 变化均为 0。

完整构建通过 `[290/290]` 和 294 文件架构门禁。Application `text/data/bss=291920/12316/402688`，总计 `706924` bytes；Flash `291720/785408`（37.1%）、DTCM `1248/131072`（1.0%）、SRAM `413956/884736`（46.8%）。未签名 BIN `304276` bytes；本次签名样本 image digest `d01789f7d1ef543f6c4f8c3eb92bdae93d09eb63f49de461b4b785c5ffc0603e`，Signed BIN `305450` bytes；Factory HEX `851194` bytes；MCUboot BIN `48236` bytes。应用向量为 `0x08040400`，ELF layout 通过，Application/MCUboot 显式 `nm -u` 均为 0，watchdog prepare/feed 链已链接，`git diff --check` 无错误。

本轮没有新增或修改测试，没有 commit、push、upload、烧录或 QGC 修改。结论边界为 `SOURCE/STATIC/WINDOWS BUILD VERIFIED`；QGC 5.1.3 参数下载、Sensors 页面、陀螺仪/加速度计/磁力计校准的启动/进度/六面/取消/完成、实际 `CAL_*0` 写入和重连持久化仍为 `BOARD/QGC PENDING`。五个已删除参数在未来板测中也必须始终不可见。

## 12. 2026-08-28 N1 单实例 PX4 EKF2 验收

N1 固定构造一个 EKF2，绑定 `vehicle_imu[0]`、`vehicle_magnetometer[0]` 与 `vehicle_gps_position[0]`，并只在 `wq:estimator` 运行。源码闭包启用 GNSS position/height/velocity/yaw、Mag Automatic、Gravity、bias、predictor 与 GSF yaw；没有 Selector、多实例、`EKF2_EN`、运行时装卸、Wheel source 或禁用观测源的发布者。UM982 的 `heading/heading_offset` 原样进入 PX4 Core，不重复扣除 `GPS_YAW_OFFSET`；`GLOBAL_POSITION_INT.relative_alt` 在当前没有 Home Topic 时采用 PX4 官方的 global altitude 回退。

参数、uORB 与 MAVLink 均从权威输入重新生成：280 个 YAML 参数、40 个 PX4 uORB 消息类型、230-message MAVLink dialect 与 14/11 条运行路由；Parameter Metadata 的 General/Parameter/Actuator 大小为 `196/10180/420` bytes。`EKF2_PREDICT_US` 与 `EKF2_DELAY_MAX` 共同决定启动期 RingBuffer 容量，均由权威 YAML 标记 `reboot_required` 并在运行期保持启动快照。realtime sensors/estimator 只发布固定参数请求快照；完整 IMU 校准组和 `EKF2_MAG_DECL` 分别由非实时 `wq:lp_default` 提交，失败限速且不直接访问 Flash。

唯一 `derivation.py` 直接不定义 Wind 状态，也不注册 Wind 公式函数；在独立临时目录用 PX4 对应 SymForce 0.9.0 重新生成的 13 个头与仓库唯一 `generated/` 目录逐文件一致。活动构建宏只有 GNSS、GNSS yaw、Magnetometer 和 Gravity fusion；IMU/GNSS/Mag/system flag 的 RingBuffer 全部在启动期建立，`wq:estimator` 首样本路径不会分配。EKF 源继续使用 `-fno-associative-math`，并继承项目统一的 `-fno-exceptions/-fno-rtti`。

Windows 原生 `E:\freertos\H743_FreeRTOS` 依次执行 `make NO_COLOR=1 clean`、`make -j4 NO_COLOR=1 dima_rover`、`make -j4 NO_COLOR=1 verify`、`make NO_COLOR=1 parameter-metadata-verify` 和 `make NO_COLOR=1 check-architecture`。fresh build 完整通过 `[337/337]`，独立 verify 通过 `[5/5]`，架构门禁为 `PASS - 396 first-party source files`。Application `text/data/bss=428112/12680/416256`，总计 `857048` bytes；Flash `427912/785408`（54.5%）、DTCM `1248/131072`（1.0%）、SRAM `427888/884736`（48.4%）。应用向量为 `0x08040400`，Application/MCUboot 未解析符号均为 0，MCUboot watchdog prepare/feed 链已链接。

本节不新增 SHA-256、文件 hash 或 image digest 依赖与文档基线；构建系统已有的来源核验、firmware identity 和 MCUboot 签名行为保持原样。本轮没有新增或修改测试文件、测试框架、runner、harness、fixture、mock、Host Test、SITL、仿真入口或测试专用 API，也没有执行 commit、push、upload 或烧录。

## 13. 2026-08-31 EKF2 状态与断言闭包收敛验收

唯一 `Dima/lib/ekf2/EKF/python/ekf_derivation/derivation.py` 直接移除未启用的 Wind 和 Terrain 状态，并由 PX4 对应 SymForce 0.9.0 正式再生成同一 `generated/` 目录的全部 13 个头；没有手改公式，也没有建立第二套公式路径。生成结果的 `StateSample` 为 22 个 float，`State::size=21`，协方差为 `21x21`，生成目录中不存在 Terrain/Wind 状态。EKF 源继续保留 `-fno-associative-math`，并继承统一的 `-fno-exceptions/-fno-rtti`。

Rover 永远向 GSF 声明 `is_fixed_wing=false`，不会使用固定翼向心加速度补偿，因此 `EKF2_GSF_TAS` 已从唯一权威 YAML、PX4 Core 参数结构和 Dima 参数加载器中删除。正式参数目标重新生成 279 个参数，源码与 `build/generated/parameters` 中该名称均为 0 命中；Parameter Metadata 的 General/Parameter/Actuator 大小为 `192/9940/420` bytes。参数名称、枚举、目录和 Metadata 仍全部由正式工具生成，没有手写派生列表。

Application 在 `Boards/H743/Src/boot_diagnostics.c` 提供强 `__assert_func`，保留断言并把文件地址、行号直接交给无格式化、无分配的启动诊断后端。该文件同时提供裸机 `abort()`，把 libgcc 不可恢复的展开失败记录为通用致命错误后复位，避免 newlib `abort -> raise -> signal` 引入不存在的 POSIX 进程系统调用。最终 ELF 保留自有 `__assert_func/abort`，且不存在 newlib `__assert_func`、`fiprintf/_vfiprintf_r`、`abort/raise/signal` 活动符号；map 中仅因宽字符 stdio 的归档解析列出 `lib_a-vfiprintf.o`，其全部段均位于 `Discarded input sections`，对可加载镜像贡献为 0 bytes。

在同一未提交 N1 源码上，修改前 Windows clean 基线为 Application `text/data/bss=427976/12680/416256`、总计 `856912` bytes、未签名 BIN `440696` bytes；本轮之后为 `419432/12316/416000`、总计 `847748` bytes、未签名 BIN `431788` bytes。对应 `text/Flash` 减少 `8544` bytes，`data` 减少 `364` bytes，`bss` 减少 `256` bytes，未签名 BIN 减少 `8908` bytes，总计减少 `9164` bytes。

Windows 原生 `E:\freertos\H743_FreeRTOS` 依次通过 `make NO_COLOR=1 clean`、`make -j4 NO_COLOR=1 dima_rover`、独立 `make -j4 NO_COLOR=1 verify`、`make NO_COLOR=1 parameter-metadata-verify` 和 `make NO_COLOR=1 check-architecture`。fresh build 完整通过 `[337/337]`，独立 verify 通过 `[5/5]`，架构门禁为 `PASS - 396 first-party source files`；Flash 为 `419232/785408`（53.4%），SRAM 为 `427268/884736`（48.3%），应用向量仍为 `0x08040400`，Application/MCUboot 未解析符号均为 0。

本节不新增 SHA-256、文件 hash 或 image digest 行为、依赖或文档基线；构建系统已有原生行为保持原样。本轮没有新增或修改测试文件、测试框架、runner、harness、fixture、mock、Host Test、SITL、仿真入口或测试专用 API，也没有执行 commit、push、upload 或烧录。
