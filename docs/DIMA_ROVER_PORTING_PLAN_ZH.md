# Dima FreeRTOS Rover 总体移植计划

- 状态：阶段 4 Commander Rover 子集已完成并通过目标编译、签名与镜像校验；阶段 1～4 目标板行为仍待人工验收
- 日期：2026-08-03
- 目标平台：STM32H743 + FreeRTOS
- 产品类型：支持前进、后退和原地旋转的差速 Rover

## 1. 项目目标

本项目不是从零设计一套差速车控制器，而是在现有 STM32H743、FreeRTOS、MCUboot、CubeMX/HAL 和六路 PWM 产品底座上，优先移植、复用和适配成熟飞控项目已经验证的模块、数据契约、控制逻辑与安全行为，形成完整且受控的差速 Rover 固件。

主要直接代码来源固定为 PX4 v1.17.0；ArduPilot Rover 用于补充 Arming、Failsafe、轮速、差速车导航和 PivotTurn 等行为参考。产品目录、构建目标和自有模块使用 Dima 命名，但不得篡改上游 API、namespace、宏、参数名、Topic 名、版权头或许可证文字。

Estimator 最终选择 PX4 EKF2。本计划取代此前所有采用 ArduPilot EKF3 的阶段性计划和讨论结论。

## 2. 固定工程原则

- 操作系统固定为 FreeRTOS，不转为 NuttX 板级移植。
- 保留当前 MCUboot 双镜像、USB 恢复、CubeMX/HAL、板级驱动和构建入口。
- 优先复用成熟上游模块；确实无法移植时才实现 Dima 适配或替代层，并记录原因。
- `Dima/` 是唯一自研应用根，负责启动壳、产品模块、适配、兼容运行时、平台层和算法库；不通过全文替换改变上游源码身份。
- `Core/`、`Boards/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `Bootloader/` 保持独立边界。
- 项目内不新建名称包含大小写不敏感 `px4` 的目录；来源说明、许可证文字和上游源码符号不受此限制。
- 不要求全系统完全静态确定。允许启动期和非实时服务受控动态分配。
- ISR、控制循环、EKF2 更新、Arming/Failsafe、Mixer 和 PWM 输出路径禁止动态分配。
- 新代码对硬件映射、时间单位、安全条件、状态转换和上游差异使用简洁中文注释。
- 默认不新增测试框架、测试文件、SITL 或仿真代码；阶段验收采用目标编译、链接检查、板上状态检查、台架和实车验证。
- 不覆盖、回退或删除工作区中与本阶段无关的现有修改。

## 3. 目标架构

```text
Dima Product Rover
  Mode / Arming Policy / Failsafe / Mission
                    ↓
成熟 Rover 模块
  RCUpdate / RcManualInput（ManualControl RC 子集）/ Commander 子集
  RoverDifferential / EKF2 / Position Control
  FunctionMotors / MixingOutput
                    ↓
Dima 公共系统层
  Parameter / px4::Param<T> / uORB / WorkQueue
  capability / events / perf / logging / allocator
                    ↓
独立平台后端
  FreeRTOS: Task / Mutex / Signal / Heap
  STM32H7: MPU / cache / DMA / Flash / UART / USB
                    ↓
FreeRTOS + STM32 HAL
                    ↓
MCUboot
```

当前目录边界：

```text
Dima/
├── application/                    启动壳、C ABI 入口和 appMainTask
├── adapters/                       只依赖 capability 的外部协议编解码/适配
├── platform/api/                   OS/MCU 无关公共契约
├── platform/freertos/              Task、同步、Heap 与 transaction 后端
├── platform/stm32h7/               system/memory/flash/serial/io 五组 MCU 后端
├── middleware/                     lifecycle、parameters、uORB、WorkQueue
├── modules/                        Parameter、Log、BootHealth、RC、MotorOutput、安全等已实现运行模块
├── lib/                            平台无关算法、容器和移植库
│   └── rover/                      当前生产链使用的 DifferentialDrive
├── messages/
└── rover/                          唯一 Rover 产品域；组合根、control、modes、navigation

Boards/H743/  Core/  Drivers/  Middlewares/  USB_DEVICE/  Bootloader/
```

已退役的顶层 `App/` 已完成目录归并。C/C++ Runtime 留在 FreeRTOS 后端，公共时间接口进入 `platform/api`，TIM2、Flash、DMA、USB 与中断实现进入 STM32H7 后端；`ApplicationContext` 只装配 capability。`Core/Boards/Drivers/Middlewares/USB_DEVICE/Bootloader` 保持独立；项目自有测试子系统已移除；尚未实现的 EKF2、PID、SlewRate 和阶段 8 控制器不建立源码占位目录。目标构建、签名镜像和目标板行为以本次验证及后续板测为准。

## 4. 阶段 0：保存计划并建立重构基线

### 4.1 文档和决策记录

阶段 0 建立并维护：

- 本总体移植计划。
- 上游源码与许可证清单。
- FreeRTOS 平台选择 ADR。
- 上游复用与 Dima 命名 ADR。
- EKF2 最终选择 ADR。

所有 ADR 日期统一为 2026-07-29。许可证最终策略暂不在阶段 0 决定，但来源保留和发布限制立即生效。

### 4.2 上游版本基线

- 正式 PX4 移植基线：v1.17.0。
- 当前本地 PX4 仅为预研快照：`release/1.16`，commit `75f9a32a12`；不得将其描述为已经验证的 v1.17.0。
- ArduPilot Rover 行为参考基线：commit `3f2e4763accb`。
- 正式导入任何文件时，必须在 Source Manifest 中补充原始路径、完整 commit、许可证和本地修改摘要。

### 4.3 资源基线

阶段 0 不调整当前 Flash 分区：

```text
MCUboot       256 KiB
Primary       768 KiB
Secondary     768 KiB
Scratch       128 KiB
Storage       128 KiB
```

使用项目正式目标构建入口记录：

- `.text`、`.rodata`、`.data`、`.bss`。
- ELF、BIN 和签名镜像大小。
- DTCM、D1、D2、D3 SRAM 使用量。
- 当前任务栈及高水位能力现状。
- Application Slot 剩余空间。

阶段 0 的历史资源数据记录在 [Dima Rover 资源基线](DIMA_RESOURCE_BASELINE_ZH.md)。目录归并后的 ELF、BIN、签名链和 Factory HEX 已于 2026-07-30 通过 Windows 本地 `make verify` 重新生成并验证；目标板运行行为仍需板测。

固件发布目标控制在 Application Slot 的 85% 以下。阶段 0 不缩小 Primary/Secondary Slot；参数掉电安全需要第二个擦除区域时，优先评估外部 NVM 或后续分区 ADR。

### 4.4 受控动态内存策略

阶段 1 已采用受监控的 FreeRTOS `heap_5`：

- 通用 Heap 首版位于 D1 AXI SRAM。
- D2 SRAM3 固定 32 KiB `.dima_dma` 由 MPU 配置为 non-cacheable，只接受 `DmaBufferView` 或平台 bounce buffer。
- 48 KiB 固定任务栈池位于 D1 且与 256 KiB Heap 分离；DTCM 不加入通用 Heap，并承载同 Bank Flash 编程例程。
- 启动期允许为模块对象、uORB Subscription、Parameter 稀疏值和 EKF2 缓冲分配内存。
- USB 命令、参数保存、日志和通信等非实时服务可受控分配。
- ISR、实时控制、Estimator 更新和执行器安全路径不得分配。
- 后续增加 malloc failed hook、最低剩余量、任务栈高水位和内存故障 Event。

## 5. 后续阶段

### 阶段 1：Dima FreeRTOS 平台兼容层（目标构建已通过，板测待完成）

已建立窄 `platform/api` capability、独立 FreeRTOS/STM32H7 后端、受控 Heap、TIM2 `hrt_absolute_time()`、持久 ApplicationContext、WorkQueue、uORB、events、perf 和 logging。公共对象由独立 include 集编译，并由 `check-architecture` 禁止底层依赖回流。板上 HRT Overflow、栈高水位和运行期 Heap 余量仍待目标板验证。

### 阶段 2：Parameter Core 与类型安全参数包装（目标构建已通过，板测待完成）

阶段 2 以 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 为唯一参数来源，已建立：

- Parameter Layer/Core、AtomicTransaction、稀疏参数层、`param_*` 和 `px4::Param<T>` 显式 bind/update 接口。
- 官方 source parser、XML/JSON 输出和 `px4_parameters.hpp` 模板语义；标准库 renderer 消除系统 Python 缺少 Jinja2 的阻塞，不回退到正则参数解析。
- TinyBSON 纯 Buffer 子集与 flashparams enumerator/visitor 适配，编码和解码热路径不动态分配。
- 300 ms debounce、至少 2 s 保存限频、最多 3 次失败重试的 Autosave 策略。
- USB CDC 固定 1024-byte SPSC RX Ring；ISR 只复制字节并立即恢复接收，命令处理位于任务/LP service 路径。
- `0x081E0000～0x08200000` 单个 128 KiB Storage 扇区的追加 Journal，保持 v1 字节格式；平台无关 Journal 与 STM32H7 raw Flash device 分离，扫描不再整段 invalidate cache，空间满返回 ENOSPC且不自动擦除。
- 通用 Flash BusFault hook 仅在活动安全读窗口、地址属于参数分区且 Bank 2 DBECC 标志匹配时恢复；其他 BusFault 记录 CFSR/ABFSR 并复位。
- 24 项 PX4 差速 Rover 参数：20 项 `RO_*` 与 4 项 `RD_*`；参数数量由官方生成目录确定，无固定 64 项上限。

2026-07-30 目录迁移后的 Windows 本地 `make verify` 已通过：`.text=112720`、`.data=2068`、`.bss=326672` bytes，Signed BIN 为 `116008` bytes；应用向量地址为 `0x08040400`。

阶段 2 的实现已存在。项目自有测试目录已移除，阶段验收采用目标编译、链接、签名和目标板行为检查；尚未进行 USB 在线调参、自动保存、掉电恢复、损坏尾部回退、扇区满和人工擦除的目标板验收。阶段 2 工作区改动已按功能拆分提交。许可证保持 `PENDING`，最终处理 `DEFERRED`，不阻塞后续阶段。

### 阶段 3：SBUS、RCUpdate 与 ManualControl

移植成熟 SBUS parser 和 SbusRc 接收逻辑，使用 STM32 UART RXINV 与循环 DMA 适配；接入 18 通道校准、MIN/MAX/TRIM/DZ/REV、功能映射、失联判断、ManualControl 和 Action Request。SBUS 模块不得直接驱动 PWM。

2026-07-31 阶段 3 实现已收敛：

```text
可配置 SBUS UART + DMA1 Stream2
→ SbusParser / SbusRc
→ input_rc
→ RCUpdate
→ rc_channels + manual_control_switches
→ RcManualInput（PX4 ManualControl RC 子集）
→ manual_control_setpoint + action_request
```

- `RC_INPUT_PROTO` 使用 `0=Disabled`、`2=SBUS`，默认 SBUS；端口按最新版 VCU-H7 硬件直接编号为 `SERIAL1..8=USART1/USART2/USART3/UART4/UART5/USART6/UART7/UART8`，每路由 `SERIALx_FUNCTION` 分配 Disabled/RC Input，默认 SERIAL6=USART6。当前板级固件不保留旧 `RC_PORT_CONFIG`、旧串口键或迁移版本参数；旧快照不迁移，开发阶段直接按当前目录重新配置。
- 原始反相 SBUS 自动使用 100000 bit/s、8E2、UART RXINV 和 RX pulldown，不再提供手动极性参数；接管前保存 UART/FIFO/RX GPIO，停用、失败回滚和 Runtime shutdown 均恢复普通 UART。
- SRAM3 non-cacheable 64-byte 循环 DMA Buffer 只由 DMA 写入；ISR 记录真实到达时间并复制到 256 项 CPU-only Ring，再经 ISR-safe callback 唤醒 `wq:io`，业务层不执行 cache maintenance。
- RC 参数由生成器扩展到 135 项总量，其中阶段 3 新增 111 项 RC 配置、18 通道校准、映射和失联参数。
- `RC_CHAN_CNT=0` 时按实际接收通道数工作，仍可通过在线参数显式限制通道数；校准与映射在 `parameter_update` 后在线刷新。
- SBUS 单帧丢失只累计丢帧统计，不误判为整条链路失联；无强 CRC 的 SBUS 在冷启动/Failsafe/UART 恢复后要求连续 3 个健康帧锁定，随后 `RCUpdate` 再要求连续健康 100 ms 才恢复控制。Failsafe、显式 `rc_lost` 或 `COM_RC_LOSS_T` 超时都会清除恢复窗口。
- Arm/Kill 开关至少要求两份严格前进的一致样本并稳定 200 ms；启动、RC 恢复及映射/阈值变化后的首个稳定样本只建立基线，之后仅在边沿发布 `action_request`。
- 阶段 3 没有连接 PWM、Mixer、Arming 或 Actuator 输出，因此完成后车辆仍不能移动。
- Windows Arm GNU Toolchain 16.1.0 下的独立 `make verify BUILD_DIR=build-phase3-final` 已通过；未新增测试、SITL 或仿真。

### 阶段 4：Commander Rover 子集（目标构建已通过，板测待完成）

阶段 4 以 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 为直接代码来源，ArduPilot `3f2e4763accb` 只作 Rover 安全行为参考，已完成：

- 完整导入 `vehicle_status`、`vehicle_control_mode` 和 `actuator_armed` 公开消息契约，三个 Topic 深度为 1，`action_request` 保持深度 8。
- Commander 复用 `wq:hp_default`，20 ms 检查 RC/参数，状态变化立即发布，静态状态至少每 500 ms 发布一次。
- 冷启动 `DISARMED + MANUAL`；Arm 除参数核心、Manual、新鲜有效 RC、throttle/yaw 中位及未 Kill/Termination 外，还要求 `actuator_output_status` 新鲜且 sequence 严格前进、后端 ready、参数无 pending、至少一右一左映射，并已实际建立 `DISARMED_NEUTRAL`。
- ARMED 时 RC/参数/执行器输出故障强制 Disarm；恢复后只清故障原因，不自动 Arm。Kill 固定为 Kill→Disarm→hard-off，Unkill 只清 Kill，必须重新产生 Arm 边沿；Termination 锁存到 MCU 重启。
- 仅 Manual 可由用户选择；Termination 只作为内部状态，Position、Auto、Offboard、VTOL 等模式均拒绝。
- Parameter Flash 保存和擦除在 ARMED 期间禁止，Autosave 保持 pending 并在 Disarm 后重试。
- 启动顺序为基础服务、Parameter、Log、Commander、RC；停止顺序为 RC、Commander、Log、Parameter。RC 链失败不停止 Commander，Commander 启动失败则回滚应用服务。
- 最终目标构建为 Application `136956/7660/331912` bytes、Signed BIN `145830` bytes，签名、Factory HEX、MCUboot 和 `0x08040400` 向量检查通过；详见 [阶段 4 资源与验收基线](DIMA_PHASE4_RESOURCE_BASELINE_ZH.md)。
- 2026-08-03 全量回归修复后，应用收敛为 SysTick + TIM2 双时基，补齐 Flash/Arming 原子互锁、Commander 发布失效安全、真实 RC DMA 到达时间、ICM42688 板级接口、参数快照回退和 WorkQueue cancel-and-drain。增量目标构建为 Application `141428/7668/332584` bytes、Signed BIN `150311` bytes；实板时序、电气和并发验收仍待完成。
- 2026-08-04 平台隔离重构把 FreeRTOS、HAL/CMSIS、cache/DMA/Flash 所有权收敛到独立后端；WorkQueue、uORB、Parameter、Console、Commander、RC 和启动任务只使用公共 capability。最终 clean 构建与实板压力验收以本次交付证据为准。
- 2026-08-05 生命周期收敛建立唯一 `Dima/rover`，补齐 Parameter cache、uORB epoch、WorkQueue owner/drain、Console、BootHealth、Fault 冷启动持久化和 SBUS DMA/CPU Ring 所有权。源码架构门禁已通过；Windows 原生 clean build、最终 ELF 门禁和同上电 Runtime restart 板测仍待完成，历史尺寸不得作为当前结果。
- 2026-08-06 SBUS 电气状态改由协议自动管理，删除手动极性参数；统一 Debug SourcePolicy 支持低优先级日志服务以不超过 10 Hz 输出最新 18 路 SBUS 数据，默认 SBUS Error 不输出，ISR/DMA 路径不格式化或写 USB；后续由 MAVLink 接入重新收敛诊断传输边界。
- 2026-08-06 目录职责再次收敛：RC 来源转换明确命名为 `modules/rc/RcManualInput`，Rover 的实际入口明确命名为 `rover/modes/ManualMode`；MotorOutput 归入执行器运行模块，纯 `DifferentialDrive` 归入 `Dima/lib/rover`。同时移除从未进入生产调用链的 `speed_to_pwm` 和阶段 8 预实现控制器、README-only 规划目录、旧 75 MHz CubeMX 工程及未参与构建的根级 newlib 锁文件，并消除两个含义不同的 `Backend.hpp`。
- 本阶段未接 PWM、Mixer、RoverDifferential 或 HAL 执行器输出，因此车辆仍不能运动；目标板行为尚未验收。

### 阶段 5：差速执行器链

阶段 5 已由 `Dima/rover/control/` 运行适配、`Dima/lib/rover/DifferentialDrive.*` 纯算法和 `Dima/modules/motor/` 安全输出模块完成以下受控子集：

- 只保留前后 `longitudinal` 和左右 `steering/yaw` 两轴。`ManualMode` 发布 `rover_motion_request`，RoverDifferential 在 100 Hz 生成右、左两路 `actuator_motors`；其余十路保持 NaN。
- `rover_motion_request` 同时预留 `SOURCE_NAVIGATION` 和 `MODE_SPEED_YAW_RATE`，但当前只接受 `SOURCE_MANUAL + MODE_NORMALIZED_AXES`。阶段 9 Navigation 必须复用该消息边界，不能直接依赖 DifferentialDrive、MotorOutput 或板级 PWM。
- 差速混控采用 PX4 v1.17.0 的两轴/消息边界，并综合 ArduPilot Rover 的倒车车头方向、转向/油门饱和优先级、静摩擦补偿、反向推力不对称和左右独立换向延时行为；当 `MOT_THR_ASYM>1` 时先在 `[-1/asymmetry, 1]` 两侧电机可行域内应用 `RD_STR_THR_MIX`，再作反向补偿，避免倒车两侧同时裁到 -1 后丢失转向。ArduPilot GPL 源码只作行为参考，没有复制。
- 六路输出只提供 Disabled、MotorRight、MotorLeft 三种功能。默认全 Disabled；每路公开 `FUNC/MIN/CENT/MAX/REV`，产品包络统一为 500～2500 us，默认仍为 1000/1500/2000 us，并允许同一 Motor function 映射多个物理口。普通 Disarmed 在参数有效的通道持续输出各自 `CENT`，Disabled 或参数无效的通道始终无脉冲；至少一右一左仍有效时允许解锁，映射不完整时仅拒绝解锁。
- 固定物理映射为 S1/PB0/TIM8_CH2N、S2/PB1/TIM8_CH3N、S3～S6/PA0～PA3/TIM5_CH1～CH4。TIM8 Update TRGO 同步复位 TIM5，二者均为 1 MHz、ARR 19999、50 Hz。
- MotorOutput 只有在完整、严格前进且新鲜的 Commander 安全快照、有效双向 Motor 命令和 `COM_ACT_LOSS_T` 双时间戳约束同时满足时才进入 `ACTIVE`。未知 `FUNC` 或无效 `MIN/CENT/MAX` 仅隔离对应通道，映射不完整由 Commander 拒绝解锁但 Runtime 与通信继续；启动、无任何有效通道、Kill、Termination、Failsafe、Armed 命令超时、发布错误、后端 Retry/Fault 和关闭进入 `HARD_SAFE_OFF`。ACTIVE inhibit 与 hard-safe inhibit 分离，单条负向安全 Topic 先到即可 fail-closed。
- 应用侧 IWDG 固定约 2048 ms、100 ms 检查，appMain 为唯一 feed owner；BootHealth 以安全 Topic 与 MotorOutput 输出 Topic 的严格进展推进健康 generation，不跨 WorkQueue 读取模块普通状态。运行期存储仅在 Disarmed、neutral/hard-safe、appMain 已 reload 且维护进度持续前进时获批，并在整个事务中阻止 Arm。MCUboot 对跨复位仍运行的 watchdog 临时扩展并在 Recovery/校验/swap/Flash/USB 长循环喂狗，原始 IWDG reset flags 跨应用桥接保留；实际复位时限和 GPIO 电气行为仍待板测。
- USB CDC 已作为系统调试日志与维护命令口，移除周期性 HelloWorld 和示例心跳；控制/ISR 热路径只上报固定结构事件，不直接格式化或阻塞发送。

Windows 原生 clean build、签名、Factory HEX、MCUboot 布局、应用 ELF 生命周期/执行器门禁和静态热路径检查已经通过。阶段状态为“源码及目标构建通过，板测待完成”；示波器确认六路频率、脉宽、相位、TIM8 N 极性、真实低电平以及 Arm/Disarm/Kill/Termination/超时波形之前，不得宣称执行器运行行为已经验收。详细数据见 [阶段 5 资源与验收基线](DIMA_PHASE5_RESOURCE_BASELINE_ZH.md)。

2026-08-19 在 Windows 原生进程、Windows 路径和项目缓存 Arm GCC 10.3.1 下重新执行 `make clean` 后完整 `make -j4 NO_COLOR=1 dima_rover`，通过 `[212/212]`：Application `233772/12284/356176` bytes、未签名 BIN `246096` bytes，MCUboot `47712/380/10192` bytes、BIN `48100` bytes，向量 `0x08040400`；两份 ELF 未解析符号为空，SBUS/Commander/MotorOutput/IWDG 与 MCUboot watchdog prepare/feed 关键符号均实际链接。同一 image digest `601c65353ffebce20cea8f040d975000e3c4caa2b27aaa15dd409529740e26ce` 的 clean build 与后续重签分别得到 `247271/247270`-byte Signed BIN、`710861/710859`-byte Factory HEX；这是 imgtool 可变长度 ECDSA P-256 DER 签名带来的合法差异，Signed/Factory 文件长度及文件 SHA-256 不作为确定性合同。该结果只证明源码、静态门禁和目标构建，不证明 SBUS 电气、六路 PWM 波形、电调握手、IWDG 约 2 秒复位或车辆行为。

同日实板首启排查修复两个串联问题：MCUboot 在无有效 D3 头时无法建立应用桥接，以及应用在 LSI/IWDG 启动前等待 SR 同步。真实记录为 `ERROR_HANDLER / APPLICATION_RUNNING / stacked_r0=2048`，无 Cortex-M fault 状态位。修复后的 Windows clean build 再次通过 `[212/212]`，Application 尺寸不变，MCUboot 为 `47864/380/10192` bytes、BIN `48252` bytes，image digest 为 `835947050b2df9062be4289bcb5b876abe0dee99b46792051827e79f123eeec3`；实板已通过冷启动双枚举和 `Dima Rover MAVLink` 身份验证。IWDG 实际复位时限、六路 PWM 和车辆行为仍待板测。

### 阶段 6：传感器层

接入 IMU、Mag、GNSS/RTK、Wheel Encoder、电池与健康状态。所有数据具有采样时间戳；DMA Buffer 位于 D2 SRAM；Estimator 只订阅消息，不直接访问 HAL。

### 阶段 7：EKF2

移植 PX4 EKF2 module、EKF core、多实例管理、参数、GSF Yaw、延迟观测缓冲、创新检查、Estimator Selector 和状态输出。首版即采用多实例架构，至少支持两个 EKF 实例；由 IMU 数据到达驱动，实际激活数量由可用 IMU/Mag 组合和参数决定。

首版输入：

```text
sensor_combined
vehicle_gps_position
vehicle_magnetometer
vehicle_air_data（硬件具备时）
distance_sensor（硬件具备时）
vehicle_visual_odometry（后续）
Dima wheel-odometry adapter（后续）
```

首版输出：

```text
vehicle_attitude
vehicle_local_position
vehicle_global_position
estimator_status
estimator_innovations
estimator_sensor_bias
estimator_reset_status
```

Wheel Encoder 不直接侵入 EKF Core；先通过独立 Dima Odometry Adapter 转换为可审查的观测输入。Estimator 只发布状态，不直接驱动控制器或 PWM。

### 阶段 8：闭环 Rover 控制

待状态估计和运行接口确定后，从 PX4 v1.17.0 重新导入 Speed、Yaw Rate、Heading 和 Stop 纯控制核，再由独立 Rover 运行模块连接状态估计、参数、消息和安全门控。当前源码树不保留无消费者的预实现控制器，也不得提前描述为生产闭环。完成后还需实车参数调整和控制状态可观测性。

### 阶段 9：Position、Waypoint、Reverse 与 PivotTurn

在 `Dima/rover/navigation/` 接入 Position、路径前视、航点、倒车、停车和原地旋转管理。PivotTurn 使用独立状态过程，不使用零速下无定义的曲率表达。

### 阶段 10：MAVLink、日志与产品化收敛

加入在线参数、状态、任务、Estimator、控制器、Fault/Event 和升级状态的现场可观测能力。纯 MAVLink 协议与 byte-stream 适配进入 `Dima/adapters/mavlink/`，uORB/Parameter/调度生命周期进入 `Dima/modules/mavlink/`，STM32 UART/DMA 后端继续留在 `Dima/platform/stm32h7/`；实现前不创建空目录。完成长时间运行、失联、传感器异常、看门狗、升级与回滚等实车验收。

## 6. 当前目录迁移验收条件

- 计划、Source Manifest 和 ADR 内容一致。
- EKF2 被明确记录为最终 Estimator 选择，旧 EKF3 计划失效。
- 正式 PX4 v1.17.0 与本地 1.16 预研快照被清楚区分。
- ArduPilot 参考 commit 被记录。
- 许可证状态为 `PENDING`，来源保留和发布限制明确。
- 阶段 0 不导入 EKF2、SBUS 或控制模块生产源码。
- Flash 地址和 MCUboot/Application 启动接口不因目录整理而改变。
- 目标编译、链接、签名镜像、Factory HEX 和目标板行为必须以当前工具链实测结果为准；完成前不得标记为通过。
- 不创建测试、SITL 或仿真代码。
- 不覆盖或回退工作区中已有的其他修改。
