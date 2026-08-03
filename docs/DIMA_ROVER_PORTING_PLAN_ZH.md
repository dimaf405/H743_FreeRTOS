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
  RCUpdate / ManualControl / Commander 子集
  RoverDifferential / EKF2 / Position Control
  FunctionMotors / MixingOutput
                    ↓
Dima FreeRTOS 兼容层
  Parameter / ModuleParams / uORB / WorkQueue
  hrt / events / perf / logging / allocator
                    ↓
STM32H743 板级适配
  UART DMA / SPI / I2C / PWM / Flash / USB
                    ↓
FreeRTOS + STM32 HAL
                    ↓
MCUboot
```

当前目录边界：

```text
Dima/
├── application/                    启动壳、C ABI 入口和 appMainTask
├── adapters/                       USB Console、MCUboot 适配
├── platform/freertos/              libc、platform_time 与 FreeRTOS 平台适配
├── middleware/                     lifecycle、parameters、uORB、WorkQueue
├── modules/                        boot_health、hello_world、RC、安全、Rover、EKF2
├── lib/                            motor、rover_control 与公共算法库
├── messages/
└── product/rover/

Boards/H743/  Core/  Drivers/  Middlewares/  USB_DEVICE/  Bootloader/
```

已退役的顶层 `App/` 已完成目录归并：原启动入口进入 `Dima/application`，BootHealth/HelloWorld 进入 `Dima/modules/{boot_health,hello_world}`，Adapter 进入 `Dima/adapters`，生命周期进入 `Dima/middleware/lifecycle`，C/C++ Runtime 与时间接口进入 `Dima/platform/freertos/libc` 和 `Dima/platform/freertos/platform_time.*`，Motor/Rover Control 进入 `Dima/lib/{motor,rover_control}`。`Core/Boards/Drivers/Middlewares/USB_DEVICE/Bootloader` 保持独立；项目自有测试子系统已移除；目标构建、签名镜像和目标板行为以本次验证及后续板测为准。

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
- D2 SRAM 保留给静态、对齐的 DMA Buffer。
- DTCM 默认不加入通用 Heap。
- 启动期允许为模块对象、uORB Subscription、Parameter 稀疏值和 EKF2 缓冲分配内存。
- USB 命令、参数保存、日志和通信等非实时服务可受控分配。
- ISR、实时控制、Estimator 更新和执行器安全路径不得分配。
- 后续增加 malloc failed hook、最低剩余量、任务栈高水位和内存故障 Event。

## 5. 后续阶段

### 阶段 1：Dima FreeRTOS 平台兼容层（目标构建已通过，板测待完成）

已建立受控 Heap、TIM2 `hrt_absolute_time()`、持久 ApplicationContext、WorkQueue、uORB、events、perf 和 logging。生产 heartbeat 已迁移到 uORB；BootHealth、HelloWorld 和日志服务已迁移到 Dima WorkQueue。目录迁移后的目标固件、签名镜像和 Factory HEX 已通过 Windows 本地 `make verify`；板上 HRT Overflow、栈高水位和运行期 Heap 余量仍待目标板验证。

### 阶段 2：Parameter 与 ModuleParams（目标构建已通过，板测待完成）

阶段 2 以 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 为唯一参数来源，已建立：

- Parameter Layer/Core、AtomicTransaction、稀疏参数层、`param_*`、`px4::Param<T>` 和 `ModuleParams` 兼容接口。
- 官方 source parser、XML/JSON 输出和 `px4_parameters.hpp` 模板语义；标准库 renderer 消除系统 Python 缺少 Jinja2 的阻塞，不回退到正则参数解析。
- TinyBSON 纯 Buffer 子集与 flashparams enumerator/visitor 适配，编码和解码热路径不动态分配。
- 300 ms debounce、至少 2 s 保存限频、最多 3 次失败重试的 Autosave 策略。
- USB CDC 固定 1024-byte SPSC RX Ring；ISR 只复制字节并立即恢复接收，命令处理位于任务/LP service 路径。
- `0x081E0000～0x08200000` 单个 128 KiB Storage 扇区的追加 Journal，包含 Sequence、长度、CRC32 和最终 Commit Marker；扫描使用 ECC 安全读与受限 BusFault 恢复，空间满返回 ENOSPC，不自动擦除。
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
→ ManualControl
→ manual_control_setpoint + action_request
```

- `RC_PORT_CONFIG` 支持 0 禁用、1 UART4/PB8、2 UART7/PE7、3 UART8/PE0、4 USART2/PD6；默认 UART4/PB8。
- 原始反相 SBUS 使用 100000 bit/s、8E2、UART RXINV、D2 SRAM 64-byte 循环 DMA Buffer；ISR 仅更新时间和唤醒 `wq:io`。
- RC 参数由生成器扩展到 135 项总量，其中阶段 3 新增 111 项 RC 配置、18 通道校准、映射和失联参数。
- `RC_CHAN_CNT=0` 时按实际接收通道数工作，仍可通过在线参数显式限制通道数；校准与映射在 `parameter_update` 后在线刷新。
- SBUS 单帧丢失只累计丢帧统计，不误判为整条链路失联；Failsafe、显式 `rc_lost` 或 `COM_RC_LOSS_T` 超时才发布失联状态。
- Arm/Kill 开关首个稳定样本只建立基线，之后仅在边沿发布 `action_request`；阶段 4 接入 Commander 前没有消费者。
- 阶段 3 没有连接 PWM、Mixer、Arming 或 Actuator 输出，因此完成后车辆仍不能移动。
- Windows Arm GNU Toolchain 16.1.0 下的独立 `make verify BUILD_DIR=build-phase3-final` 已通过；未新增测试、SITL 或仿真。

### 阶段 4：Commander Rover 子集（目标构建已通过，板测待完成）

阶段 4 以 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` 为直接代码来源，ArduPilot `3f2e4763accb` 只作 Rover 安全行为参考，已完成：

- 完整导入 `vehicle_status`、`vehicle_control_mode` 和 `actuator_armed` 公开消息契约，三个 Topic 深度为 1，`action_request` 保持深度 8。
- Commander 复用 `wq:hp_default`，20 ms 检查 RC/参数，状态变化立即发布，静态状态至少每 500 ms 发布一次。
- 冷启动 `DISARMED + MANUAL`；Arm 要求参数核心、Manual、新鲜有效 RC、throttle/yaw 中位以及未 Kill/Termination。
- ARMED 时 RC/参数故障强制 Disarm；RC 恢复不自动 Arm。Kill/Unkill 可逆，Termination 锁存到 MCU 重启。
- 仅 Manual 可由用户选择；Termination 只作为内部状态，Position、Auto、Offboard、VTOL 等模式均拒绝。
- Parameter Flash 保存和擦除在 ARMED 期间禁止，Autosave 保持 pending 并在 Disarm 后重试。
- 启动顺序为基础服务、Parameter、Log、Commander、RC；停止顺序为 RC、Commander、Log、Parameter。RC 链失败不停止 Commander，Commander 启动失败则回滚应用服务。
- 最终目标构建为 Application `136956/7660/331912` bytes、Signed BIN `145830` bytes，签名、Factory HEX、MCUboot 和 `0x08040400` 向量检查通过；详见 [阶段 4 资源与验收基线](DIMA_PHASE4_RESOURCE_BASELINE_ZH.md)。
- 2026-08-03 全量回归修复后，应用收敛为 SysTick + TIM2 双时基，补齐 Flash/Arming 原子互锁、Commander 发布失效安全、真实 RC DMA 到达时间、ICM42688 板级接口、参数快照回退和 WorkQueue cancel-and-drain。增量目标构建为 Application `141428/7668/332584` bytes、Signed BIN `150311` bytes；实板时序、电气和并发验收仍待完成。
- 本阶段未接 PWM、Mixer、RoverDifferential 或 HAL 执行器输出，因此车辆仍不能运动；目标板行为尚未验收。

### 阶段 5：差速执行器链

移植 RoverDifferential、FunctionMotors、MixingOutput 和 OutputLimit，接入六路 PWM，实现 Manual、倒车、普通转弯和零速原地旋转。保留成熟的 armed 门控、failsafe 输出、限幅、反向、slew 和解锁 ramp。

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

接入 Speed、Yaw Rate、Heading 和 Stop 控制链，完成实车参数调整和控制状态可观测性。

### 阶段 9：Position、Waypoint、Reverse 与 PivotTurn

接入 Position、路径前视、航点、倒车、停车和原地旋转管理。PivotTurn 使用独立状态过程，不使用零速下无定义的曲率表达。

### 阶段 10：MAVLink、日志与产品化收敛

加入在线参数、状态、任务、Estimator、控制器、Fault/Event 和升级状态的现场可观测能力，完成长时间运行、失联、传感器异常、看门狗、升级与回滚等实车验收。

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
