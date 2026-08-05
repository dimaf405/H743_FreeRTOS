# Dima H743 FreeRTOS Rover 软件架构与边界

## 1. 产品定位

本项目是在 STM32H743 + FreeRTOS 上构建的完整、受控差速 Rover 固件。运行平台、产品装配和硬件适配由 Dima 维护；参数、消息、调度、遥控、状态估计和 Rover 控制优先复用成熟上游模块及数据契约，不从零重复实现同类核心。

直接代码来源以 PX4 v1.17.0 为正式基线，ArduPilot Rover 作为 Arming、Failsafe、轮速和 PivotTurn 等行为参考。目录和产品名称使用 Dima；上游源码中的 namespace、宏、参数名、Topic 名、版权头和许可证文字必须保持可追踪，不进行品牌式全文替换。

Estimator 固定采用 PX4 EKF2。此前涉及 ArduPilot EKF3 的阶段计划均由 `DIMA_ROVER_PORTING_PLAN_ZH.md` 取代。

## 2. 项目代码结构

```text
Dima/                         唯一自研应用根、兼容层和产品装配
├── application/              启动壳、C ABI 入口和 appMainTask
├── adapters/                 仅依赖公共 capability 的外部协议适配
├── platform/api/             OS/MCU 无关 capability 与 opaque handle
├── platform/freertos/        Task、同步、Heap 和 transaction 后端
├── platform/stm32h7/         MPU/cache/DMA/Flash/时钟/USB/串口后端
├── middleware/               Parameter、uORB、WorkQueue、Event、Perf、Log
│   ├── lifecycle/            Module 生命周期
├── modules/                  Parameter、Log、boot_health、RC、安全、EKF2
├── lib/                      motor、rover_control 与公共算法库
├── messages/                 共享消息契约
└── rover/                    唯一 Rover 产品域（ApplicationContext、control、navigation）

Boards/H743/                  板级初始化、Flash 布局和外设适配
Core/                         CubeMX/HAL 应用生成层
Drivers/                      CMSIS 与 STM32 HAL 厂商代码
Middlewares/                  FreeRTOS、MCUboot、ST USB 等第三方代码
USB_DEVICE/                   CubeMX USB Device 集成层
Bootloader/                   独立 MCUboot 固件镜像
Linker/、make/、tools/        链接、构建、签名和升级工具
docs/                         计划、架构、ADR、来源和维护文档
```

已退役的顶层 `App/` 已迁入 `Dima/`。C/C++ Runtime 位于 `Dima/platform/freertos/libc`；公共时间契约位于 `Dima/platform/api/Time.hpp`，TIM2 实现位于 `Dima/platform/stm32h7/Clock.cpp`。参数 Journal 与 STM32H7 raw Flash、USB Console 与 STM32 USB transport、SBUS 模块与 STM32 UART/DMA 驱动均已拆分，不再保留混合后端。

## 3. 依赖规则

- `Dima/rover` 是唯一 Rover 产品域，可依赖 modules、middleware、messages、lib 和 `platform/api`，但不包含具体后端类型；`Boards/H743/Src/platform_composition.cpp` 是后端组合根。
- `Dima/modules` 承载具有独立生命周期和运行状态的功能块；Parameter、Log 等系统功能与 RC、Commander 一样实现统一的 ModuleBase 契约。
- `Dima/modules` 和 `Dima/middleware` 禁止反向依赖 `Dima/rover`。
- `application/rover/modules/middleware/messages/lib/adapters` 只允许依赖标准库、内部公共契约和 `Dima/platform/api`，禁止 FreeRTOS、HAL、CMSIS、SCB/NVIC、Core、Board 与 USB 生成头。
- `Dima/platform/api` 只定义整数、尺寸、opaque handle、callback 和窄 capability，不暴露 OS/MCU/厂商类型。
- `Dima/platform/freertos` 只连接 `platform/api` 与 FreeRTOS；`Dima/platform/stm32h7` 只连接 `platform/api`、HAL/CMSIS 和板级定义，二者禁止互相包含。
- `Boards/H743` 负责 MCU、引脚、DMA、PWM、Flash 和总线接线，不依赖上层控制模块。
- `Core/` 与 `USB_DEVICE/` 的生成区只保留必要接线，禁止写入业务逻辑。
- `make check-architecture` 检查底层 include/API、cache/DMA/Flash 操作所有权、依赖方向和私有 include 集，并是 `firmware/verify/dima_rover` 的强制前置条件。
- 上游移植文件必须保留原始版权头，并在 Source Manifest 中记录原始路径、版本和本地修改。
- 项目内不新建名称包含大小写不敏感 `px4` 的目录；该规则不适用于源码符号、许可证和来源说明。

## 4. 启动与运行链

当前启动链为：

1. CRT 复制 data/清零 bss 之前，`SystemInit()` 调用无全局状态的 early-memory hook，规范化已有 cache/MPU 状态，配置 MPU Region 6/7，再依次启用 I-cache、D-cache；
2. `main()` 在 HAL 与调度器之前验证 `SCB->CCR`、`MPU->CTRL` 和 Region 6/7，失败则写启动诊断并 fail-closed；
3. `board_vector_table_init()`、`HAL_Init()`、系统/外设时钟和 `board_init()`；
4. H743 组合根建立 FreeRTOS 与 STM32H7 后端并安装 `platform::Services`；
5. `osKernelInitialize()` → `TaskRuntime::create(appMainTask)` → `osKernelStart()`；
6. `app_main` 进入 `ApplicationContext`，所有模块只通过 capability 运行。

### 4.1 双时基与调度边界

- SysTick 固定 1 kHz，同时推进 HAL 32 位毫秒计数和 FreeRTOS tick；`HAL_SuspendTick()` 只暂停 HAL 逻辑计数，不得关闭 SysTick 或停止任务调度。
- TIM2 固定为 1 MHz、32 位 HRT，并由溢出中断扩展为 64 位 `hrt_absolute_time()`；TIM2 及 CH1 保留给 HRT 和未来 compare，不得分配给 PWM、编码器或输入捕获。
- 当前禁止 tickless sleep、STOP 模式补偿和运行期动态改频。恢复这些能力前，必须同时证明 SysTick 与 TIM2 在低功耗和变频边界上的连续性。
- 普通 WorkQueue 由 1 ms FreeRTOS tick 唤醒，截止时间向上取整，因此不得提前执行；周期任务以前一截止时间锁相并跳过错过周期，不执行突发补偿。
- IMU、Estimator 和 Rate Controller 等高频链必须由 DMA、EXTI 或消息事件唤醒。确需亚毫秒 one-shot 时只扩展 TIM2 CH1 compare，不再增加第三套系统时基。

后续将形成独立执行域：

```text
wq:rate_ctrl     Rover 两轴差速控制
wq:estimator     EKF2
wq:sensors       传感器采样与处理
wq:nav           Position、Waypoint、PivotTurn
wq:io            六路 MotorOutput、SBUS、GNSS 和串口协议
service:param    参数和 Flash
service:console  USB 命令
service:logger   非实时日志
```

USB、Flash、SD 和阻塞日志不得运行在控制或 Estimator WorkQueue。

### 4.2 Application Runtime 生命周期

平台资源分为上电期、Application Runtime 和跨 Runtime 保留三类，禁止把三者的 cache、句柄或有效性标志混用：

```text
整次上电：MPU/cache、SysTick、TIM2 HRT、平台 Services、Heap、Flash 互锁、USB transport
每次 Runtime：Console 前端、WorkQueue、uORB Buffer、Parameter Core/Journal、ModuleManager
跨 Runtime：Commander Termination、D3 Fault 记录和明确标注的累计诊断
```

Runtime 初始化顺序固定为 `Console → WorkQueue → uORB → Parameter Journal/Core → Module registration`；启动顺序固定为 `Parameter → Log → MotorOutput safe-off → Commander → SBUS → RCUpdate → ManualControl → ManualMotionAdapter → RoverDifferential → BootHealth`。关闭时先停 BootHealth 和 RC 链，再确认 MotorOutput 已停止 TIM5/TIM8 并恢复六路 GPIO 低电平，然后停止控制生产者、Commander、Log 和 Parameter，最后按 `Parameter Core/Journal → uORB → WorkQueue → Console` 释放 Runtime 资源。

- `ApplicationContext` 只接受 owner task 执行 init/start/shutdown；部分初始化和 Error 状态按成功步骤逆序回滚，清理失败时不得伪装为 Stopped 或重新 init。
- `Param<T>` 构造不访问 Parameter Core；模块每次 start 必须 `bind()`，shutdown 清除 ready、used、unsaved、值 cache、动态 Layer、callback 和运行期同步对象。Journal 下次 initialize/load 必须重新扫描并复验 Header、Commit Marker 和 payload CRC。
- uORB 每次成功 initialize 推进上电期单调 epoch。Publication、Subscription、instance、generation 和 callback 发现 epoch 变化后丢弃旧 Runtime 状态；深度队列从当前最旧有效样本恢复，generation 0 不得复制空槽。
- WorkQueue 由 Runtime owner 创建和关闭；ISR、worker 自身或非 owner 无权销毁。外部 stop 在释放订阅和后端前 cancel-and-drain；Signal 和 task slot 由 shutdown 显式回收，不依赖全局析构器。
- Commander Termination 是唯一跨 Runtime 保留的模块安全状态，只能由 MCU reset 清除；RC 边沿、Failsafe 临时原因、BootHealth 窗口和参数绑定状态每次 Runtime 重建。

### 4.3 Rover 控制与六路输出边界

阶段 5 的运行链固定为：

```text
manual_control_setpoint
→ rover_motion_request（前后、左右两轴）
→ RoverDifferential
→ actuator_motors（Motor1 右侧、Motor2 左侧）
→ MotorOutput
→ platform::ActuatorPwm
→ TIM8/TIM5 六路普通 PWM
```

- Manual 和未来 Navigation 只能通过 `rover_motion_request` 进入控制层；当前只接受 `SOURCE_MANUAL + MODE_NORMALIZED_AXES`，Navigation 的 source/mode 只保留接口，不创建空模块，也不得绕过差速混控直接访问 `actuator_motors` 或 PWM。
- MotorOutput 只有在 Commander 三 Topic 为同时间戳、严格前进、新鲜且完整表达 `ARMED + MANUAL + !kill + !termination + !lockdown + !failsafe`，两个 Motor 命令均有限、可逆并且发布/采样时间都不超过 `COM_ACT_LOSS_T` 时，才允许启动 PWM。
- S1～S6 只允许 Disabled、MotorRight、MotorLeft；默认全部 Disabled。零命令映射到各通道 `CENT`，Disarm、Kill、Termination、状态/命令超时、参数无效、发布失败和后端 Retry/Fault 都回到物理 safe-off。
- `board_init()` 在调度器和产品 Runtime 之前确认 TIM5/TIM8 已停止、CCR 为 0、六路 GPIO 为低；Application shutdown 只有在 MotorOutput 停止且 `safe_off_confirmed()` 成功后才能释放 Runtime 资源。
- BootHealth 除 Commander 三 Topic 外还要求 `actuator_output_status` 严格前进、新鲜、后端就绪、状态为 SAFE_OFF、有效掩码和六路脉宽全为 0。ARMED、Active、Kill、Termination 或 Failsafe 会重新开始完整 5 秒窗口。

## 5. 内存与实时边界

本项目不再要求全系统完全静态，但动态内存必须受控：

- 启动期和非实时服务允许使用位于 D1 AXI SRAM 的受监控 FreeRTOS `heap_5`。
- ISR、控制循环、EKF2 更新、Arming/Failsafe、Mixer 和 PWM 输出禁止分配。
- 通用 Heap 已固定为 D1 AXI SRAM 中 256 KiB 的 `.dima_heap`。
- D2 普通内存与 SRAM3 固定 32 KiB `.dima_dma` 分离；MPU 将 `0x30040000～0x30047FFF` 配置为 Normal、Shareable、Non-cacheable、XN，DMA 只接受 `DmaBufferView` 或平台 bounce buffer。
- 48 KiB 固定任务栈池位于 D1 的 `.dima_task_pool`，与 256 KiB `.dima_heap` 分离；DTCM 用于普通 data/bss 与同 Bank Flash 编程例程，不加入通用 Heap。
- D3 `0x38000000～0x3800FFFF` 为 non-cacheable 跨复位诊断区。
- 已启用 malloc failed hook、Heap 统计和内存故障 Event；任务栈高水位待目标板采集。
- C++ exceptions 和 RTTI 继续关闭。

`_sbrk()` 继续 fail-closed；C++ `new/delete` 和启动期 Topic Buffer 只通过 Dima allocator 使用受控 Heap。

## 6. 消息、参数和状态估计边界

- 生产消息接口统一采用 uORB 兼容 Publication/Subscription；启动健康观察 Commander 三个安全 Topic 和 `actuator_output_status`，不建立示例心跳 Topic。
- 参数系统采用 PX4 Parameter + ModuleParams，参数数量由生成器产生，不设置固定 64 项上限。
- 在线参数通过 USB，后续增加 MAVLink；Flash 写入由非实时服务执行。
- 参数核心只依赖公共 execution/memory/synchronization 接口；`ParameterJournal` 与 STM32H7 raw Flash device 分离，保持 `0x081E0000/128 KiB`、Journal v1 字节格式和 ENOSPC 语义不变。
- 参数扫描不执行整段 cache invalidate；raw Flash 仅在 program/erase 成功后处理实际修改范围，D-cache 关闭时中央 helper no-op。
- 每次 load 都重新验证 Header、Commit Marker 和 payload CRC；最新记录损坏时重扫并回退上一条有效快照。BusFault 仅在活动安全读窗口、分区地址和 Bank 2 DBECC 三条件同时成立时恢复。
- Estimator 采用 EKF2，首版即保留多实例与 Estimator Selector，至少支持两个 EKF 实例；实际激活数量由可用 IMU/Mag 组合和参数决定。控制器只消费 `vehicle_attitude`、`vehicle_local_position`、`vehicle_global_position` 和健康状态，不直接访问 EKF 内部对象。
- Wheel Encoder 先通过 Dima Odometry Adapter 转为受支持的速度或里程计观测，不直接修改 EKF Core。
- Arming 状态与 PWM 外设是否启动分离；RoverDifferential 只发布两路双向 Motor 命令，最终六路输出必须再经过 MotorOutput 的独立 Failsafe、命令新鲜度和板级 safe-off Gate。

## 7. Flash、构建与恢复边界

当前 Flash 分区保持不变：

```text
MCUboot       256 KiB
Primary       768 KiB
Secondary     768 KiB
Scratch       128 KiB
Storage       128 KiB
```

- `Boards/H743/Inc/boot_layout.h` 是应用和 Bootloader 共用的 Flash 布局权威定义。
- 固件目标占用不超过 Application Slot 的 85%。
- 阶段 0 不缩小升级 Slot，也不改变 MCUboot 地址。
- 根目录 `H743_FreeRTOS.ioc` 是唯一 CubeMX 配置源。
- 默认构建读取 `GNUmakefile` 和 `make/project.mk`；禁止使用 `make -f Makefile` 绕过项目叠加层。
- `tools/check_architecture.py` 在源码阶段拒绝重复 Rover 根、逆向依赖、越权硬件访问、生命周期契约缺失、非零 PWM compare 和未授权执行器消费者；`tools/verify_application_elf.py` 在最终应用 ELF 上检查向量、ISR 强弱绑定、section 地址/容量、SBUS DMA/CPU Ring、生命周期符号、初始化数组白名单，并要求唯一六路安全 PWM 链的 HAL、board 和 MotorOutput 符号实际链接。源码扫描通过不等于 ELF、目标构建或板测通过。
- VS Code 的 Microsoft C/C++ 插件只使用 `make intellisense` 从真实 Make 配方生成的主机本地 `compile_commands.json`。数据库同时覆盖 Application、MCUboot 和各层私有 include/define；源码清单或编译参数变化后必须重新生成。可移植的 `.vscode` 配置纳入源码，包含绝对路径的数据库和符号索引继续忽略。
- MCUboot CDC + `mcumgr` 和 ROM USB DFU 恢复链不得因 Dima 重构而改变。
- 参数存储与 MCUboot confirm 共用 Flash transaction 和 Armed/Flash coordinator；锁顺序固定为存储/Journal → transaction → interlock，confirm 使用非阻塞 transaction 并保留 `DEFERRED`。
- MCUboot 可在跳转前关闭 cache，每个应用镜像必须自行且幂等地重建 MPU/cache 契约。
- 启动诊断 v2 增加 ABFSR、SCB CCR、MPU CTRL 和上一故障 ABFSR；Flash record 仍为 256 bytes，读取工具兼容 v1/v2。

目录迁移已完成。2026-07-30 已在 Windows 本地使用 GNU Make 4.4.1、Arm GNU Toolchain 16.1.0 和 binutils 2.47 执行正式项目入口，目标编译、链接、签名、MCUboot 地址一致性和 Factory HEX 验证通过；目标板运行行为仍需板测。项目不新增 Host Test、SITL 或仿真入口：

```powershell
make verify
```

操作和恢复要求见 [MCUboot USB 升级与恢复手册](MCUBOOT_USB_RECOVERY_ZH.md)。完整迁移路线见 [Dima Rover 移植计划](DIMA_ROVER_PORTING_PLAN_ZH.md)，阶段 0 实测资源见 [Dima Rover 资源基线](DIMA_RESOURCE_BASELINE_ZH.md)，当前执行器链资源和板测边界见 [阶段 5 资源与验收基线](DIMA_PHASE5_RESOURCE_BASELINE_ZH.md)。
