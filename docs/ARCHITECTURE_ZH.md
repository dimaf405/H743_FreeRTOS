# Dima H743 FreeRTOS Rover 软件架构与边界

## 1. 产品定位

本项目是在 STM32H743 + FreeRTOS 上构建的完整、受控差速 Rover 固件。运行平台、产品装配和硬件适配由 Dima 维护；参数、消息、调度、遥控、状态估计和 Rover 控制优先复用成熟上游模块及数据契约，不从零重复实现同类核心。

直接代码来源以 PX4 v1.17.0 为正式基线，ArduPilot Rover 作为 Arming、Failsafe、轮速和 PivotTurn 等行为参考。目录和产品名称使用 Dima；上游源码中的 namespace、宏、参数名、Topic 名、版权头和许可证文字必须保持可追踪，不进行品牌式全文替换。

Estimator 固定采用 PX4 EKF2。此前涉及 ArduPilot EKF3 的阶段计划均由 `DIMA_ROVER_PORTING_PLAN_ZH.md` 取代。

## 2. 项目代码结构

```text
Dima/                         唯一自研应用根、兼容层和产品装配
├── application/              启动壳、C ABI 入口和 appMainTask
├── adapters/                 USB Console、MCUboot 等外部适配
├── platform/freertos/        FreeRTOS 平台适配（libc、platform_time）
├── middleware/               Parameter、uORB、WorkQueue、Event、Perf、Log
│   ├── lifecycle/            Module 生命周期
├── modules/                  Parameter、Log、boot_health、hello_world、RC、安全、EKF2
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

已退役的顶层 `App/` 已迁入 `Dima/`：启动壳位于 `Dima/application`，功能模块位于 `Dima/modules/{boot_health,hello_world}`，外部适配位于 `Dima/adapters`，生命周期位于 `Dima/middleware/lifecycle`，C/C++ Runtime 与平台时间位于 `Dima/platform/freertos/libc` 和 `Dima/platform/freertos/platform_time.*`，Motor 与 Rover Control 位于 `Dima/lib/{motor,rover_control}`。已退役的顶层 `App/` 不再是当前架构根；目录迁移已完成；Windows ARM 工具链已配置，构建和运行结果以本次目标编译为准。

## 3. 依赖规则

- `Dima/rover` 是唯一 Rover 产品域，保留 `ApplicationContext` 产品装配根，并在 `control/`、`navigation/` 承载 Rover 专属功能。
- `Dima/modules` 承载具有独立生命周期和运行状态的功能块；Parameter、Log 等系统功能与 RC、Commander 一样实现统一的 ModuleBase 契约。
- `Dima/modules` 和 `Dima/middleware` 禁止反向依赖 `Dima/rover`。
- `Dima/modules` 可依赖 Dima middleware、messages、lib 和明确的平台适配接口，不直接包含 STM32 HAL 全局句柄。
- `Dima/lib` 保持算法属性，不依赖 HAL、USB、MCUboot 或具体板卡。
- `Dima/middleware` 直接拥有 lifecycle、Parameter、uORB、WorkQueue、Event、Perf 和 Logging，不再依赖顶层 `App`。
- `Dima/platform/freertos` 连接 FreeRTOS、时间、内存和同步原语，不承载 Rover 业务逻辑。
- `Boards/H743` 负责 MCU、引脚、DMA、PWM、Flash 和总线接线，不依赖上层控制模块。
- `Core/` 与 `USB_DEVICE/` 的生成区只保留必要接线，禁止写入业务逻辑。
- 上游移植文件必须保留原始版权头，并在 Source Manifest 中记录原始路径、版本和本地修改。
- 项目内不新建名称包含大小写不敏感 `px4` 的目录；该规则不适用于源码符号、许可证和来源说明。

## 4. 启动与运行链

当前启动链保持不变：

1. `board_vector_table_init()` 设置应用向量表；
2. `HAL_Init()`；
3. 系统和外设时钟初始化；
4. `board_init()`；
5. `osKernelInitialize()` → `Dima/application/app_bootstrap.cpp` 中的 `app_bootstrap_create()` → `osKernelStart()`；
6. `Dima/application/app_main.cpp` 进入 `Dima/rover/ApplicationContext`，由 Rover 装配根初始化 USB 和运行模块。

### 4.1 双时基与调度边界

- SysTick 固定 1 kHz，同时推进 HAL 32 位毫秒计数和 FreeRTOS tick；`HAL_SuspendTick()` 只暂停 HAL 逻辑计数，不得关闭 SysTick 或停止任务调度。
- TIM2 固定为 1 MHz、32 位 HRT，并由溢出中断扩展为 64 位 `hrt_absolute_time()`；TIM2 及 CH1 保留给 HRT 和未来 compare，不得分配给 PWM、编码器或输入捕获。
- 当前禁止 tickless sleep、STOP 模式补偿和运行期动态改频。恢复这些能力前，必须同时证明 SysTick 与 TIM2 在低功耗和变频边界上的连续性。
- 普通 WorkQueue 由 1 ms FreeRTOS tick 唤醒，截止时间向上取整，因此不得提前执行；周期任务以前一截止时间锁相并跳过错过周期，不执行突发补偿。
- IMU、Estimator 和 Rate Controller 等高频链必须由 DMA、EXTI 或消息事件唤醒。确需亚毫秒 one-shot 时只扩展 TIM2 CH1 compare，不再增加第三套系统时基。

后续将形成独立执行域：

```text
wq:rate_ctrl     Rover 控制和执行器输出
wq:estimator     EKF2
wq:sensors       传感器采样与处理
wq:nav           Position、Waypoint、PivotTurn
wq:io            SBUS、GNSS 和串口协议
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

Runtime 初始化顺序固定为 `Console → WorkQueue → uORB → Parameter Journal/Core → Module registration`；启动顺序固定为 `Parameter → Log → Commander → SBUS → RCUpdate → ManualControl → HelloWorld（启用时）→ BootHealth`。关闭时严格反向停止生产者和模块，再按 `Parameter Core/Journal → uORB → WorkQueue → Console` 释放 Runtime 资源。

- `ApplicationContext` 只接受 owner task 执行 init/start/shutdown；部分初始化和 Error 状态按成功步骤逆序回滚，清理失败时不得伪装为 Stopped 或重新 init。
- `Param<T>` 构造不访问 Parameter Core；模块每次 start 必须 `bind()`，shutdown 清除 ready、used、unsaved、值 cache、动态 Layer、callback 和运行期同步对象。Journal 下次 initialize/load 必须重新扫描并复验 Header、Commit Marker 和 payload CRC。
- uORB 每次成功 initialize 推进上电期单调 epoch。Publication、Subscription、instance、generation 和 callback 发现 epoch 变化后丢弃旧 Runtime 状态；深度队列从当前最旧有效样本恢复，generation 0 不得复制空槽。
- WorkQueue 由 Runtime owner 创建和关闭；ISR、worker 自身或非 owner 无权销毁。外部 stop 在释放订阅和后端前 cancel-and-drain；Signal 和 task slot 由 shutdown 显式回收，不依赖全局析构器。
- Commander Termination 是唯一跨 Runtime 保留的模块安全状态，只能由 MCU reset 清除；RC 边沿、Failsafe 临时原因、BootHealth 窗口和参数绑定状态每次 Runtime 重建。

## 5. 内存与实时边界

本项目不再要求全系统完全静态，但动态内存必须受控：

- 启动期和非实时服务允许使用位于 D1 AXI SRAM 的受监控 FreeRTOS `heap_5`。
- ISR、控制循环、EKF2 更新、Arming/Failsafe、Mixer 和 PWM 输出禁止分配。
- 通用 Heap 已固定为 D1 AXI SRAM 中 256 KiB 的 `.dima_heap`。
- D2 SRAM 保留给显式 DMA Buffer；DMA Buffer 不从通用 Heap 获取。
- DTCM 默认用于快速非 DMA 状态和任务栈，不加入通用 Heap。
- 已启用 malloc failed hook、Heap 统计和内存故障 Event；任务栈高水位待目标板采集。
- C++ exceptions 和 RTTI 继续关闭。

`_sbrk()` 继续 fail-closed；C++ `new/delete` 和启动期 Topic Buffer 只通过 Dima allocator 使用受控 Heap。

## 6. 消息、参数和状态估计边界

- 生产消息接口统一采用 uORB 兼容 Publication/Subscription；`app_heartbeat` 使用生产 uORB 链。
- 参数系统采用 PX4 Parameter + ModuleParams，参数数量由生成器产生，不设置固定 64 项上限。
- 在线参数通过 USB，后续增加 MAVLink；Flash 写入由非实时服务执行。
- 参数 Journal 每次 load 都重新验证 Header、Commit Marker 和 payload CRC；最新记录损坏时重扫并回退上一条有效快照。
- Estimator 采用 EKF2，首版即保留多实例与 Estimator Selector，至少支持两个 EKF 实例；实际激活数量由可用 IMU/Mag 组合和参数决定。控制器只消费 `vehicle_attitude`、`vehicle_local_position`、`vehicle_global_position` 和健康状态，不直接访问 EKF 内部对象。
- Wheel Encoder 先通过 Dima Odometry Adapter 转为受支持的速度或里程计观测，不直接修改 EKF Core。
- Arming 状态与 PWM 外设是否启动分离，最终输出必须经过统一 Failsafe 和 Actuator Gate。

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
- `tools/check_architecture.py` 在源码阶段拒绝重复 Rover 根、逆向依赖、越权硬件访问、生命周期契约缺失和非零 PWM compare；`tools/verify_application_elf.py` 在最终应用 ELF 上检查向量、ISR 强弱绑定、section 地址/容量、SBUS DMA/CPU Ring、生命周期符号、初始化数组白名单以及无执行器消费者。源码扫描通过不等于 ELF、目标构建或板测通过。
- MCUboot CDC + `mcumgr` 和 ROM USB DFU 恢复链不得因 Dima 重构而改变。
- Arm 与应用 Flash 操作由同一原子互锁协调；模块停止先关闭新调度并 drain 正在执行的 `Run()`，再释放订阅、DMA、Perf、Flash 或日志资源。

目录迁移已完成。2026-07-30 已在 Windows 本地使用 GNU Make 4.4.1、Arm GNU Toolchain 16.1.0 和 binutils 2.47 执行正式项目入口，目标编译、链接、签名、MCUboot 地址一致性和 Factory HEX 验证通过；目标板运行行为仍需板测。项目不新增 Host Test、SITL 或仿真入口：

```powershell
make verify
```

操作和恢复要求见 [MCUboot USB 升级与恢复手册](MCUBOOT_USB_RECOVERY_ZH.md)。完整迁移路线见 [Dima Rover 移植计划](DIMA_ROVER_PORTING_PLAN_ZH.md)，阶段 0 实测资源见 [Dima Rover 资源基线](DIMA_RESOURCE_BASELINE_ZH.md)。
