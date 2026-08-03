# ADR-0001：采用 FreeRTOS 构建 Dima 差速 Rover

- 状态：Accepted
- 日期：2026-07-29
- 决策者：Dima Rover 项目

## 背景

当前项目已经以 STM32H743、FreeRTOS、CubeMX/HAL、MCUboot、USB 恢复和六路 PWM 为产品底座。项目目标是完整、受控的差速 Rover，而不是将硬件重新移植为 NuttX/PX4 Board。

后续会复用 Parameter、uORB、WorkQueue、RC、Commander Rover 子集、RoverDifferential、执行器和 EKF2 等成熟模块，因此需要在保留当前产品底座的同时提供足够稳定的兼容层。

## 决策

固定使用 FreeRTOS 作为 Dima Rover 的实时操作系统，不切换到 NuttX。

在 FreeRTOS 上建立 Dima 平台兼容层，承载：

- 单调微秒时间。
- WorkQueue 和 ScheduledWorkItem 兼容接口。
- uORB Publication/Subscription 兼容接口。
- Parameter、ModuleParams、Event、Perf 和 Logging 后端。
- STM32 HAL 的 UART、DMA、PWM、Flash、USB 和传感器适配。

不再要求全系统完全静态确定：启动期和非实时服务允许受控动态分配；ISR、控制循环、EKF2 更新、Arming/Failsafe、Mixer 和 PWM 输出路径禁止动态分配。

阶段 1 计划使用受监控的 FreeRTOS `heap_5`，通用 Heap 首版放在 D1 AXI SRAM；D2 SRAM 用于静态 DMA Buffer；DTCM 默认不加入通用 Heap。

## 备选方案

### NuttX/PX4 Board Port

优点是更接近 PX4 原生运行环境，完整模块复用和上游同步成本更低。未采用原因是会重做当前板级、启动、驱动、USB、构建和产品运行时，且不符合继续保留现有 FreeRTOS 产品底座的目标。

### 继续使用现有轻量运行时但不建立兼容层

初期改动较少，但每个上游模块都需要单独改写平台依赖，长期会形成大量不可维护的分叉，因此不采用。

## 影响

### 正面影响

- 保留 MCUboot、CubeMX/HAL、USB 恢复和现有板级成果。
- 产品任务、内存、外设和升级行为保持可控。
- 可以按阶段只移植差速 Rover 所需能力。

### 代价与风险

- 必须长期维护 FreeRTOS 与上游 API 的兼容层。
- 上游升级时需要审查参数、消息、调度和模块依赖变化。
- 动态内存需要严格监控和实时路径隔离。

## 实施约束

- USB、Flash、SD 和阻塞日志不得运行在控制或 Estimator WorkQueue。
- DMA Buffer 不从通用 Heap 分配。
- 动态分配失败必须产生可观测 Fault/Event。
- 当前启动和 Flash 分区在阶段 0 不修改。

## 验证方式

- 使用项目正式目标构建入口完成编译和链接。
- 记录 Flash/RAM、任务栈和 Application Slot 余量。
- 后续在目标板检查 Heap 最低余量、任务栈高水位和 WorkQueue deadline。
- 不为本 ADR 新增测试或仿真代码。

## 相关记录

- 总体计划：`../DIMA_ROVER_PORTING_PLAN_ZH.md`
- 来源清单：`../DIMA_SOURCE_MANIFEST.md`
