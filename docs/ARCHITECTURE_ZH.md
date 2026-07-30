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
│   ├── messaging/            Topic 兼容接口
│   └── scheduling/           兼容调度实现
├── modules/                  boot_health、hello_world、RC、安全、Rover、EKF2
├── lib/                      motor、rover_control 与公共算法库
├── messages/                 共享消息契约
└── product/rover/            产品配置、模式和装配

Boards/H743/                  板级初始化、Flash 布局和外设适配
Core/                         CubeMX/HAL 应用生成层
Drivers/                      CMSIS 与 STM32 HAL 厂商代码
Middlewares/                  FreeRTOS、MCUboot、ST USB 等第三方代码
USB_DEVICE/                   CubeMX USB Device 集成层
Bootloader/                   独立 MCUboot 固件镜像
Linker/、make/、tools/        链接、构建、签名和升级工具
docs/                         计划、架构、ADR、来源和维护文档
```

已退役的顶层 `App/` 已迁入 `Dima/`：启动壳位于 `Dima/application`，功能模块位于 `Dima/modules/{boot_health,hello_world}`，外部适配位于 `Dima/adapters`，兼容运行时位于 `Dima/middleware/{lifecycle,messaging,scheduling}`，C/C++ Runtime 与平台时间位于 `Dima/platform/freertos/libc` 和 `Dima/platform/freertos/platform_time.*`，Motor 与 Rover Control 位于 `Dima/lib/{motor,rover_control}`。已退役的顶层 `App/` 不再是当前架构根；目录迁移后的结构检查已通过；构建和运行验证待工具链恢复后执行。

## 3. 依赖规则

- `Dima/product/rover` 是最终产品装配层，只装配所需的 Dima 和上游兼容模块。
- `Dima/modules` 可依赖 Dima middleware、messages、lib 和明确的平台适配接口，不直接包含 STM32 HAL 全局句柄。
- `Dima/lib` 保持算法属性，不依赖 HAL、USB、MCUboot 或具体板卡。
- `Dima/middleware` 直接拥有 `lifecycle`、`messaging`、`scheduling` 兼容运行时及 Parameter、uORB、WorkQueue、Event、Perf、Logging，不再依赖顶层 `App`。
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
6. `Dima/application/app_main.cpp` 进入 `Dima/product/rover/ApplicationContext`，由产品装配根初始化 USB、运行时服务和产品模块。

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

- 生产消息接口采用 uORB 兼容 Publication/Subscription；`app_heartbeat` 使用生产 uORB 链，`Dima/middleware/messaging/topic.hpp` 仅作为兼容接口或 Host seam 保留。
- 参数系统采用 PX4 Parameter + ModuleParams，参数数量由生成器产生，不设置固定 64 项上限。
- 在线参数通过 USB，后续增加 MAVLink；Flash 写入由非实时服务执行。
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
- MCUboot CDC + `mcumgr` 和 ROM USB DFU 恢复链不得因 Dima 重构而改变。

目录迁移后的结构检查已通过；目标编译、链接、签名和镜像一致性待工具链恢复后验证。正式验证必须使用项目构建入口，不新增测试、SITL 或仿真入口：

```bash
make firmware GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
```

操作和恢复要求见 [MCUboot USB 升级与恢复手册](MCUBOOT_USB_RECOVERY_ZH.md)。完整迁移路线见 [Dima Rover 移植计划](DIMA_ROVER_PORTING_PLAN_ZH.md)，阶段 0 实测资源见 [Dima Rover 资源基线](DIMA_RESOURCE_BASELINE_ZH.md)。
