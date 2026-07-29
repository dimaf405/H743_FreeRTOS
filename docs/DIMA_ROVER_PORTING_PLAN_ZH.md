# Dima FreeRTOS Rover 总体移植计划

- 状态：阶段 0、阶段 1 已完成；阶段 2 待执行
- 日期：2026-07-29
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
- Dima 目录只负责产品组织和平台适配，不通过全文替换改变上游源码身份。
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

规划目录：

```text
Dima/
├── platform/freertos/
├── middleware/
│   ├── parameters/
│   ├── uorb/
│   ├── work_queue/
│   ├── events/
│   ├── perf/
│   └── logging/
├── modules/
│   ├── rc/
│   ├── safety/
│   ├── rover/
│   └── estimator/ekf2/
├── lib/
│   ├── estimator/
│   ├── math/
│   ├── pid/
│   ├── slew_rate/
│   └── rover_control/
├── messages/
└── product/rover/
```

当前 `App/domain/rover_control` 作为迁移期实现保留并冻结，不继续加入 Parameter、SBUS、Arming、Estimator 或 Position 功能。新模块链达到对应实车里程碑后，再逐项替换旧链。

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

阶段 0 的实测结果记录在 [Dima Rover 资源基线](DIMA_RESOURCE_BASELINE_ZH.md)。应用 ELF/BIN 已重新生成；完整签名链当前受 `build/host-python` 的 WSL/Windows 目录重命名权限阻塞，必须作为独立构建问题处理。

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

### 阶段 1：Dima FreeRTOS 平台兼容层（已完成）

已建立受控 Heap、TIM2 `hrt_absolute_time()`、持久 ApplicationContext、WorkQueue、uORB、events、perf 和 logging。生产 heartbeat 已迁移到 uORB；BootHealth、HelloWorld 和日志服务已迁移到 Dima WorkQueue。目标固件、签名镜像和 Factory HEX 已通过验证，板上 HRT Overflow、栈高水位和运行期 Heap 余量仍需人工验收。

### 阶段 2：Parameter 与 ModuleParams

以 PX4 Parameter 为唯一参数系统，移植 `PARAM_DEFINE_*`、`param_find/get/set/reset`、`px4::Param<T>`、`ModuleParams`、`DEFINE_PARAMETERS` 和 `parameter_update`。参数数量由生成结果决定，不设置 64 项上限。接入内部 Flash 和 USB 在线命令：

```text
param show
param get
param set
param save
param reset
param status
```

### 阶段 3：SBUS、RCUpdate 与 ManualControl

移植成熟 SBUS parser 和 SbusRc 接收逻辑，使用 STM32 UART RXINV 与循环 DMA 适配；接入 18 通道校准、MIN/MAX/TRIM/DZ/REV、功能映射、失联判断、ManualControl 和 Action Request。SBUS 模块不得直接驱动 PWM。

### 阶段 4：Commander Rover 子集

建立 `action_request → vehicle_status / vehicle_control_mode / actuator_armed`，实现 Rover 所需的 Arming、Disarming、Emergency Stop 和统一 Failsafe。Manual 模式不强制位置有效；Position/Auto 模式按需检查 Estimator。

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

## 6. 阶段 0 验收条件

- 计划、Source Manifest 和 ADR 内容一致。
- EKF2 被明确记录为最终 Estimator 选择，旧 EKF3 计划失效。
- 正式 PX4 v1.17.0 与本地 1.16 预研快照被清楚区分。
- ArduPilot 参考 commit 被记录。
- 许可证状态为 `PENDING`，来源保留和发布限制明确。
- 阶段 0 不导入 EKF2、SBUS 或控制模块生产源码。
- 不修改当前 Flash 地址、启动流程或迁移期控制代码。
- 不创建测试、SITL 或仿真代码。
- 不覆盖或回退工作区中已有的其他修改。
