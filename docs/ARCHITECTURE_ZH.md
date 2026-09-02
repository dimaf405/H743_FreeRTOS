# Dima H743 FreeRTOS Rover 软件架构与边界

## 1. 产品定位

本项目是在 STM32H743 + FreeRTOS 上构建的完整、受控差速 Rover 固件。运行平台、产品装配和硬件适配由 Dima 维护；参数、消息、调度、遥控、状态估计和 Rover 控制优先复用成熟上游模块及数据契约，不从零重复实现同类核心。

直接代码来源以 PX4 v1.17.0 为正式基线，ArduPilot Rover 作为 Arming、Failsafe、轮速和 PivotTurn 等行为参考。目录和产品名称使用 Dima；上游源码中的 namespace、宏、参数名、Topic 名、版权头和许可证文字必须保持可追踪，不进行品牌式全文替换。

Estimator 固定采用 PX4 EKF2。此前涉及 ArduPilot EKF3 的阶段计划均由 `DIMA_ROVER_PORTING_PLAN_ZH.md` 取代。

## 2. 项目代码结构

```text
Dima/                         唯一自研应用根、兼容层和产品装配
├── application/              启动壳、C ABI 入口和 appMainTask
├── adapters/                 MAVLink、USB Console 等外部协议/数据面适配
├── drivers/                  GPS、IMU、磁力计、RC 的设备状态机与配置
├── platform/api/             OS/MCU 无关 capability 与 opaque handle
├── platform/common/          公共 capability 的 MCU/OS 无关实现
├── platform/freertos/        Task、同步、Heap、transaction 与 FatFs 文件后端
│   └── storage/              AtomicFileStore/LogFileStore 的唯一 FatFs volume owner
├── platform/stm32h7/         MCU 基础后端，按总线/中断/资源职责拆分
│   ├── system/ memory/ flash/
│   └── serial/ can/ spi/ interrupts/ pwm/ usb/
├── middleware/               Parameter、uORB、WorkQueue、Event、Perf、Log
│   ├── lifecycle/            Module 生命周期
├── modules/                  具有独立生命周期的运行模块
│   ├── boot_health/          BootHealth 启动健康观察
│   ├── logging/              LogService、Topic ULog producer 与异步文件 writer
│   ├── ekf2/                 单实例 PX4 EKF2 运行模块
│   ├── mavlink/              MAVLink v2.0（心跳、命令、参数、Mission、时间同步）
│   ├── mission/              固定 64 项任务仓库、编解码与持久事务
│   ├── motor/                安全门控后的六路 MotorOutput
│   ├── parameters/           ParameterService 参数持久化与在线管理
│   ├── rc/                   RC 校准、通道映射与手动输入转换
│   ├── sensors/              校准及 vehicle_imu/vehicle_magnetometer 前端
│   ├── serial/               板级串口参数绑定、校验与普通配置应用
│   └── safety/               Commander 安全状态机（Arming、Failsafe、Termination）
├── lib/                      平台无关算法、容器和移植库
│   ├── containers/           平台无关容器
│   ├── protocols/            SBUS、UM982、DroneCAN 纯协议逻辑
│   ├── sensors/              旋转与磁力计纯算法
│   ├── serial/               串口分配只读合同
│   ├── format/               轻量格式化包装
│   ├── rover/                Pure Pursuit、四控制器与 DifferentialDrive 纯算法
│   ├── timesync/             时间同步库
│   └── tinybson/             TinyBSON 编解码
├── messages/                 uORB schema 权威定义，Topic 与 catalog 由工具生成
└── rover/                    唯一 Rover 产品域
    ├── control/              RoverDifferential 消息/参数/安全运行适配
    └── modes/                ManualMode 与 50 Hz AutoMode

Boards/H743/                  板级初始化、Flash 布局、外设和 FatFs SDMMC 适配
Core/                         CubeMX/HAL 应用生成层
Drivers/                      CMSIS 与 STM32 HAL 厂商代码
Middlewares/                  FreeRTOS、MCUboot、ST USB 等第三方代码
USB_DEVICE/                   CubeMX USB Device 集成层
Bootloader/                   独立 MCUboot 固件镜像
Linker/、make/、tools/        链接、构建、签名和升级工具（project/release 分层）
docs/                         计划、架构、ADR、来源和维护文档
```

已退役的顶层 `App/` 已迁入 `Dima/`。C/C++ Runtime 位于 `Dima/platform/freertos/libc`；公共时间契约位于 `Dima/platform/api/Time.hpp`，TIM2 实现位于 `Dima/platform/stm32h7/system/Clock.cpp`。Parameter/Mission 共用的原子 FileStorage 与 FatFs file backend、FatFs disk ABI 与 H743 SDMMC、`flash/` raw Flash、USB Console 与 `usb/` transport 均已拆分。SBUS、UM982、ICM-42688-P 和 DroneCAN 的协议/设备策略位于 `lib/protocols` 与 `drivers`，`platform/stm32h7` 只保留通用总线配置、读写、中断和统计。PX4 v1.17.0 EKF2 已按 N1 单实例闭包落入 `Dima/modules/ekf2` 与 `Dima/lib/{ekf2,matrix,mathlib,geo,lat_lon_alt,world_magnetic_model}`；Pure Pursuit、Speed PI、Heading P、YawRate PI、停车确认和原地转向已有正式运行消费者，Commander 只在任务、参数、EKF、AutoMode 与 Armed 条件同时成立时接受 `MAV_CMD_MISSION_START`。源码和 Windows 构建通过不代表 QGC/SD/目标板/实车动态验收通过。

## 3. 依赖规则

- `Dima/rover` 是唯一 Rover 产品域，可依赖 modules、middleware、messages、lib 和 `platform/api`，但不包含具体后端类型；`Boards/H743/Src/platform_composition.cpp` 是后端组合根。
- `Dima/modules` 承载具有独立生命周期和运行状态的功能块；Parameter、Mission、EKF2、Log、RC、MotorOutput、Commander 与 MavlinkService 均实现统一的 ModuleBase 契约。`RcManualInput` 明确只转换 RC 来源，不代表 Rover 模式或 MAVLink 入口。
- `Dima/rover/modes` 只把通用输入契约转换为 Rover 产品请求；`Dima/rover/control` 只消费 `rover_motion_request`、调用 `Dima/lib/rover` 纯算法并产生双 Motor 命令，二者都不得直接访问 PWM 后端。
- `Dima/lib/rover` 不得依赖 uORB、Parameter、WorkQueue 或 ModuleBase；`AutoMode` 调用 Pure Pursuit、Heading P 与 Driving 状态机，`RoverDifferential` 调用 Speed PI、YawRate PI 与 `DifferentialDrive`。运行接入和目标构建证据不得被描述为板端闭环或轨迹证明。
- `Dima/modules` 和 `Dima/middleware` 禁止反向依赖 `Dima/rover`。
- `application/rover/modules/middleware/messages/lib/adapters` 只允许依赖标准库、内部公共契约和 `Dima/platform/api`，禁止 FreeRTOS、HAL、CMSIS、SCB/NVIC、Core、Board 与 USB 生成头。
- `Dima/platform/api` 只定义整数、尺寸、opaque handle、callback 和窄 capability，不暴露 OS/MCU/厂商类型。
- `Dima/platform/api` 按 Execution、Synchronization、TaskRuntime、Memory、Flash、AtomicFileStore、Console、Serial、Boot、ActuatorPwm 等 capability 分头提供契约；`Services` 只负责安装和取得组合后的 capability，不保留重新聚合全部接口的 umbrella 头。
- `Dima/platform/freertos` 连接公共 capability 与 FreeRTOS；仅 `storage/` 额外连接对象私有的 FatFs API。`Dima/platform/stm32h7` 只连接公共 capability、HAL/CMSIS 和板级定义，二者禁止互相包含。STM32H7 根目录只保留 `HardwareServices.hpp` 工厂声明，具体实现按 `system/memory/flash/serial/can/spi/interrupts/pwm/usb` 下沉。
- 首方 include 只允许 `Header.hpp` 或 `domain/Header.hpp`；两层及以上路径由 target-private include root 收窄，`make check-architecture` 对 Dima、Board、Core、USB、Bootloader 和 tests 强制执行该约束。
- `Boards/H743` 负责 MCU、引脚、DMA、PWM、Flash、总线接线和 FatFs `disk_*` 到 SDMMC/HAL 的板级端口，不依赖上层控制模块。
- `Core/` 与 `USB_DEVICE/` 的生成区只保留必要接线，禁止写入业务逻辑。
- `Core/Src/stm32h7xx_hal_msp.c` 与 `USB_DEVICE/Target/usbd_conf.c` 分别由唯一 `H743_FreeRTOS.ioc` 的 MCU MSP/USB Device 配置集中生成，是当前单文件 600 行上限的必要生成区例外；禁止为追求行数而手工分源，也禁止借此例外加入业务逻辑。
- `Core/Inc/FreeRTOSConfig.h` 只是 CubeMX/FreeRTOS 查找约定所需的转发头，唯一配置实现为 `Dima/platform/freertos/FreeRTOSConfig.h`；STM32H7 capability 工厂统一由 `HardwareServices.hpp` 声明。
- 根目录 `H743_FreeRTOS.ioc` 是唯一 CubeMX 配置源；禁止第二份 `.ioc`、README-only 源码目录、Dima 同名源码文件和未列入 `make/project.mk` 的翻译单元。
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
wq:lp_default    Log、MAVLink 和校准非实时服务（4 KiB 静态栈）
wq:storage       Parameter、Autosave、FlashFS、SD 参数镜像、ULog 写入与日志下载（4 KiB 静态栈）
service:param    参数、Flash 主副本和 SD 镜像
service:mission  PX4 Dataman 双 bank、Mission State、板载 Flash/RAM backend
service:console  USB MAVLink
service:logger   实时结构化日志、SD ULog 与 MAVLink Onboard Log
```

USB、Flash、SD 和阻塞日志不得运行在控制或 Estimator WorkQueue。
`wq:lp_default` 的 Parameter/结构化日志/MAVLink 共用调用链在 QGC 连接时曾以 2 KiB 栈越界至少 56 bytes；栈已固定为 4 KiB，并由 R228 禁止回退。Topic ULog producer 在该队列只向固定 64 KiB SPSC Ring 发布完整消息；运行期 SD 重初始化、ULog 分片写入/同步/关闭、目录枚举和分块读取可能进入同步 HAL/FatFs，因此全部隔离到优先级更低的 `wq:storage`。MAVLink 只交换固定请求/响应 Ring，该队列阻塞不得影响 HEARTBEAT、传感器遥测或实时结构化日志。板上没有 card-detect GPIO，`disk_status` 不能证明物理在位；复用旧挂载前必须执行最长 500 ms 的主动 `CTRL_SYNC`，其成功结果只表示“最近一次探测可用”。八个 WorkQueue 加 `appMainTask` 合计 36 KiB，仍位于固定 48 KiB `.dima_task_pool` 内。

### 4.2 Application Runtime 生命周期

平台资源分为上电期、Application Runtime 和跨 Runtime 保留三类，禁止把三者的 cache、句柄或有效性标志混用：

```text
整次上电：MPU/cache、SysTick、TIM2 HRT、平台 Services、Heap、Flash 互锁、USB transport
每次 Runtime：Console 前端、WorkQueue、uORB Buffer、Parameter Core/Journal、ModuleManager
跨 Runtime：Commander Termination、D3 Fault 记录和明确标注的累计诊断
```

Runtime 初始化顺序固定为 `Console → WorkQueue → uORB → Log structured sink → Parameter Journal/Core → Module registration`；启动主链固定为 `Parameter → Mission → Log → SerialConfig → MotorOutput safe-off → Commander → MavlinkService → RC 链 → Manual/Rover control → BootHealth → IMU/Mag/GNSS → EKF2 → AutoMode → SensorCalibration`。Mission、EKF2 或 AutoMode 失败只锁闭 AUTO；传感器启动失败只降级对应数据源，均不阻断 Manual。关闭时先停止 AutoMode 和 EKF2，再按消费者到生产者及执行器安全顺序逆向释放。

- `ApplicationContext` 只接受 owner task 执行 init/start/shutdown；部分初始化和 Error 状态按成功步骤逆序回滚，清理失败时不得伪装为 Stopped 或重新 init。
- `Param<T>` 构造不访问 Parameter Core；模块每次 start 必须 `bind()`，shutdown 清除 ready、used、unsaved、值 cache、动态 Layer、callback 和运行期同步对象。Journal 下次 initialize/load 必须重新扫描并复验 Header、Commit Marker 和 payload CRC。
- uORB 每次成功 initialize 推进上电期单调 epoch。Publication、Subscription、instance、generation 和 callback 发现 epoch 变化后丢弃旧 Runtime 状态；深度队列从当前最旧有效样本恢复，generation 0 不得复制空槽。
- WorkQueue 由 Runtime owner 创建和关闭；ISR、worker 自身或非 owner 无权销毁。外部 stop 在释放订阅和后端前 cancel-and-drain；Signal 和 task slot 由 shutdown 显式回收，不依赖全局析构器。
- Commander Termination 是唯一跨 Runtime 保留的模块安全状态，只能由 MCU reset 清除；RC 边沿、Failsafe 临时原因、BootHealth 窗口和参数绑定状态每次 Runtime 重建。
- MavlinkService 每次 start/stop 都复位 parser/channel、heartbeat、参数流、Timesync、ACK/reboot 和连接边沿；不得把 GCS 或旧 Runtime 状态带入下一次启动。

### 4.2.1 IWDG 健康所有权

- 应用 IWDG 固定为约 2048 ms，健康检查周期 100 ms，不提供运行期关闭参数。`appMainTask` 是应用侧唯一硬件喂狗 owner；BootHealth 只依据 Parameter ready、安全三 Topic 与 MotorOutput 输出 Topic 的严格时间戳/sequence 进展推进单调 generation，不跨 WorkQueue 读取模块普通状态。运行期维护还必须满足 Disarmed 与 neutral/hard-safe 输出，且 `appMainTask` 完成一次真实 reload 后才激活票据。
- Runtime 完整启动后立即启动或收窄 IWDG；只有观察到新 generation 才喂狗。BootHealth/WorkQueue 停滞、appMain 停滞、Runtime Error 或健康快照不再前进都会停止喂狗。受控 shutdown 只有在 MotorOutput 已确认物理 hard-safe-off 后才允许继续喂狗。
- IWDG 不能由 MCU reset 关闭。MCUboot 入口会把已经运行的应用 watchdog 临时扩展到约 32 s，但不会在普通上电时主动启动一个原本未运行的 watchdog；Recovery、镜像校验、swap、Flash 和 USB 串行长循环统一调用 `boot_watchdog_feed()`。应用 Runtime 建立后先写 start key 启动冷上电时尚未运行的 LSI/IWDG，再写 PR/RLR/WINR 并等待同步，将窗口重新配置为约 2048 ms；跨复位已运行的 IWDG 允许重复 start key。
- `RCC->RSR` 的 IWDG 原始复位原因先写入 D3 启动诊断；MCUboot 的一次性应用桥接软件复位和同次 hot handoff 不得覆盖该原始原因。冷上电、全片擦除或 ROM DFU 后 D3 尚无有效头时，MCUboot 必须先用当前 `RCC->RSR` 建立最小 v2 bridge 记录，再执行软件复位，禁止把“Application 尚未初始化 D3”误判成桥接失败而永久停在 Recovery。调试暂停通过 DBGMCU 冻结 IWDG。上述时限、复位后 TIM/GPIO 电气行为仍属于 `BOARD PENDING`。

### 4.3 Rover 控制与六路输出边界

Manual/AUTO 共用的运行链固定为：

```text
Manual: manual_control_setpoint → ManualMode → normalized axes ┐
AUTO: Mission + vehicle_local_position + vehicle_odometry health
                                                               │
      → AutoMode(Pure Pursuit + Heading P + Driving state)     │
      → speed_m_s + yaw_rate_rad_s                             ├→ rover_motion_request
                                                               └→ RoverDifferential
                                                                  (Speed PI + YawRate PI)
→ actuator_motors（Motor1 右侧、Motor2 左侧）
→ MotorOutput
→ platform::ActuatorPwm
→ TIM8/TIM5 六路普通 PWM
```

- Manual 只允许 `SOURCE_MANUAL + MODE_NORMALIZED_AXES`，AUTO/Hold 只允许 `SOURCE_NAVIGATION + MODE_SPEED_YAW_RATE`；非当前模式的无效帧只更新自己的来源缓存。模式切换后请求必须晚于 `nav_state_timestamp`，任何来源都不得绕过差速混控直接访问 `actuator_motors` 或 PWM。
- Commander 的 `AUTO_MISSION` 固定投影 auto/position/velocity/attitude/rates，`AUTO_LOITER` 保持同一控制标志并在估计健康时发布物理量零请求；Manual、AUTO、Termination 三套逐字段投影在 Commander、RoverDifferential 和 MotorOutput 中精确匹配。MotorOutput 只有在同代安全快照、有效 Motor 命令和无 Kill/Termination/Failsafe 时允许 `ACTIVE`。
- AUTO 外环同时要求 `vehicle_local_position` 的 xy/vxy/heading/global/fresh/non-dead-reckoning 闭包，以及 `vehicle_odometry.angular_velocity[2]` 的有限值、新鲜度和 reset 代际；导航请求的 `timestamp_sample` 取两路估计输入中最旧样本。Pure Pursuit 的动态前视距离使用 NED 水平地速模长 `hypot(v_body_x,v_body_y)`，Speed PI 才使用带车体前向符号的 `sign(v_body_x)×hypot(v_body_x,v_body_y)`；这两个量的物理语义不同，纯横向运动不得把前视距离错误压到最小值。四环原子参数快照还要求 `RO_YAW_RATE_TH(rad/s) < RO_YAW_P × RD_TRANS_TRN_DRV(rad)`，确保 SpotTurning 在进入退出滞回前仍能生成未被死区归零的 yaw-rate 目标；不满足时保持 AUTO 锁闭。任一路 reset、失鲜、样本时间回退或输出无效都先清控制状态，AutoMode 固定先发布同代故障状态、再发布全 NaN 请求，由 Commander 降级 Hold。100 Hz 内环再次复核同一位置/速度/航向/yaw-rate 健康闭包，先计算 steering，再限制 `|longitudinal|≤1-|steering|`；原地转向保持 longitudinal 为零，左右轮换向继续经过 `MOT_REV_DELAY`。
- S1～S6 只允许 Disabled、MotorRight、MotorLeft；默认全部 Disabled，普通 PWM 产品包络统一为 500～2500 us，默认 `MIN/CENT/MAX` 仍为 1000/1500/2000 us。参数协议保留普通有限原值，MotorOutput 在完整 Disarmed 快照中消费校验：未知 `FUNC` 或无效脉宽配置只禁用对应通道，`MIN/MAX` 反序只交换运行时有效端点。普通 Disarmed 在其余有效通道输出各自 `CENT`；至少一右一左仍有效时允许解锁，映射不完整时仅拒绝解锁。无任何有效通道、Kill、Termination、Failsafe、Armed 命令超时、发布失败、后端 Retry/Fault、关闭和 watchdog 复位路径进入 `HARD_SAFE_OFF`，停止 TIM5/TIM8、CCR 清零并拉低六路 GPIO。
- MotorOutput 分离“禁止 ACTIVE”和“必须 HARD_SAFE_OFF”观察锁存：普通 Disarm 的任一新 Topic 立即阻断 ACTIVE，完整一致快照后才允许 neutral；任一 Kill/Termination/Failsafe Topic 先到即同时禁止 neutral。只有当前 Commander 代际之后仍新鲜到达、12 路精确全 NaN 的 `actuator_motors` 帧，在物理停波成功后标记为 `CONTROL_INHIBITED`；旧有限命令、生产者超时、结构错误仍是普通 `HARD_SAFE_OFF`，Retry/Fault 保持独立故障态。Commander 只在 `AUTO_MISSION` 正准备因同代导航故障切 Hold，或 `AUTO_LOITER` 持续收到同 mission_id/count 的参数、EKF、reset、时间回退、非有限输出故障报告时接受该状态并保持 Armed；报告停更、任务/模式错误、后端/映射故障、非零 PWM 和普通 Hard Safe Off 仍强制 Disarm。RC Loss 继续固定 Disarm，Kill 保持 Kill→Disarm，Unkill 不自动重新 Arm，Termination 的不可恢复停波优先级不变。
- `board_init()` 在调度器和产品 Runtime 之前确认 TIM5/TIM8 已停止、CCR 为 0、六路 GPIO 为低；Application shutdown 只有在 MotorOutput 停止且 `safe_off_confirmed()` 成功后才能释放 Runtime 资源。
- BootHealth 除 Commander 三 Topic 外还要求 `actuator_output_status` sequence 严格前进、新鲜且与当前安全状态一致：健康 Disarmed 为合法 `DISARMED_NEUTRAL` 帧，Armed 通常为命令有效的 `ACTIVE` 帧，`AUTO_LOITER` 可接受映射完整、控制失效流仍新鲜且六路全零的 `CONTROL_INHIBITED`，Kill/Termination/Failsafe 只能是六路全零的 `HARD_SAFE_OFF`。镜像确认仍只允许健康 Disarmed、无 Kill/Termination/Failsafe 且输出为 neutral 或 hard-off 的完整 5 秒窗口；确认完成后 BootHealth 继续推进运行期健康 generation。

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
- 参数系统采用 Parameter Core 与显式 bind/update 的 `dima::Param<T>`，参数数量由生成器从权威定义计算，不设置固定运行期上限或需要手工同步的总数门禁。生成器只扫描 Make 显式输入并按参数名排序；GPS、UAVCAN、Magnetometer、Sensors、Sensor Calibration 与 Serial 使用 PX4/QGC group 分类，全部 `CAL_*` 依上游语义标记为 `System` category，offset/scale 同时带 `volatile`、三位小数和有效范围；不保留旧固件 handle、stable-tail、旧键或迁移版本参数，固件目录与公开 Metadata 使用同一顺序。
- 传感器发布分层固定采用 PX4 单实例子集：ICM42688P 驱动发布原始 `sensor_accel/sensor_gyro`，`VehicleImu` 应用 correction、旋转和积分后发布 `vehicle_imu/vehicle_imu_status`；DroneCAN Mag2 驱动只发布原始 `sensor_mag`，独立 `VehicleMagnetometer` 按 device ID 选择匹配校准或 identity correction 后发布 `vehicle_magnetometer`。校准事务持有 arming interlock 时，两个前端直接在 Disarmed 状态应用 correction，不得二次获取同一不可重入互锁；提交与回滚均以同一 `parameter_update.instance` 和 active correction 逐项匹配作为完成握手，在前端确认前不得释放互锁。gyro/accel/mag 校准命令与 QGC `[cal]` 状态机运行于非实时 `wq:lp_default`，与 PX4 Commander worker 的非实时职责一致；项目日志层连 RAW 记录也拒绝实时上下文格式化，因此禁止把校准事务放入 `wq:sensors`。Accel 固定六面稳定采样；Mag 固定六面、每面 0.5 rad 净旋转、7 s 内 40 个空间去重点，共 240 点，scale 范围与 PX4 Metadata/前端统一为 0.1..3.0。
- 单路 USB CDC 的 Application data plane 由 MavlinkService 独占；在线参数使用 MAVLink Classic/Ext 协议。General Metadata 声明 Parameter type 1 和 Actuator type 5，MavlinkService 提供 General/Parameter/Actuator 三个只读 FTP 虚拟文件，不提供目录、写操作或 Event/Peripheral Metadata。Actuator Metadata 只开放六路 PWM 分配与参数编辑，MotorRight/MotorLeft 排除执行器测试；固件不实现 `MAV_CMD_ACTUATOR_TEST` 或 `SERVO_OUTPUT_RAW`。原始 RC 样本新鲜且通道数有效时把校准前 `input_rc` 以 5 Hz 发布为 `RC_CHANNELS`；完全无帧或样本超时才停流，恢复立即发送。PX4 USB/QGC 配置流的单实例子集以 50 Hz `HIGHRES_IMU` 发布校准/旋转后的 `vehicle_imu` 与 `vehicle_magnetometer`（SI/Gauss），并以 25 Hz `SCALED_IMU` 发布第 1 套 `vehicle_imu` 与原始 `sensor_mag`（mG/mrad/s/milliGauss）；GPS 以 5 Hz `GPS_RAW_INT` 发布，1 Hz `SYS_STATUS` 分别表达 gyro/accel/mag/GPS 的 present、enabled 与 health。周期流相互独立，不以 health 位作为发送门禁；均支持不受周期 last-send 限制的 one-shot `MAV_CMD_REQUEST_MESSAGE` 以及 PX4 `SET/GET_MESSAGE_INTERVAL` 语义。固定 PX4 v1.17 没有注册 `RAW_IMU`，因此不把 SI 数据伪装成比例未定义的 raw 消息。MAVLink 手柄、`PARAM_MAP_RC` 和 Offboard 仍不声明。
- EKF2 导航输出通过权威 `mavlink_runtime.yaml` 生成调度合同：`ATTITUDE` 50 Hz、`LOCAL_POSITION_NED` 30 Hz、`GLOBAL_POSITION_INT` 10 Hz、`ESTIMATOR_STATUS` 5 Hz。UM982 的原始 `GPS_RAW_INT` 不以 EKF 融合资格为发送门禁；EKF2 是 `estimator_gps_status` 唯一发布者，`SYS_STATUS` GPS health 要求原始流新鲜且 EKF `checks_passed`，从而把接收机可见性与融合健康分层表达。
- Onboard Log 对照 PX4 v1.17.0：生成链自动从权威 `.msg` catalog 产生压缩字段合同，Logger 定义段依次写 header/Flag Bits、`F`、used 参数 `P` 和 current/system default `Q`；每个 Topic 实例首次数据写 `A`、后续按 `o_size_no_padding` 写 `D`，`mavlink_log` 单独映射为 `L`，并写 500 ms `S` 与 Ring 拥塞 `O`。producer 每 5 ms 在 `wq:lp_default` 有界扫描，64 KiB SPSC consumer 在 `wq:storage` 以最大 4096-byte 分片写 `sessNNN/log100.ulg` 并每 1 s `f_sync`；介质/文件 generation 变化后丢弃旧 Ring、ID 和 Topic generation，从 header 全量重建。`LOG_REQUEST_LIST/DATA/END/ERASE` 由权威 runtime YAML 路由；列表与 90-byte 分片 I/O 在 `wq:storage`，USB owner 每轮最多发送四片。`MAV_CMD_REQUEST_MESSAGE(STORAGE_INFORMATION)` 及 PX4 deprecated 兼容命令使用同一 storage Ring，`f_getfree` 不进入 MAVLink 队列；无卡时回复 `EMPTY/count=0`。无卡/无日志仍发送 `num_logs=0`，板上无 RTC 时不伪造 UTC 日期。
- Parameter 继续以 FlashFS 为主存储、SD 为 generation 镜像；Mission 不再访问 FatFs。Mission 严格采用 PX4 `SYS_DM_BACKEND`：`-1` 禁用、`0` 使用板载 FlashFS、`1` 使用非持久 RAM。上传逐项写入 `DM_KEY_WAYPOINTS_OFFBOARD_0/1` 语义的 inactive bank，每项成功后才请求下一项；最后通过 Mission State 的 FlashFS commit marker 原子切换 active bank，清空任务同样切 bank，`MISSION_SET_CURRENT` 只更新 Mission State。掉电发生在 state commit 前时仍加载旧 bank；无 SD 卡不影响默认 backend 的上传、回读或 AUTO 任务门控。
- 参数扫描不执行整段 cache invalidate；raw Flash 仅在 program/erase 成功后处理实际修改范围，D-cache 关闭时中央 helper no-op。
- 每次 load 都重新验证有效 payload 长度、条目 CRC 和最终 commit；最新记录损坏时回退到更早有效记录。FlashFS 物理格式不兼容 ParameterJournal v1，首次部署必须执行参数导出/迁移，初始化失败不得自动擦除旧扇区。BusFault 仅在活动安全读窗口、分区地址和 Bank 2 DBECC 三条件同时成立时恢复。
- Estimator 固定为 PX4 v1.17.0 单实例 EKF2，绑定 IMU/Mag/GNSS instance 0，永久运行于 `wq:estimator`；不保留 Selector、多实例、`EKF2_EN` 或运行时装卸。编译闭包只启用 GNSS position/height/velocity/yaw、Mag Automatic、Gravity、GSF yaw、bias 和 predictor，排除 Wind、Airspeed、Barometer、Flow、Range/Terrain fusion、EV、Sideslip、Drag、Aux 与 Wheel。AUTO 只消费公开的 `vehicle_local_position` 与 `vehicle_odometry`，不访问 EKF 内部对象；EKF 故障只使 AUTO 降级 Hold/失效请求，不反向门控 Manual、BootHealth、IWDG 或 PWM 安全链。
- Arming 状态与 PWM 外设是否启动分离；RoverDifferential 只发布两路双向 Motor 命令，最终六路输出必须再经过 MotorOutput 的独立 Failsafe、命令新鲜度和板级 safe-off Gate。
- 控制来源包括 RC Manual 与持久任务 AUTO；`COM_RC_IN_MODE` 仍只接受 `0=RC only`，AUTO 不是第二种 RC 输入。`SET_MODE` 只接受 PX4 Manual 与 AUTO Mission 两种 custom mode；其中 AUTO Mission 只是 QGC 兼容入口，与显式 `MAV_CMD_MISSION_START` 共用同一个 Commander 事务，必须先 Armed，再通过任务持久化、参数、EKF 与 AutoMode 门控，绝不隐式 Arm。Armed 期间收到的新控制参数冻结为 pending，不改变正在执行任务的已验证配置；pending 尚未在 Disarmed 状态原子应用和校验时，拒绝新的 Mission Start。`AUTO_LOITER` 只由完成或导航故障进入。首版不含倒车航段、`DO_CHANGE_SPEED`、RTL、避障或地理围栏。RC 丢失策略固定 `NAV_RCL_ACT=6`（Disarm），GCS 丢失策略固定 `NAV_DLL_ACT=0`（Disabled），因此 QGC 断链不终止已冻结任务。
- MAVLink HEARTBEAT 从 Commander 的 `vehicle_status`/`vehicle_control_mode` 投影 PX4 custom mode，准确区分 Manual、AUTO Mission、AUTO Loiter 与 Termination；AUTO 同时设置 `MAV_MODE_FLAG_AUTO_ENABLED`。Mission 协议只接收 `MAV_MISSION_TYPE_MISSION` 与 `MAV_CMD_NAV_WAYPOINT`；`MISSION_ITEM_INT` 上传兼容 QGC 使用的 `GLOBAL/GLOBAL_RELATIVE_ALT` 及对应 `_INT` frame，x/y 始终按消息定义解释为 `1e-7 deg`，进入任务仓库前统一规范化为两种 `_INT` frame。协议支持上传/回读/清空/设置当前、`MISSION_CURRENT` 与一次性 `MISSION_ITEM_REACHED`；到达事件只允许由执行态保持 `Active → Active` 的索引推进产生，Disarm/Manual 后的人工前移不得伪造到达。QGC 对设置当前项先尝试 `MAV_CMD_DO_SET_MISSION_CURRENT(224)`，产品明确返回 unsupported 后由 QGC 回退到 `MISSION_SET_CURRENT`；PX4 同名 `VehicleCommand.msg` 保持与 pinned upstream 逐字一致，不为兼容入口手写命令常量。参数、消息和路由只从权威 YAML/`.msg` 生成。

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
- MAVLink XML 固定为 upstream `33af200d`，pymavlink 固定为 `fcaa2c7d`/2.4.47；`dima.xml` 只 include 固定 common 方言，正式构建直接运行原始 mavgen 生成当前 230-message wire 闭包。运行频率与 handler 由独立 YAML 策略派生，不能反向定义 ID、CRC 或 payload。`make clean` 删除 `build/generated/mavlink` 与 `build/generated/component_metadata`，不读取源码树历史生成物。
- Parameter 的唯一源码定义为通过锁定上游 schema 的 `Dima/middleware/parameters/definitions/module_*.yaml`；串口与 DroneCAN 都直接位于该目录，不生成中间 YAML 或专用参数合同，并统一进入 YAML→中间 C→XML/JSON→Header 链。上游原始暂存头随后机械安装为 `dima_parameters.hpp`，运行时只公开 `dima::params`/`dima::Param<T>`。uORB 的唯一消息输入为 PX4 原生 `.msg`，Topic ID/hash/JSON/注册目录由 PX4 v1.17 原始工具生成。三条链的上游目录和派生产物均由逐文件 SHA-256 门禁闭合。
- `tools/check_architecture.py` 在源码阶段拒绝重复 Rover 根、逆向依赖、越权硬件访问、生命周期契约缺失、非零 PWM compare 和未授权执行器消费者；`tools/verify_application_elf.py` 在最终应用 ELF 上检查向量、ISR 强弱绑定、section 地址/容量、SBUS DMA/CPU Ring、生命周期符号、初始化数组白名单，并要求唯一六路安全 PWM 链的 HAL、board 和 MotorOutput 符号实际链接。源码扫描通过不等于 ELF、目标构建或板测通过。
- VS Code 的 Microsoft C/C++ 插件只使用 `make intellisense` 从真实 Make 配方生成的主机本地 `compile_commands.json`。数据库同时覆盖 Application、MCUboot 和各层私有 include/define；源码清单或编译参数变化后必须重新生成。`.vscode/` 中的编译器绝对路径、数据库、符号索引和主机配置全部保持本地，不纳入源码。
- MCUboot CDC + `mcumgr` 和 ROM USB DFU 恢复链不得因 Dima 重构而改变。
- 参数存储与 MCUboot confirm 共用 Flash transaction 和 Armed/Flash coordinator；锁顺序固定为存储/Journal → transaction → interlock，confirm 使用非阻塞 transaction 并保留 `DEFERRED`。
- MCUboot 可在跳转前关闭 cache，每个应用镜像必须自行且幂等地重建 MPU/cache 契约。
- 启动诊断 v2 增加 ABFSR、SCB CCR、MPU CTRL 和上一故障 ABFSR；Flash record 仍为 256 bytes，读取工具兼容 v1/v2。

目录迁移已完成。2026-07-30 已在 Windows 本地使用 GNU Make 4.4.1、Arm GNU Toolchain 16.1.0 和 binutils 2.47 执行正式项目入口，目标编译、链接、签名、MCUboot 地址一致性和 Factory HEX 验证通过；目标板运行行为仍需板测。项目不新增 Host Test、SITL 或仿真入口：

2026-08-19 又在 Windows 原生进程中使用 GNU Make 4.4.1 与项目缓存 Arm GCC 10.3.1 对当前 SBUS→PWM→电机安全链执行 `make clean` 后完整 `make -j4 NO_COLOR=1 dima_rover`，通过 225 文件架构门禁、Application/MCUboot 编译链接、签名、Factory HEX、`0x08040400`、两份 ELF 未解析符号以及 SBUS/Commander/MotorOutput/IWDG/MCUboot-feed 符号门禁。未签名 Application BIN 固定为 246096 bytes，MCUboot BIN 固定为 48100 bytes；同一 image digest `601c65353ffebce20cea8f040d975000e3c4caa2b27aaa15dd409529740e26ce` 的两次合法 ECDSA P-256 重签分别产生 247271/247270-byte Signed BIN 和 710861/710859-byte Factory HEX。imgtool 使用可变长度 DER 签名，因此 Signed/Factory 文件长度和文件 SHA-256 不是确定性构建合同，验收以未签名 ELF/BIN、image digest、签名校验、地址布局和未解析符号为准。该结果仍不替代目标板电气、波形、复位时限和车辆行为验收。

同日 ROM DFU 首启实板暴露并修复两项启动缺陷：全片擦除后无有效 D3 头会令 MCUboot 应用桥接永久停在 Recovery；应用 IWDG 又曾在写 start key 前等待 SR 同步，导致冷启动 LSI 未运行时 `start(2048)` 主动 panic。真实持久记录为 `ERROR_HANDLER / APPLICATION_RUNNING / stacked_r0=2048 / CFSR=HFSR=ABFSR=0`。修复后的 Windows 原生 clean build 再次通过 `[212/212]`，Application `233772/12284/356176` bytes，MCUboot `47864/380/10192` bytes、BIN 48252 bytes，image digest `835947050b2df9062be4289bcb5b876abe0dee99b46792051827e79f123eeec3`；实板完成 MCUboot→bridge reset→Application 双枚举，并由只读 preflight 识别为 `Dima Rover MAVLink`。实际 IWDG 复位时限和 PWM/GPIO 行为仍属于 `BOARD PENDING`。

同日继续把 STM32H7 扁平后端收敛为 `system/memory/flash/serial/io` 五个真实职责目录；`HardwareServices.hpp` 仍只是工厂声明，唯一组合根没有离开 `Boards/H743`，`flash/flash_fault.h` 则成为 Core 弱钩子与 Flash 强实现共同包含的唯一 C ABI 声明。迁移后 Windows 原生 clean build 通过 `[230/230]` 和 261 文件架构检查，Application/MCUboot 两份 BIN、program headers、定义符号表及参数/Metadata 生成树均与迁移前逐字节一致；仅 ELF 的 DWARF 源路径随目录变化。`make intellisense` 已同步为新路径，本次没有执行新的目标板电气或车辆行为验证。

```powershell
make verify
```

操作和恢复要求见 [MCUboot USB 升级与恢复手册](MCUBOOT_USB_RECOVERY_ZH.md)。完整迁移路线见 [Dima Rover 移植计划](DIMA_ROVER_PORTING_PLAN_ZH.md)，阶段 0 实测资源见 [Dima Rover 资源基线](DIMA_RESOURCE_BASELINE_ZH.md)，当前执行器链资源和板测边界见 [阶段 5 资源与验收基线](DIMA_PHASE5_RESOURCE_BASELINE_ZH.md)。
