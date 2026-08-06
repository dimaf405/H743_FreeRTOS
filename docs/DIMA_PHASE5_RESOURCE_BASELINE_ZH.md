# Dima 阶段 5 差速执行器链资源与验收基线

日期：2026-08-06

状态：源码、Windows 原生目标构建和镜像门禁通过；目标板电气、时序与车辆行为验收待完成。

## 1. 完成范围与控制链

阶段 5 只建立 Rover Manual 两轴控制与六路普通 PWM 安全输出链：

```text
manual_control_setpoint
→ ManualMode
→ rover_motion_request（前后、左右两轴）
→ RoverDifferential / DifferentialDrive
→ actuator_motors（Motor1 右侧、Motor2 左侧）
→ MotorOutput
→ platform::ActuatorPwm
→ S1～S6 / TIM8 + TIM5
```

- USB CDC 已收敛为系统调试日志与维护命令口，不再周期打印 HelloWorld，也不再发布示例心跳。唯一编译期 `debug_config.hpp` 默认把 SBUS 限制为 Error；100 ms 最新样本入口保留但默认不输出，后续由 MAVLink 接入重新定义诊断传输边界。ICM42688 仅预留关闭的策略入口。
- Manual 输入只保留 `longitudinal` 和 `steering/yaw` 两轴；`actuator_motors` 只使用 Motor1、Motor2，其余十路保持 NaN。
- MotorOutput 只允许 Disabled、MotorRight、MotorLeft 三种通道功能，不导入通用 Mixer、MixingOutput、FunctionMotors 或 ActuatorOutput 框架。
- 本阶段没有增加 FreeRTOS Task；ManualMode 复用 `wq:hp_default`，RoverDifferential 复用 `wq:rate_ctrl`，MotorOutput 复用 `wq:io`。

## 2. 上游来源与许可证边界

阶段 5 的直接代码和消息契约来源固定为 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`：

- `msg/versioned/ActuatorMotors.msg` 对应本地 `actuator_motors` 完整公开契约。
- `src/modules/rover_differential/RoverDifferential.cpp`、`DifferentialDriveModes/DifferentialManualMode/` 和 `DifferentialActControl/` 分别映射到本地 `Dima/rover/control/RoverDifferential.*` 运行适配、`Dima/rover/modes/ManualMode.*` 产品模式和 `Dima/lib/rover/DifferentialDrive.*` 纯差速算法；Motor 命令边界仍由消息隔离。
- PX4 actuator function/output 只保留本项目需要的 MotorRight/MotorLeft 到六路 PWM 的固定存储子集。

ArduPilot commit `3f2e4763accb` 的 `libraries/AR_Motors/AP_MotorsUGV.cpp` 仅作为 Rover 行为参考，用于倒车时车头方向、转向与油门饱和优先级、slew、`MOT_THR_MIN` 静摩擦补偿、反向推力不对称和左右独立换向延时；没有把 GPL 源码复制进本目录。

PX4 来源文件继续保留原始版权头。当前许可证状态仍为 `PENDING`，最终产品分发与 Notice 收敛为 `DEFERRED`；这两个状态不应被目标构建通过替代。

## 3. 两轴控制与 Navigation 预留接口

`rover_motion_request` 是 Manual 与未来 Navigation 唯一允许进入 Rover 控制层的边界：

- 当前生产者固定发布 `SOURCE_MANUAL + MODE_NORMALIZED_AXES`。
- 接口预留 `SOURCE_NAVIGATION + MODE_SPEED_YAW_RATE`，但阶段 5 不创建空 Navigation 模块，也不接受该组合驱动输出。
- 阶段 9 Navigation 必须发布 `rover_motion_request`，不得直接依赖 DifferentialDrive、`actuator_motors`、MotorOutput 或板级 PWM。

中心双向油门保持 Rover 语义：零命令对应停车中心点，负值表示反向，正值表示正向。Manual 倒车转向由 `RD_REV_STEER` 控制；混控先应用 `RD_STR_THR_MIX` 饱和优先级，再应用最小非零输出、最大输出、slew、推力曲线、反向不对称、独立换向延时和解锁 ramp。所有中间值都要求有限并限制在归一化范围内。

## 4. 六路 PWM 电气契约与参数

固定物理映射如下：

| 逻辑口 | MCU 引脚 | 定时器通道 |
|---|---|---|
| S1 | PB0 | TIM8_CH2N |
| S2 | PB1 | TIM8_CH3N |
| S3 | PA0 | TIM5_CH1 |
| S4 | PA1 | TIM5_CH2 |
| S5 | PA2 | TIM5_CH3 |
| S6 | PA3 | TIM5_CH4 |

TIM8 与 TIM5 的目标配置均为 240 MHz 输入、prescaler 239、1 MHz 计数、ARR 19999 和 50 Hz 周期。TIM8 Update 作为 TRGO，TIM5 使用 ITR3 Reset slave，使两组计数器共享帧起点。S1/S2 使用互补输出，极性由 TIM8 N 通道配置决定。

每路公开 `PWM_Sn_FUNC/MIN/CENT/MAX/REV`：

- 默认 `FUNC=Disabled`，因此默认配置即使车辆 ARMED 也不产生物理 PWM。
- 默认 `MIN/CENT/MAX=1000/1500/2000 us`，有效配置必须满足 `MIN < CENT < MAX`，参数元数据允许 800～2200 us。
- 零命令严格映射到 `CENT`；负向在 `CENT→MIN` 区间映射，正向在 `CENT→MAX` 区间映射，`REV` 在通道映射前反转归一化方向。
- 同一个 Motor function 可以映射到多个物理口；未启用通道的有效掩码和脉宽必须保持 0。

阶段 5 新增 41 项参数：10 项两轴/油门保护参数、六路各 5 项 PWM 参数，以及 `COM_ACT_LOSS_T`。`COM_ACT_LOSS_T` 默认 0.10 s、范围 0.02～1.00 s，同时约束 `actuator_motors.timestamp` 和 `timestamp_sample`；任一时间戳为零、未来值、顺序反转或超时都禁止输出。后续 SBUS 电气收敛删除手动极性参数，当前参数总数由 177 降为 176。

## 5. 生命周期与安全门控

Application Runtime 的相关启动顺序固定为：

```text
Parameter → Log → MotorOutput safe-off → Commander
→ SbusRc → RCUpdate → RcManualInput
→ ManualMode → RoverDifferential → BootHealth
```

关闭时先停 BootHealth 和 RC 链，再让 MotorOutput 停止 TIM5/TIM8、清零 CCR 并把六路恢复为 GPIO 低电平，然后停止 RoverDifferential、ManualMode、Commander、Log 和 Parameter。`board_init()` 在调度器与 Runtime 启动前也会确认相同的物理 safe-off。

MotorOutput 只有同时满足以下条件才允许启动或维持 PWM：

1. Commander 的 `actuator_armed → vehicle_control_mode → vehicle_status` 是同时间戳、完整、严格前进且新鲜的快照。
2. 状态为 `ARMED + MANUAL + !kill + !termination + !lockdown + !failsafe`。
3. Motor1/Motor2 命令有限、归一化且 reversible flags 完整。
4. 发布和采样时间都不超过 `COM_ACT_LOSS_T`。
5. 参数快照有效，板级后端 ready，并且至少一个物理通道已配置功能。

Disarm、Kill、Termination、Failsafe、非 Manual、命令超时、参数无效、发布失败和后端 Retry/Fault 都进入 fail-closed safe-off。负向安全字段单 Topic 到达即可立即抑制输出；从不安全状态恢复时必须等待下一组完整、严格更新且同时间戳的 Commander 快照。

BootHealth 除 Commander 三 Topic 外还要求 `actuator_output_status` 持续前进、年龄不超过 250 ms、后端 ready、状态为 SAFE_OFF，并且 active mask 与六路脉宽全为 0。ARMED、Active、Kill、Termination 或 Failsafe 都重新开始完整 5 秒镜像确认窗口；默认全 Disabled 时发生过 ARMED 也同样重置窗口。

## 6. Windows 原生构建与阶段增量

阶段 5 起点为 `3568b84`。该提交已在独立临时 worktree 使用 Windows 原生进程、Windows 路径和 Windows ARM 工具链执行 clean build；阶段 5 最终状态使用相同边界执行 clean verify。WSL 只负责发起命令，不作为编译验收环境。

| 资源 | 阶段 5 起点 | 当前收敛 | 增量 |
|---|---:|---:|---:|
| 生成参数 | 136 | 176 | +40 |
| 一方源文件 | 184 | 193 | +9 |
| Application text | 166596 | 188912 | +22316 bytes |
| Application data | 12044 | 12212 | +168 bytes |
| Application bss | 356016 | 359584 | +3568 bytes |
| Signed App | 179855 | 202339 | +22484 bytes |
| MCUboot BIN | 47884 | 47884 | 0 bytes |
| `.init_array` | 13 | 15 | +2 项 |

最终镜像与布局门禁结果：

```text
Factory HEX                 602222 bytes
Application vector         0x08040400
Application Flash usage    188720 bytes
DTCM usage                  58356 bytes
SRAM usage                  313632 bytes
DMA usage                   2112 bytes
D3 diagnostics              224 bytes
Image digest                ce85eafb4351952aef1ab10a4bcdfaad90bf259da191f1b2fe7441edd4e54a04
```

Signed App 占 768 KiB Primary Slot 约 25.73%，低于 85% 发布控制线。MCUboot 大小未变化；构建已检查应用/MCUboot 无未解析符号、签名、Factory HEX、Slot 地址、向量和镜像验证。

## 7. uORB、初始化数组与任务资源

阶段 5 起点 uORB 启动 Buffer 为 2496 bytes，最终为 5280 bytes，净增 2784 bytes：

| Topic 变化 | 结构大小 | 队列深度 | 四实例 Buffer 变化 |
|---|---:|---:|---:|
| 移除 `app_heartbeat` | 16 | 1 | -64 bytes |
| 新增 `actuator_motors` | 72 | 1 | +288 bytes |
| 新增 `rover_motion_request` | 40 | 8 | +1280 bytes |
| 新增 `actuator_output_status` | 40 | 8 | +1280 bytes |

`.init_array` 从 13 项变为 15 项：仍为 2 个工具链项、1 个无平台访问的 Parameter ConstLayer 构造，uORB metadata registrar 因移除一个示例 Topic、增加三个阶段 5 Topic，从 10 个净增到 12 个。

阶段 5 没有新增 FreeRTOS Task，`.dima_task_pool` 继续固定为 48 KiB。控制模块复用既有 WorkQueue；启动期 Topic Buffer 继续由受控 D1 Heap 分配，控制和 ISR 热路径不得动态分配。

## 8. 静态与目标 ELF 门禁

- 架构扫描要求唯一控制链和六路映射，禁止底层反向依赖 `Dima/rover`，并拒绝非零初始 compare、额外执行器消费者及重复 Rover 根。
- 目标 ELF 要求 HAL PWM、`board_motor_pwm_start/write/stop`、MotorOutput start/safe-off 和平台 `ActuatorPwm` 符号实际链接，同时继续拒绝 Mixer、MixingOutput、FunctionMotors 和通用 ActuatorOutput。
- SBUS 门禁要求 `RC_INPUT_PROTO=0/2`、自动 `100000/8E2 + RXINV + pulldown`、UART/FIFO/RX GPIO 事务恢复和任务上下文单次故障日志；UART/DMA ISR 后端不得格式化日志或写 USB。
- 启用 SBUS Debug 后，连续数据只能由低优先级 LogService 通过 `input_rc` 输出，周期不得小于 100 ms；最坏 18 路记录为 209 bytes，低于固定 256-byte 格式缓冲。默认 Error 策略不产生该数据，后续传输边界交由 MAVLink 方案收敛。
- MotorOutput、ApplicationContext 和 BootHealth 热路径没有 `new`、`malloc` 或 `free` 调用。目标文件中编译器为虚析构生成的 deleting-destructor `operator delete` 重定位不在 `Run()` 调用路径。
- 应用向量保持 `0x08040400`，`.dima_task_pool`、`.dima_dma`、D3 diagnostics、SysTick/TIM2 ISR、TIM12 非 HAL tick 和生命周期符号继续由目标 ELF 门禁检查。
- 没有新增 Host Test、Mock、Fixture、测试目标、SITL 或仿真入口。

## 9. 证据边界与目标板待验收项

本文件证明当前源码通过静态检查，并可由 Windows 原生工具链完成编译、链接、签名、Factory HEX 和目标 ELF/镜像门禁。它不证明以下目标板事实：

1. 六路实际频率均为 50 Hz，TIM8/TIM5 帧起点和更新相位符合设计。
2. 默认和边界配置实际产生 1000/1500/2000 us 脉宽，且 TIM8 N 通道极性正确。
3. 冷启动、Runtime 启动失败和 stop 后六路真实保持低电平，没有窄脉冲或残余输出。
4. Arm/Disarm、Kill/Unkill、Termination、Failsafe、RC 丢失和命令超时波形符合 fail-closed 行为。
5. 油门中位、正反向、倒车转向、静摩擦补偿、换向延时、slew、解锁 ramp 和左右多路映射符合实车驱动器要求。
6. USB 日志在高负载、断开、Busy 和 Ring 满时不阻塞控制 WorkQueue。
7. 四个候选口自动进入原始反相 SBUS 电气状态，禁用、失败回滚和 Runtime shutdown 后真实恢复普通 UART。
8. 未接接收机时不再产生周期性后端故障洪泛；真实 UART/DMA 故障只记录一次，收到有效帧后才开启下一故障周期。
9. 显式启用 SBUS Debug 时，USB 上的 18 路 SBUS 数据不超过 10 Hz，断开不影响 RC/控制调度，重连不补发历史样本；MAVLink 接入后按新传输契约重新验收。
10. 既有 HRT、Parameter、Runtime restart、SBUS/RC、Flash/Arm 互锁和 ICM42688 板级待验项目。

完成逻辑分析仪、示波器和目标车人工验收之前，阶段结论只能表述为“源码及目标构建通过，板测待完成”。
