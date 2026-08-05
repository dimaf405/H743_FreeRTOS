# Dima 阶段 4 Commander 资源与验收基线

日期：2026-08-03

状态：目标构建通过，目标板行为验收待完成。

## 1. 完成范围

阶段 4 已建立以下安全状态链：

```text
manual_control_setpoint + action_request
→ Commander（wq:hp_default）
→ actuator_armed
→ vehicle_control_mode
→ vehicle_status
```

- 冷启动为 `DISARMED + MANUAL`；无新鲜 RC 时预解锁检查失败，但冷启动本身不置 Failsafe。
- Arm 要求 Parameter Core 和 Commander 参数有效、Manual 模式、RC 新鲜有效、throttle/yaw 有限且位于 `COM_ARM_STICK_DZ` 中位范围内，并且未 Kill/Termination。
- Action Request 深度保持 8；Commander 使用循环逐条消费，并在每次状态变化后按 `actuator_armed → vehicle_control_mode → vehicle_status` 发布。
- RC/参数故障在 ARMED 时强制 Disarm 并置 Failsafe；恢复只清除可恢复原因，不自动 Arm。
- Kill/Unkill 可逆且不单独置 Failsafe；Termination 进入内部终止模式并锁存到 MCU 重启。
- 用户只可选择 Manual；Position、Auto、Offboard、VTOL 和其他未实现模式全部拒绝。
- Parameter Flash 保存和擦除在 ARMED 期间禁止；Autosave 保持 pending，并在 Disarm 后重试。
- 本阶段没有 PWM、Mixer、RoverDifferential 或 HAL 执行器消费者，车辆仍不能运动。

## 2. 构建结果与阶段增量

阶段 4 前从 `feature/dima-phase3`、commit `cd9b41f` 执行：

```text
make clean && make -j4 verify
```

实测基线为 Application `132460/7600/330832` bytes，Signed BIN `141275` bytes，MCUboot `45568/380/9788` bytes，应用向量 `0x08040400`。计划草案中的 Signed BIN `141274` 与实测相差 1 byte，本文件以实测为准。

Commander 接入后的目标构建结果：

```text
Application（text/data/bss）  136956 / 7660 / 331912 bytes
ELF                            1,883,968 bytes（含调试段）
BIN                              144,656 bytes
Signed BIN                       145,830 bytes
Factory HEX                      455,592 bytes
MCUboot（text/data/bss）        45568 / 380 / 9788 bytes
MCUboot BIN                       45,956 bytes
应用向量地址                  0x08040400
```

相对阶段 4 前基线：

| 资源 | 基线 | 阶段 4 | 增量 |
|---|---:|---:|---:|
| Application text | 132460 | 136956 | +4496 |
| Application data | 7600 | 7660 | +60 |
| Application bss | 330832 | 331912 | +1080 |
| Signed BIN | 141275 | 145830 | +4555 |

Signed BIN 占 768 KiB Application Slot 约 18.5%，低于 85% 控制线。构建已通过链接、签名、Factory HEX 合并、MCUboot 地址布局和 `0x08040400` 向量检查。

## 3. 调度、静态对象与 uORB 资源

- Commander 不创建 FreeRTOS Task，复用现有 `wq:hp_default` 2,048-byte 静态栈；七个 WorkQueue 栈总量不变。
- Commander 静态对象目标 ABI 大小为 400 bytes；状态机热路径不动态分配。
- 新增三个 Topic 的静态 runtime instance 数组共增加 672 bytes BSS；metadata、registrar 和初始化表合计对应 60 bytes data 增量。
- uORB 启动分配器仍从固定 D1 Heap 分配消息 Buffer；三个 Topic 的四实例 Buffer 共增加 480 bytes。

| Topic | 结构大小 | 队列深度 | 单实例 Buffer | 四实例启动分配 |
|---|---:|---:|---:|---:|
| `actuator_armed` | 16 | 1 | 16 | 64 |
| `vehicle_control_mode` | 24 | 1 | 24 | 96 |
| `vehicle_status` | 80 | 1 | 80 | 320 |
| 合计 | 120 | — | 120 | 480 |

阶段 3 的全部 Topic Buffer 为 2,016 bytes；阶段 4 增加后为 2,496 bytes。`action_request` 仍为 16-byte 结构、深度 8、四实例共 512 bytes，不因 Commander 接入扩大。

## 4. 静态验收

- 参数生成器输出 136 项参数；`COM_ARM_STICK_DZ` 的默认值、`0.00～0.50` 范围和 metadata 已进入 XML、JSON、C++ 枚举及最终 ELF。
- Commander 对 Action Request 使用 `while (copy())`，不是只读取队尾最新值；Arm、Disarm、Toggle、Kill、Unkill、Termination、Manual 模式和未知动作均有显式分支。
- 三个状态 Topic 和 Commander 符号均已链接进入最终 ELF；发布调用顺序固定为 armed、control mode、vehicle status。
- Commander 源码及 Run/状态转换调用路径不调用 `new`、`malloc`、`free`、PWM、Mixer、RoverDifferential 或 HAL 输出。由于公共 `WorkItem` 基类具有虚析构，目标文件仍带有编译器生成的 deleting-destructor `operator delete` 重定位；它不在 Commander Run/状态转换调用路径中，也与其他现有 WorkItem 子类一致。
- 没有新增 Host Test、Mock、Fixture、测试目标、SITL 或仿真入口。

## 5. 证据边界与待办

本文件只证明目标源码可编译、链接、签名并满足静态镜像布局；不证明以下目标板运行行为已经通过：

1. 冷启动无 RC 时保持 DISARMED、非 Failsafe。
2. throttle/yaw 任一不在中位时拒绝 Arm。
3. 正常 Arm、Disarm 和 Toggle 幂等行为。
4. ARMED 时 RC 失联强制 Disarm，恢复后不自动 Arm，Arm 开关必须重新产生 OFF→ON 边沿。
5. Kill/Unkill 在 ARMED 状态下保持可逆，并正确限制未来执行器许可。
6. Termination 锁存、Failsafe 和模式拒绝行为。
7. ARMED 期间人工保存、擦除和 Autosave 延后，Disarm 后重试。
8. 阶段 1～3 尚未完成的 HRT、Parameter、SBUS 和 RC 链目标板验收。
9. 全阶段 PWM 保持未启动。

完成上述人工项目之前，阶段状态必须保持“目标构建通过、板测待完成”。

## 6. 全量回归修复追加基线

2026-08-03 在阶段 4 之后完成了六批源码修复：

- 时基收敛为 `SysTick + TIM2`。SysTick 统一 HAL/FreeRTOS 1 ms tick，TIM2 专职 1 MHz 32 位 HRT 和 64 位单调扩展，TIM12 已释放。
- WorkQueue 周期以前一截止时间锁相并跳过错过周期；停止使用持久调度屏障和 cancel-and-drain，外部 stop 等待 `Run()` 返回，self-stop 不自等待。
- Arm 与 Parameter/MCUboot Flash 写统一通过原子互锁；Commander/uORB 发布失败、新鲜 Action Request 和连续 BootHealth 已按 fail-closed 处理。
- SBUS UART/DMA 使用真实到达时间并逐帧发布；`COM_RC_LOSS_T` 下界和 Arm/Kill 端点比较已校正。
- SPI4 固定 8-bit、约 7.5 MHz 和软件 GPIO CS；PE10/PE15 rising EXTI 只记录 HRT 时间并通知 WorkItem，本轮不导入完整 IMU 驱动。
- Parameter Journal 每次 load 复验 Header、Commit Marker 和 payload CRC；最新记录损坏时重扫并回退上一条有效快照。

以第四批 RC 修复后的 `348c706` 为本轮代码基线，应用从 `140132/7668/332552` bytes、Signed BIN `149015` bytes变为：

```text
Application（text/data/bss）  141428 / 7668 / 332584 bytes
Signed BIN                       150311 bytes
MCUboot（当前工作区）             46060 bytes
应用向量地址                  0x08040400
```

本轮应用增量为 text `+1296` bytes、data `0`、bss `+32` bytes、Signed BIN `+1296` bytes；Signed BIN 占 768 KiB Slot 约 19.1%，仍低于 85% 控制线。MCUboot `46060` bytes 包含保留在工作区、未混入本轮提交的 Recovery 请求改动，不能归因于上述应用修复。

`make -j4 verify`、链接、签名、Factory HEX、MCUboot 地址校验、`0x08040400` 向量检查和 `git diff --check` 已通过。以下项目没有由目标构建证明，仍必须在目标板执行：HAL/FreeRTOS tick 同速、TIM2 加速回绕、10/20/500 ms 抖动统计、Flash/Arm 并发、损坏快照回退、stop/run 竞争、RC DMA 积压，以及 ICM42688 的 SPI/CS/INT 示波器检查。当前结论仍为“源码及目标构建通过，板测待完成”。

## 7. 生命周期收敛后的验证状态

日期：2026-08-05

状态：生命周期修复源码及架构静态门禁通过；Windows 原生 clean build、最终 ELF 门禁和目标板行为均待完成。本节不覆盖第 2、6 节的历史构建记录，也不把历史镜像尺寸外推为当前结果。

- Rover 产品域已收敛为唯一 `Dima/rover`；阶段 5 控制组合进入 `Dima/rover/control`，阶段 9 导航进入 `Dima/rover/navigation`，modules 下不再保留重复 Rover 子目录。
- Application Runtime 已补齐 Parameter bind/shutdown、Journal cache 失效、uORB lifecycle epoch、WorkQueue owner/destroy、Console shutdown 和逆序 rollback；同一次上电内的 `shutdown → init → start` 不再继承上一 Runtime 的参数、Topic 或 RC 有效性。
- BootHealth 只接受 Commander 三个安全 Topic 同时间戳、一致、新鲜且严格前进的快照，不再依赖 HelloWorld 或 `app_heartbeat`；连续 5 秒窗口在任一条件失效后重新计时。
- Application Fault/Panic 只写 non-cacheable D3 记录并复位；诊断 Flash 持久化由 MCUboot 冷启动独占。SBUS 使用 64-byte DMA Buffer 向 256 项 CPU-only SPSC Ring 复制，接收 epoch 变化时同时清 Ring 和 parser。
- TIM5/TIM8 可以保留 CubeMX 初始化，但六路 compare 初值固定为 0；当前应用不得链接 PWM start、Motor backend 消费者、Mixer 或 RoverDifferential。

| 当前门禁或资源 | 生命周期收敛状态 |
|---|---|
| 源码架构扫描 | `PASS`，184 个一方源文件 |
| `.init_array` 目标白名单 | 13 项：2 个工具链项、1 个 Parameter ConstLayer 构造、10 个 uORB registrar |
| `.fini_array` 目标白名单 | 仅 `__do_global_dtors_aux` |
| uORB 启动分配 | 没有新增 Topic；最终 Heap 实际值待 clean build/ELF 复核 |
| Task Pool | 链接上限仍为 48 KiB；最终 section 使用量待 clean build/ELF 复核 |
| SBUS 内存所有权 | `g_dma_buffer` 必须位于 `.dima_dma`；`g_receive_ring` 必须位于 CPU `.bss` |
| Application Flash/RAM、Signed BIN 增量 | 待 Windows 原生 clean build，当前不填入推测值 |
| 目标板 Runtime restart、Fault、SBUS 和 Termination | 待人工验收 |

当前 `build/H743_FreeRTOS.elf` 只用于 ELF 解析器烟雾检查：新门禁按预期拒绝其中旧 WorkQueue 带来的第 14 个 `.init_array` 项；它不是本节源码的重建产物，不能作为当前链接或目标构建证据。
