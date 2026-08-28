# Dima Rover 资源基线与目录迁移后验证

- 历史采集日期：2026-07-29
- 目录说明更新：2026-07-30
- 分支：`feature/dima-phase1`
- 平台：STM32H743VI + FreeRTOS
- 历史工具链：GNU Arm Embedded 10.2.1 (`gcc-arm-none-eabi-10-2020-q4-major`)
- 当前验证工具链：GNU Make 4.4.1、Arm GNU Toolchain 16.1.0、binutils 2.47
- 镜像版本：`0.1.0+0`
- 构建入口：`GNUmakefile` + `make/project.mk`
- 当前状态：2026-07-30 Windows 本地 `make verify` 已通过；本文同时保留迁移前历史基线，目标板运行验收仍待完成

## 1. 历史构建与镜像记录

2026-07-29 17:13（Asia/Shanghai）曾执行：

```text
make firmware GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
make verify   GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
```

以下仅保留历史镜像元数据和尺寸，不作为目录迁移后的验证结论：

```text
Image version: 0.1.0+0
Image digest: 3f158564b2727589ee117173d2cb20356455e41616bf3a663f581239bfbb808a
bootloader: 45,956 bytes @ 0x08000000
signed app: 68,378 bytes @ 0x08040000
app vector: 0x08040400
```

产物：

```text
build/H743_FreeRTOS.elf          1,287,176 bytes（含调试段）
build/H743_FreeRTOS.bin             67,204 bytes
build/H743_FreeRTOS.hex            188,996 bytes
build/H743_FreeRTOS_signed.bin      68,378 bytes
build/H743_FreeRTOS_signed.hex     192,386 bytes
build/H743_FreeRTOS_factory.hex    271,620 bytes
```

主机 Python 工具位于 WSL 原生缓存：

```text
/root/.cache/dima-rover/host-tools/host-python
```

连续执行 `host-tools` 时不会因为工程 Makefile 变化重复运行 pip；旧 `build/host-python` 尚未自动删除。

### 1.1 目录迁移后 Windows 本地验证

2026-07-30（Asia/Shanghai）在当前完整工作区执行：

```powershell
make verify
```

结果：应用 ELF/BIN、MCUboot、签名镜像和 Factory HEX 全部生成并通过地址与签名检查。

```text
Image version: 0.1.0+0
Image digest: 5bd241f52cae861007498ec5e703e0b2bd3265366c86a0e0afd1d8353f786177
bootloader: 48,952 bytes @ 0x08000000
signed app: 113,471 bytes @ 0x08040000
app vector: 0x08040400
```

当前产物：

```text
build/H743_FreeRTOS.elf          1,822,080 bytes（含调试段）
build/H743_FreeRTOS.bin           112,296 bytes
build/H743_FreeRTOS_signed.bin    113,471 bytes
build/H743_FreeRTOS_factory.hex   390,826 bytes
build/mcuboot/mcuboot.elf       2,638,728 bytes（含调试段）
build/mcuboot/mcuboot.hex         137,668 bytes
```

该结果证明当前源码能够完成目标编译、链接、签名和镜像组合，不代表目标板外设、时序、Flash 掉电恢复或实车行为已经验收。

## 2. Flash 使用量

`arm-none-eabi-size`：

```text
text     64,752 bytes
data      2,412 bytes
bss     321,936 bytes
```

主要 Flash/RAM 段：

```text
.isr_vector          664 bytes  @ 0x08040400
.text             63,296 bytes  @ 0x080406C0
.rodata              784 bytes  @ 0x0804FE00
.data               2,388 bytes  @ 0x20000000，LMA 0x08050130
.bss               58,764 bytes  @ 0x20000958
._user_heap_stack   1,028 bytes  @ 0x2000EEE4
.dima_heap        262,144 bytes  @ 0x24000000，NOLOAD
```

Application Slot：

```text
Slot 容量                    786,432 bytes（768 KiB）
未签名 BIN                    67,204 bytes，占 8.55%
Signed BIN                    68,378 bytes，占 8.69%
未签名 BIN 物理剩余          719,228 bytes
85% 发布预算                 668,467 bytes
到 85% 预算线余量            601,263 bytes
```

Flash 分区未改变：

```text
MCUboot       0x08000000  256 KiB
Primary       0x08040000  768 KiB
Secondary     0x08100000  768 KiB
Scratch       0x081C0000  128 KiB
Storage       0x081E0000  128 KiB
```

## 3. SRAM 使用量

### DTCM

```text
容量                         131,072 bytes
.data + .bss + 保留栈区       62,180 bytes
占用                           47.44%
链接期剩余                    68,892 bytes
```

DTCM 主要增长来自七个静态 WorkQueue 栈、Event Ring、Log Ring 和 uORB runtime 状态。DTCM 不加入通用 Heap，也不能作为 DMA Buffer 默认区域。

### D1 AXI SRAM

```text
.dima_heap 起点              0x24000000
.dima_heap 终点              0x24040000
容量                         262,144 bytes（256 KiB）
对齐                              32 bytes
```

首个 uORB Topic `app_heartbeat`：

```text
对象大小                         16 bytes
Queue Depth                        1
最大实例                           4
启动期 Topic Buffer payload       64 bytes
```

heap_5 的管理头和对齐会使实际消耗略高于 64 bytes；准确 free/minimum-ever/largest-block 必须由目标板上的 `heap_stats()` 读取。

### D2/D3

阶段 1 未建立通用 Heap 输出段。D2 继续保留给后续显式 DMA Buffer 和 Cache/MPU 策略；D3 暂未分配产品专用输出段。

## 4. WorkQueue 与任务栈

生产模块使用 Dima/PX4 兼容 WorkQueue。启动壳位于 `Dima/application`；BootHealth/HelloWorld 位于 `Dima/modules/{boot_health,hello_world}`；生命周期位于 `Dima/middleware/lifecycle`。生产任务创建和调度状态以目标构建与后续板测为准。

```text
wq:estimator    priority 8    8,192 bytes    实时/禁止分配
wq:rate_ctrl    priority 7    4,096 bytes    实时/禁止分配
wq:sensors      priority 6    4,096 bytes    实时/禁止分配
wq:io           priority 5    4,096 bytes
wq:nav          priority 4    4,096 bytes
wq:hp_default   priority 3    2,048 bytes
wq:lp_default   priority 2    2,048 bytes
```

七个队列合计静态栈：`28,672 bytes`。此外：

```text
appMainTask（Dima/application）   2,048 bytes
Idle Task                          512 bytes
Timer Task                       1,024 bytes
```

按设计，`Dima/modules/boot_health` 运行在 `wq:hp_default`，`Dima/modules/hello_world` 和 USB Log Flush 运行在 `wq:lp_default`；实际运行关系待目标板验证。USB、格式化日志和其他可能阻塞的服务不得进入 estimator、rate-control 或 sensor 队列。

实际栈高水位尚未上板采集。

## 5. 中间件静态资源

```text
Event Ring：128 条，ELF 静态状态约 4,192 bytes
Log Ring：8 KiB，ELF 静态状态约 8,220 bytes
Perf Pool：容量 64；当前没有生产计数器引用，链接器可裁剪未引用实现
uORB Topic metadata：由 PX4 `uORBTopics.hpp/.cpp` 和每 Topic 官方源文件生成
uORB Runtime Buffer：按 schema 队列深度与实际实例在初始化期分配
```

早期基线使用 linker section 与静态构造器注册单个 Topic，该实现已经退役。当前 Runtime 直接消费官方生成的 `orb_get_topics()` 与 `orb_topics_count()`；不再存在专用 metadata section、边界符号或对应 ELF 保留断言，实际 Flash/Heap 占用以本轮 Windows clean build 为准。

uORB 初始化使用带 `allocate/deallocate` 的受控 D1 Heap backend；初始化中途失败和正常 shutdown 都会释放已创建 Buffer、清除 generation、advertise 状态和 callback。

## 6. 阶段 1 实现边界

历史基线记录的实现项：

- WSL 原生 host-tools 缓存。
- D1 256 KiB `heap_5` 受控 Heap、失败 Hook、统计和实时任务分配拒绝。
- TIM2 1 MHz、32 位 HRT 与 64 位溢出扩展。
- 持久 `ApplicationContext`。
- 七队列 Dima/PX4 WorkQueue 兼容层。
- uORB Publication、Subscription、Callback、Queue Depth 和最多四实例。
- heartbeat 生产链迁移到 uORB。
- Events、Perf、8 KiB Logging 与 LP USB 刷新服务。
- 目录迁移后 Signed BIN、MCUboot HEX 和 Factory HEX 已于 2026-07-30 通过 Windows 本地 `make verify` 重新生成并验证。

尚需目标板人工确认：

- TIM2 HRT 连续性、单调性和 32 位 Overflow（约每 71 分 34.967 秒一次）。
- Heap 启动后空闲量、最低余量和最大连续空闲块。
- 七个 WorkQueue 的实际栈高水位、最大执行时间和 deadline miss。
- USB 断开、Busy 和日志 Ring 满时不影响实时队列。
- BootHealth 通过 uORB heartbeat 在稳定窗口后确认 MCUboot 镜像。

这些是板级运行验收，不通过新增测试框架、SITL 或仿真实现。

## 7. 后续阶段

阶段 2 开始移植 Parameter Core，并将成熟上游显式 bind/update 语义公开为 `dima::Param<T>`。参数系统继续复用锁定的 Parameter YAML 生成逻辑，不设置固定 64 项容量，并提供 USB 在线 `param get/set/show/save/reset/status`。

## 8. 阶段 2 目录迁移前后构建资源

| 项目 | 迁移前记录（bytes） | 2026-07-30 当前验证（bytes） | 变化（bytes） |
|---|---:|---:|---:|
| `.text` | 111,020 | 110,184 | -836 |
| `.data` | 2,828 | 2,068 | -760 |
| `.bss` | 325,776 | 326,448 | +672 |
| MCUboot Signed BIN | 115,064 | 113,471 | -1,593 |

当前数值来自目录迁移后 Windows 本地 `make verify`，签名镜像、应用向量地址、MCUboot 镜像和 Factory HEX 验证通过。GCC 16.1.0 仍报告 newlib 未实现系统调用及 ELF RWX LOAD 段警告，后续需单独收敛；这些警告未导致本次构建失败。

阶段 2 新增资源主要来自 Parameter Layer/Core、24 项 metadata、官方生成结果、TinyBSON/flashparams、Autosave、USB RX Ring 和单扇区 Flash Journal，以及参数区 ECC 安全读/BusFault 受控恢复。参数数量由生成结果确定，不存在固定 64 项容量。

项目自有测试目录已移除，不保留 Host Test、SITL 或仿真入口。阶段 2 尚未完成目标板实车验收，包括 USB 在线调参、自动保存时序、掉电恢复、CRC 尾部损坏回退、ENOSPC 和人工擦除流程。
