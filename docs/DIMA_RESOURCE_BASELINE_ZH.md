# Dima Rover 阶段 1 资源基线

- 日期：2026-07-29
- 分支：`feature/dima-phase1`
- 平台：STM32H743VI + FreeRTOS
- 工具链：GNU Arm Embedded 10.2.1 (`gcc-arm-none-eabi-10-2020-q4-major`)
- 镜像版本：`0.1.0+0`
- 构建入口：`GNUmakefile` + `make/project.mk`
- 验证范围：目标编译、链接、签名和 MCUboot 镜像一致性；未新增或运行测试、SITL、仿真

## 1. 最终构建与镜像验证

2026-07-29 17:13（Asia/Shanghai）执行：

```text
make firmware GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
make verify   GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin
```

结果：

```text
Image was correctly validated
Image version: 0.1.0+0
Image digest: 3f158564b2727589ee117173d2cb20356455e41616bf3a663f581239bfbb808a
MCUboot image verification passed
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

生产模块现在只启动 Dima/PX4 兼容 WorkQueue；旧 `App/runtime/scheduling` 保留为迁移期代码和 Host seam，但不再由 `ApplicationContext` 初始化。

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
appMainTask                      2,048 bytes
Idle Task                          512 bytes
Timer Task                       1,024 bytes
```

BootHealth 运行在 `wq:hp_default`；HelloWorld 和 USB Log Flush 运行在 `wq:lp_default`。USB、格式化日志和其他可能阻塞的服务不得进入 estimator、rate-control 或 sensor 队列。

实际栈高水位尚未上板采集。

## 5. 中间件静态资源

```text
Event Ring：128 条，ELF 静态状态约 4,192 bytes
Log Ring：8 KiB，ELF 静态状态约 8,220 bytes
Perf Pool：容量 64；当前没有生产计数器引用，链接器可裁剪未引用实现
uORB metadata：1 个 Topic，16 bytes，地址 0x20000944～0x20000954
uORB heartbeat runtime：4 个实例，静态状态 224 bytes
```

`.dima_orb_meta` 已在链接脚本 `.data` 中显式 `KEEP`，当前注册仍由 `MetadataRegistrar` 在静态初始化阶段建立链表；section 同时用于保留和资源审计。

uORB 初始化使用带 `allocate/deallocate` 的受控 D1 Heap backend；初始化中途失败和正常 shutdown 都会释放已创建 Buffer、清除 generation、advertise 状态和 callback。

## 6. 阶段 1 已完成边界

已完成：

- WSL 原生 host-tools 缓存。
- D1 256 KiB `heap_5` 受控 Heap、失败 Hook、统计和实时任务分配拒绝。
- TIM2 1 MHz、32 位 HRT 与 64 位溢出扩展。
- 持久 `ApplicationContext`。
- 七队列 Dima/PX4 WorkQueue 兼容层。
- uORB Publication、Subscription、Callback、Queue Depth 和最多四实例。
- heartbeat 生产链迁移到 uORB。
- Events、Perf、8 KiB Logging 与 LP USB 刷新服务。
- Signed BIN、MCUboot HEX 和 Factory HEX 重新生成并验证。

尚需目标板人工确认：

- TIM2 HRT 连续性、单调性和 32 位 Overflow（约每 71 分 34.967 秒一次）。
- Heap 启动后空闲量、最低余量和最大连续空闲块。
- 七个 WorkQueue 的实际栈高水位、最大执行时间和 deadline miss。
- USB 断开、Busy 和日志 Ring 满时不影响实时队列。
- BootHealth 通过 uORB heartbeat 在稳定窗口后确认 MCUboot 镜像。

这些是板级运行验收，不通过新增测试框架、SITL 或仿真实现。

## 7. 后续阶段

阶段 2 开始移植 Parameter 与 ModuleParams。参数系统必须复用成熟 PX4 Parameter API 和生成逻辑，不设置固定 64 项容量，并提供 USB 在线 `param get/set/show/save/reset/status`。
