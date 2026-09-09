# DTCM 任务栈与 CPU 对象迁移

日期：2026-09-09。执行已确认的 48 KiB 任务栈池及 Ekf2、RoverDifferential、VehicleImu 迁移方案。

## 布局与生命周期

- DTCM 低 64 KiB 依次放置 .dima_ramfunc、.dima_task_pool、.dima_dtcm_bss 和最小栈容量检查区；上部 64 KiB 保留 MSP，初始 MSP 仍为 0x20020000。
- 48 KiB 任务栈池保持原 bitmap 分配、创建前清栈、任务优先级、栈大小与启停回收。九个平台注册任务配置使用 36 KiB，内核 Idle/Timer 的 1.5 KiB 小栈继续由 D2 静态回调提供。
- 三个 CPU 对象使用按类型对齐的原始字节存储。共享 Reset_Handler 在任何 C++ 构造前清零新段，ApplicationContext 在原成员构造时点通过 placement new 建立对象，避免 NOLOAD 丢失非零初值，也不提前到 Services/heap 安装前构造。
- ApplicationContext 持有对象引用，原模块注册、启动、停止、回滚顺序保持。实例随固件进程常驻，动态 EKF RingBuffer 继续由 D1 heap 提供。
- D1 仍保留 256 KiB heap；LogService 紧随 heap，SD 双缓冲再按 8 KiB 对齐。任务池迁出后的 48 KiB 被回收，不保留旧地址空洞。
- MPU Region 5 的 SD 地址继续来自链接符号；SD/UART/SPI DMA 专用存储和缓存策略保持。DTCM 栈上的 SD I/O 走已有中转路径，D1 日志 Ring 仍可直接 DMA。
- MCUboot 提供空 DTCM CPU 对象区间，并拒绝意外链接应用对象；共用的启动循环先比较边界，空区间不写内存。

## 生产验证合同

链接器与 tools/elf_support/layout.py 验证任务池恰好 48 KiB、CPU 段与最小栈预算不越过低 64 KiB、三份存储实际位于 DTCM、MSP 边界、D1 日志紧随 heap，以及 SD DMA NOLOAD/对齐/强 IRQ 绑定。内存摘要分别显示 DTCM 静态预算、剩余低区容量与 MSP 预留；这些不是实际栈高水位。

没有新增测试文件、测试框架、模拟环境或测试专用接口。参数、消息及派生契约由既有正式 Make 生成链维护。本次不修改控制、采样、日志和 CPU 统计策略。

## 基线与验收状态

- 源码、原始产物与逐文件哈希：C:\Users\master\AppData\Local\Temp\h743-dtcm-apply-2olf0uyh。
- 迁移前独立 Windows 工作区：E:\freertos\h743_dtcm_baseline_2olf0uyh。
- 迁移前 make dima_rover 全量 377/377 通过：应用 BIN 616784 bytes；DTCM 1248 B、D1 401408 B、D2 182560 B、DMA 6208 B、D3 224 B。
- 迁移后 make dima_rover 与 make intellisense 已通过；最终刷新后的前/后构建分别为 23/23、7/7，架构门禁通过 454 个第一方源文件，IDE 数据库 356 条命令/335 个源文件。应用与 MCUboot 无未解析符号，签名和 factory 布局验证通过。
- 新布局未刷入目标板，MSP/PSP 高水位、任务 CPU、deadline、ULog dropout 与 SD 直接命中率仍待板端比较。性能比较必须使用 IMU/EKF 持续正常更新的同业务负载；静态容量变化不能证明 CPU 降幅。


## 最终内存结果

| 区域 | 迁移前 B | 迁移后 B | 迁移后占比 | 迁移后链接未分配 |
|---|---:|---:|---:|---:|
| DTCM | 1248 | 60896 | 46.46% | 70176 B，含 MSP 65536 B 预算 |
| D1 AXI SRAM | 401408 | 352256 | 67.19% | 172032 B，168 KiB |
| D2 SRAM1/2 | 182560 | 172096 | 65.65% | 90048 B，87.94 KiB |
| D2 SRAM3 DMA | 6208 | 6208 | 18.95% | 26560 B |
| D3 诊断 | 224 | 224 | 0.34% | 65312 B |

DTCM 低区剩余 65536-60896=4640 B（4.53 KiB），上部 MSP 预算完整保留。D1 释放 49152 B；D2 释放 10464 B，ApplicationContext 从 91968 B 减至 81504 B。新的静态引用/guard 和对齐导致总链接 RAM 跨度净增 32 B，这次是存放位置优化。

| 内容 | 起址 | 字节 |
|---|---|---:|
| .dima_ramfunc | 0x20000000 | 224 |
| .dima_task_pool | 0x200000e0 | 49152 |
| Ekf2 原始存储 | 0x2000c0e0 | 5976 |
| RoverDifferential 原始存储 | 0x2000d838 | 2920 |
| VehicleImu 原始存储 | 0x2000e3a0 | 1592 |
| .dima_dtcm_bss 整段（含尾部对齐） | 0x2000c0e0 | 10496 |
| ._user_heap_stack 最小容量检查 | 0x2000e9e0 | 1024 |
| MSP 预算 | 0x20010000–0x20020000（半开） | 65536 |
| D1 heap | 0x24000000 | 262144 |
| D1 LogService | 0x24040000 | 73952 |
| D1 SD 双缓冲 | 0x24054000 | 8192 |

三份对象行是 .dima_dtcm_bss 的子项，不重复计入总量。检查 ApplicationContext.o 确认三份输入存储均为全零、类型对齐为 8；最终输出为 NOLOAD。应用 init_array 大小未增加，MCUboot 新清零边界为相等的 0x20002430，启动循环跳过空区间。

## 前后产物与边界

最终 before/after 目录位于上述快照目录，artifact_manifest.json 保存 section、符号、原始存储检查、源文件哈希与所有产物 SHA-256。业务源码通过逐文件核对，差异为本任务的内存迁移；构建过程中共享分支被其他会话提交，baseline 的 Git 标识 e1f839ec989b9841 与迁移版 10650e7dd5a08bb8 不同，不能把两套镜像标成相同的 Git 身份。CPU/任务统计实现保持同一口径。

| 产物 | B | SHA-256 |
|---|---:|---|
| 迁移前原始 BIN | 616744 | 97920647b138fc3ac16dbb6323352075070fdcdee46dfa5d7acd8335d212dee5 |
| 迁移后原始 BIN | 616752 | bb87e483a3f8dd01301376398ff508a30d46ccf7a8c04085b0c0775eebc7427d |
| 迁移前签名 BIN | 617920 | f23a775c0f96a655b6df9b49bd59ea9548939fc739592c5ca22c6fc8f9f3d346 |
| 迁移后签名 BIN | 617926 | d6c98cb3d7c096f323f8b32925d90fa09f1138b949501f5ebaa38eddd3962899 |

当前 Windows 串口清单为空，也未发现 QGroundControl/STM32CubeProgrammer/openocd 进程。本轮没有连接或刷写设备，没有板端性能、冷/热启动或高水位证据。两套镜像可供后续使用相同参数、正常 IMU/EKF 数据、相同日志 Profile/SD 卡和负载进行比较。
