# SDMMC IDMA、直接传输与 CPU 统计实施及验收记录

日期：2026-09-08。范围：H743 板级 SD 数据通路、平台 CPU 统计及资源验证。

## 交付状态

| 计划项 | 当前结果 | 证据边界 |
|---|---|---|
| 保存共享工作区和原始产物 | 完成；保留原有未提交修改 | 本地快照与逐文件 SHA-256 |
| 轮询版接入 CPU 统计 | 完成；已用最终采样代码重建独立基线 | Windows 编译产物；没有板端负载数据 |
| 双缓冲、直接 IDMA、cache、IRQ、恢复 | 完整实现 | 源码审查与目标编译 |
| 构建归属、MPU/内存合同、IDE 数据库 | 完成 | Windows 架构检查、ELF 校验、正式生成入口 |
| 单扇区、尾块、回绕、竞争和错误路径 | 已完成源级逐路径审查 | 不等同于硬件读回和故障注入通过 |
| SD/QGC/参数镜像、冷/热启动、拔卡恢复 | 待板端验收 | 当前 Windows 串口清单为空，未打开串口或刷写 |
| CPU、IRQ、控制/EKF 时序、ULog dropout 对比 | 待实测 | 不预设 CPU 下降百分比 |

本次没有新增测试文件、测试框架、模拟环境或测试专用接口。参数、Topic 和 MAVLink 继续通过正式工具生成。本次不提交或推送，也不覆盖其他并行工作。

## 最终数据通路

生产者继续在原队列编码 ULog 并写入 D1 的 64 KiB SPSC Ring；storage worker 的连续提交上限从 4096 增至 8192 bytes。低流量仍提交实际已有数据，不等填满；采样频率、同步周期和停机冲刷语义不变。

FatFs 的同步 C ABI 保留。完整位于 D1、起址按 32 bytes 对齐、长度为完整扇区的数据直接进入 SDMMC1 IDMA。TX 先 clean；RX 先 clean-invalidate，完成或硬件静默后再 invalidate。Ring 消费指针只有在 append_log 成功后才推进，确保 DMA 期间源数据不会被生产者覆盖。部分扇区仍由 FatFs 拼接；直接命中率还取决于实际文件位置和缓冲对齐，不能仅凭 Ring 位于 D1 预设为 100%。

其他来源使用两个 4 KiB 非缓存专用半区。超过一个半区的请求开启硬件 IDMA 双缓冲，每笔 DLEN 最多 8192 bytes。半区 0 释放后，可以在半区 1 传输时复制已收到的数据或预填下一批 TX 数据。即使任务被抢占、两个半区完成通知合并，硬件也不会在这一笔事务中再次访问半区 0。新批次只有在数据准备完成后才能重新启动。

SDMMC1_IRQHandler 为唯一强绑定入口，逻辑优先级 7。它捕获/锁存事件、关闭必要数据通路并通过平台 Signal 唤醒任务，不调用 HAL_SD_IRQHandler，不执行 CMD12 等待、FatFs 或大块 memcpy。卡识别和总线配置沿用 HAL，运行期命令使用 LL 配置及终态后的 R1 解析。

状态推进为 Idle → Prepare → Command → Data → Stop/CardReady → Complete；失败进入 Quiesce → ReinitializeRequired。正常完成时如果静默检查仍需要复位，也必须返回失败并撤销旧会话。

每次 disk_read、disk_write 和 CTRL_SYNC 的全部分块、CMD12/CMD13 和卡忙检查共用 500 ms 软件截止。Signal 承担常规阻塞等待，卡状态重查之间至少延迟一个内核 tick。该值是软件截止预算；任务重新获得调度的延迟不构成硬实时返回保证。HAL 识卡流程也不据此宣称具有 500 ms 总时限。

错误优先于完成事件。恢复先关闭 IDMA/DPSM、清理外设及 NVIC 待处理中断，必要时复位 SDMMC1，然后释放缓冲并撤销会话。软件事件使用事务代次过滤；硬件事件没有代次字段，依靠外设静默、清标志和重新布置事务隔离迟到 IRQ。故障后不回退到轮询方式。FlashFS 主存储、SD 镜像降级与 storage 队列隔离保持。

## D1 地址分配

下表使用半开区间，末地址不属于该区域。所有数值来自最终 ELF。

| 用途 | 起址 | 末地址 | 字节 |
|---|---|---|---:|
| 固定 heap_5 预留 | 0x24000000 | 0x24040000 | 262144 |
| 固定任务栈池 | 0x24040000 | 0x2404c000 | 49152 |
| LogService 独立静态实例 | 0x2404c000 | 0x2405e0e0 | 73952 |
| MPU 对齐空隙 | 0x2405e0e0 | 0x24060000 | 7968 |
| SD IDMA 两个半区 | 0x24060000 | 0x24062000 | 8192 |
| 连续未分配余量 | 0x24062000 | 0x24080000 | 122880 |

.dima_sd_dma 为 SHT_NOBITS/NOLOAD，按 8192 bytes 对齐。MPU Region 5 的 RASR 为 0x130c0019，即 Normal、Shareable、不可缓存、禁止执行。地址来自链接符号。Region 6 的 D2 UART/SPI DMA 与 Region 7 的 D3 诊断属性保留。这里的 MPU 属性证据来自源代码、链接边界及已链接的启动验证逻辑；没有板端寄存器读取证据。

LogService 继续使用原有 D1 清零及构造流程。SD 原始缓冲不依赖启动清零：TX 先准备本笔长度，RX 只在对应硬件完成后读取。

## 最终资源余量

| 区域 | 容量 bytes | 已占用 bytes | 剩余 bytes |
|---|---:|---:|---:|
| 应用可链接 Flash | 782336 | 633064 | 149272 |
| D1 AXI SRAM | 524288 | 401408 | 122880 |
| D2 普通静态数据 | 262144 | 182368 | 79776 |
| D2 SRAM3 UART/SPI DMA | 32768 | 6208 | 26560 |
| DTCM | 131072 | 1248 | 129824 |
| D3 诊断区 | 65536 | 224 | 65312 |

RAM 按最高已分配地址到区域基址的跨度统计，包含段间空隙。D1 的 heap 和任务池属于已预留，池内运行时空闲量不计入“未分配余量”。DTCM 剩余需要承载 MSP 增长，不能全部当作可随意扩张的静态区。

Flash 使用实际有内容 section 的 LMA 跨度，包含 .data 与 .dima_ramfunc 的加载副本和链接对齐空隙，排除 NOLOAD 与 ELF 文件头。当前应用 BIN 大小正好为 633064 bytes。报表容量复用正式 ELF 验证器的 764 KiB 应用上限，不把槽位尾部 3072 bytes 保留空间算成应用增长余量。

| 应用 Flash 构成 | bytes |
|---|---:|
| 向量表 | 664 |
| .text | 540352 |
| .rodata | 77880 |
| ARM unwind 表 | 1216 |
| init/fini 数组 | 24 |
| DTCM 运行代码的加载副本 | 224 |
| .data 初始化副本 | 12664 |
| 加载对齐空隙 | 40 |
| 合计 | 633064 |

2 MiB 物理 Flash 的分区仍为：MCUboot 128 KiB、启动诊断 128 KiB、主槽 768 KiB、升级槽 768 KiB、交换区 128 KiB、持久存储 128 KiB。升级/交换/持久存储分区是既有保留用途，不能把“未出现在应用 ELF”解释为应用可用空闲。当前签名应用为 634240 bytes；MCUboot BIN 为 48308 bytes。

## CPU 统计与诊断读取口径

FreeRTOSConfig.h 启用运行时间统计，复用既有 TIM2 1 MHz 时钟，不增加定时器，不启用统计格式化函数。TaskRuntime 提供平台中性的 CpuUsage 和 TaskCpuUsage，只使用固定容量表。

采样在非实时任务中最多每秒一次，枚举固定平台任务槽与内核 Idle/Timer，通过 vTaskGetInfo(..., pdFALSE, ...) 禁止栈高水位扫描。首次采样、计数回绕、任务代次变化、计数不完整及超过 60 s 的窗口重新建立基线，不发布伪造的负载尖峰。

有效窗口中，总负载千分比为 1000 - 1000 × idle_delta / window；各任务为 1000 × task_delta / window。乘法使用 64-bit 中间量。SYS_STATUS.load 复用原有发送节奏和字段；无效窗口保留上次有效值。任务明细按索引读取；需要跨多次调用拼成同一快照时，应核对前后 CpuUsage.timestamp_us 一致。

统计采用 FreeRTOS 在任务切换时的运行时间归账，包含被计入当前任务的中断时间，不是扣除了全部 IRQ 的“纯线程时间”。板级 dima_sdmmc_get_io_stats 提供独立只读诊断，已通过链接 KEEP 与 ELF 门禁保证不会被裁掉：

- 直接/中转读写成功字节、事务数、双缓冲使用数、流水复制字节。
- 错误、超时、恢复、强制复位、无效请求及事务代次。
- SD IRQ 次数、累计/最大耗时和最后硬件状态。
- 当前会话、事务阶段及两个半区的所有权状态。

IRQ 耗时使用 TIM2 在该 ISR 入口/出口之间测量，可能包含更高优先级 IRQ 抢占。它用于观察 SD 中断开销和延迟，不能简单从任务比例中相减得到精确的排他 CPU 时间。包含 Signal 挂起时间的 storage Run 墙钟耗时不作为 CPU 使用率。

## Windows 证据与对比基线

正式工作区：E:\freertos\H743_FreeRTOS。独立轮询工作区：E:\freertos\h743_sd_poll_ddxtpix4。两者使用同一 HEAD 3ac342f808e847fe5b9c223ccb9457fec33f29e8 及当前业务源文件快照，保留共享工作区未提交修改。验收期间检测到的自动标定并行修改已保留并同步到两套工作区，最终通过逐文件哈希确认业务源一致；不能只用 HEAD 代替未提交源码快照。

主工作区执行 make -j4 NO_COLOR=1 dima_rover 和 make intellisense。架构检查通过 454 个第一方源文件；IDE 数据库为 356 条命令、335 个源文件，其中恰好一个 fatfs_diskio.cpp 条目，旧 fatfs_diskio.c 条目为零。

最终 ELF 核验：SDMMC1 向量指向强符号 0x080ba691；dima_sdmmc_get_io_stats 位于 0x080406c1；CPU counter、vTaskGetInfo、cache 维护和 MPU 验证函数均已链接。HAL_SD_IRQHandler、HAL_SD_ReadBlocks、HAL_SD_WriteBlocks 与旧的 uxTaskGetSystemState 均不在最终应用中。应用/MCUboot 无未解析符号，签名与 factory 布局校验通过。

轮询基线在独立工作区恢复旧磁盘端口、cache/MPU/链接规则和 4 KiB 日志提交上限，并保持最终 CPU API、FreeRTOS 配置/Backend、MAVLink CPU 消费者五个文件逐字一致。基线完成 make dima_rover 全量构建；同步并行自动标定更新后，主工作区最终 16/16、轮询基线最终 18/18 验证均通过。早期 polling_with_cpu_stats 快照使用过旧采样实现；后续 A/B 应使用这里的 polling_final。

| 同业务源和同 CPU 统计口径 | 轮询版 | 完整 IDMA 版 | 差值 bytes |
|---|---:|---:|---:|
| 应用 Flash bytes | 629792 | 633064 | +3272 |
| D1 预留跨度 bytes | 385248 | 401408 | +16160 |
| D2 普通数据 bytes | 182176 | 182368 | +192 |
| D2 SRAM3 DMA bytes | 6720 | 6208 | -512 |

D1 增量 = 8192 bytes 缓冲 + 7968 bytes MPU 对齐空隙。上述是资源成本，不是 CPU 性能测量结果。

快照目录：C:\Users\master\AppData\Local\Temp\h743-sd-idma-ddxtpix4。idma_final 与 polling_final 保存 ELF/BIN/map/签名/factory 及 manifest.json；baseline.json 和 work_hashes.json 记录原始状态与本任务预期哈希；polling_final_source.json 记录同业务源快照和恢复文件清单。签名私钥未复制进基线工作区。

| 产物 | SHA-256 |
|---|---|
| IDMA 应用 BIN | 9a5341f362cadabbbfd3fc3651464512a8d523b8aee012f570ecbe94332f7931 |
| IDMA 签名 BIN | 9b45c08dd1b3c1797f3f256329479aa0beb560f0acfaafaf2fc38eb16611adf0 |
| 轮询应用 BIN | ab8d9adaafbffdc4c5ba3951c0657f745e1efe8a772db9383cc54d170113b618 |
| 轮询签名 BIN | ab98825556e98372d32218f00af5d2c265d7a8cf64ce4432abccd95812907f94 |

## 已审查路径与板端待办

| 场景 | 源级处理 | 板端证据 |
|---|---|---|
| 单扇区 | 单缓冲 IDMA；D1 对齐则直接，其余 staging | 待读回 |
| 16 扇区完整缓冲对 | 非直接请求使用两个 4 KiB 半区；DLEN 有限 | 待读回及中断观察 |
| 9–15 扇区尾块 | 第二半区只准备/复制有效扇区长度 | 待读回 |
| 超过 16 扇区 | 分批共用同一截止；允许前一批释放半区后预取 | 待读回 |
| 未对齐/D2/DTCM 来源 | 统一进入专用 staging 缓冲 | 待读回 |
| Ring 回绕及 FatFs 簇边界 | Ring 连续区、8 KiB 上限与 FatFs 自身裁剪共同生效；成功后才消费 | 待 ULog 解码 |
| 完成早于等待/通知合并 | 二值信号仅提示唤醒，事件锁存供重查 | 待调度观测 |
| 错误和 DATAEND 同时出现 | 错误优先，先关数据通路，再由任务撤销会话 | 待硬件故障证据 |
| 超时后的迟到 IRQ | 停止/复位、清外设和 NVIC、撤销缓冲所有权，下一代重新布置 | 待拔卡/故障证据 |
| 冷/热启动与移除后重新插入 | 保留完整识卡/挂载和 FlashFS/SD 降级边界 | 待板端流程 |

板端使用现有日志生成、参数镜像和 QGC 日志下载流程验证文件读回一致性、ULog 可解析性、参数恢复和拔卡后的新会话。固定 SD 卡、日志 Profile、采样频率、任务负载、参数和供电条件，以 polling_final 与 idma_final 分别建立有效采样窗口，再比较 SYS_STATUS.load、storage/控制/EKF 任务占比、直接 DMA 命中率、SD IRQ 累计/峰值、控制/EKF deadline 和 ULog dropout。

在这些数据取得前，交付结论限于“完整实现与 Windows 构建/产物验证通过”，板端稳定性和 CPU 收益保持待验证。
