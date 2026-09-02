# PX4 风格结构化日志与 ULog

本目录保留 PX4 v1.17.0 `PX4_INFO/WARN/ERR/PANIC/DEBUG`、`PX4_INFO_RAW` 和 SourcePolicy 兼容面，但不拥有 USB transport。`Dima/modules/logging/` 在此实时文本面之外实现产品完整 Topic ULog；Topic 名、ID、字段格式和 catalog 全部来自权威 `.msg` 生成链，不维护第二份手写日志清单。

## 运行契约

- ISR 和实时 WorkQueue 在格式化前拒绝；所有接受的记录使用固定 256-byte 格式缓冲，不动态分配。
- `debug_config.hpp` 统一控制 System、SBUS、ICM42688 的最低等级和周期；默认 System=Info、SBUS=Error、ICM42688=Off。
- uORB 初始化后、Parameter 初始化前，LogService 注册唯一 structured sink。普通日志和 `PX4_INFO_RAW` 都发布为深度 8 的 `mavlink_log`；RAW 保留原正文并使用调用级别，不添加模块前缀，但只绕过普通等级过滤，不绕过 ISR/实时 WorkQueue 的格式化禁令。QGC `[cal]` 协议因此必须由非实时 `wq:lp_default` 的校准事务产生。
- MavlinkService 独占 USB CDC，并把 `mavlink_log` 转为 STATUSTEXT。断线时 uORB 保留有界最新记录，重连后不发送超过 5 s 的旧记录。
- `SdLogWriter` producer 在 `wq:lp_default` 每 5 ms 有界扫描生成的 Topic catalog；除 `mavlink_log` 映射为 `L` 外，首次实例数据写 `A`、后续按 `o_size_no_padding` 写 `D`。启动定义段写 header/Flag Bits、自动生成的 `F`、used 参数 `P` 以及 current/system default `Q`，Active 段写 `L/O`、变化参数 `P`，并每 500 ms 写 `S` sync marker。
- `LogWriter` consumer 独占 `wq:storage`，使用固定 64 KiB SPSC 字节 Ring，每次最多向 FatFs 提交 4096 bytes，并每 1 s 执行 `f_sync`。producer 不调用任何 FatFs/SDMMC API；Ring 满时写标准 `O` dropout，而不是静默拼接损坏流。
- 每个新介质/文件都推进 session generation，清空旧 Ring、Topic generation 与 message ID，并从 ULog header 全量重建。介质失败按 3 s 重试，只停止 SD 副本，不影响实时 STATUSTEXT/Event。
- H743 板没有 card-detect GPIO，无法证明“物理卡在位”。已挂载会话通过最长 500 ms 的 `CTRL_SYNC` 主动命令确认“最近一次探测可用”；失败立即撤销全部 FIL/DIR 与挂载，下一次重试执行完整 SDMMC/FatFs 初始化。
- sink 不存在或 uORB 发布失败时只推进 `sink_dropped_records`；Critical Event 仍由独立 Event Ring 和故障锁存保存。
- LogService 在低优先级队列每轮最多转储 4 条 Event；SBUS 连续数据入口仍受 100 ms 最小周期限制。

## 文件清单

| 文件 | 职责 |
|---|---|
| `logging.hpp/cpp` | PX4 宏/C ABI、格式过滤、structured sink 和饱和统计 |
| `debug_config.hpp` | 编译期 SourcePolicy、最低等级和连续数据周期 |
| `modules/logging/LogService.*` | sink 生命周期、Event/SBUS 生产和 `mavlink_log` 发布 |
| `modules/logging/SdLogWriter.*` | 生成 catalog 驱动的 `F/P/Q/A/D/L/S/O` ULog producer |
| `modules/logging/LogWriter.*` | 64 KiB SPSC Ring、storage consumer、同步/关闭与介质会话恢复 |

禁止重新加入直接 Console write、USB 字节 Ring、`ServiceWriter`、`service_flush` 或手写 Topic/参数清单。上游基线为 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`：`messages.h`、`uORBMessageFields.*`、`Array.hpp`、官方 compressed-fields 生成器和 Python heatshrink encoder 保持逐字同步；产品调度、固定 Ring 和 FatFs capability 是 FreeRTOS 适配。解码器来自 PX4 heatshrink commit `052e6de72f67f1777198bce98f3de62f7f3c16a0`。
