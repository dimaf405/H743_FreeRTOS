# PX4 风格结构化日志与 ULog

本目录保留 PX4 v1.17.0 `PX4_INFO/WARN/ERR/PANIC/DEBUG`、`PX4_INFO_RAW` 和 SourcePolicy 兼容面，但不拥有 USB transport。`Dima/modules/logging/` 在此实时文本面之外实现产品 Profile 驱动的 SD ULog；Topic 名、ID、字段格式和采样策略全部来自权威 `.msg` 与 `logger_topics.yaml` 生成链，不维护第二份 C++ 日志清单，也不提供 MAVLink 实时 ULog backend。

## 运行契约

- ISR 和实时 WorkQueue 在格式化前拒绝；所有接受的记录使用固定 256-byte 格式缓冲，不动态分配。
- `debug_config.hpp` 统一控制 System、SBUS、ICM42688 的最低等级和周期；默认 System=Info、SBUS=Error、ICM42688=Off。
- uORB 初始化后、Parameter 初始化前，LogService 注册唯一 structured sink。普通日志和 `PX4_INFO_RAW` 都发布为深度 8 的 `mavlink_log`；RAW 保留原正文并使用调用级别，不添加模块前缀，但只绕过普通等级过滤，不绕过 ISR/实时 WorkQueue 的格式化禁令。QGC `[cal]` 协议因此必须由非实时 `wq:lp_default` 的校准事务产生。
- MavlinkService 独占 USB CDC，并把 `mavlink_log` 转为 STATUSTEXT。断线时 uORB 保留有界最新记录，重连后不发送超过 5 s 的旧记录。
- Logger 只读取一次重启参数快照：`SDLOG_MODE` 控制四种会话生命周期，`SDLOG_PROFILE` 合并通用 Rover、EKF2 回放和系统辨识采样策略，`SDLOG_DIRS_MAX` 给出包含当前会话的有限目录上限。三项定义只存在于 `module_logger.yaml`，QGC Metadata 与 C++ 合同均由正式参数工具生成。
- 生成的 Topic 策略只保存运行期需要的分类、逐 Profile 索引和停止刷新标志；相同 kind/interval 共用只读采样表，间隔仍为完整微秒值，不做量化或运行期合并。Topic 名称/ID 继续直接使用 uORB metadata，回放注册继续使用独立生成索引，不保存无人读取的副本。
- `SdLogWriter` producer 在 `wq:lp_default` 每 5 ms 有界扫描生成的 Topic catalog；`mavlink_log` 只映射为 `L`，`parameter_update` 只触发 `P/Q`，普通实例首次写 `A`、后续按 `o_size_no_padding` 写 `D`。启动定义段写 header/Flag Bits、硬件 UID、自动生成的 `F`、完整生成参数目录的 `P` 以及 current/system default `Q`，Active 段写 `L/O`、变化参数 `P`，并每 500 ms 写 `S` sync marker。
- 当前 Profile 没有选中任何别名的格式组，在完整解码后使用上游 `clearFormatFromBuffer()` 跳过字符串展开；保留下一组剩余字节与单轮解码预算。有输出的组仍整体预留空间，并在背压时重试同一组。
- 普通 Topic 发送共用结果处理：固定频率通过原有时间门禁后最多写一条最新样本，源频率仍按队列深度及每轮预算逐 generation 排空。停止边沿仍取最新状态，Blocked 重试原 slot，dropout 与失败处理不改变。
- 参数 `P/Q` 写入共用 key 长度校验；默认值相同只发合并类型的单条 `Q`，不同默认值按 setup 再 system 写入。保留双记录空间预留、与当前值相同则省略及遇背压立即重试的规则。
- `P/Q` key 与硬件 UID 的纯字符串/整数格式化复用现有 `dima::format::format_to`，保留返回长度、截断拒绝、补零、大写十六进制和末尾 NUL；不再为两处缓冲格式化引入 newlib 的第二套 formatter。ULog 记录顺序、路由、过滤和背压不变，stdout/setvbuf 与 newlib 链接策略没有改动。
- `LogWriter` consumer 独占 `wq:storage`，使用固定 64 KiB SPSC 字节 Ring，每次最多向 FatFs 提交 8192 bytes，并每 1 s 执行 `f_sync`；活动写入与关闭前的 UTC 侧车更新共用代次确认路径，写入失败不推进确认代次。producer 不调用任何 FatFs/SDMMC API；Ring 满时写标准 `O` dropout，而不是静默拼接损坏流。
- producer/consumer 的立即唤醒与 uORB 回调必须保留各自的 5 ms/20 ms 周期。否则开机记录时 producer 可能先于文件创建执行并返回，consumer 只创建 0-byte 文件后也停止，且没有 I/O 错误可报告；文件缺少 ULog magic 时不会进入 QGC 列表。保留周期后，等待文件、Ring 暂空以及同步/重试分支都能继续推进。
- 每个新介质/文件都推进 session generation，清空旧 Ring、Topic generation 与 message ID，并从 ULog header 全量重建。普通介质失败在 Mode 仍有记录意图时按 3 s 重试；低空间暂停按 60 s 复查，只停止 SD 副本，不影响实时 STATUSTEXT/Event。
- `sessNNN/log100.ulg` 使用最多三条 64-byte CRC `meta.bin` 记录保存全局顺序、硬件 UID、关闭/恢复状态、最终文件大小/CRC 和可选 GPS UTC。恢复、`sessNNN -> delNNN` 删除及目录上限均由 `wq:storage` 分步推进。空间策略对照 PX4 v1.17 `logger/util.cpp::check_free_space`：回收目标为 `min(容量×10%, 300 MiB)`，停止记录门限独立为 50 MiB；本地小卷将回收目标抬到至少 50 MiB，以保证先回收再停写。无可删历史时允许使用回收目标与停止门限之间的空间。
- 写入按下一块新增 FAT 簇提前判断；触线前先校正实际空闲计数、每轮至多回收一个会话并返回 `-EAGAIN`，consumer 保留 Ring 原字节重试。新建会话先预留 sess/sidecar/父目录扩展的三簇预算，创建后再次校正。只有无安全候选且下一次分配会突破 50 MiB 停止线时才因空间暂停。告警带实际 free/total MiB；ENOSPC 也可能表示受保护目录占满名额，不能仅凭提示判断卡的标称容量。
- 未知文件、当前 writer 和仍在传输的 QGC reader 不自动删除。正常下载完成不依赖地面站发送 `LOG_REQUEST_END`：reader 在请求区间完成后保留 5 s 补传窗口，随后由 `wq:storage` 关闭并解除回收保护；响应 Ring 的独立字节副本不受关闭影响。
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
| `modules/logging/logger_topics.yaml` | 唯一 Topic 分类、Profile 采样、回放队列和 sidecar 布局策略 |
| `tools/logging/generate_logger_contract.py` | 全集校验 uORB/参数/上游来源并生成 Logger、ULog wire 与 sidecar 合同 |

禁止重新加入直接 Console write、USB 字节 Ring、`ServiceWriter`、`service_flush` 或手写 Topic/参数清单。上游基线为 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`：原始 `messages.h` 位于固定上游快照并生成 `ulog_messages.hpp`，`uORBMessageFields.*`、`Array.hpp`、官方 compressed-fields 生成器和 Python heatshrink encoder 保持来源可追踪；产品调度、固定 Ring 和 FatFs capability 是 FreeRTOS 适配。解码器来自 PX4 heatshrink commit `052e6de72f67f1777198bce98f3de62f7f3c16a0`。

日志 Ring 的消费指针仍在同步 append_log 成功后推进，因而满足直接 IDMA 的源缓冲所有权。块端口对 D1 整 cache line 数据执行直接传输，其余输入使用两个 4 KiB 专用缓冲；格式化、部分扇区拼接和 sidecar CRC 仍由 CPU 完成。低流量立即提交当前长度，不等待凑满 8 KiB。
