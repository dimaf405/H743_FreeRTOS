# PX4 风格结构化日志

本目录保留 PX4 v1.17.0 `PX4_INFO/WARN/ERR/PANIC/DEBUG`、`PX4_INFO_RAW` 和 SourcePolicy 兼容面，但不拥有 USB transport。

## 运行契约

- ISR 和实时 WorkQueue 在格式化前拒绝；所有接受的记录使用固定 256-byte 格式缓冲，不动态分配。
- `debug_config.hpp` 统一控制 System、SBUS、ICM42688 的最低等级和周期；默认 System=Info、SBUS=Error、ICM42688=Off。
- uORB 初始化后、Parameter 初始化前，LogService 注册唯一 structured sink。普通日志和 `PX4_INFO_RAW` 都发布为深度 8 的 `mavlink_log`；RAW 保留原正文并使用调用级别，不添加模块前缀，但只绕过普通等级过滤，不绕过 ISR/实时 WorkQueue 的格式化禁令。QGC `[cal]` 协议因此必须由非实时 `wq:lp_default` 的校准事务产生。
- MavlinkService 独占 USB CDC，并把 `mavlink_log` 转为 STATUSTEXT。断线时 uORB 保留有界最新记录，重连后不发送超过 5 s 的旧记录。
- sink 不存在或 uORB 发布失败时只推进 `sink_dropped_records`；Critical Event 仍由独立 Event Ring 和故障锁存保存。
- LogService 在低优先级队列每轮最多转储 4 条 Event；SBUS 连续数据入口仍受 100 ms 最小周期限制。

## 文件清单

| 文件 | 职责 |
|---|---|
| `logging.hpp/cpp` | PX4 宏/C ABI、格式过滤、structured sink 和饱和统计 |
| `debug_config.hpp` | 编译期 SourcePolicy、最低等级和连续数据周期 |
| `modules/logging/LogService.*` | sink 生命周期、Event/SBUS 生产和 `mavlink_log` 发布 |

禁止重新加入直接 Console write、USB 字节 Ring、`ServiceWriter` 或 `service_flush`。上游基线为 PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
