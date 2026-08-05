# Dima Logging

PX4 v1.17.0 默认/Release 构建日志兼容层，固定 8 KiB 非阻塞字节环形。

- 模块日志严格输出 `%-5s [%s] ` 前缀，例如 `INFO  [commander] armed\n`。
- `PX4_INFO_RAW` 不增加级别、模块名或换行。
- 提供 `PX4_INFO/WARN/ERR/PANIC/DEBUG`、`PX4_LOG_NAMED` 和 `px4_log_modulename/px4_log_raw`。
- 默认构建裁剪 `PX4_DEBUG`；`RELEASE_BUILD` 同时裁剪 WARN，匹配 PX4 宏语义。
- TRACE/DEBUG_BUILD 专用格式尚未移植，当前编译时 fail-closed。
- ISR 和实时 WorkQueue 在格式化前拒绝；USB 输出由 `modules/logging/LogService` 在 LP WorkQueue 刷新。
- USB CDC 不再输出 HelloWorld；日志包含 Runtime 启动、模块状态变化、结构化 Event 和可格式化故障。每个服务周期最多转储 4 条 Event、最多刷新 512 bytes，USB 断开时保留环形内容。
- Log/Event Ring 由 `ApplicationContext` 在每次 Runtime init/shutdown 边界重置，`LogService::start()` 不得丢弃 Parameter 启动阶段已经产生的记录。

上游基线：PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
