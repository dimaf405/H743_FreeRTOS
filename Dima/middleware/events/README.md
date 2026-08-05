# Dima Events

固定容量结构化事件通道，不在上报路径执行字符串格式化、USB 或 Flash I/O。

- `events.hpp`：`DimaEvent`、128 条环形缓冲、统计和关键故障锁存 API。
- `events.cpp`：任务/ISR 可调用的有界实现；满时优先覆盖最旧非关键记录。
- `modules/logging/LogService` 仅在 USB 就绪时按固定每周期上限消费，并在 LP WorkQueue 格式化为调试日志；ISR 和实时路径不执行格式化或 USB I/O。
- 关键故障保存首次和最近一次事件，必须显式调用 `clear_critical_fault()` 清除。
- 时间戳来自阶段 1 的全局 `hrt_absolute_time()`。
