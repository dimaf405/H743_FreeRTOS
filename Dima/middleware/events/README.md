# Dima Events

固定容量结构化事件通道，不在上报路径执行字符串格式化、USB 或 Flash I/O。

- `events.hpp`：`DimaEvent`、128 条环形缓冲、统计和关键故障锁存 API。
- `events.cpp`：任务/ISR 可调用的有界实现；满时优先覆盖最旧非关键记录。
- 关键故障保存首次和最近一次事件，必须显式调用 `clear_critical_fault()` 清除。
- 时间戳来自阶段 1 的全局 `hrt_absolute_time()`。
