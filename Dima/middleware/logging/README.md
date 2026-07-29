# Dima Logging

固定 8 KiB 非阻塞字节环形，生产路径不执行 USB、Flash 或其他 service I/O。

- `PX4_DEBUG/INFO/WARN/ERR` 映射到 `writef()`。
- `writef()` 在 ISR 或 `dima::platform::in_realtime_context()` 为真时，在调用 `snprintf/vsnprintf` 前直接拒绝。
- `write_literal()` 不格式化，可写入预先准备好的文本；环形满时覆盖最旧字节。
- `service_flush()` 仅供 LP/service 任务使用，通过 `ServiceWriter` 对接 USB、串口或其他输出。
- service writer 短写时，当前块未接收部分记入 `service_dropped_bytes`，不会反向阻塞生产者。
- 时间戳来自阶段 1 的全局 `hrt_absolute_time()`。
