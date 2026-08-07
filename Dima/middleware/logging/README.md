# Dima Logging

PX4 v1.17.0 风格日志兼容层，固定 8 KiB 非阻塞字节环形。

- 模块日志严格输出 `%-5s [%s] ` 前缀，例如 `INFO  [commander] armed\n`。
- `PX4_INFO_RAW` 不增加级别、模块名或换行。
- 提供 `PX4_INFO/WARN/ERR/PANIC/DEBUG`、`PX4_LOG_NAMED` 和 `px4_log_modulename/px4_log_raw`。
- `debug_config.hpp` 是唯一的编译期调试策略文件。它统一设置 USB 最低等级，并为 `System`、`Sbus`、`Icm42688` 保存各自的最低等级、数据输出开关和周期；后续外设继续加入同一文件。
- 默认 USB 最低等级为 Debug，System 为 Info，SBUS 为 Error，ICM42688 为 Off。SBUS 仍保留 100 ms 周期数据入口，但只有显式把 SBUS 最低等级降到 Debug 后才会输出；后续 MAVLink 接入将重新定义诊断传输边界。`PX4_DEBUG` 保留真实路径，由全局和 Source 两级策略过滤，不再由构建宏直接裁剪。
- TRACE/DEBUG_BUILD 专用格式尚未移植，当前编译时 fail-closed。
- ISR 和实时 WorkQueue 在格式化前拒绝；USB 输出由 `modules/logging/LogService` 在 LP WorkQueue 刷新。
- USB CDC 不再输出 HelloWorld；日志包含 Runtime 启动、模块状态变化、结构化 Event 和可格式化故障。每个服务周期最多转储 4 条 Event、最多刷新 512 bytes，USB 断开时保留环形内容。
- `LogService` 在低优先级队列订阅 `input_rc`。USB 就绪时最多每 100 ms 打印一个严格更新的最新 SBUS 样本，包含 18 路 PWM、Failsafe、RC lost、总帧数和丢帧数；断线期间不生成连续数据，重连不补发历史样本。
- `PX4_INFO_RAW`、参数查询结果和 USB 控制响应绕过诊断等级过滤。Error/Critical 结构化 Event 仍先进入独立 Event Ring 和故障锁存，外设 Debug 策略不改变安全故障记录。
- Log/Event Ring 由 `ApplicationContext` 在每次 Runtime init/shutdown 边界重置，`LogService::start()` 不得丢弃 Parameter 启动阶段已经产生的记录。

## 禁止事项

- 不反向依赖 `rover/`，不直接包含 HAL/CMSIS/Core/Board 头文件。
- 在 `namespace dima::logging` 内引用 `dima::platform` 符号时**必须使用 `::dima::platform::` 全局前缀**，避免被解析为 `dima::logging::dima::platform::`。

## 文件清单

| 文件 | 职责 |
|---|---|
| `logging.hpp` | 公开 API——`writef`、`write_module`、`write_literal`、`service_flush`、`stats`、`reset`、`set_structured_sink`；`LogStats`/`WriteResult`/`ServiceWriter` 类型定义 |
| `logging.cpp` | 8 KiB 环形缓冲实现、中断/实时上下文过滤、结构化 sink 分发、PX4 C ABI 桥接 |
| `debug_config.hpp` | 编译期 Source 级调试策略——USB/System/SBUS/ICM42688 最低等级与数据周期 |

上游基线：PX4 v1.17.0 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
