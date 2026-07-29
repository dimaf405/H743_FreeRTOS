# Dima Perf

提供 PX4 风格的 `perf_alloc/free/begin/end/count/set_elapsed` 接口。

- 64 个计数器固定对象池；`perf_alloc()` 仅用于初始化或模块启动。
- `perf_count()`、`perf_begin()`、`perf_end()`、`perf_set_elapsed()` 不分配内存。
- `PC_ELAPSED` 和 `PC_INTERVAL` 使用微秒级 `hrt_absolute_time()`。
- 名称仅保存 `const char *`，调用方必须保证字符串具有静态生命周期。
- `perf_get_snapshot()` 提供无格式化统计快照，输出和格式化由 service 层完成。
