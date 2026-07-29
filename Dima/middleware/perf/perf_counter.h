#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum perf_counter_type {
    PC_COUNT = 0,
    PC_ELAPSED,
    PC_INTERVAL,
};

struct perf_ctr_header;
typedef struct perf_ctr_header *perf_counter_t;

struct perf_counter_snapshot {
    enum perf_counter_type type;
    const char *name;
    uint64_t event_count;
    uint64_t total;
    uint64_t minimum;
    uint64_t maximum;
    uint64_t last;
    bool active;
};

// perf_alloc 使用固定对象池，仅允许在初始化或模块启动阶段调用。
perf_counter_t perf_alloc(enum perf_counter_type type, const char *name);
void perf_free(perf_counter_t handle);

// 以下运行期操作不进行任何动态分配。
void perf_begin(perf_counter_t handle);
void perf_end(perf_counter_t handle);
void perf_count(perf_counter_t handle);
void perf_set_elapsed(perf_counter_t handle, int64_t elapsed);

bool perf_get_snapshot(perf_counter_t handle,
                       struct perf_counter_snapshot *snapshot);
void perf_reset(perf_counter_t handle);
size_t perf_allocated_count(void);

#ifdef __cplusplus
}
#endif
