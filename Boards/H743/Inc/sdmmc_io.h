#ifndef DIMA_BOARD_SDMMC_IO_H
#define DIMA_BOARD_SDMMC_IO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 板级只读诊断，不是参数或消息协议；计数不改变传输/恢复策略。 */
typedef struct {
    uint64_t direct_read_bytes;
    uint64_t direct_write_bytes;
    uint64_t staged_read_bytes;
    uint64_t staged_write_bytes;
    uint64_t pipeline_copy_bytes;
    uint64_t irq_total_us;
    uint32_t requests;
    uint32_t completed_requests;
    uint32_t rejected_requests;
    uint32_t direct_transfers;
    uint32_t staged_transfers;
    uint32_t double_buffer_transfers;
    uint32_t errors;
    uint32_t timeouts;
    uint32_t recoveries;
    uint32_t forced_resets;
    uint32_t irq_count;
    uint32_t spurious_irqs;
    uint32_t irq_max_us;
    uint32_t generation;
    uint32_t last_error;
    uint32_t last_status;
    uint32_t phase;
    uint32_t buffer_state[2];
    bool media_ready;
    bool active;
} DimaSdIoStats;

bool dima_sdmmc_get_io_stats(DimaSdIoStats *statistics);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_BOARD_SDMMC_IO_H */
