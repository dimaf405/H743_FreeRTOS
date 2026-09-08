/*-----------------------------------------------------------------------
 * H743 SDMMC1 FatFs port.
 * Copyright (C) 2017, ChaN; portions Copyright (C) STMicroelectronics.
 * Portions Copyright (C) Dima Project.
 *
 * 卡识别/接线复用 HAL；运行期命令与 IDMA 参考当前 STM32 HAL/LL 的寄存器
 * 顺序，改为任务等待的事务。HAL_SD_IRQHandler 内的等待式 CMD12 不进入 ISR。
 *-----------------------------------------------------------------------*/
#include "diskio.h"
#include "sdmmc.h"
#include "sdmmc_io.h"
#include "api/Services.hpp"
#include "api/Execution.hpp"
#include "api/Synchronization.hpp"
#include "api/TaskRuntime.hpp"
#include "api/Time.hpp"
#include "api/platform_config.h"
#include "memory/cache.h"
#include "memory/early_memory.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <limits>

#ifndef DIMA_SD_BLOCKING_TIMEOUT_MS
#define DIMA_SD_BLOCKING_TIMEOUT_MS 500U
#endif

namespace {

constexpr std::size_t kSectorBytes = 512U;
constexpr std::size_t kHalfBytes = DIMA_SD_DMA_REGION_SIZE / 2U;
constexpr std::uint32_t kHalfBlocks = kHalfBytes / kSectorBytes;
constexpr std::uint32_t kBatchBlocks = 2U * kHalfBlocks;
constexpr std::uint32_t kIrqPriority = 7U;
constexpr std::uint64_t kTimeoutUs =
    static_cast<std::uint64_t>(DIMA_SD_BLOCKING_TIMEOUT_MS) * 1000ULL;
constexpr std::uint64_t kReadyPollUs = 1000000ULL / DIMA_KERNEL_TICK_HZ;
constexpr std::uint32_t kCommandFlags =
    SDMMC_FLAG_CMDREND | SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT |
    SDMMC_FLAG_BUSYD0END;
constexpr std::uint32_t kDataErrors =
    SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_RXOVERR |
    SDMMC_FLAG_TXUNDERR | SDMMC_FLAG_IDMATE;
constexpr std::uint32_t kDataInterrupts =
    SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR |
    SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND | SDMMC_IT_IDMABTC;
constexpr std::uint32_t kEventCommand = 1UL << 0U;
constexpr std::uint32_t kEventData = 1UL << 1U;
constexpr std::uint32_t kEventHalf0 = 1UL << 2U;
constexpr std::uint32_t kEventHalf1 = 1UL << 3U;
constexpr std::uint32_t kEventError = 1UL << 4U;

static_assert(kHalfBytes == 4096U && kHalfBytes % kSectorBytes == 0U);
static_assert(kIrqPriority >= DIMA_MAX_SYSCALL_INTERRUPT_PRIORITY);

extern "C" std::uint8_t __dima_axi_sram_start__;
extern "C" std::uint8_t __dima_axi_sram_end__;

// MPU 覆盖独立的 8 KiB 区域；TX 先写满本次长度，RX 完成后才读取，不依赖初值。
alignas(DIMA_SD_DMA_REGION_SIZE) std::uint8_t
    g_dma_buffers[2][kHalfBytes] __attribute__((section(".dima_sd_dma")));

enum class Phase : std::uint32_t {
    Idle, Prepare, Command, Data, Stop, CardReady, Complete,
    Quiesce, ReinitializeRequired,
};
enum class BufferState : std::uint32_t {
    Free, Filling, Ready, InDma, CpuCopy,
};

class InterruptLock {
public:
    InterruptLock() noexcept : saved_(__get_PRIMASK()) { __disable_irq(); }
    ~InterruptLock() { if (saved_ == 0U) { __enable_irq(); } }
private:
    std::uint32_t saved_;
};

std::uint32_t data_error(std::uint32_t flags) noexcept
{
    std::uint32_t error{};
    if ((flags & SDMMC_FLAG_DCRCFAIL) != 0U) error |= SDMMC_ERROR_DATA_CRC_FAIL;
    if ((flags & SDMMC_FLAG_DTIMEOUT) != 0U) error |= SDMMC_ERROR_DATA_TIMEOUT;
    if ((flags & SDMMC_FLAG_RXOVERR) != 0U) error |= SDMMC_ERROR_RX_OVERRUN;
    if ((flags & SDMMC_FLAG_TXUNDERR) != 0U) error |= SDMMC_ERROR_TX_UNDERRUN;
    if ((flags & SDMMC_FLAG_IDMATE) != 0U) error |= SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
    return error;
}

bool direct_range(const void *buffer, std::size_t bytes) noexcept
{
    const auto begin = reinterpret_cast<std::uintptr_t>(buffer);
    const auto end = begin + bytes;
    // 只有完整、独占 cache line 的 D1 扇区数据才能直接 DMA；其他内存走双缓冲。
    return buffer != nullptr && bytes != 0U && end >= begin &&
           (begin & 31U) == 0U && bytes % kSectorBytes == 0U &&
           begin >= reinterpret_cast<std::uintptr_t>(&__dima_axi_sram_start__) &&
           end <= reinterpret_cast<std::uintptr_t>(&__dima_axi_sram_end__);
}

class SdPort;
SdPort *g_irq_owner{nullptr};

class SdPort {
public:
    DSTATUS status() const noexcept
    {
        // 无 card-detect：这里只表示旧会话未失效，不能证明当前物理插卡。
        return ready_.load(std::memory_order_acquire) ? 0U : STA_NOINIT;
    }

    DSTATUS initialize() noexcept
    {
        if (ready_.load(std::memory_order_acquire)) return 0U;
        bool expected = false;
        if (!active_.compare_exchange_strong(expected, true)) return STA_NOINIT;
        if (!runtime_ready()) {
            active_.store(false);
            return STA_NOINIT;
        }

        set_phase(Phase::Prepare);
        stop_hardware(false);
        if (hsd1.Instance == SDMMC1) {
            (void)HAL_SD_DeInit(&hsd1);
        }
        hsd1.Instance = SDMMC1;
        hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
        hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
        hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
        hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
        hsd1.Init.ClockDiv = 0U;
        // HAL 的识卡/电气初始化保持原实现；它不属于运行期 500 ms 事务预算。
        const bool initialized = HAL_SD_Init(&hsd1) == HAL_OK;
        if (!initialized) {
            stop_hardware(true);
        }
        {
            const InterruptLock lock;
            if (initialized && ever_ready_) ++statistics_.recoveries;
            if (initialized) ever_ready_ = true;
            statistics_.last_error = initialized ? 0U : hsd1.ErrorCode;
        }
        ready_.store(initialized, std::memory_order_release);
        active_.store(false, std::memory_order_release);
        set_phase(initialized ? Phase::Idle : Phase::ReinitializeRequired);
        return status();
    }

    DRESULT transfer(std::uint8_t *destination, const std::uint8_t *source,
                     std::uint32_t sector, std::uint32_t count) noexcept
    {
        const bool writing = source != nullptr;
        const void *const buffer = writing ? static_cast<const void *>(source)
                                           : static_cast<const void *>(destination);
        const std::uint64_t total_bytes =
            static_cast<std::uint64_t>(count) * kSectorBytes;
        const auto address = reinterpret_cast<std::uintptr_t>(buffer);
        if (buffer == nullptr || count == 0U ||
            total_bytes > std::numeric_limits<std::uintptr_t>::max() - address) {
            return reject();
        }
        if (!begin_request()) return RES_NOTRDY;
        if (sector >= hsd1.SdCard.LogBlockNbr ||
            count > hsd1.SdCard.LogBlockNbr - sector) {
            finish_request(true, false);
            return reject();
        }
        const std::uint64_t deadline = clock_->now_us() + kTimeoutUs;
        bool prefetched = false;
        std::size_t offset{};
        std::uint32_t remaining = count;
        bool success = card_ready(deadline);
        while (success && remaining != 0U) {
            const std::uint32_t blocks = std::min(remaining, kBatchBlocks);
            const std::size_t bytes = blocks * kSectorBytes;
            const std::uint8_t *const tx = writing ? source + offset : nullptr;
            std::uint8_t *const rx = writing ? nullptr : destination + offset;
            const bool direct = direct_range(writing ? tx : rx, bytes);
            const bool dual = !direct && blocks > kHalfBlocks;
            set_phase(Phase::Prepare);

            if (direct) {
                // RX 必须先清理旧脏行并失效；完成/终止后再失效，避免脏行回写覆盖
                // DMA 新数据。调用方在整个同步 disk_* 返回前保有这段内存。
                if (writing) dima_stm32_cache_clean_range(tx, bytes);
                else dima_stm32_cache_clean_invalidate_range(rx, bytes);
            } else if (writing) {
                if (!prefetched) {
                    set_buffer(0U, BufferState::Filling);
                    std::memcpy(g_dma_buffers[0], tx, std::min(bytes, kHalfBytes));
                }
                prefetched = false;
                set_buffer(0U, BufferState::Ready);
                if (dual) {
                    set_buffer(1U, BufferState::Filling);
                    std::memcpy(g_dma_buffers[1], tx + kHalfBytes, bytes - kHalfBytes);
                    set_buffer(1U, BufferState::Ready);
                }
            }
            {
                const InterruptLock lock;
                if (direct) ++statistics_.direct_transfers;
                else ++statistics_.staged_transfers;
                if (dual) ++statistics_.double_buffer_transfers;
            }
            const std::uint8_t *const next_source =
                writing && remaining > blocks ? source + offset + bytes : nullptr;
            const std::size_t next_bytes = next_source != nullptr
                ? std::min<std::size_t>((remaining - blocks) * kSectorBytes, kHalfBytes)
                : 0U;
            std::uint8_t *dma_address = direct
                ? (writing ? const_cast<std::uint8_t *>(tx) : rx)
                : g_dma_buffers[0];
            bool copied_first = false;
            success = start_data(writing, dma_address, sector, blocks, dual, deadline);
            if (success) {
                success = wait_data(writing, rx, bytes, dual, next_source, next_bytes,
                                    copied_first, prefetched, deadline);
            }
            if (success && blocks > 1U) {
                set_phase(Phase::Stop);
                success = command(SDMMC_CMD_STOP_TRANSMISSION, 0U, false,
                                  deadline, nullptr);
            }
            if (success) success = card_ready(deadline);
            if (!success) {
                // 必须先让硬件失去缓冲访问权，才允许 RX 失效或调用方复用。
                finish_request(false);
                if (direct && !writing) {
                    dima_stm32_cache_invalidate_range(rx, bytes);
                }
                return RES_ERROR;
            }
            if (!writing) {
                if (direct) {
                    dima_stm32_cache_invalidate_range(rx, bytes);
                } else {
                    if (!copied_first) {
                        set_buffer(0U, BufferState::CpuCopy);
                        std::memcpy(rx, g_dma_buffers[0], std::min(bytes, kHalfBytes));
                    }
                    if (dual) {
                        set_buffer(1U, BufferState::CpuCopy);
                        std::memcpy(rx + kHalfBytes, g_dma_buffers[1], bytes - kHalfBytes);
                    }
                }
            }
            if (remaining_us(deadline) == 0U) {
                finish_request(false);
                return RES_ERROR;
            }
            {
                const InterruptLock lock;
                if (direct && writing) statistics_.direct_write_bytes += bytes;
                else if (direct) statistics_.direct_read_bytes += bytes;
                else if (writing) statistics_.staged_write_bytes += bytes;
                else statistics_.staged_read_bytes += bytes;
            }
            if (!prefetched) set_buffer(0U, BufferState::Free);
            set_buffer(1U, BufferState::Free);
            remaining -= blocks;
            sector += blocks;
            offset += bytes;
        }
        success = finish_request(success);
        return success ? RES_OK : RES_ERROR;
    }

    DRESULT sync() noexcept
    {
        if (!begin_request()) return RES_NOTRDY;
        bool success = card_ready(clock_->now_us() + kTimeoutUs);
        success = finish_request(success);
        return success ? RES_OK : RES_ERROR;
    }

    DRESULT control(BYTE command_id, void *buffer) noexcept
    {
        if (command_id == CTRL_SYNC) return sync();
        if (buffer == nullptr) return reject();
        if (status() != 0U || active_.load()) return RES_NOTRDY;
        HAL_SD_CardInfoTypeDef info{};
        HAL_SD_GetCardInfo(&hsd1, &info);
        switch (command_id) {
        case GET_SECTOR_COUNT: {
            const DWORD value = info.LogBlockNbr;
            std::memcpy(buffer, &value, sizeof(value));
            return RES_OK;
        }
        case GET_SECTOR_SIZE: {
            const WORD value = static_cast<WORD>(info.LogBlockSize);
            std::memcpy(buffer, &value, sizeof(value));
            return RES_OK;
        }
        case GET_BLOCK_SIZE: {
            const DWORD value = info.LogBlockSize / kSectorBytes;
            std::memcpy(buffer, &value, sizeof(value));
            return RES_OK;
        }
        case MMC_GET_TYPE:
            *static_cast<BYTE *>(buffer) =
                info.CardType == CARD_SDHC_SDXC ? 2U : 1U;
            return RES_OK;
        default:
            return reject();
        }
    }

    void interrupt() noexcept
    {
        const std::uint64_t entered = clock_ != nullptr ? clock_->now_us() : 0U;
        const std::uint32_t status_flags = SDMMC1->STA;
        const std::uint32_t pending = status_flags & SDMMC1->MASK;
        const std::uint32_t dma_control = SDMMC1->IDMACTRL;
        bool notify = false;
        {
            const InterruptLock lock;
            ++statistics_.irq_count;
            statistics_.last_status = status_flags;
            if (!active_.load() || pending == 0U) {
                ++statistics_.spurious_irqs;
                if (!active_.load()) {
                    SDMMC1->MASK = 0U;
                    SDMMC1->ICR = SDMMC_STATIC_FLAGS;
                }
            } else {
                // 命令标志留给任务中的 LL 响应解析，只屏蔽来源，避免重复进入。
                if ((pending & kCommandFlags) != 0U) {
                    SDMMC1->MASK &= ~kCommandFlags;
                    events_.fetch_or(kEventCommand, std::memory_order_release);
                }
                const std::uint32_t error = data_error(status_flags & kDataErrors);
                if (error != 0U) {
                    // 同时出现错误与 DATAEND 时错误优先；ISR 只停止数据通路，
                    // CMD12/CMD13 与恢复交给任务，绝不在此调用等待式 HAL IRQ。
                    error_.fetch_or(error, std::memory_order_release);
                    events_.fetch_or(kEventError, std::memory_order_release);
                    SDMMC1->MASK &= ~kDataInterrupts;
                    SDMMC1->IDMACTRL = SDMMC_DISABLE_IDMA;
                    SDMMC1->DCTRL = 0U;
                    __SDMMC_CMDTRANS_DISABLE(SDMMC1);
                } else {
                    if ((pending & SDMMC_FLAG_IDMABTC) != 0U) {
                        const std::uint32_t finished =
                            (dma_control & SDMMC_IDMA_IDMABACT) != 0U
                                ? kEventHalf0 : kEventHalf1;
                        events_.fetch_or(finished, std::memory_order_release);
                    }
                    if ((pending & SDMMC_FLAG_DATAEND) != 0U) {
                        SDMMC1->MASK &= ~kDataInterrupts;
                        SDMMC1->IDMACTRL = SDMMC_DISABLE_IDMA;
                        SDMMC1->DCTRL = 0U;
                        SDMMC1->DLEN = 0U;
                        __SDMMC_CMDTRANS_DISABLE(SDMMC1);
                        events_.fetch_or(kEventData, std::memory_order_release);
                    }
                }
                SDMMC1->ICR = status_flags &
                    (SDMMC_STATIC_DATA_FLAGS | SDMMC_FLAG_IDMATE | SDMMC_FLAG_IDMABTC);
                event_generation_.store(generation_.load(), std::memory_order_release);
                notify = true;
            }
        }
        if (notify) completion_.notify_from_isr();
        const std::uint64_t elapsed = clock_ != nullptr ? clock_->now_us() - entered : 0U;
        {
            const InterruptLock lock;
            statistics_.irq_total_us += elapsed;
            statistics_.irq_max_us = std::max(statistics_.irq_max_us,
                static_cast<std::uint32_t>(std::min<std::uint64_t>(
                    elapsed, std::numeric_limits<std::uint32_t>::max())));
        }
    }

    void snapshot(DimaSdIoStats &output) const noexcept
    {
        const InterruptLock lock;
        output = statistics_;
        output.media_ready = ready_.load();
        output.active = active_.load();
        output.generation = generation_.load();
    }

private:
    bool runtime_ready() noexcept
    {
        auto *services = dima::platform::try_services();
        if (services == nullptr || !services->execution.scheduler_running() ||
            services->execution.in_realtime_task()) return false;
        if (!completion_.valid() &&
            !completion_.initialize(services->synchronization)) return false;
        clock_ = &services->clock;
        tasks_ = &services->tasks;
        {
            const InterruptLock lock;
            g_irq_owner = this;
        }
        return true;
    }

    bool begin_request() noexcept
    {
        if (!ready_.load()) return false;
        bool expected = false;
        if (!active_.compare_exchange_strong(expected, true)) return false;
        // 单请求所有权也覆盖首次 Signal 创建，避免并发初始化占用多个资源槽。
        if (!runtime_ready() || !ready_.load()) {
            active_.store(false);
            return false;
        }
        if (stop_hardware(false)) {
            // 上次会话若仍残留硬件活动，复位已破坏卡配置，禁止继续发送新命令。
            error_.store(SDMMC_ERROR_GENERAL_UNKNOWN_ERR);
            (void)finish_request(false);
            return false;
        }
        (void)completion_.wait(dima::platform::Timeout::from_us(0U));
        {
            const InterruptLock lock;
            std::uint32_t next = generation_.load() + 1U;
            if (next == 0U) next = 1U;
            generation_.store(next);
            event_generation_.store(next);
            events_.store(0U);
            error_.store(0U);
            ++statistics_.requests;
            statistics_.last_error = 0U;
            statistics_.phase = static_cast<std::uint32_t>(Phase::Prepare);
            NVIC_SetPriority(SDMMC1_IRQn, kIrqPriority);
            NVIC_ClearPendingIRQ(SDMMC1_IRQn);
            NVIC_EnableIRQ(SDMMC1_IRQn);
        }
        return true;
    }

    void set_phase(Phase phase) noexcept
    {
        const InterruptLock lock;
        statistics_.phase = static_cast<std::uint32_t>(phase);
    }

    void set_buffer(std::size_t index, BufferState state) noexcept
    {
        const InterruptLock lock;
        statistics_.buffer_state[index] = static_cast<std::uint32_t>(state);
    }

    DRESULT reject() noexcept
    {
        const InterruptLock lock;
        ++statistics_.rejected_requests;
        return RES_PARERR;
    }

    bool failed() noexcept
    {
        const std::uint32_t status_flags = SDMMC1->STA;
        const std::uint32_t flags = status_flags & kDataErrors;
        if (flags != 0U) {
            error_.fetch_or(data_error(flags));
            const InterruptLock lock;
            statistics_.last_status = status_flags;
        }
        return error_.load(std::memory_order_acquire) != 0U;
    }

    std::uint64_t remaining_us(std::uint64_t deadline) noexcept
    {
        const std::uint64_t now = clock_->now_us();
        if (now >= deadline) {
            error_.fetch_or(SDMMC_ERROR_TIMEOUT);
            return 0U;
        }
        return deadline - now;
    }

    bool wait_signal(std::uint64_t deadline) noexcept
    {
        if (failed()) return false;
        const std::uint64_t remaining = remaining_us(deadline);
        if (remaining == 0U) return false;
        (void)completion_.wait(dima::platform::Timeout::from_us(remaining));
        // 信号只是唤醒提示，事件/代次才是事实；先检查错误，覆盖超时边沿的 IRQ。
        if (failed()) return false;
        // 旧的二值信号只触发重查；current_events 会过滤不属于当前代次的事件。
        return remaining_us(deadline) != 0U;
    }

    std::uint32_t current_events() const noexcept
    {
        const InterruptLock lock;
        return event_generation_.load(std::memory_order_acquire) == generation_.load()
            ? events_.load(std::memory_order_acquire) : 0U;
    }

    bool delay_ready(std::uint64_t deadline) noexcept
    {
        const std::uint64_t remaining = remaining_us(deadline);
        if (remaining == 0U || failed()) return false;
        tasks_->delay(dima::platform::Timeout::from_us(std::min(remaining, kReadyPollUs)));
        return remaining_us(deadline) != 0U && !failed();
    }

    bool command(std::uint8_t index, std::uint32_t argument, bool data_command,
                 std::uint64_t deadline, std::uint32_t *response) noexcept
    {
        if (failed() || remaining_us(deadline) == 0U) return false;
        SDMMC_CmdInitTypeDef config{};
        config.Argument = argument;
        config.CmdIndex = index;
        config.Response = SDMMC_RESPONSE_SHORT;
        config.WaitForInterrupt = SDMMC_WAIT_NO;
        config.CPSM = SDMMC_CPSM_ENABLE;
        {
            const InterruptLock lock;
            events_.fetch_and(~kEventCommand);
            SDMMC1->ICR = SDMMC_STATIC_CMD_FLAGS;
            if (data_command) __SDMMC_CMDTRANS_ENABLE(SDMMC1);
            else __SDMMC_CMDTRANS_DISABLE(SDMMC1);
            if (index == SDMMC_CMD_STOP_TRANSMISSION) __SDMMC_CMDSTOP_ENABLE(SDMMC1);
            else __SDMMC_CMDSTOP_DISABLE(SDMMC1);
            SDMMC1->MASK |= kCommandFlags;
            (void)SDMMC_SendCommand(SDMMC1, &config);
            __DSB();
        }
        // 非阻塞发命令；只在终态已到且 CPSM 空闲后复用 LL 的 R1 解码。
        // HAL 默认的 5 s/超长 CMD12 等待不进入本事务，也不会发生在中断中。
        for (;;) {
            if (failed() || remaining_us(deadline) == 0U) return false;
            const std::uint32_t status_flags = SDMMC1->STA;
            const bool normal_terminal = (status_flags &
                (SDMMC_FLAG_CMDREND | SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CTIMEOUT)) != 0U;
            const bool busy_terminal = (status_flags & SDMMC_FLAG_BUSYD0END) != 0U &&
                SDMMC_GetCommandResponse(SDMMC1) == index;
            if ((normal_terminal || busy_terminal) &&
                (status_flags & SDMMC_FLAG_CMDACT) == 0U) break;
            if ((current_events() & kEventCommand) != 0U) {
                if (!delay_ready(deadline)) return false;
            } else if (!wait_signal(deadline)) {
                return false;
            }
        }
        {
            const InterruptLock lock;
            SDMMC1->MASK &= ~kCommandFlags;
        }
        const std::uint32_t value = SDMMC_GetResponse(SDMMC1, SDMMC_RESP1);
        std::uint32_t error = SDMMC_GetCmdResp1(SDMMC1, index, 1U);
        {
            const InterruptLock lock;
            __SDMMC_CMDSTOP_DISABLE(SDMMC1);
            events_.fetch_and(~kEventCommand);
        }
        // 沿用 LL 的 CMD12 末地址处理，不把它扩展到实际读写命令。
        if (index == SDMMC_CMD_STOP_TRANSMISSION &&
            error == SDMMC_ERROR_ADDR_OUT_OF_RANGE) error = SDMMC_ERROR_NONE;
        if (error != SDMMC_ERROR_NONE) {
            error_.fetch_or(error);
            return false;
        }
        if (response != nullptr) *response = value;
        return !failed() && remaining_us(deadline) != 0U;
    }

    bool card_ready(std::uint64_t deadline) noexcept
    {
        set_phase(Phase::CardReady);
        for (;;) {
            std::uint32_t response{};
            if (!command(SDMMC_CMD_SEND_STATUS,
                         static_cast<std::uint32_t>(hsd1.SdCard.RelCardAdd) << 16U,
                         false, deadline, &response)) return false;
            const std::uint32_t state = (response >> 9U) & 0x0FU;
            // R1 的 READY_FOR_DATA(bit 8) 与 TRANSFER 状态必须同时满足，避免
            // 卡仍忙时提前承认 CTRL_SYNC 完成或开始下一笔读写。
            if (state == HAL_SD_CARD_TRANSFER && (response & (1UL << 8U)) != 0U) {
                return true;
            }
            if (!delay_ready(deadline)) return false;
        }
    }

    bool start_data(bool writing, std::uint8_t *address, std::uint32_t sector,
                    std::uint32_t blocks, bool dual, std::uint64_t deadline) noexcept
    {
        if (failed() || remaining_us(deadline) == 0U) return false;
        SDMMC_DataInitTypeDef config{};
        config.DataTimeOut = SDMMC_DATATIMEOUT;
        config.DataLength = blocks * kSectorBytes;
        config.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
        config.TransferDir = writing ? SDMMC_TRANSFER_DIR_TO_CARD
                                     : SDMMC_TRANSFER_DIR_TO_SDMMC;
        config.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
        config.DPSM = SDMMC_DPSM_DISABLE;
        {
            const InterruptLock lock;
            events_.fetch_and(~(kEventData | kEventHalf0 | kEventHalf1));
            SDMMC1->MASK &= ~kDataInterrupts;
            SDMMC1->ICR = SDMMC_STATIC_DATA_FLAGS;
            SDMMC1->DCTRL = 0U;
            (void)SDMMC_ConfigData(SDMMC1, &config);
            if (!writing && dual) {
                // 与当前 HAL_SDEx 的双缓冲读初始化一致，清除上一笔 FIFO 状态。
                SDMMC1->DCTRL |= SDMMC_DCTRL_FIFORST;
            }
            SDMMC1->IDMABASE0 = reinterpret_cast<std::uint32_t>(address);
            SDMMC1->IDMABASE1 = dual
                ? reinterpret_cast<std::uint32_t>(g_dma_buffers[1]) : 0U;
            SDMMC1->IDMABSIZE = dual ? kHalfBytes : 0U;
            __DSB();
            SDMMC1->IDMACTRL = dual ? SDMMC_ENABLE_IDMA_DOUBLE_BUFF0
                                   : SDMMC_ENABLE_IDMA_SINGLE_BUFF;
            SDMMC1->MASK |= kDataInterrupts & (dual ? UINT32_MAX : ~SDMMC_IT_IDMABTC);
            if (address == g_dma_buffers[0]) {
                statistics_.buffer_state[0] = static_cast<std::uint32_t>(BufferState::InDma);
            }
            if (dual) {
                statistics_.buffer_state[1] = static_cast<std::uint32_t>(BufferState::InDma);
            }
        }
        hsd1.State = HAL_SD_STATE_BUSY;
        hsd1.Context = SD_CONTEXT_DMA | (writing
            ? (blocks > 1U ? SD_CONTEXT_WRITE_MULTIPLE_BLOCK : SD_CONTEXT_WRITE_SINGLE_BLOCK)
            : (blocks > 1U ? SD_CONTEXT_READ_MULTIPLE_BLOCK : SD_CONTEXT_READ_SINGLE_BLOCK));
        const std::uint32_t argument = hsd1.SdCard.CardType == CARD_SDHC_SDXC
            ? sector : sector * kSectorBytes;
        const std::uint8_t index = writing
            ? (blocks > 1U ? SDMMC_CMD_WRITE_MULT_BLOCK : SDMMC_CMD_WRITE_SINGLE_BLOCK)
            : (blocks > 1U ? SDMMC_CMD_READ_MULT_BLOCK : SDMMC_CMD_READ_SINGLE_BLOCK);
        set_phase(Phase::Command);
        return command(index, argument, true, deadline, nullptr);
    }

    bool wait_data(bool writing, std::uint8_t *destination, std::size_t bytes,
                   bool dual, const std::uint8_t *next_source, std::size_t next_bytes,
                   bool &copied_first, bool &prefetched, std::uint64_t deadline) noexcept
    {
        set_phase(Phase::Data);
        bool first_handled = false;
        for (;;) {
            if (failed() || remaining_us(deadline) == 0U) return false;
            const std::uint32_t events = current_events();
            if (dual && !first_handled &&
                (events & (kEventHalf0 | kEventHalf1 | kEventData)) != 0U) {
                // 每次 DLEN 最多两个半区，因此半区 0 完成后硬件不会再次访问它。
                // 即使 storage 被抢占错过中间 IRQ，也只失去流水重叠，不会覆盖数据。
                const bool overlap = (events & kEventData) == 0U &&
                    (SDMMC1->IDMACTRL & SDMMC_IDMA_IDMAEN) != 0U;
                set_buffer(0U, BufferState::Free);
                std::size_t copied{};
                if (writing && next_source != nullptr && next_bytes != 0U) {
                    set_buffer(0U, BufferState::Filling);
                    std::memcpy(g_dma_buffers[0], next_source, next_bytes);
                    set_buffer(0U, BufferState::Ready);
                    prefetched = true;
                    copied = next_bytes;
                } else if (!writing) {
                    set_buffer(0U, BufferState::CpuCopy);
                    std::memcpy(destination, g_dma_buffers[0], std::min(bytes, kHalfBytes));
                    set_buffer(0U, BufferState::Free);
                    copied_first = true;
                    copied = std::min(bytes, kHalfBytes);
                }
                if (overlap) {
                    const InterruptLock lock;
                    statistics_.pipeline_copy_bytes += copied;
                }
                first_handled = true;
                events_.fetch_and(~(kEventHalf0 | kEventHalf1));
            }
            if ((current_events() & kEventData) != 0U) {
                return !failed();
            }
            if (!wait_signal(deadline)) return false;
        }
    }

    bool stop_hardware(bool reset) noexcept
    {
        bool reset_performed = false;
        const InterruptLock lock;
        NVIC_DisableIRQ(SDMMC1_IRQn);
        if (hsd1.Instance == SDMMC1) {
            SDMMC1->MASK = 0U;
            SDMMC1->IDMACTRL = SDMMC_DISABLE_IDMA;
            SDMMC1->DCTRL = 0U;
            SDMMC1->DLEN = 0U;
            __SDMMC_CMDTRANS_DISABLE(SDMMC1);
            __SDMMC_CMDSTOP_DISABLE(SDMMC1);
            __DSB();
            if (reset || (SDMMC1->STA &
                (SDMMC_FLAG_DPSMACT | SDMMC_FLAG_CMDACT)) != 0U) {
                // 不调用可能再次等待卡响应的 HAL_SD_Abort；复位后才撤销所有权，
                // 旧硬件中断与旧数据请求不能落进下一代缓冲使用期。
                __HAL_RCC_SDMMC1_FORCE_RESET();
                __DSB();
                __HAL_RCC_SDMMC1_RELEASE_RESET();
                __DSB();
                ++statistics_.forced_resets;
                reset_performed = true;
            }
            SDMMC1->ICR = SDMMC_STATIC_FLAGS;
        }
        NVIC_ClearPendingIRQ(SDMMC1_IRQn);
        __DSB();
        return reset_performed;
    }

    bool finish_request(bool success, bool completed = true) noexcept
    {
        set_phase(success ? Phase::Complete : Phase::Quiesce);
        if (stop_hardware(!success) && success) {
            // 即使已收到完成，只要最终静默检查需要复位，旧会话就不能继续使用，
            // 同步接口也必须返回失败，不能把已丢失的外设配置当成正常完成。
            success = false;
            error_.fetch_or(SDMMC_ERROR_GENERAL_UNKNOWN_ERR);
        }
        const std::uint32_t error = error_.load();
        {
            const InterruptLock lock;
            if (success && completed) ++statistics_.completed_requests;
            if (!success) {
                ++statistics_.errors;
                if ((error & (SDMMC_ERROR_TIMEOUT | SDMMC_ERROR_CMD_RSP_TIMEOUT |
                              SDMMC_ERROR_DATA_TIMEOUT)) != 0U) ++statistics_.timeouts;
                statistics_.last_error = error;
                ready_.store(false, std::memory_order_release);
            }
            statistics_.buffer_state[0] = static_cast<std::uint32_t>(BufferState::Free);
            statistics_.buffer_state[1] = static_cast<std::uint32_t>(BufferState::Free);
            statistics_.phase = static_cast<std::uint32_t>(
                success ? Phase::Complete : Phase::ReinitializeRequired);
            hsd1.ErrorCode = error;
            hsd1.Context = SD_CONTEXT_NONE;
            hsd1.State = success ? HAL_SD_STATE_READY : HAL_SD_STATE_RESET;
            active_.store(false, std::memory_order_release);
        }
        return success;
    }

    dima::platform::Signal completion_{};
    dima::platform::MonotonicClock *clock_{nullptr};
    dima::platform::TaskRuntime *tasks_{nullptr};
    std::atomic<std::uint32_t> events_{0U};
    std::atomic<std::uint32_t> error_{0U};
    std::atomic<std::uint32_t> generation_{0U};
    std::atomic<std::uint32_t> event_generation_{0U};
    std::atomic<bool> active_{false};
    std::atomic<bool> ready_{false};
    bool ever_ready_{false};
    DimaSdIoStats statistics_{};
};

SdPort &port() noexcept
{
    static SdPort instance;
    return instance;
}

} // namespace

extern "C" DSTATUS disk_initialize(BYTE drive)
{
    return drive == 0U ? port().initialize() : STA_NOINIT;
}

extern "C" DSTATUS disk_status(BYTE drive)
{
    return drive == 0U ? port().status() : STA_NOINIT;
}

extern "C" DRESULT disk_read(BYTE drive, BYTE *buffer, DWORD sector, UINT count)
{
    return drive == 0U ? port().transfer(buffer, nullptr, sector, count) : RES_PARERR;
}

extern "C" DRESULT disk_write(BYTE drive, const BYTE *buffer, DWORD sector, UINT count)
{
    return drive == 0U ? port().transfer(nullptr, buffer, sector, count) : RES_PARERR;
}

extern "C" DRESULT disk_ioctl(BYTE drive, BYTE command_id, void *buffer)
{
    return drive == 0U ? port().control(command_id, buffer) : RES_PARERR;
}

extern "C" bool dima_sdmmc_get_io_stats(DimaSdIoStats *statistics)
{
    if (statistics == nullptr) return false;
    port().snapshot(*statistics);
    return true;
}

extern "C" void SDMMC1_IRQHandler(void)
{
    if (g_irq_owner != nullptr) {
        g_irq_owner->interrupt();
    } else {
        NVIC_DisableIRQ(SDMMC1_IRQn);
        NVIC_ClearPendingIRQ(SDMMC1_IRQn);
    }
}
