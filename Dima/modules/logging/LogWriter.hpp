/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2026 Dima Project. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "api/LogFileStore.hpp"
#include "containers/atomic.h"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::logging {

/**
 * PX4 LogWriterFile 的 FreeRTOS/FatFs 适配。
 *
 * Logger producer 只有一个，storage worker 也只有一个，因此使用单生产者/
 * 单消费者字节 Ring。producer 只复制 RAM 并发布写指针，任何 f_write/f_sync/
 * close 和最多 500 ms 的 SDMMC 等待都固定留在 wq:storage。
 */
class LogWriter final {
public:
    explicit LogWriter(dima::platform::LogFileStore &store) noexcept;

    bool start() noexcept;
    void request_stop() noexcept;
    void stop() noexcept;

    bool ready() const noexcept;
    std::uint32_t session_generation() const noexcept;
    std::size_t available_bytes() const noexcept;

    /**
     * 原子发布一条完整 ULog 消息。返回 false 表示本条未进入 Ring，调用方必须
     * 对 header/format 重试，或对飞行数据开始 PX4 dropout 计时。
     */
    bool write_message(const void *message, std::size_t size) noexcept;

private:
    class StorageWorker final : public px4::ScheduledWorkItem {
    public:
        explicit StorageWorker(LogWriter &owner) noexcept;

    protected:
        void Run() override;

    private:
        LogWriter &owner_;
    };

    static constexpr std::uint32_t kRingCapacity = 64U * 1024U;
    static constexpr std::uint32_t kRingMask = kRingCapacity - 1U;
    static constexpr std::uint32_t kWriteChunkBytes = 4096U;
    static constexpr std::uint32_t kRunIntervalUs = 20000U;
    static constexpr std::uint64_t kRetryIntervalUs = 3000000ULL;
    static constexpr std::uint64_t kSyncIntervalUs = 1000000ULL;

    static_assert((kRingCapacity & kRingMask) == 0U,
                  "ULog ring capacity must be a power of two");

    std::uint32_t pending_bytes() const noexcept;
    void discard_ring() noexcept;
    bool append_one_chunk() noexcept;
    void handle_storage_failure(std::uint64_t now_us) noexcept;
    bool open_file(std::uint64_t now_us) noexcept;
    void finish_stop() noexcept;
    void run_storage() noexcept;

    dima::platform::LogFileStore &store_;
    StorageWorker worker_;
    alignas(32) std::uint8_t ring_[kRingCapacity]{};
    px4::atomic<std::uint32_t> read_position_{0U};
    px4::atomic<std::uint32_t> write_position_{0U};
    px4::atomic<std::uint32_t> session_generation_{0U};
    px4::atomic_bool accepting_{false};
    px4::atomic_bool running_{false};
    px4::atomic_bool stop_requested_{false};
    std::uint64_t last_open_attempt_us_{0U};
    std::uint64_t last_sync_us_{0U};
};

} // namespace dima::modules::logging
