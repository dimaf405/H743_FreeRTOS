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

#include "LogWriter.hpp"

#include "api/Services.hpp"
#include "api/TaskRuntime.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cstring>

namespace dima::modules::logging {

LogWriter::StorageWorker::StorageWorker(LogWriter &owner) noexcept
    : ScheduledWorkItem("ulog_file", px4::wq_configurations::storage),
      owner_(owner)
{
}

void LogWriter::StorageWorker::Run()
{
    owner_.run_storage();
}

LogWriter::LogWriter(dima::platform::LogFileStore &store) noexcept
    : store_(store), worker_(*this)
{
}

bool LogWriter::start() noexcept
{
    if (running_.load()) {
        return true;
    }

    accepting_.store(false);
    stop_requested_.store(false);
    read_position_.store(0U);
    write_position_.store(0U);
    last_open_attempt_us_ = 0U;
    last_sync_us_ = 0U;
    running_.store(true);

    if (!worker_.ScheduleEnable() ||
        !worker_.ScheduleOnInterval(kRunIntervalUs, 1U)) {
        running_.store(false);
        worker_.ScheduleCancelAndDrain();
        return false;
    }
    return true;
}

void LogWriter::stop() noexcept
{
    if (!running_.load()) {
        worker_.ScheduleCancelAndDrain();
        return;
    }

    /* producer 已在调用方先 drain；此处封住新写入，再让 storage worker 把 Ring
     * 尾部同步并关闭。这里与 PX4 LogWriterFile::thread_stop() 的 pthread_join
     * 语义一致：必须等 consumer 明确完成 close，不能在固定超时边沿取消一个
     * 尚未开始的 storage Run，否则 FIL 会保持打开。底层每次 HAL 等待仍受
     * 500 ms 上限约束，因此正常停机不会被单次坏卡访问无限阻塞。 */
    request_stop();

    while (running_.load()) {
        dima::platform::services().tasks.delay(
            dima::platform::Timeout::from_ms(1U));
    }

    /* running_ 只会在 storage worker 完成 close 后清零；此时再取消周期调度，
     * drain 不会从当前调用线程直接触碰 FatFs，也不会丢掉最后一次关闭任务。 */
    worker_.ScheduleCancelAndDrain();
    running_.store(false);
    stop_requested_.store(false);
    accepting_.store(false);
    discard_ring();
}

void LogWriter::request_stop() noexcept
{
    if (!running_.load()) {
        return;
    }
    accepting_.store(false);
    stop_requested_.store(true);
    (void)worker_.ScheduleNow();
}

bool LogWriter::ready() const noexcept
{
    return running_.load() && accepting_.load();
}

std::uint32_t LogWriter::session_generation() const noexcept
{
    return session_generation_.load();
}

std::uint32_t LogWriter::pending_bytes() const noexcept
{
    const std::uint32_t written = write_position_.load();
    const std::uint32_t read = read_position_.load();
    const std::uint32_t pending = written - read;
    return pending <= kRingCapacity ? pending : kRingCapacity;
}

std::size_t LogWriter::available_bytes() const noexcept
{
    return ready() ? kRingCapacity - pending_bytes() : 0U;
}

bool LogWriter::write_message(const void *message, std::size_t size) noexcept
{
    if (message == nullptr || size == 0U || size > kRingCapacity ||
        !ready()) {
        return false;
    }

    const std::uint32_t generation = session_generation_.load();
    const std::uint32_t read = read_position_.load();
    const std::uint32_t written = write_position_.load();
    const std::uint32_t pending = written - read;
    if (pending > kRingCapacity || size > kRingCapacity - pending) {
        return false;
    }

    /* SPSC 发布顺序：consumer 在 read_position 前的字节未释放；producer 先复制
     * 完整消息，再原子推进 write_position。consumer 因而永远看不到半条 ULog
     * 消息，慢速 f_write 期间也不会被覆盖正在读取的连续片段。 */
    const std::uint32_t index = written & kRingMask;
    const std::uint32_t first = static_cast<std::uint32_t>(
        std::min<std::size_t>(size, kRingCapacity - index));
    std::memcpy(&ring_[index], message, first);
    if (first < size) {
        std::memcpy(ring_, static_cast<const std::uint8_t *>(message) + first,
                    size - first);
    }

    /* 介质失效会先撤销 accepting 并推进 session generation；若它恰好发生在
     * RAM copy 中，禁止把旧文件的尾部发布进新文件。 */
    if (!accepting_.load() || session_generation_.load() != generation) {
        return false;
    }
    write_position_.store(written + static_cast<std::uint32_t>(size));
    (void)worker_.ScheduleNow();
    return true;
}

void LogWriter::discard_ring() noexcept
{
    const std::uint32_t written = write_position_.load();
    read_position_.store(written);
}

bool LogWriter::append_one_chunk() noexcept
{
    const std::uint32_t read = read_position_.load();
    const std::uint32_t written = write_position_.load();
    const std::uint32_t pending = written - read;
    if (pending == 0U) {
        return true;
    }
    if (pending > kRingCapacity) {
        return false;
    }

    const std::uint32_t index = read & kRingMask;
    const std::uint32_t contiguous = std::min(
        std::min(pending, kRingCapacity - index), kWriteChunkBytes);
    if (store_.append_log(&ring_[index], contiguous) != 0) {
        return false;
    }
    read_position_.store(read + contiguous);
    return true;
}

void LogWriter::handle_storage_failure(std::uint64_t now_us) noexcept
{
    /* 与 PX4 writer 的 had_write_error 边界一致：失败文件不再追加。先禁止
     * producer，再丢弃仅属于旧 ULog 会话的 RAM 尾部；3 s 后完整重挂载并由
     * producer 重新写 header/format，绝不把两条 ULog 流拼成一个文件。 */
    accepting_.store(false);
    discard_ring();
    (void)store_.close_log();
    last_open_attempt_us_ = now_us;
    last_sync_us_ = 0U;
}

bool LogWriter::open_file(std::uint64_t now_us) noexcept
{
    if (last_open_attempt_us_ != 0U && now_us >= last_open_attempt_us_ &&
        now_us - last_open_attempt_us_ < kRetryIntervalUs) {
        return false;
    }
    last_open_attempt_us_ = now_us;
    if (store_.initialize() != 0 || store_.start_log() != 0) {
        return false;
    }

    read_position_.store(0U);
    write_position_.store(0U);
    std::uint32_t next = session_generation_.load() + 1U;
    if (next == 0U) {
        next = 1U;
    }
    session_generation_.store(next);
    last_sync_us_ = now_us;
    accepting_.store(true);
    return true;
}

void LogWriter::finish_stop() noexcept
{
    accepting_.store(false);
    (void)store_.close_log();
    discard_ring();
    last_sync_us_ = 0U;
    running_.store(false);
}

void LogWriter::run_storage() noexcept
{
    if (!running_.load()) {
        return;
    }

    const std::uint64_t now = hrt_absolute_time();
    if (!store_.log_open()) {
        accepting_.store(false);
        discard_ring();
        if (stop_requested_.load()) {
            finish_stop();
            return;
        }
        (void)open_file(now);
        return;
    }

    if (stop_requested_.load()) {
        /* producer 已停止，故 pending 只会单调下降。一次 storage Run 完整冲刷，
         * 让外部 stop 无需在非 storage 线程调用任何 FatFs API。 */
        while (pending_bytes() != 0U) {
            if (!append_one_chunk()) {
                handle_storage_failure(now);
                break;
            }
        }
        finish_stop();
        return;
    }

    if (pending_bytes() != 0U && !append_one_chunk()) {
        handle_storage_failure(now);
        return;
    }

    if (last_sync_us_ == 0U || now < last_sync_us_ ||
        now - last_sync_us_ >= kSyncIntervalUs) {
        if (store_.sync_log() != 0) {
            handle_storage_failure(now);
            return;
        }
        last_sync_us_ = now;
    }

    if (pending_bytes() != 0U) {
        (void)worker_.ScheduleNow();
    }
}

} // namespace dima::modules::logging
