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
#include "api/Synchronization.hpp"
#include "api/TaskRuntime.hpp"
#include "api/Time.hpp"

#include <px4_platform_common/log.h>

#include <algorithm>
#include <cerrno>
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

bool LogWriter::start(
    const dima::platform::LogSessionContext &context) noexcept
{
    if (running_.load()) {
        return true;
    }
    if (context.maximum_directories == 0U ||
        context.maximum_directories > 999U) {
        return false;
    }
    if (store_.configure_log_maintenance(context.maximum_directories) != 0) {
        return false;
    }

    base_context_ = context;
    base_context_.start_monotonic_us = 0U;
    published_context_ = {};
    time_reference_ = context.time_reference;
    time_reference_generation_.store(context.time_reference.valid ? 1U : 0U);
    applied_time_reference_generation_ = 0U;
    accepting_.store(false);
    session_requested_.store(false);
    close_requested_.store(false);
    stop_requested_.store(false);
    read_position_.store(0U);
    write_position_.store(0U);
    last_open_attempt_us_ = 0U;
    retry_interval_us_ = kRetryIntervalUs;
    last_sync_us_ = 0U;
    last_open_warning_us_ = 0U;
    last_open_error_ = 0;
    initial_probe_attempted_ = false;
    volume_ready_ = false;
    running_.store(true);

    if (!worker_.ScheduleEnable() ||
        !worker_.ScheduleOnInterval(kRunIntervalUs, 1U)) {
        running_.store(false);
        worker_.ScheduleCancelAndDrain();
        return false;
    }
    return true;
}

void LogWriter::set_recording_intent(bool enabled) noexcept
{
    if (!running_.load()) {
        return;
    }
    if (enabled) {
        session_requested_.store(true);
        (void)worker_.ScheduleNow();
        return;
    }
    end_session();
}

void LogWriter::end_session() noexcept
{
    if (!running_.load()) {
        return;
    }

    /* 关闭边沿先撤销 producer 接收权，再让 storage worker 排空已经完整发布的
     * ULog 消息。等待标志只由 storage worker 在 f_sync/close/侧车提交结束后
     * 清除，因此 Mode 0 的下一次解锁不会与上一会话尾部交叉。 */
    session_requested_.store(false);
    accepting_.store(false);
    close_requested_.store(true);
    (void)worker_.ScheduleNow();

    while (running_.load() && close_requested_.load()) {
        dima::platform::services().tasks.delay(
            dima::platform::Timeout::from_ms(1U));
    }
}

void LogWriter::stop() noexcept
{
    if (!running_.load()) {
        worker_.ScheduleCancelAndDrain();
        return;
    }

    session_requested_.store(false);
    accepting_.store(false);
    close_requested_.store(true);
    stop_requested_.store(true);
    (void)worker_.ScheduleNow();

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

void LogWriter::update_time_reference(
    const dima::platform::LogTimeReference &reference) noexcept
{
    if (!running_.load() || !reference.valid) {
        return;
    }
    {
        /* Cortex-M7 不保证 64-bit 原子读写；极短临界区整体发布 UTC 映射，避免
         * storage worker 观察到来自两个 GPS 候选的高低 32 位组合。 */
        dima::platform::CriticalGuard guard;
        time_reference_ = reference;
    }
    (void)time_reference_generation_.fetch_add(1U);
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

dima::platform::LogSessionContext LogWriter::session_context() const noexcept
{
    dima::platform::CriticalGuard guard;
    return published_context_;
}

dima::platform::LogTimeReference LogWriter::time_reference_snapshot() const noexcept
{
    dima::platform::CriticalGuard guard;
    return time_reference_;
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

int LogWriter::append_one_chunk(std::uint32_t maximum_bytes) noexcept
{
    const std::uint32_t read = read_position_.load();
    const std::uint32_t written = write_position_.load();
    const std::uint32_t pending = written - read;
    if (pending == 0U) {
        return 0;
    }
    if (pending > kRingCapacity) {
        return -EIO;
    }

    const std::uint32_t index = read & kRingMask;
    const std::uint32_t contiguous = std::min(
        std::min(pending, kRingCapacity - index), maximum_bytes);
    if (contiguous == 0U) {
        return -EINVAL;
    }
    const int result = store_.append_log(&ring_[index], contiguous);
    if (result != 0) {
        return result;
    }
    read_position_.store(read + contiguous);
    return 0;
}

void LogWriter::report_open_failure(int error, std::uint64_t now_us) noexcept
{
    if (error == 0 || error == -EAGAIN) {
        return;
    }

    const bool changed = error != last_open_error_;
    const bool warning_due = changed || last_open_warning_us_ == 0U ||
        now_us < last_open_warning_us_ ||
        now_us - last_open_warning_us_ >= kOpenWarningIntervalUs;
    last_open_error_ = error;
    if (!warning_due) {
        return;
    }

    /* 文件尚未创建时没有 close/failure 路径可代为告警。首次失败、错误类型
     * 变化及每 60 s 各提示一次，既让无卡和低空间可见，也避免 3 s 重试刷屏。 */
    if (error == -ENOSPC) {
        PX4_WARN("ULog waiting for SD free space");
    } else {
        PX4_WARN("ULog waiting for SD storage (%d)", error);
    }
    last_open_warning_us_ = now_us;
}

void LogWriter::handle_storage_failure(int error,
                                       std::uint64_t now_us) noexcept
{
    accepting_.store(false);
    if (error == -ENOSPC && store_.log_open()) {
        /* 先封住 producer，再逐步缩小写块，把当前已分配 FAT 簇尚可容纳的
         * Ring 前缀尽量写完。一次分配会被后端限制在 threshold-1 cluster，
         * 因此这里只丢弃确实无法在空间合同内落盘的尾部。 */
        std::uint32_t attempt = kWriteChunkBytes;
        while (pending_bytes() != 0U) {
            const int appended = append_one_chunk(attempt);
            if (appended == 0) {
                attempt = kWriteChunkBytes;
                continue;
            }
            if (appended == -ENOSPC && attempt > 1U) {
                attempt = std::max<std::uint32_t>(1U, attempt / 2U);
                continue;
            }
            if (appended != -ENOSPC) {
                error = appended;
            }
            break;
        }
    }
    discard_ring();
    const int close_error = store_.close_log();
    last_open_attempt_us_ = now_us;
    last_sync_us_ = 0U;
    if (close_error != 0) {
        /* 原始原因可能只是低空间或正常轮转，但 sync/close/sidecar 提交又暴露
         * 介质错误时，必须撤销已挂载假设并按拔卡路径重建，不能立即复用 FIL。 */
        retry_interval_us_ = kRetryIntervalUs;
        volume_ready_ = false;
        PX4_WARN("ULog close failed (%d); remounting", close_error);
        return;
    }
    if (error == -EFBIG) {
        /* 4 GiB 是协议/file-size 边界，不是介质损坏。旧文件已同步关闭并提交
         * CLOSED 侧车，下一轮立即创建带全新 ULog Definitions 的会话。 */
        retry_interval_us_ = 0U;
        last_open_error_ = 0;
        PX4_WARN("ULog reached 4 GiB; rotating session");
    } else if (error == -ENOSPC) {
        /* 空间保护在跨过阈值前拒绝整块写入；保留旧文件有效前缀，并按 60 s
         * 周期让维护器重查 QGC 删除、换卡或其他空间恢复。 */
        retry_interval_us_ = kSpaceRetryIntervalUs;
        last_open_error_ = error;
        last_open_warning_us_ = now_us;
        PX4_WARN("ULog paused at SD free-space reserve");
    } else {
        /* I/O 会话一旦失败就不再追加：Ring 尾部只属于旧 ULog，三秒后重挂载
         * 并从 header 开始新文件，绝不把新数据拼接到损坏前缀。 */
        retry_interval_us_ = kRetryIntervalUs;
        volume_ready_ = false;
        last_open_error_ = error;
        last_open_warning_us_ = now_us;
        PX4_WARN("ULog SD I/O failed (%d); retrying", error);
    }
}

bool LogWriter::open_file(std::uint64_t now_us) noexcept
{
    if (last_open_attempt_us_ != 0U && now_us >= last_open_attempt_us_ &&
        now_us - last_open_attempt_us_ < retry_interval_us_) {
        /* 普通拔卡按 3 s 重试，因保护空闲线暂停则按 60 s 重试，4 GiB 正常
         * 轮转使用 0 间隔。这里必须使用故障路径选出的间隔，不能退回固定值。 */
        return false;
    }
    if (!volume_ready_) {
        last_open_attempt_us_ = now_us;
        const int initialized = store_.initialize();
        if (initialized != 0) {
            retry_interval_us_ = kRetryIntervalUs;
            report_open_failure(initialized, now_us);
            return false;
        }
        volume_ready_ = true;
    }

    const int maintenance = store_.service_log_maintenance();
    if (maintenance == -EAGAIN) {
        last_open_attempt_us_ = 0U;
        (void)worker_.ScheduleNow();
        return false;
    }
    if (maintenance != 0) {
        volume_ready_ = false;
        last_open_attempt_us_ = now_us;
        retry_interval_us_ = kRetryIntervalUs;
        report_open_failure(maintenance, now_us);
        return false;
    }

    dima::platform::LogSessionContext context = base_context_;
    context.start_monotonic_us = now_us;
    /* generation 必须在 UTC RAM 快照之前取得。若高优先级 Logger 恰在二者
     * 之间发布新映射，本次 context 至多携带“更新的值+旧 generation”，下轮
     * 会安全地重复提交；绝不能在 open 后读取新 generation 并误称其已落侧车。 */
    const std::uint32_t opened_reference_generation =
        time_reference_generation_.load();
    context.time_reference = time_reference_snapshot();
    const int opened = store_.start_log(context);
    if (opened == -EAGAIN) {
        last_open_attempt_us_ = 0U;
        (void)worker_.ScheduleNow();
        return false;
    }
    if (opened != 0) {
        last_open_attempt_us_ = now_us;
        retry_interval_us_ = opened == -ENOSPC
                                 ? kSpaceRetryIntervalUs
                                 : kRetryIntervalUs;
        if (opened != -ENOSPC) {
            volume_ready_ = false;
        }
        report_open_failure(opened, now_us);
        return false;
    }

    read_position_.store(0U);
    write_position_.store(0U);
    std::uint32_t next = session_generation_.load() + 1U;
    if (next == 0U) {
        next = 1U;
    }
    {
        dima::platform::CriticalGuard guard;
        published_context_ = context;
    }
    session_generation_.store(next);
    applied_time_reference_generation_ = opened_reference_generation;
    last_open_attempt_us_ = 0U;
    retry_interval_us_ = kRetryIntervalUs;
    last_sync_us_ = now_us;
    accepting_.store(true);
    if (last_open_error_ != 0) {
        PX4_INFO("ULog SD recording resumed");
    } else {
        PX4_INFO("ULog SD session started");
    }
    last_open_error_ = 0;
    last_open_warning_us_ = 0U;
    return true;
}

int LogWriter::apply_time_reference(std::uint32_t generation) noexcept
{
    if (generation == applied_time_reference_generation_) {
        return 0;
    }
    // 调用者先取代次，再在临界区复制完整 UTC 映射；并发发布最多导致下一轮
    // 重复提交，不能把尚未落盘的新代标成已提交。写入失败也不推进确认代次。
    const dima::platform::LogTimeReference reference = time_reference_snapshot();
    if (!reference.valid) {
        return 0;
    }
    const int result = store_.update_log_time(reference);
    if (result == 0) {
        applied_time_reference_generation_ = generation;
    }
    return result;
}

void LogWriter::finish_session() noexcept
{
    accepting_.store(false);
    if (store_.close_log() != 0) {
        /* 正常关闭也可能在 f_sync/侧车追加阶段发现拔卡；下次有记录意图时
         * 必须完整重挂载，不能复用后端已经失效的 FATFS 对象。 */
        volume_ready_ = false;
    }
    discard_ring();
    last_sync_us_ = 0U;
    close_requested_.store(false);
}

void LogWriter::finish_worker_stop() noexcept
{
    accepting_.store(false);
    session_requested_.store(false);
    close_requested_.store(false);
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
        close_requested_.store(false);
        if (stop_requested_.load()) {
            finish_worker_stop();
            return;
        }

        if (!session_requested_.load()) {
            /* Mode 0/3 尚无记录意图时只在服务启动做一次挂载探测；卡缺失后不
             * 周期重试。若一次挂载成功，则继续以有限步骤完成遗留 del/recovery，
             * 让 QGC 即使未解锁也能看到已恢复的历史日志。 */
            if (!initial_probe_attempted_) {
                initial_probe_attempted_ = true;
                volume_ready_ = store_.initialize() == 0;
            }
            if (volume_ready_) {
                const int maintenance = store_.service_log_maintenance();
                if (maintenance == -EAGAIN) {
                    (void)worker_.ScheduleNow();
                } else if (maintenance != 0) {
                    volume_ready_ = false;
                }
            }
            return;
        }

        (void)open_file(now);
        return;
    }

    /* 活动会话每轮先推进一次目录/空间维护。低于保护线时 service 会逐个删除
     * 最旧已关闭会话并返回 EAGAIN；只有确实没有安全候选才返回 ENOSPC。
     * 因而本轮不会用删除前的保守空闲估算误关当前文件。 */
    const int maintenance = store_.service_log_maintenance();
    if (maintenance == -EAGAIN) {
        (void)worker_.ScheduleNow();
        return;
    }
    if (maintenance != 0) {
        handle_storage_failure(maintenance, now);
        return;
    }

    if (stop_requested_.load() || close_requested_.load() ||
        !session_requested_.load()) {
        /* producer 已停止，故 pending 只会单调下降。一次 storage Run 完整冲刷，
         * 让锁定/停机调用无需在非 storage 线程触碰 FatFs。 */
        while (pending_bytes() != 0U) {
            const int appended = append_one_chunk();
            if (appended != 0) {
                handle_storage_failure(appended, now);
                break;
            }
        }
        if (store_.log_open()) {
            // 关闭侧车前补交最新 UTC，避免 ULog 已有时间而 QGC 仍显示 UnknownDate。
            const int updated = apply_time_reference(time_reference_generation_.load());
            if (updated != 0) {
                handle_storage_failure(updated, now);
            }
        }
        if (store_.log_open()) {
            finish_session();
        } else {
            close_requested_.store(false);
        }
        if (stop_requested_.load()) {
            finish_worker_stop();
        }
        return;
    }

    if (pending_bytes() != 0U) {
        const int appended = append_one_chunk();
        if (appended != 0) {
            handle_storage_failure(appended, now);
            return;
        }
    }

    const std::uint32_t reference_generation = time_reference_generation_.load();
    const int updated = apply_time_reference(reference_generation);
    if (updated != 0) {
        handle_storage_failure(updated, now);
        return;
    }
    applied_time_reference_generation_ = reference_generation;

    if (last_sync_us_ == 0U || now < last_sync_us_ ||
        now - last_sync_us_ >= kSyncIntervalUs) {
        const int synchronized = store_.sync_log();
        if (synchronized != 0) {
            handle_storage_failure(synchronized, now);
            return;
        }
        last_sync_us_ = now;
    }

    if (pending_bytes() != 0U) {
        (void)worker_.ScheduleNow();
    }
}

} // namespace dima::modules::logging
