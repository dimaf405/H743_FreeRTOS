/****************************************************************************
 *
 *   Copyright (c) 2014-2024 PX4 Development Team. All rights reserved.
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

#include "MavlinkLogHandler.hpp"

#include "api/Execution.hpp"
#include "api/Services.hpp"
#include "api/TaskRuntime.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cerrno>
#include <limits>

namespace dima::modules::mavlink {

MavlinkLogHandler::MavlinkLogHandler(
    dima::platform::LogFileStore &store, SendCallback sender,
    void *sender_context) noexcept
    : ScheduledWorkItem("mav_log", px4::wq_configurations::storage),
      store_(store), sender_(sender), sender_context_(sender_context)
{
}

bool MavlinkLogHandler::start() noexcept
{
    if (running_) {
        return true;
    }
    if (sender_ == nullptr || !ScheduleEnable()) {
        return false;
    }
    reset_worker_state();
    {
        dima::platform::CriticalGuard guard;
        request_head_ = request_tail_ = request_count_ = 0U;
        response_head_ = response_tail_ = response_count_ = 0U;
        next_response_sequence_ = 0U;
        reset_requested_ = false;
        stop_requested_ = false;
        running_ = true;
    }
    return true;
}

void MavlinkLogHandler::stop() noexcept
{
    bool active = false;
    {
        dima::platform::CriticalGuard guard;
        active = running_;
        if (!active) {
            stop_requested_ = false;
            reset_requested_ = false;
        } else {
            /* reader 的 f_close 必须仍由 storage worker 执行。stop 只发布请求，
             * running_ 要等 Run 完成 close 后才清零，等价于专用线程 join。 */
            stop_requested_ = true;
            reset_requested_ = true;
        }
        request_head_ = request_tail_ = request_count_ = 0U;
        response_head_ = response_tail_ = response_count_ = 0U;
    }

    if (active && ScheduleNow()) {
        for (;;) {
            {
                dima::platform::CriticalGuard guard;
                active = running_;
            }
            if (!active) {
                break;
            }
            dima::platform::services().tasks.delay(
                dima::platform::Timeout::from_ms(1U));
        }
    } else if (active) {
        /* WorkQueue 已停止时不能跨执行域补做 FatFs；只撤销内存状态，介质会话
         * 将随平台存储卸载失效。正常生命周期不会进入此降级分支。 */
        dima::platform::CriticalGuard guard;
        running_ = false;
        stop_requested_ = false;
        reset_requested_ = false;
    }
    ScheduleCancelAndDrain();
    reset_worker_state();
}

void MavlinkLogHandler::reset_link() noexcept
{
    {
        dima::platform::CriticalGuard guard;
        request_head_ = request_tail_ = request_count_ = 0U;
        response_head_ = response_tail_ = response_count_ = 0U;
        reset_requested_ = true;
    }
    (void)ScheduleNow();
}

void MavlinkLogHandler::handle_message(
    const mavlink_message_t &message) noexcept
{
    Request request{};
    switch (message.msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST: {
        mavlink_log_request_list_t decoded{};
        mavlink_msg_log_request_list_decode(&message, &decoded);
        request.type = RequestType::List;
        request.first_id = decoded.start;
        request.last_id = decoded.end;
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA: {
        mavlink_log_request_data_t decoded{};
        mavlink_msg_log_request_data_decode(&message, &decoded);
        request.type = RequestType::Data;
        request.id = decoded.id;
        request.offset = decoded.ofs;
        request.count = decoded.count;
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        request.type = RequestType::End;
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        request.type = RequestType::Erase;
        break;
    default:
        return;
    }
    (void)enqueue_request(request);
}

bool MavlinkLogHandler::request_storage_information(
    std::uint8_t storage_id) noexcept
{
    // PX4 STORAGE_INFORMATION 只公开一块默认存储：0 表示请求全部，
    // 1 表示第一块。其他索引必须让 REQUEST_MESSAGE 返回 DENIED。
    if (storage_id > 1U) {
        return false;
    }
    Request request{};
    request.type = RequestType::StorageInformation;
    request.storage_id = storage_id;
    return enqueue_request(request);
}

bool MavlinkLogHandler::enqueue_request(const Request &request) noexcept
{
    bool accepted = false;
    {
        dima::platform::CriticalGuard guard;
        if (!running_ || stop_requested_) {
            return false;
        }
        const bool terminal_request = request.type == RequestType::End ||
                                      request.type == RequestType::Erase;
        if (terminal_request) {
            /* PX4 在接收线程中立即处理 END/ERASE；本平台把文件操作搬到 storage
             * worker 后，必须用同等的抢占屏障丢弃旧请求和待发分片。reset 标志会
             * 让正在运行的预读循环尽快退出，但随后到达的 ERASE -> LIST 仍按序保留。 */
            request_head_ = request_tail_ = request_count_ = 0U;
            clear_log_responses_locked();
            reset_requested_ = true;
        }
        if (request_count_ < kRequestQueueCapacity) {
            request_queue_[request_tail_] = request;
            request_tail_ = static_cast<std::uint8_t>(
                (request_tail_ + 1U) % kRequestQueueCapacity);
            ++request_count_;
            accepted = true;
        }
    }
    if (accepted) {
        (void)ScheduleNow();
    }
    return accepted;
}

bool MavlinkLogHandler::pop_request(Request &request) noexcept
{
    dima::platform::CriticalGuard guard;
    if (request_count_ == 0U) {
        return false;
    }
    request = request_queue_[request_head_];
    request_head_ = static_cast<std::uint8_t>(
        (request_head_ + 1U) % kRequestQueueCapacity);
    --request_count_;
    return true;
}

bool MavlinkLogHandler::enqueue_response(const Response &response) noexcept
{
    dima::platform::CriticalGuard guard;
    if (!running_ || reset_requested_ ||
        response_count_ == kResponseQueueCapacity) {
        return false;
    }
    response_queue_[response_tail_] = response;
    response_queue_[response_tail_].sequence = next_response_sequence_++;
    response_tail_ = static_cast<std::uint8_t>(
        (response_tail_ + 1U) % kResponseQueueCapacity);
    ++response_count_;
    return true;
}

bool MavlinkLogHandler::peek_response(
    Response &response, std::size_t index,
    std::uint32_t first_sequence) noexcept
{
    dima::platform::CriticalGuard guard;
    if (reset_requested_ || index >= response_count_) {
        return false;
    }
    // 批编码期间 storage 可能替换请求；首项 sequence 改变后立即停止取片，
    // 避免把新旧请求的 Ring 索引拼成同一批。临界区只复制一项，不含编码/USB。
    if (index != 0U &&
        response_queue_[response_head_].sequence != first_sequence) {
        return false;
    }
    response = response_queue_[
        (response_head_ + index) % kResponseQueueCapacity];
    return true;
}

void MavlinkLogHandler::pop_response(
    std::uint32_t expected_sequence) noexcept
{
    dima::platform::CriticalGuard guard;
    /* send() 在 USB 写期间不持临界区；若 storage worker 此时用新请求清空并重建
     * Ring，只允许弹出仍是刚才那一项的 sequence，绝不能误删新事务首包。 */
    if (response_count_ != 0U &&
        response_queue_[response_head_].sequence == expected_sequence) {
        response_head_ = static_cast<std::uint8_t>(
            (response_head_ + 1U) % kResponseQueueCapacity);
        --response_count_;
    }
}

void MavlinkLogHandler::clear_responses() noexcept
{
    dima::platform::CriticalGuard guard;
    clear_log_responses_locked();
}

void MavlinkLogHandler::clear_log_responses_locked() noexcept
{
    // LOG_LIST/DATA/END/ERASE 只能取消旧的日志分片。STORAGE_INFORMATION
    // 是已被 COMMAND_ACK 接受的独立请求，必须在同一固定 Ring 内保留。
    const std::uint8_t original_count = response_count_;
    for (std::uint8_t index = 0U; index < original_count; ++index) {
        const Response response = response_queue_[response_head_];
        response_head_ = static_cast<std::uint8_t>(
            (response_head_ + 1U) % kResponseQueueCapacity);
        --response_count_;
        if (response.type == ResponseType::StorageInformation) {
            // 每次先 pop 再 push，tail 始终指向已经释放的槽位；
            // 不需要额外栈数组，也不会覆盖尚未检查的原始项。
            response_queue_[response_tail_] = response;
            response_tail_ = static_cast<std::uint8_t>(
                (response_tail_ + 1U) % kResponseQueueCapacity);
            ++response_count_;
        }
    }
    if (response_count_ == 0U) {
        response_head_ = response_tail_ = 0U;
    }
}

bool MavlinkLogHandler::response_space_available() noexcept
{
    dima::platform::CriticalGuard guard;
    /* USB 断开或 END/ERASE 抢占后不再读取下一片。介质调用本身仍遵守后端有限
     * 超时，返回后 worker 会先关闭旧 reader，再处理保留下来的终止/刷新请求。 */
    return running_ && !reset_requested_ &&
           response_count_ < kResponseQueueCapacity;
}

bool MavlinkLogHandler::work_pending() noexcept
{
    dima::platform::CriticalGuard guard;
    return running_ &&
           (reset_requested_ || request_count_ != 0U ||
            worker_state_ == WorkerState::PreparingList ||
            worker_state_ == WorkerState::Erasing ||
            ((worker_state_ == WorkerState::Listing ||
              worker_state_ == WorkerState::SendingData) &&
             response_count_ < kResponseQueueCapacity));
}

void MavlinkLogHandler::reset_worker_state() noexcept
{
    set_worker_state(WorkerState::Idle);
    list_first_id_ = 0U;
    list_last_id_ = 0U;
    list_current_id_ = 0U;
    number_of_logs_ = 0U;
    current_log_id_ = 0xffffU;
    current_log_size_ = 0U;
    data_offset_ = 0U;
    data_end_offset_ = 0U;
    last_data_activity_us_ = 0U;
    logs_listed_ = false;
    storage_initialized_ = false;
}

void MavlinkLogHandler::set_worker_state(WorkerState state) noexcept
{
    /* send() 会跨 WorkQueue 判断生产者是否仍需补帧；状态发布与读取必须经过同一
     * 临界区，避免 lp_default 观察到 storage worker 的中间写入。 */
    dima::platform::CriticalGuard guard;
    worker_state_ = state;
}

void MavlinkLogHandler::enqueue_empty_list() noexcept
{
    Response response{};
    response.type = ResponseType::Entry;
    response.entry.id = 0U;
    response.entry.num_logs = 0U;
    response.entry.last_log_num = 0U;
    response.entry.time_utc = 0U;
    response.entry.size = 0U;
    (void)enqueue_response(response);
}

void MavlinkLogHandler::process_request(const Request &request) noexcept
{
    switch (request.type) {
    case RequestType::End:
        clear_responses();
        store_.close_log_transfer();
        set_worker_state(WorkerState::Idle);
        current_log_id_ = 0xffffU;
        return;

    case RequestType::Erase:
        clear_responses();
        store_.close_log_transfer();
        reset_worker_state();
        storage_initialized_ = store_.initialize() == 0;
        if (storage_initialized_) {
            const int erased = store_.erase_logs();
            if (erased == -EAGAIN) {
                set_worker_state(WorkerState::Erasing);
            }
        }
        return;

    case RequestType::List: {
        clear_responses();
        store_.close_log_transfer();
        current_log_id_ = 0xffffU;
        number_of_logs_ = 0U;
        list_first_id_ = request.first_id;
        list_last_id_ = request.last_id;
        list_current_id_ = 0U;
        storage_initialized_ = false;
        set_worker_state(WorkerState::PreparingList);
        process_list_preparation();
        return;
    }

    case RequestType::Data: {
        clear_responses();
        if (!logs_listed_ || request.id >= number_of_logs_) {
            store_.close_log_transfer();
            current_log_id_ = 0xffffU;
            set_worker_state(WorkerState::Idle);
            return;
        }
        dima::platform::LogFileEntry entry{};
        const int opened = store_.open_log(request.id, entry);
        if (opened != 0 || request.offset >= entry.size_bytes) {
            // open 成功但 offset 越界也必须关闭，否则没有有效 ID 可供空闲回收。
            store_.close_log_transfer();
            set_worker_state(WorkerState::Idle);
            current_log_id_ = 0xffffU;
            return;
        }
        current_log_id_ = request.id;
        current_log_size_ = entry.size_bytes;
        last_data_activity_us_ = hrt_absolute_time();
        data_offset_ = request.offset;
        const std::uint64_t requested_end =
            static_cast<std::uint64_t>(request.offset) + request.count;
        data_end_offset_ = static_cast<std::uint32_t>(std::min<std::uint64_t>(
            requested_end, current_log_size_));
        if (data_end_offset_ <= data_offset_) {
            const std::uint64_t one_packet_end =
                static_cast<std::uint64_t>(data_offset_) +
                MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN;
            data_end_offset_ = static_cast<std::uint32_t>(
                std::min<std::uint64_t>(current_log_size_, one_packet_end));
        }
        set_worker_state(WorkerState::SendingData);
        return;
    }

    case RequestType::StorageInformation:
        process_storage_information(request);
        return;
    }
}

void MavlinkLogHandler::process_list_preparation() noexcept
{
    if (!storage_initialized_) {
        if (store_.initialize() != 0) {
            set_worker_state(WorkerState::Idle);
            enqueue_empty_list();
            return;
        }
        storage_initialized_ = true;
    }
    const int listed = store_.create_log_list(number_of_logs_);
    if (listed == -EAGAIN) {
        /* 恢复、CRC 或 delNNN 清理每轮只推进有限 FatFs 操作；保持本状态并
         * 立即重调度，避免一次 LOG_REQUEST_LIST 长时间独占 storage worker。 */
        return;
    }
    if (listed != 0 || number_of_logs_ == 0U) {
        /* common.xml 要求无日志也返回 id=0/num_logs=0，否则 QGC 会一直等待。 */
        logs_listed_ = listed == 0;
        set_worker_state(WorkerState::Idle);
        enqueue_empty_list();
        return;
    }
    list_last_id_ = list_last_id_ == 0xffffU
                        ? static_cast<std::uint16_t>(number_of_logs_ - 1U)
                        : list_last_id_;
    logs_listed_ = true;
    set_worker_state(WorkerState::Listing);
}

void MavlinkLogHandler::process_erase() noexcept
{
    const int maintained = store_.service_log_maintenance();
    if (maintained == -EAGAIN) {
        return;
    }
    set_worker_state(WorkerState::Idle);
}

void MavlinkLogHandler::process_storage_information(
    const Request &request) noexcept
{
    // f_getfree 可能访问 FAT，因此必须留在 wq:storage。响应 Ring 已满时
    // 先把请求放回有界队列，禁止 ACK 已接受却静默丢失最终消息。
    if (request.storage_id > 1U) {
        return;
    }
    {
        dima::platform::CriticalGuard guard;
        if (!running_ || reset_requested_) {
            return;
        }
        if (response_count_ == kResponseQueueCapacity) {
            // 当前请求已从 request Ring 弹出，因此请求队列至少应有
            // 一个空位；仍保留边界检查，
            // 任何不满足容量合同的状态都宁可取消本次响应，也不能越界覆盖旧分片。
            if (request_count_ < kRequestQueueCapacity) {
                request_queue_[request_tail_] = request;
                request_tail_ = static_cast<std::uint8_t>(
                    (request_tail_ + 1U) % kRequestQueueCapacity);
                ++request_count_;
            }
            return;
        }
    }

    dima::platform::StorageInformation information{};
    const int initialized = store_.initialize();
    const int result = initialized == 0
                           ? store_.storage_information(information)
                           : initialized;

    Response response{};
    response.type = ResponseType::StorageInformation;
    mavlink_storage_information_t &storage = response.storage_information;
    storage.time_boot_ms = static_cast<std::uint32_t>(
        hrt_absolute_time() / 1000ULL);
    storage.storage_id = 1U;

    // 严格保持 PX4 v1.17 语义：请求 0/1 都回复 storage_id=1。不可用时
    // 仍发送 EMPTY/count=0，使 QGC 得到确定结果，而不是等待超时。
    if (result == 0 && information.total_bytes != 0U &&
        information.available_bytes <= information.total_bytes) {
        constexpr double kBytesPerMebibyte = 1024.0 * 1024.0;
        storage.storage_count = 1U;
        storage.status = static_cast<std::uint8_t>(STORAGE_STATUS_READY);
        storage.total_capacity = static_cast<float>(
            static_cast<double>(information.total_bytes) /
            kBytesPerMebibyte);
        storage.available_capacity = static_cast<float>(
            static_cast<double>(information.available_bytes) /
            kBytesPerMebibyte);
        storage.used_capacity = storage.total_capacity -
                                storage.available_capacity;
    } else {
        storage.storage_count = 0U;
        storage.status = static_cast<std::uint8_t>(STORAGE_STATUS_EMPTY);
    }

    (void)enqueue_response(response);
}

void MavlinkLogHandler::process_listing() noexcept
{
    while (worker_state_ == WorkerState::Listing &&
           response_space_available()) {
        if (list_current_id_ >= number_of_logs_) {
            list_current_id_ = 0U;
            set_worker_state(WorkerState::Idle);
            return;
        }
        const std::uint16_t id = list_current_id_++;
        if (id < list_first_id_) {
            continue;
        }
        dima::platform::LogFileEntry entry{};
        if (store_.read_log_entry(id, entry) != 0) {
            set_worker_state(WorkerState::Idle);
            return;
        }
        Response response{};
        response.type = ResponseType::Entry;
        response.entry.id = id;
        response.entry.num_logs = number_of_logs_;
        response.entry.last_log_num = list_last_id_;
        response.entry.time_utc = entry.time_utc;
        response.entry.size = entry.size_bytes;
        if (!enqueue_response(response)) {
            --list_current_id_;
            return;
        }
    }
}

void MavlinkLogHandler::process_data() noexcept
{
    // 即使 USB 在本轮读取期间持续消费，也只补一整个 Ring，随后让出 storage
    // 队列给日志写入和参数持久化；不能因扩大预取窗口而形成无限读循环。
    for (std::size_t count = 0U;
         count < kResponseQueueCapacity &&
         worker_state_ == WorkerState::SendingData &&
         response_space_available(); ++count) {
        if (data_offset_ >= data_end_offset_ ||
            data_offset_ >= current_log_size_) {
            set_worker_state(WorkerState::Idle);
            return;
        }
        Response response{};
        response.type = ResponseType::Data;
        response.data.id = current_log_id_;
        response.data.ofs = data_offset_;
        /* 保持 PX4 原始分片语义：count 只决定本次 burst 的停止边界，每个非 EOF
         * LOG_DATA 仍尽量装满 mavgen 生成的 data 字段；count=0 也发送一片。 */
        const std::size_t requested = std::min<std::size_t>(
            sizeof(response.data.data), current_log_size_ - data_offset_);
        std::size_t read_size = 0U;
        if (store_.read_log(data_offset_, response.data.data,
                            requested, read_size) != 0 ||
            read_size == 0U || read_size > UINT8_MAX) {
            set_worker_state(WorkerState::Idle);
            return;
        }
        response.data.count = static_cast<std::uint8_t>(read_size);
        if (!enqueue_response(response)) {
            return;
        }
        data_offset_ += static_cast<std::uint32_t>(read_size);
        last_data_activity_us_ = hrt_absolute_time();
    }
}

void MavlinkLogHandler::release_idle_reader() noexcept
{
    if (worker_state_ != WorkerState::Idle || current_log_id_ == 0xffffU) {
        return;
    }
    // QGC 的正常下载完成路径不保证发送 LOG_REQUEST_END。保留 5 s 供下一段
    // 请求/补传，之后由 storage 关闭 reader，解除最旧日志的回收保护。
    // 已预读的响应拥有独立字节副本；关闭 FIL 不会破坏 USB 尚未完成的缓冲。
    const std::uint64_t now = hrt_absolute_time();
    if (now < last_data_activity_us_ ||
        now - last_data_activity_us_ >= kReaderIdleTimeoutUs) {
        store_.close_log_transfer();
        current_log_id_ = 0xffffU;
        last_data_activity_us_ = 0U;
        return;
    }
    (void)ScheduleDelayed(static_cast<std::uint32_t>(
        kReaderIdleTimeoutUs - (now - last_data_activity_us_)));
}

void MavlinkLogHandler::Run()
{
    bool reset = false;
    bool active = false;
    bool stop = false;
    {
        dima::platform::CriticalGuard guard;
        active = running_;
        reset = reset_requested_;
        stop = stop_requested_;
        if (reset) {
            /* reset_link 已在置位标志的同一临界区清空所有 Ring；
             * END/ERASE 则只清理日志分片并保留独立存储响应。此处只消费
             * worker reset 边沿，禁止再次无区别清空响应。 */
            reset_requested_ = false;
        }
    }
    if (!active) {
        return;
    }
    if (reset) {
        store_.close_log_transfer();
        reset_worker_state();
    }
    if (stop) {
        dima::platform::CriticalGuard guard;
        running_ = false;
        stop_requested_ = false;
        return;
    }

    Request request{};
    if (pop_request(request)) {
        process_request(request);
    }
    if (worker_state_ == WorkerState::Listing) {
        process_listing();
    } else if (worker_state_ == WorkerState::SendingData) {
        process_data();
    } else if (worker_state_ == WorkerState::PreparingList) {
        process_list_preparation();
    } else if (worker_state_ == WorkerState::Erasing) {
        process_erase();
    }
    release_idle_reader();
    // 先安排空闲回收再检查请求：若 RX 在上述过程中已 ScheduleNow，必须
    // 重新发布立即调度，避免延迟回收覆盖新请求的唤醒；之后到达的 RX 会自行唤醒。
    if (work_pending()) {
        (void)ScheduleNow();
    }
}

void MavlinkLogHandler::send() noexcept
{
    std::uint32_t sequences[kMaximumResponsesPerSend]{};
    std::size_t response_count = 0U;
    std::size_t batch_size = 0U;
    // PX4 按 TX 可用空间连续发 LOG_DATA；本地 Console 是有完成确认的同步接口，
    // 因此先用官方 codec 合并完整帧，再一次提交。预留生成库的最大帧空间，
    // 不手写 payload/CRC/消息列表，也不在 lp_default 的任务栈上放大块缓冲。
    while (response_count < kMaximumResponsesPerSend &&
           sizeof(tx_batch_) - batch_size >= MAVLINK_MAX_PACKET_LEN) {
        Response response{};
        if (!peek_response(response, response_count, sequences[0])) {
            break;
        }
        mavlink_message_t message{};
        if (response.type == ResponseType::Entry) {
            mavlink_msg_log_entry_encode(MAVLINK_SYSTEM_ID,
                                         MAVLINK_COMPONENT_ID,
                                         &message, &response.entry);
        } else if (response.type == ResponseType::Data) {
            mavlink_msg_log_data_encode(MAVLINK_SYSTEM_ID,
                                        MAVLINK_COMPONENT_ID,
                                        &message, &response.data);
        } else {
            mavlink_msg_storage_information_encode(
                MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                &message, &response.storage_information);
        }
        batch_size += mavlink_msg_to_send_buffer(
            tx_batch_ + batch_size, &message);
        sequences[response_count++] = response.sequence;
    }

    // 整批 USB 完成才消费响应；超时/断线保留分片供协议重试，Console staging
    // 继续保护尚在传输的字节。逐项核对 sequence，禁止误弹 storage 新事务的响应。
    if (batch_size != 0U && sender_(sender_context_, tx_batch_, batch_size)) {
        for (std::size_t index = 0U; index < response_count; ++index) {
            pop_response(sequences[index]);
        }
    }

    // TX Ring 腾出空间后唤醒 storage worker，继续按 PX4 burst 语义预取下一批。
    if (work_pending()) {
        (void)ScheduleNow();
    }
}

} // namespace dima::modules::mavlink
