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

#pragma once

#include "mavlink/MavlinkBridge.h"
#include "api/Console.hpp"
#include "api/LogFileStore.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::mavlink {

// 单个 LogFileStore reader 的跨链路租约；释放期间仍保持独占，直到 storage 完成 close。
class MavlinkLogLease {
public:
    bool acquire(std::uint8_t channel, std::uint64_t now) noexcept;
    bool owns(std::uint8_t channel) const noexcept;
    void touch(std::uint8_t channel, std::uint64_t now) noexcept;
    void begin_close(std::uint8_t channel) noexcept;
    bool begin_expiry(std::uint8_t channel, std::uint64_t now,
                      std::uint32_t timeout_us) noexcept;
    std::uint64_t activity() const noexcept;
    void release(std::uint8_t channel) noexcept;
private:
    std::uint8_t owner_{UINT8_MAX};
    bool releasing_{false};
    std::uint64_t activity_us_{0U};
};

/**
 * PX4 v1.17 MavlinkLogHandler 的 FreeRTOS/FatFs 薄适配。
 *
 * LOG_* 协议状态、0 起始 ID 和 90-byte LOG_DATA 分片保持 PX4 行为；文件系统
 * 操作转移到 wq:storage，MAVLink owner 只在自己的队列中发送固定响应 Ring。
 */
class MavlinkLogHandler final : public px4::ScheduledWorkItem {
public:
    using SendCallback = bool (*)(void *context, const std::uint8_t *data,
                                  std::size_t length) noexcept;

    MavlinkLogHandler(dima::platform::LogFileStore &store, MavlinkLogLease &lease,
                      SendCallback sender, void *sender_context, std::uint8_t channel = MAVLINK_COMM_0) noexcept;

    bool start() noexcept;
    void stop() noexcept;
    void reset_link() noexcept;
    bool handle_message(const mavlink_message_t &message) noexcept;
    bool request_storage_information(std::uint8_t storage_id) noexcept;
    void send(std::size_t maximum_responses = 16U) noexcept;

protected:
    void Run() override;

private:
    std::uint8_t channel_{MAVLINK_COMM_0};
    MavlinkLogLease &lease_;
    std::uint64_t last_busy_warning_us_{0U};
    enum class RequestType : std::uint8_t {
        List,
        Data,
        End,
        Erase,
        StorageInformation,
    };

    struct Request {
        RequestType type{RequestType::End};
        std::uint16_t first_id{0U};
        std::uint16_t last_id{0U};
        std::uint16_t id{0U};
        std::uint8_t storage_id{0U};
        std::uint32_t offset{0U};
        std::uint32_t count{0U};
    };

    enum class ResponseType : std::uint8_t {
        Entry,
        Data,
        StorageInformation,
    };

    struct Response {
        std::uint32_t sequence{0U};
        ResponseType type{ResponseType::Entry};
        mavlink_log_entry_t entry{};
        mavlink_log_data_t data{};
        mavlink_storage_information_t storage_information{};
    };

    enum class WorkerState : std::uint8_t {
        Idle,
        PreparingList,
        Listing,
        SendingData,
        Erasing,
    };

    bool enqueue_request(const Request &request) noexcept;
    bool pop_request(Request &request) noexcept;
    bool enqueue_response(const Response &response) noexcept;
    bool peek_response(Response &response, std::size_t index,
                       std::uint32_t first_sequence) noexcept;
    void pop_response(std::uint32_t expected_sequence) noexcept;
    void clear_responses() noexcept;
    void clear_log_responses_locked() noexcept;
    bool response_space_available() noexcept;
    bool work_pending() noexcept;

    void reset_worker_state() noexcept;
    void set_worker_state(WorkerState state) noexcept;
    void process_request(const Request &request) noexcept;
    void process_list_preparation() noexcept;
    void process_erase() noexcept;
    void process_listing() noexcept;
    void process_data() noexcept;
    void release_idle_reader() noexcept;
    void process_storage_information(const Request &request) noexcept;
    void enqueue_empty_list() noexcept;

    static constexpr std::size_t kRequestQueueCapacity = 4U;
    static constexpr std::uint32_t kReaderIdleTimeoutUs = 5000000U;
    // 双批预取吸收 storage/USB 调度相位差；每轮最多合并 16 帧，只等待一次
    // USB 完成。100 Hz 下满 LOG_DATA 的载荷预算为 16 * 90 * 100 = 144000 B/s。
    static constexpr std::size_t kMaximumResponsesPerSend = 16U;
    static constexpr std::size_t kResponseQueueCapacity =
        2U * kMaximumResponsesPerSend;

    dima::platform::LogFileStore &store_;
    SendCallback sender_{nullptr};
    void *sender_context_{nullptr};

    Request request_queue_[kRequestQueueCapacity]{};
    Response response_queue_[kResponseQueueCapacity]{};
    std::uint8_t tx_batch_[dima::platform::Console::kWriteCapacity]{};
    std::uint8_t request_head_{0U};
    std::uint8_t request_tail_{0U};
    std::uint8_t request_count_{0U};
    std::uint8_t response_head_{0U};
    std::uint8_t response_tail_{0U};
    std::uint8_t response_count_{0U};
    std::uint32_t next_response_sequence_{0U};
    bool reset_requested_{false};
    bool release_on_reset_{false};
    bool stop_requested_{false};
    bool running_{false};

    WorkerState worker_state_{WorkerState::Idle};
    std::uint16_t list_first_id_{0U};
    std::uint16_t list_last_id_{0U};
    std::uint16_t list_current_id_{0U};
    std::uint16_t number_of_logs_{0U};
    std::uint16_t current_log_id_{0xffffU};
    std::uint32_t current_log_size_{0U};
    std::uint32_t data_offset_{0U};
    std::uint32_t data_end_offset_{0U};
    std::uint64_t last_data_activity_us_{0U};
    bool logs_listed_{false};
    bool storage_initialized_{false};
};

} // namespace dima::modules::mavlink
