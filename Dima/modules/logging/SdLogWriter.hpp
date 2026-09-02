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

#include "LogWriter.hpp"
#include "messages.h"

#include "mavlink_log.hpp"
#include "parameter_update.hpp"
#include "parameters/param.h"
#include "uORB/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <uORB/topics/uORBTopics.hpp>

#include <cstddef>
#include <cstdint>

namespace uORB {
class MessageFormatReader;
} // namespace uORB

namespace dima::modules::logging {

/**
 * PX4 v1.17 Logger 的产品 Topic producer。
 *
 * Topic、别名、ID 和字段格式全部来自 uORB 生成目录；本类只按 PX4 ULog v1
 * 顺序写 header/flags、F、P/Q、A/D、L、S 和 O。它不调用 FatFs，实际文件 I/O
 * 由 LogWriter 的 wq:storage consumer 完成。
 */
class SdLogWriter final : public px4::ScheduledWorkItem {
public:
    explicit SdLogWriter(dima::platform::LogFileStore &store) noexcept;

    bool start() noexcept;
    void stop() noexcept;

protected:
    void Run() override;

private:
    enum class SessionPhase : std::uint8_t {
        Header,
        Formats,
        Parameters,
        ParameterDefaults,
        Active,
    };

    enum class StepResult : std::uint8_t {
        Skipped,
        Emitted,
        Blocked,
        Failed,
    };

    enum class TopicResult : std::uint8_t {
        NoData,
        Written,
        Blocked,
        Failed,
    };

    static constexpr std::uint32_t kRunIntervalUs = 5000U;
    static constexpr std::uint64_t kSyncMarkerIntervalUs = 500000ULL;
    static constexpr std::size_t kMaximumFormatGroupsPerRun = 4U;
    static constexpr std::size_t kMaximumParametersPerRun = 8U;
    static constexpr std::size_t kMaximumChangedParametersPerRun = 8U;
    static constexpr std::size_t kMaximumTextRecordsPerRun = 8U;
    static constexpr std::size_t kMaximumTopicMessagesPerRun = 48U;
    static constexpr std::size_t kMaximumMessagesPerInstancePerRun = 4U;
    static constexpr std::uint16_t kInvalidMessageId = UINT16_MAX;
    static constexpr std::size_t kCatalogSlots =
        ORB_TOPICS_COUNT * uORB::kMaximumInstances;
    static constexpr std::size_t kMessageBufferSize =
        sizeof(ulog_message_format_s);
    static constexpr std::size_t kFormatReaderStorageSize = 512U;

    bool validate_catalog() const noexcept;
    void reset_session(std::uint32_t generation,
                       std::uint64_t now_us) noexcept;
    void reset_format_reader() noexcept;
    void destroy_format_reader() noexcept;
    void fail_stream(const char *reason) noexcept;

    bool write_file_header(std::uint64_t now_us) noexcept;
    StepResult write_format_group() noexcept;
    void process_formats() noexcept;
    StepResult write_current_parameter(param_t parameter,
                                       bool require_unsaved) noexcept;
    StepResult write_parameter_defaults(param_t parameter) noexcept;
    bool process_initial_parameters(bool defaults) noexcept;
    bool process_changed_parameters() noexcept;

    void note_dropout(std::uint64_t now_us) noexcept;
    bool write_active_message(const void *message, std::size_t size,
                              std::uint64_t now_us) noexcept;
    bool append_text_records(std::uint64_t now_us) noexcept;
    bool append_sync_marker(std::uint64_t now_us) noexcept;
    TopicResult append_topic(std::size_t topic_index,
                             std::uint8_t instance,
                             std::uint64_t now_us) noexcept;
    bool drain_topics(std::uint64_t now_us) noexcept;

    LogWriter writer_;
    uORB::SubscriptionData<mavlink_log_s> log_subscription_{
        ORB_ID(mavlink_log)};
    uORB::SubscriptionData<parameter_update_s> parameter_subscription_{
        ORB_ID(parameter_update)};
    /* MessageFormatReader 的 heatshrink 类型只在 .cpp 可见，避免第三方 ABI 沿
     * LogService.hpp 泄漏到组合根；cpp 中以 static_assert 核对尺寸和对齐。 */
    alignas(std::max_align_t)
        std::uint8_t format_reader_storage_[kFormatReaderStorageSize]{};
    uORB::MessageFormatReader *format_reader_{nullptr};
    ulog_message_format_s format_message_{};
    alignas(8) std::uint8_t message_buffer_[kMessageBufferSize]{};
    std::uint64_t topic_generations_[kCatalogSlots]{};
    std::uint16_t message_ids_[kCatalogSlots]{};
    std::uint32_t writer_session_generation_{0U};
    std::uint16_t next_message_id_{0U};
    std::size_t parameter_index_{0U};
    std::size_t changed_parameter_index_{0U};
    std::size_t scan_cursor_{0U};
    std::uint64_t last_sync_marker_us_{0U};
    std::uint64_t dropout_start_us_{0U};
    std::uint32_t message_gaps_{0U};
    SessionPhase phase_{SessionPhase::Header};
    bool changed_parameter_scan_pending_{false};
    bool running_{false};
};

} // namespace dima::modules::logging
