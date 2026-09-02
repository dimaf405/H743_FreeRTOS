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

#include "SdLogWriter.hpp"

#include "api/Time.hpp"
#include "parameters/param.h"
#include "uORB/uORBMessageFields.hpp"

#include <px4_platform_common/log.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <new>

namespace dima::modules::logging {
namespace {

constexpr std::uint8_t kUlogMagic[]{
    'U', 'L', 'o', 'g', 0x01U, 0x12U, 0x35U, 0x01U};
constexpr std::uint8_t kSyncMagic[]{
    0x2FU, 0x73U, 0x13U, 0x20U, 0x25U, 0x0CU, 0xBBU, 0x12U};

std::size_t bounded_length(const char *text, std::size_t capacity) noexcept
{
    if (text == nullptr) {
        return 0U;
    }
    std::size_t length = 0U;
    while (length < capacity && text[length] != '\0') {
        ++length;
    }
    return length;
}

void write_u16_le(std::uint8_t *destination, std::uint16_t value) noexcept
{
    destination[0] = static_cast<std::uint8_t>(value & 0xffU);
    destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

const char *parameter_type_name(param_type_t type) noexcept
{
    switch (type) {
    case PARAM_TYPE_INT32: return "int32_t";
    case PARAM_TYPE_FLOAT: return "float";
    default: return nullptr;
    }
}

bool read_parameter(param_t parameter, param_type_t type,
                    param_value_u &value) noexcept
{
    if (type == PARAM_TYPE_INT32) {
        return param_get(parameter, &value.i) == 0;
    }
    if (type == PARAM_TYPE_FLOAT) {
        return param_get(parameter, &value.f) == 0;
    }
    return false;
}

bool read_parameter_default(param_t parameter, param_type_t type,
                            bool system, param_value_u &value) noexcept
{
    void *destination = type == PARAM_TYPE_INT32
                            ? static_cast<void *>(&value.i)
                            : static_cast<void *>(&value.f);
    return system ? param_get_system_default_value(parameter, destination) == 0
                  : param_get_default_value(parameter, destination) == 0;
}

} // namespace

SdLogWriter::SdLogWriter(dima::platform::LogFileStore &store) noexcept
    : ScheduledWorkItem("sd_logger", px4::wq_configurations::lp_default),
      writer_(store)
{
}

bool SdLogWriter::validate_catalog() const noexcept
{
    const orb_metadata *const *topics = orb_get_topics();
    if (topics == nullptr || orb_topics_count() != ORB_TOPICS_COUNT) {
        return false;
    }

    /* Logger 不保存 Topic 尺寸副本；启动时直接验证生成 metadata 是否能放进
     * PX4 最大 ULog format buffer。未来 schema 超过边界会明确拒绝启动，而不是
     * 截断 D 消息并生成表面可打开、实际字段错位的文件。 */
    for (std::size_t index = 0U; index < orb_topics_count(); ++index) {
        const orb_metadata *metadata = topics[index];
        if (metadata == nullptr || metadata->o_id != index ||
            metadata->o_name == nullptr || metadata->o_size == 0U ||
            metadata->o_size_no_padding == 0U ||
            metadata->o_size_no_padding > metadata->o_size ||
            metadata->max_instances != uORB::kMaximumInstances ||
            sizeof(ulog_message_data_s) + metadata->o_size >
                sizeof(message_buffer_)) {
            return false;
        }
        const std::size_t name_length = bounded_length(
            metadata->o_name, sizeof(ulog_message_add_logged_s::message_name));
        if (name_length == 0U ||
            name_length == sizeof(ulog_message_add_logged_s::message_name)) {
            return false;
        }
    }
    return true;
}

bool SdLogWriter::start() noexcept
{
    if (running_) {
        return true;
    }
    if (!validate_catalog() || !writer_.start()) {
        return false;
    }

    running_ = true;
    if (!ScheduleEnable() ||
        !ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        running_ = false;
        ScheduleCancelAndDrain();
        writer_.stop();
        return false;
    }
    return true;
}

void SdLogWriter::stop() noexcept
{
    running_ = false;
    ScheduleCancelAndDrain();
    /* producer 已完全退出后才请求 storage consumer 冲刷，保证 SPSC Ring 不会在
     * stop 边沿再出现新数据；本线程自身仍不调用任何 FatFs API。 */
    writer_.stop();
    destroy_format_reader();
    writer_session_generation_ = 0U;
    dropout_start_us_ = 0U;
}

void SdLogWriter::destroy_format_reader() noexcept
{
    if (format_reader_ != nullptr) {
        format_reader_->~MessageFormatReader();
        format_reader_ = nullptr;
    }
}

void SdLogWriter::reset_format_reader() noexcept
{
    static_assert(sizeof(uORB::MessageFormatReader) <=
                      sizeof(format_reader_storage_),
                  "PX4 MessageFormatReader exceeds static Logger storage");
    static_assert(alignof(uORB::MessageFormatReader) <=
                      alignof(std::max_align_t),
                  "PX4 MessageFormatReader alignment exceeds static storage");
    destroy_format_reader();
    format_message_ = ulog_message_format_s{};
    format_reader_ = new (format_reader_storage_) uORB::MessageFormatReader(
        format_message_.format, sizeof(format_message_.format));
}

void SdLogWriter::reset_session(std::uint32_t generation,
                                std::uint64_t now_us) noexcept
{
    /* 每个新 FatFs 文件都是独立 ULog 流：Topic generation 从 0 重新取当前
     * retained 数据，A 的内部 msg_id 也从 0 重建。旧卡/旧文件的 ID 和格式状态
     * 绝不跨介质会话复用。 */
    std::fill(std::begin(topic_generations_), std::end(topic_generations_), 0U);
    std::fill(std::begin(message_ids_), std::end(message_ids_),
              kInvalidMessageId);
    std::memset(message_buffer_, 0, sizeof(message_buffer_));
    reset_format_reader();
    writer_session_generation_ = generation;
    next_message_id_ = 0U;
    parameter_index_ = 0U;
    changed_parameter_index_ = 0U;
    scan_cursor_ = 0U;
    last_sync_marker_us_ = now_us;
    dropout_start_us_ = 0U;
    message_gaps_ = 0U;
    changed_parameter_scan_pending_ = false;
    phase_ = SessionPhase::Header;
}

void SdLogWriter::fail_stream(const char *reason) noexcept
{
    /* 格式/目录合同错误是确定性软件错误，不能每 3 s 新建一个同样损坏的文件。
     * 仅停止 SD 副本；structured log、Event Ring 和 MAVLink STATUSTEXT 继续工作。 */
    PX4_ERR("ULog stream stopped: %s", reason == nullptr ? "error" : reason);
    running_ = false;
    ScheduleClear();
    writer_.request_stop();
}

bool SdLogWriter::write_file_header(std::uint64_t now_us) noexcept
{
    ulog_file_header_s header{};
    std::memcpy(header.magic, kUlogMagic, sizeof(kUlogMagic));
    header.timestamp = now_us;

    ulog_message_flag_bits_s flags{};
    flags.compat_flags[0] = ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK;
    flags.msg_size = sizeof(flags) - ULOG_MSG_HEADER_LEN;
    flags.msg_type = static_cast<std::uint8_t>(ULogMessageType::FLAG_BITS);

    /* Flag Bits 必须紧跟 16-byte 文件头。合并为一次 Ring 发布，使 consumer 即使
     * 立即运行，也不可能在二者之间观察到其他 producer 字节。 */
    std::uint8_t initial[sizeof(header) + sizeof(flags)]{};
    std::memcpy(initial, &header, sizeof(header));
    std::memcpy(initial + sizeof(header), &flags, sizeof(flags));
    return writer_.write_message(initial, sizeof(initial));
}

SdLogWriter::StepResult SdLogWriter::write_format_group() noexcept
{
    if (format_reader_ == nullptr) {
        return StepResult::Failed;
    }

    switch (format_reader_->readMore()) {
    case uORB::MessageFormatReader::State::ReadOrbIDs:
    case uORB::MessageFormatReader::State::ReadingFormat:
        return StepResult::Skipped;

    case uORB::MessageFormatReader::State::Complete:
        phase_ = SessionPhase::Parameters;
        parameter_index_ = 0U;
        return StepResult::Skipped;

    case uORB::MessageFormatReader::State::Failure:
        return StepResult::Failed;

    case uORB::MessageFormatReader::State::FormatComplete:
        break;
    }

    std::size_t output_aliases = 0U;
    for (const orb_id_size_t id : format_reader_->orbIDs()) {
        if (id >= orb_topics_count()) {
            return StepResult::Failed;
        }
        if (get_orb_meta(static_cast<ORB_ID>(id)) != ORB_ID(mavlink_log)) {
            ++output_aliases;
        }
    }
    /* 一份字段定义最多对应多个 Topic alias。先按 PX4 最大 F 结构保守预留整组
     * 空间，之后所有 alias 要么全部写入，要么在修改 reader buffer 前整体重试。 */
    if (writer_.available_bytes() <
        output_aliases * sizeof(ulog_message_format_s)) {
        return StepResult::Blocked;
    }

    unsigned format_length = format_reader_->formatLength();
    const unsigned leftover_length =
        format_reader_->moveLeftoverToBufferEnd();
    const int expanded = uORB::MessageFormatReader::expandMessageFormat(
        format_message_.format, format_length,
        sizeof(format_message_.format) - leftover_length);
    if (expanded < 0) {
        return StepResult::Failed;
    }
    format_length = static_cast<unsigned>(expanded);

    int last_name_length = 0;
    for (const orb_id_size_t id : format_reader_->orbIDs()) {
        const orb_metadata *metadata = get_orb_meta(static_cast<ORB_ID>(id));
        if (metadata == ORB_ID(mavlink_log)) {
            /* mavlink_log 按 PX4 logger 映射为 L 文本，不重复声明/写入普通 D Topic。 */
            continue;
        }
        if (metadata == nullptr) {
            return StepResult::Failed;
        }

        const int name_length =
            static_cast<int>(std::strlen(metadata->o_name)) + 1;
        if (format_length + name_length - last_name_length + 1U >
            sizeof(format_message_.format) - leftover_length) {
            return StepResult::Failed;
        }
        if (last_name_length != name_length) {
            std::memmove(format_message_.format + name_length,
                         format_message_.format + last_name_length,
                         format_length + 1U - last_name_length);
            format_message_.format[name_length - 1] = ':';
            format_length = static_cast<unsigned>(
                static_cast<int>(format_length) + name_length -
                last_name_length);
            last_name_length = name_length;
        }
        std::memcpy(format_message_.format, metadata->o_name,
                    static_cast<std::size_t>(name_length - 1));

        const std::size_t message_size =
            sizeof(format_message_) - sizeof(format_message_.format) +
            format_length;
        format_message_.msg_size = static_cast<std::uint16_t>(
            message_size - ULOG_MSG_HEADER_LEN);
        format_message_.msg_type =
            static_cast<std::uint8_t>(ULogMessageType::FORMAT);
        if (!writer_.write_message(&format_message_, message_size)) {
            /* 空间已在整组写入前预留；此处失败只可能是 SD 会话在发布期间
             * 被撤销。等待新 generation 从 header 重建，不能把热插拔误判成
             * 确定性的字段格式错误并永久停掉 Logger。 */
            return StepResult::Blocked;
        }
    }

    format_reader_->clearFormatAndRestoreLeftover();
    return output_aliases == 0U ? StepResult::Skipped : StepResult::Emitted;
}

void SdLogWriter::process_formats() noexcept
{
    std::size_t emitted = 0U;
    std::size_t decoder_steps = 0U;
    while (running_ && phase_ == SessionPhase::Formats &&
           emitted < kMaximumFormatGroupsPerRun && decoder_steps < 32U) {
        ++decoder_steps;
        const StepResult result = write_format_group();
        if (result == StepResult::Emitted) {
            ++emitted;
        } else if (result == StepResult::Blocked) {
            return;
        } else if (result == StepResult::Failed) {
            fail_stream("uORB format contract");
            return;
        }
    }
}

SdLogWriter::StepResult SdLogWriter::write_current_parameter(
    param_t parameter, bool require_unsaved) noexcept
{
    if (!param_used(parameter) ||
        (require_unsaved && !param_value_unsaved(parameter))) {
        return StepResult::Skipped;
    }
    if (writer_.available_bytes() < sizeof(ulog_message_parameter_s)) {
        return StepResult::Blocked;
    }

    const param_type_t type = param_type(parameter);
    const char *type_name = parameter_type_name(type);
    const char *name = param_name(parameter);
    if (type_name == nullptr || name == nullptr) {
        return StepResult::Skipped;
    }

    ulog_message_parameter_s message{};
    const int key_length = std::snprintf(
        message.key_value_str, sizeof(message.key_value_str),
        "%s %s", type_name, name);
    if (key_length <= 0 ||
        key_length >= static_cast<int>(sizeof(message.key_value_str)) ||
        key_length > std::numeric_limits<std::uint8_t>::max()) {
        return StepResult::Failed;
    }
    message.key_len = static_cast<std::uint8_t>(key_length);

    param_value_u value{};
    if (!read_parameter(parameter, type, value)) {
        return StepResult::Failed;
    }
    const std::size_t value_size = param_size(parameter);
    std::size_t message_size =
        sizeof(message) - sizeof(message.key_value_str) +
        static_cast<std::size_t>(key_length);
    std::memcpy(reinterpret_cast<std::uint8_t *>(&message) + message_size,
                &value, value_size);
    message_size += value_size;
    message.msg_size = static_cast<std::uint16_t>(
        message_size - ULOG_MSG_HEADER_LEN);
    message.msg_type = static_cast<std::uint8_t>(ULogMessageType::PARAMETER);
    return writer_.write_message(&message, message_size)
               ? StepResult::Emitted
               : StepResult::Blocked;
}

SdLogWriter::StepResult SdLogWriter::write_parameter_defaults(
    param_t parameter) noexcept
{
    if (!param_used(parameter) || param_is_volatile(parameter)) {
        return StepResult::Skipped;
    }
    if (writer_.available_bytes() <
        2U * sizeof(ulog_message_parameter_default_s)) {
        return StepResult::Blocked;
    }

    const param_type_t type = param_type(parameter);
    const char *type_name = parameter_type_name(type);
    const char *name = param_name(parameter);
    if (type_name == nullptr || name == nullptr) {
        return StepResult::Skipped;
    }

    param_value_u current{};
    param_value_u setup_default{};
    param_value_u system_default{};
    if (!read_parameter(parameter, type, current) ||
        !read_parameter_default(parameter, type, false, setup_default) ||
        !read_parameter_default(parameter, type, true, system_default)) {
        return StepResult::Failed;
    }

    ulog_message_parameter_default_s message{};
    const int key_length = std::snprintf(
        message.key_value_str, sizeof(message.key_value_str),
        "%s %s", type_name, name);
    if (key_length <= 0 ||
        key_length >= static_cast<int>(sizeof(message.key_value_str)) ||
        key_length > std::numeric_limits<std::uint8_t>::max()) {
        return StepResult::Failed;
    }
    message.key_len = static_cast<std::uint8_t>(key_length);
    message.msg_type =
        static_cast<std::uint8_t>(ULogMessageType::PARAMETER_DEFAULT);
    const std::size_t value_size = param_size(parameter);
    const std::size_t value_offset =
        sizeof(message) - sizeof(message.key_value_str) +
        static_cast<std::size_t>(key_length);
    const std::size_t message_size = value_offset + value_size;
    message.msg_size = static_cast<std::uint16_t>(
        message_size - ULOG_MSG_HEADER_LEN);

    const bool same_defaults =
        std::memcmp(&setup_default, &system_default, value_size) == 0;
    bool emitted = false;
    if (same_defaults) {
        if (std::memcmp(&current, &setup_default, value_size) != 0) {
            std::memcpy(reinterpret_cast<std::uint8_t *>(&message) +
                            value_offset,
                        &setup_default, value_size);
            message.default_types =
                ulog_parameter_default_type_t::current_setup |
                ulog_parameter_default_type_t::system;
            if (!writer_.write_message(&message, message_size)) {
                return StepResult::Blocked;
            }
            emitted = true;
        }
    } else {
        if (std::memcmp(&current, &setup_default, value_size) != 0) {
            std::memcpy(reinterpret_cast<std::uint8_t *>(&message) +
                            value_offset,
                        &setup_default, value_size);
            message.default_types =
                ulog_parameter_default_type_t::current_setup;
            if (!writer_.write_message(&message, message_size)) {
                return StepResult::Blocked;
            }
            emitted = true;
        }
        if (std::memcmp(&current, &system_default, value_size) != 0) {
            std::memcpy(reinterpret_cast<std::uint8_t *>(&message) +
                            value_offset,
                        &system_default, value_size);
            message.default_types = ulog_parameter_default_type_t::system;
            if (!writer_.write_message(&message, message_size)) {
                return StepResult::Blocked;
            }
            emitted = true;
        }
    }
    return emitted ? StepResult::Emitted : StepResult::Skipped;
}

bool SdLogWriter::process_initial_parameters(bool defaults) noexcept
{
    std::size_t processed = 0U;
    const std::size_t count = param_count();
    while (parameter_index_ < count &&
           processed < kMaximumParametersPerRun) {
        const param_t parameter = param_for_index(
            static_cast<unsigned>(parameter_index_));
        const StepResult result = defaults
                                      ? write_parameter_defaults(parameter)
                                      : write_current_parameter(parameter,
                                                                false);
        if (result == StepResult::Blocked) {
            return false;
        }
        if (result == StepResult::Failed) {
            fail_stream(defaults ? "parameter default" : "parameter value");
            return false;
        }
        ++parameter_index_;
        ++processed;
    }

    if (parameter_index_ >= count) {
        parameter_index_ = 0U;
        if (defaults) {
            phase_ = SessionPhase::Active;
            last_sync_marker_us_ = hrt_absolute_time();
        } else {
            phase_ = SessionPhase::ParameterDefaults;
        }
    }
    return true;
}

bool SdLogWriter::process_changed_parameters() noexcept
{
    if (parameter_subscription_.update()) {
        /* 与 PX4 write_changed_parameters 一致，从权威参数目录扫描 used+unsaved；
         * 新通知到达时从 0 重扫，避免上一轮有界切片漏掉更低索引的新变化。 */
        changed_parameter_scan_pending_ = true;
        changed_parameter_index_ = 0U;
    }
    if (!changed_parameter_scan_pending_) {
        return true;
    }

    std::size_t processed = 0U;
    const std::size_t count = param_count();
    while (changed_parameter_index_ < count &&
           processed < kMaximumChangedParametersPerRun) {
        const param_t parameter = param_for_index(
            static_cast<unsigned>(changed_parameter_index_));
        const StepResult result = write_current_parameter(parameter, true);
        if (result == StepResult::Blocked) {
            return false;
        }
        if (result == StepResult::Failed) {
            fail_stream("changed parameter");
            return false;
        }
        ++changed_parameter_index_;
        ++processed;
    }
    if (changed_parameter_index_ >= count) {
        changed_parameter_scan_pending_ = false;
        changed_parameter_index_ = 0U;
    }
    return true;
}

void SdLogWriter::note_dropout(std::uint64_t now_us) noexcept
{
    if (dropout_start_us_ == 0U) {
        dropout_start_us_ = now_us;
    }
}

bool SdLogWriter::write_active_message(const void *message, std::size_t size,
                                       std::uint64_t now_us) noexcept
{
    const std::size_t dropout_size =
        dropout_start_us_ == 0U ? 0U : sizeof(ulog_message_dropout_s);
    if (writer_.available_bytes() < size + dropout_size) {
        note_dropout(now_us);
        return false;
    }

    if (dropout_start_us_ != 0U) {
        ulog_message_dropout_s dropout{};
        const std::uint64_t elapsed_ms =
            now_us >= dropout_start_us_
                ? (now_us - dropout_start_us_) / 1000ULL
                : 0U;
        dropout.duration = static_cast<std::uint16_t>(
            std::min<std::uint64_t>(elapsed_ms,
                                    std::numeric_limits<std::uint16_t>::max()));
        if (!writer_.write_message(&dropout, sizeof(dropout))) {
            return false;
        }
    }
    if (!writer_.write_message(message, size)) {
        note_dropout(now_us);
        return false;
    }
    dropout_start_us_ = 0U;
    return true;
}

bool SdLogWriter::append_text_records(std::uint64_t now_us) noexcept
{
    for (std::size_t count = 0U; count < kMaximumTextRecordsPerRun; ++count) {
        if (!log_subscription_.update()) {
            return true;
        }
        const mavlink_log_s &record = log_subscription_.get();
        const std::size_t text_length =
            bounded_length(record.text, sizeof(record.text));
        if (text_length == 0U) {
            continue;
        }

        ulog_message_logging_s message{};
        message.log_level = static_cast<std::uint8_t>(
            '0' + std::min<std::uint8_t>(record.severity, 7U));
        message.timestamp = record.timestamp;
        const std::size_t copy = std::min(
            text_length, sizeof(message.message));
        std::memcpy(message.message, record.text, copy);
        const std::size_t message_size =
            sizeof(message) - sizeof(message.message) + copy;
        message.msg_size = static_cast<std::uint16_t>(
            message_size - ULOG_MSG_HEADER_LEN);
        message.msg_type = static_cast<std::uint8_t>(ULogMessageType::LOGGING);
        if (!write_active_message(&message, message_size, now_us)) {
            return false;
        }
    }
    return true;
}

bool SdLogWriter::append_sync_marker(std::uint64_t now_us) noexcept
{
    ulog_message_sync_s message{};
    message.msg_size = sizeof(message) - ULOG_MSG_HEADER_LEN;
    message.msg_type = static_cast<std::uint8_t>(ULogMessageType::SYNC);
    std::memcpy(message.sync_magic, kSyncMagic, sizeof(kSyncMagic));
    if (!write_active_message(&message, sizeof(message), now_us)) {
        return false;
    }
    last_sync_marker_us_ = now_us;
    return true;
}

SdLogWriter::TopicResult SdLogWriter::append_topic(
    std::size_t topic_index, std::uint8_t instance,
    std::uint64_t now_us) noexcept
{
    const orb_metadata *const metadata = orb_get_topics()[topic_index];
    if (metadata == ORB_ID(mavlink_log)) {
        return TopicResult::NoData;
    }
    const std::size_t slot =
        topic_index * uORB::kMaximumInstances + instance;
    const std::size_t name_length = std::strlen(metadata->o_name);
    const std::size_t add_size =
        sizeof(ulog_message_add_logged_s) -
        sizeof(ulog_message_add_logged_s::message_name) + name_length;
    const std::size_t data_size =
        sizeof(ulog_message_data_s) + metadata->o_size_no_padding;
    const std::size_t dropout_size =
        dropout_start_us_ == 0U ? 0U : sizeof(ulog_message_dropout_s);
    const std::size_t required = data_size + dropout_size +
        (message_ids_[slot] == kInvalidMessageId ? add_size : 0U);
    if (writer_.available_bytes() < required) {
        note_dropout(now_us);
        return TopicResult::Blocked;
    }

    const std::uint64_t previous_generation = topic_generations_[slot];
    if (!uORB::orb_copy(metadata, instance, topic_generations_[slot],
                        message_buffer_ + sizeof(ulog_message_data_s))) {
        return TopicResult::NoData;
    }

    if (previous_generation != 0U &&
        topic_generations_[slot] > previous_generation + 1U) {
        const std::uint64_t gap = topic_generations_[slot] -
                                  previous_generation - 1U;
        const std::uint64_t total =
            static_cast<std::uint64_t>(message_gaps_) + gap;
        message_gaps_ = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(
                total, std::numeric_limits<std::uint32_t>::max()));
    }

    if (message_ids_[slot] == kInvalidMessageId) {
        if (next_message_id_ == kInvalidMessageId) {
            return TopicResult::Failed;
        }
        ulog_message_add_logged_s add{};
        add.multi_id = instance;
        add.msg_id = next_message_id_;
        std::memcpy(add.message_name, metadata->o_name, name_length);
        add.msg_size = static_cast<std::uint16_t>(
            add_size - ULOG_MSG_HEADER_LEN);
        add.msg_type =
            static_cast<std::uint8_t>(ULogMessageType::ADD_LOGGED_MSG);
        if (!writer_.write_message(&add, add_size)) {
            return TopicResult::Blocked;
        }
        message_ids_[slot] = next_message_id_++;
    }

    const std::uint16_t payload_size = static_cast<std::uint16_t>(
        sizeof(std::uint16_t) + metadata->o_size_no_padding);
    write_u16_le(message_buffer_, payload_size);
    message_buffer_[2] = static_cast<std::uint8_t>(ULogMessageType::DATA);
    write_u16_le(message_buffer_ + ULOG_MSG_HEADER_LEN, message_ids_[slot]);
    if (!write_active_message(message_buffer_, data_size, now_us)) {
        return TopicResult::Blocked;
    }
    return TopicResult::Written;
}

bool SdLogWriter::drain_topics(std::uint64_t now_us) noexcept
{
    const std::size_t total_slots = kCatalogSlots;
    std::size_t checked = 0U;
    std::size_t written = 0U;
    while (checked < total_slots &&
           written < kMaximumTopicMessagesPerRun) {
        const std::size_t slot = scan_cursor_;
        scan_cursor_ = (scan_cursor_ + 1U) % total_slots;
        ++checked;

        const std::size_t topic_index =
            slot / uORB::kMaximumInstances;
        const std::uint8_t instance = static_cast<std::uint8_t>(
            slot % uORB::kMaximumInstances);
        const orb_metadata *metadata = orb_get_topics()[topic_index];
        if (metadata == ORB_ID(mavlink_log) ||
            instance >= metadata->max_instances) {
            continue;
        }

        const std::size_t burst = std::min<std::size_t>(
            metadata->o_queue, kMaximumMessagesPerInstancePerRun);
        for (std::size_t index = 0U;
             index < burst && written < kMaximumTopicMessagesPerRun;
             ++index) {
            const TopicResult result = append_topic(
                topic_index, instance, now_us);
            if (result == TopicResult::Written) {
                ++written;
            } else if (result == TopicResult::NoData) {
                break;
            } else if (result == TopicResult::Blocked) {
                scan_cursor_ = slot;
                return false;
            } else {
                fail_stream("Topic message ID or size");
                return false;
            }
        }
    }
    return true;
}

void SdLogWriter::Run()
{
    if (!running_ || !writer_.ready()) {
        return;
    }

    const std::uint64_t now = hrt_absolute_time();
    const std::uint32_t generation = writer_.session_generation();
    if (generation == 0U) {
        return;
    }
    if (generation != writer_session_generation_) {
        reset_session(generation, now);
    }

    switch (phase_) {
    case SessionPhase::Header:
        if (write_file_header(now)) {
            phase_ = SessionPhase::Formats;
        }
        return;

    case SessionPhase::Formats:
        process_formats();
        return;

    case SessionPhase::Parameters:
        (void)process_initial_parameters(false);
        return;

    case SessionPhase::ParameterDefaults:
        (void)process_initial_parameters(true);
        return;

    case SessionPhase::Active:
        break;
    }

    if (!process_changed_parameters()) {
        note_dropout(now);
        return;
    }
    if (!append_text_records(now) || !drain_topics(now)) {
        return;
    }
    if (last_sync_marker_us_ == 0U || now < last_sync_marker_us_ ||
        now - last_sync_marker_us_ >= kSyncMarkerIntervalUs) {
        (void)append_sync_marker(now);
    }
}

} // namespace dima::modules::logging
