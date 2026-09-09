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

#include "api/BoardIdentity.hpp"
#include "api/Services.hpp"
#include "api/TaskRuntime.hpp"
#include "api/Time.hpp"
#include "format/Format.hpp"
#include "parameters/param.h"
#include "uORB/uORBMessageFields.hpp"

#include <px4_platform_common/log.h>

#include <algorithm>
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
constexpr std::uint64_t kUtc2020Us = 1577836800000000ULL;
constexpr std::uint64_t kMavlinkMaximumUtcUs =
    static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) *
        1000000ULL +
    999999ULL;
constexpr std::uint64_t kUtcSampleMaximumAgeUs = 1000000ULL;
constexpr std::uint64_t kUtcConfirmationToleranceUs = 1000000ULL;
constexpr std::uint64_t kUtcJumpToleranceUs = 2000000ULL;
constexpr std::uint64_t kUtcWarningIntervalUs = 60000000ULL;

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

int format_parameter_key(char *key, std::size_t capacity,
                         const char *type_name, const char *name) noexcept
{
    // 类型/名称来自参数目录，缓冲容量由生成的 P/Q 消息结构提供；两条写入路径
    // 共用长度门禁，避免截断字符串或 uint8 key_len 回绕进入 ULog。
    // 复用现有缓冲格式化器；返回完整所需长度并保留截断/NUL 语义，不改变 ULog 路由。
    const int length = dima::format::format_to(key, capacity, "%s %s", type_name, name);
    if (length <= 0 || length >= static_cast<int>(capacity) ||
        length > std::numeric_limits<std::uint8_t>::max()) {
        return -1;
    }
    return length;
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

bool SdLogWriter::topic_enabled(std::size_t topic_index) const noexcept
{
    return topic_index < ORB_TOPICS_COUNT &&
           generated::sampling_policy(topic_index, profile_).kind !=
               generated::SamplingKind::Excluded;
}

bool SdLogWriter::register_replay_callbacks() noexcept
{
    if (replay_callbacks_registered_ ||
        (profile_ & generated::kEkf2ReplayProfile) == 0U) {
        return true;
    }
    std::size_t registered = 0U;
    for (const std::size_t topic_index :
         generated::kReplayCallbackTopicIndices) {
        if (topic_index >= orb_topics_count() ||
            !uORB::orb_register_callback(
                orb_get_topics()[topic_index], 0U, *this)) {
            while (registered > 0U) {
                --registered;
                const std::size_t rollback_index =
                    generated::kReplayCallbackTopicIndices[registered];
                uORB::orb_unregister_callback(
                    orb_get_topics()[rollback_index], 0U, *this);
            }
            return false;
        }
        ++registered;
    }
    replay_callbacks_registered_ = true;
    return true;
}

void SdLogWriter::unregister_replay_callbacks() noexcept
{
    if (!replay_callbacks_registered_) {
        return;
    }
    for (const std::size_t topic_index :
         generated::kReplayCallbackTopicIndices) {
        if (topic_index < orb_topics_count()) {
            uORB::orb_unregister_callback(
                orb_get_topics()[topic_index], 0U, *this);
        }
    }
    replay_callbacks_registered_ = false;
}

bool SdLogWriter::start(const Configuration &configuration) noexcept
{
    if (running_) {
        return true;
    }
    if (configuration.profile > generated::kProfileMask ||
        configuration.maximum_directories == 0U ||
        configuration.maximum_directories > 999U || !validate_catalog()) {
        return false;
    }

    profile_ = configuration.profile;
    hardware_uid_ = configuration.hardware_uid;
    /* 模块可在同一次上电中 stop/start。Writer 的 session generation 会从 1
     * 重新计数，因此 UTC 写入代际和两点确认状态也必须重新建立，不能误把上次
     * 会话的 generation=1 当成新文件已经写过 boot_time_utc_us。 */
    pending_boot_utc_us_ = 0U;
    confirmed_boot_utc_us_ = 0U;
    last_utc_jump_warning_us_ = 0U;
    boot_time_written_generation_ = 0U;
    pending_utc_candidate_ = false;
    utc_confirmed_ = false;
    writer_session_generation_ = 0U;
    dima::platform::LogSessionContext context{};
    context.maximum_directories = configuration.maximum_directories;
    context.hardware_uid = configuration.hardware_uid;
    if (!writer_.start(context)) {
        return false;
    }

    running_ = true;
    recording_intent_ = false;
    stream_failed_ = false;
    if (!ScheduleEnable() ||
        !ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        running_ = false;
        ScheduleCancelAndDrain();
        writer_.stop();
        return false;
    }
    return true;
}

void SdLogWriter::set_recording_intent(bool enabled) noexcept
{
    if (!running_ || stream_failed_ || enabled == recording_intent_) {
        return;
    }
    if (enabled) {
        recording_intent_ = true;
        if (!register_replay_callbacks()) {
            /* 5 ms 周期扫描仍能覆盖八槽/400 Hz 的 20 ms 保留窗口；回调注册失败
             * 只降低唤醒及时性，不得让非关键 Logger 阻止整机启动。 */
            PX4_WARN("ULog replay callback unavailable; using 5 ms scan");
        }
        writer_.set_recording_intent(true);
        (void)ScheduleNow();
        return;
    }

    /* 极短会话也必须先形成完整的 Definitions：否则一个已经带 ULog magic、会被
     * QGC 列出的文件可能缺少 UID、F 或 P/Q。这里仍只写 RAM Ring，等待动作让
     * storage worker 排空；任何 FatFs 调用都不会越过线程边界。 */
    if (writer_.ready()) {
        const std::uint32_t generation = writer_.session_generation();
        if (generation != 0U && generation != writer_session_generation_) {
            reset_session(generation, writer_.session_context());
        }
        while (writer_.ready() && !stream_failed_ &&
               phase_ != SessionPhase::Active) {
            const std::uint64_t now_us = hrt_absolute_time();
            process_gps_time(now_us);
            advance_definition_phase();
            if (phase_ != SessionPhase::Active) {
                dima::platform::services().tasks.delay(
                    dima::platform::Timeout::from_ms(1U));
            }
        }
    }

    /* Mode 的停止边沿先补写已确认 UTC 和安全状态 Topic 的最新 generation，
     * 再封住 producer。writer_.end_session() 后只由 storage worker 清空 Ring
     * 并提交 CLOSED 侧车。 */
    if (writer_.ready() && phase_ == SessionPhase::Active) {
        process_gps_time(hrt_absolute_time());
        while (writer_.ready() && !stream_failed_ &&
               !write_late_boot_time()) {
            dima::platform::services().tasks.delay(
                dima::platform::Timeout::from_ms(1U));
        }
        /* Ring 满时 storage worker 仍在异步排空。保持 producer 权限并短暂让出
         * CPU，直到三个生成标记的安全状态均推进到最新 generation；介质失败
         * 会撤销 ready 并结束等待，绝不从本线程越界调用 FatFs。 */
        while (writer_.ready() && !stream_failed_ &&
               !flush_stop_topics(hrt_absolute_time())) {
            dima::platform::services().tasks.delay(
                dima::platform::Timeout::from_ms(1U));
        }
    }
    recording_intent_ = false;
    unregister_replay_callbacks();
    writer_.end_session();
    writer_session_generation_ = 0U;
}

void SdLogWriter::stop() noexcept
{
    if (!running_) {
        ScheduleCancelAndDrain();
        writer_.stop();
        return;
    }
    set_recording_intent(false);
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

void SdLogWriter::reset_session(
    std::uint32_t generation,
    const dima::platform::LogSessionContext &context) noexcept
{
    /* 每个新 FatFs 文件都是独立 ULog 流：Topic generation 从 0 重新取当前
     * retained 数据，A 的内部 msg_id 也从 0 重建。旧卡/旧文件的 ID 和格式状态
     * 绝不跨介质会话复用。 */
    std::fill(std::begin(topic_generations_), std::end(topic_generations_), 0U);
    std::fill(std::begin(last_topic_write_us_),
              std::end(last_topic_write_us_), 0U);
    std::fill(std::begin(message_ids_), std::end(message_ids_),
              kInvalidMessageId);
    std::memset(message_buffer_, 0, sizeof(message_buffer_));
    reset_format_reader();
    writer_session_generation_ = generation;
    next_message_id_ = 0U;
    parameter_index_ = 0U;
    changed_parameter_index_ = 0U;
    scan_cursor_ = 0U;
    information_step_ = 0U;
    ulog_header_monotonic_us_ = context.start_monotonic_us;
    last_sync_marker_us_ = ulog_header_monotonic_us_;
    dropout_start_us_ = 0U;
    changed_parameter_scan_pending_ = false;
    format_group_ready_ = false;
    phase_ = SessionPhase::Header;
}

void SdLogWriter::fail_stream(const char *reason) noexcept
{
    /* 格式/目录合同错误是确定性软件错误，不能每 3 s 新建一个同样损坏的文件。
     * 仅停止 SD 副本；structured log、Event Ring 和 MAVLink STATUSTEXT 继续工作。 */
    PX4_ERR("ULog stream stopped: %s", reason == nullptr ? "error" : reason);
    stream_failed_ = true;
    recording_intent_ = false;
    unregister_replay_callbacks();
    ScheduleClear();
    writer_.end_session();
}

bool SdLogWriter::write_file_header() noexcept
{
    ulog_file_header_s header{};
    std::memcpy(header.magic, kUlogMagic, sizeof(kUlogMagic));
    header.timestamp = ulog_header_monotonic_us_;

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

SdLogWriter::StepResult SdLogWriter::write_info(
    const char *key, const void *value, std::size_t value_size) noexcept
{
    if (key == nullptr || value == nullptr || value_size == 0U) {
        return StepResult::Failed;
    }
    const std::size_t key_length = std::strlen(key);
    if (key_length == 0U ||
        key_length > std::numeric_limits<std::uint8_t>::max() ||
        key_length + value_size > sizeof(ulog_message_info_s::key_value_str)) {
        return StepResult::Failed;
    }
    const std::size_t message_size =
        sizeof(ulog_message_info_s) -
        sizeof(ulog_message_info_s::key_value_str) + key_length + value_size;
    if (writer_.available_bytes() < message_size) {
        return StepResult::Blocked;
    }

    ulog_message_info_s message{};
    message.key_len = static_cast<std::uint8_t>(key_length);
    std::memcpy(message.key_value_str, key, key_length);
    std::memcpy(message.key_value_str + key_length, value, value_size);
    message.msg_size = static_cast<std::uint16_t>(
        message_size - ULOG_MSG_HEADER_LEN);
    message.msg_type = static_cast<std::uint8_t>(ULogMessageType::INFO);
    return writer_.write_message(&message, message_size)
               ? StepResult::Emitted
               : StepResult::Blocked;
}

bool SdLogWriter::process_initial_information() noexcept
{
    /* ULog 信息值是 key 后紧接二进制 payload：sys_uuid 固定写与
     * AUTOPILOT_VERSION.uid 相同的 64-bit 数值，格式化为 16 位大写十六进制；
     * UID=0 仍写全零并由 LogService 告警，不让身份异常阻断日志。 */
    while (information_step_ < 3U) {
        StepResult result = StepResult::Skipped;
        if (information_step_ == 0U) {
            char uuid[17]{};
            // 现有格式子集支持 64-bit 大写十六进制与补零，仍保留 16 字节正文及末尾 NUL。
            const int length = dima::format::format_to(
                uuid, sizeof(uuid), "%016llX",
                static_cast<unsigned long long>(hardware_uid_));
            if (length != 16) {
                fail_stream("hardware UID format");
                return false;
            }
            result = write_info("char[16] sys_uuid", uuid, 16U);
        } else if (information_step_ == 1U) {
            const std::int32_t utc_offset_minutes = 0;
            result = write_info("int32_t time_ref_utc", &utc_offset_minutes,
                                sizeof(utc_offset_minutes));
        } else if (utc_confirmed_) {
            result = write_info("uint64_t boot_time_utc_us",
                                &confirmed_boot_utc_us_,
                                sizeof(confirmed_boot_utc_us_));
            if (result == StepResult::Emitted) {
                boot_time_written_generation_ = writer_session_generation_;
            }
        }

        if (result == StepResult::Blocked) {
            return false;
        }
        if (result == StepResult::Failed) {
            fail_stream("ULog information");
            return false;
        }
        ++information_step_;
    }
    phase_ = SessionPhase::Formats;
    return true;
}

bool SdLogWriter::write_late_boot_time() noexcept
{
    if (!utc_confirmed_ ||
        boot_time_written_generation_ == writer_session_generation_) {
        return true;
    }
    const StepResult result = write_info(
        "uint64_t boot_time_utc_us", &confirmed_boot_utc_us_,
        sizeof(confirmed_boot_utc_us_));
    if (result == StepResult::Emitted) {
        /* ULog 允许 I 消息出现在 Data section；这使冷启动后才确认的 GPS UTC
         * 无需伪造启动时间，也无需关闭正在写入的有效日志。 */
        boot_time_written_generation_ = writer_session_generation_;
        return true;
    }
    if (result == StepResult::Failed) {
        fail_stream("late boot UTC information");
    }
    return false;
}

void SdLogWriter::advance_definition_phase() noexcept
{
    /* Definitions 的每一步都有固定上限；正常 Run 每次推进一段，极短会话的关闭
     * 路径可复用同一状态机补齐合同，而不会维护第二套 UID/F/P/Q 写入顺序。 */
    switch (phase_) {
    case SessionPhase::Header:
        if (write_file_header()) {
            phase_ = SessionPhase::Information;
        }
        break;
    case SessionPhase::Information:
        (void)process_initial_information();
        break;
    case SessionPhase::Formats:
        process_formats();
        break;
    case SessionPhase::Parameters:
        (void)process_initial_parameters(false);
        break;
    case SessionPhase::ParameterDefaults:
        (void)process_initial_parameters(true);
        break;
    case SessionPhase::Active:
        break;
    }
}

SdLogWriter::StepResult SdLogWriter::write_format_group() noexcept
{
    if (format_reader_ == nullptr) {
        return StepResult::Failed;
    }

    if (!format_group_ready_) {
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
            /* MessageFormatReader 要求 FormatComplete 后必须 clear；若 Ring
             * 暂时不足，保留此标志并重试同一组，不能再次 readMore() 把正常
             * 背压误判成 Invalid API calls。 */
            format_group_ready_ = true;
            break;
        }
    }

    std::size_t output_aliases = 0U;
    for (const orb_id_size_t id : format_reader_->orbIDs()) {
        if (id >= orb_topics_count()) {
            return StepResult::Failed;
        }
        if (topic_enabled(id)) {
            ++output_aliases;
        }
    }
    if (output_aliases == 0U) {
        // 当前 Profile 不输出此组时，直接消费完整压缩格式并保留下组剩余字节；
        // 不展开或搬移字段字符串。清零 ready 后下一轮才可继续 readMore。
        format_reader_->clearFormatFromBuffer();
        format_group_ready_ = false;
        return StepResult::Skipped;
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
        if (!topic_enabled(id)) {
            /* 同组中重复 payload 或当前 Profile 未选 Topic 不写 F/A/D；
             * 保留项和采样策略都来自生成合同。 */
            continue;
        }
        const orb_metadata *metadata = get_orb_meta(static_cast<ORB_ID>(id));
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
    format_group_ready_ = false;
    return StepResult::Emitted;
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
    if (require_unsaved && !param_value_unsaved(parameter)) {
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
    const int key_length = format_parameter_key(
        message.key_value_str, sizeof(message.key_value_str), type_name, name);
    if (key_length < 0) {
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
    if (param_is_volatile(parameter)) {
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
    const int key_length = format_parameter_key(
        message.key_value_str, sizeof(message.key_value_str), type_name, name);
    if (key_length < 0) {
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
    // 相同默认值只写合并类型的一条 Q；不同默认值保持 setup -> system 顺序。
    // 上面的双记录空间预留仍生效；任一次写入背压立即返回，不跳过未发送项。
    const auto write_default = [&](const param_value_u &value,
                                   ulog_parameter_default_type_t types) noexcept {
        if (std::memcmp(&current, &value, value_size) == 0) {
            return StepResult::Skipped;
        }
        std::memcpy(reinterpret_cast<std::uint8_t *>(&message) + value_offset,
                    &value, value_size);
        message.default_types = types;
        return writer_.write_message(&message, message_size)
                   ? StepResult::Emitted : StepResult::Blocked;
    };
    const auto setup_types = same_defaults
        ? ulog_parameter_default_type_t::current_setup | ulog_parameter_default_type_t::system
        : ulog_parameter_default_type_t::current_setup;
    const StepResult setup = write_default(setup_default, setup_types);
    if (setup == StepResult::Blocked || same_defaults) {
        return setup;
    }
    const StepResult system = write_default(
        system_default, ulog_parameter_default_type_t::system);
    if (system == StepResult::Blocked) {
        return system;
    }
    return setup == StepResult::Emitted || system == StepResult::Emitted
               ? StepResult::Emitted : StepResult::Skipped;
}

bool SdLogWriter::process_initial_parameters(bool defaults) noexcept
{
    /* Logger 早于多数业务模块启动，不能用瞬时 param_used 集合决定文件合同。
     * 直接遍历生成参数目录可保证每个会话都有完整初值；Q 默认仍按 PX4 规则
     * 只写与当前值不同的 setup/system 值，volatile 项不声明持久默认。 */
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
            initialize_active_generations();
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
        /* 与 PX4 write_changed_parameters 一致，从权威参数目录扫描 unsaved；
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
    std::uint64_t now_us, generated::SamplingKind kind,
    bool newest_only) noexcept
{
    const orb_metadata *const metadata = orb_get_topics()[topic_index];
    if (kind == generated::SamplingKind::Excluded ||
        generated::topic_policy(topic_index).disposition !=
            generated::TopicDisposition::Record) {
        return TopicResult::NoData;
    }
    const std::size_t slot =
        topic_index * uORB::kMaximumInstances + instance;
    if (!uORB::orb_updated(metadata, instance, topic_generations_[slot])) {
        /* 先确认确有新 generation，再用最坏情况预留 A/D/O 空间。否则 Ring
         * 接近满时，一个从未发布或没有更新的 Topic 会制造假 dropout，并
         * 阻塞排在其后的真实数据与停止边沿状态。 */
        return TopicResult::NoData;
    }
    const std::size_t name_length = std::strlen(metadata->o_name);
    const std::size_t add_size =
        sizeof(ulog_message_add_logged_s) -
        sizeof(ulog_message_add_logged_s::message_name) + name_length;
    const std::size_t data_size =
        sizeof(ulog_message_data_s) + metadata->o_size_no_padding;
    const bool source_rate = kind == generated::SamplingKind::SourceRate;
    const std::size_t dropout_size =
        dropout_start_us_ == 0U && !source_rate
            ? 0U
            : sizeof(ulog_message_dropout_s);
    const std::size_t required = data_size + dropout_size +
        (message_ids_[slot] == kInvalidMessageId ? add_size : 0U);
    if (writer_.available_bytes() < required) {
        note_dropout(now_us);
        return TopicResult::Blocked;
    }

    const std::uint64_t previous_generation = topic_generations_[slot];
    // 仅正常 source-rate 排空历史；固定频率和停止边沿都只取最新状态。
    const bool copied = source_rate && !newest_only
                            ? uORB::orb_copy(
                                  metadata, instance, topic_generations_[slot],
                                  message_buffer_ + sizeof(ulog_message_data_s))
                            : uORB::orb_copy_latest(
                                  metadata, instance, topic_generations_[slot],
                                  message_buffer_ + sizeof(ulog_message_data_s));
    if (!copied) {
        return TopicResult::NoData;
    }

    const bool source_generation_gap = source_rate &&
        ((previous_generation == 0U && topic_generations_[slot] > 1U) ||
         (topic_generations_[slot] > previous_generation &&
          topic_generations_[slot] - previous_generation > 1U));
    if (source_generation_gap) {
        /* source-rate generation 被队列覆盖才是真实丢失。用该 Topic 上次成功写入
         * 的单调时刻作为 dropout 起点；固定频率主动跳过的 generation 永不走
         * 此分支，因而不会把产品降采样伪装成 SD 性能故障。 */
        const std::uint64_t lost_since = last_topic_write_us_[slot];
        note_dropout(lost_since != 0U ? lost_since : now_us);
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
    last_topic_write_us_[slot] = now_us;
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
        const generated::SamplingPolicy sampling =
            generated::sampling_policy(topic_index, profile_);
        if (sampling.kind == generated::SamplingKind::Excluded ||
            instance >= metadata->max_instances) {
            continue;
        }

        const bool fixed_rate = sampling.kind == generated::SamplingKind::FixedRate;
        if (fixed_rate) {
            const std::uint64_t last = last_topic_write_us_[slot];
            if (last != 0U && now_us >= last &&
                now_us - last < sampling.interval_us) {
                continue;
            }
        }

        // 固定频率本轮最多一条；源频率受队列深度和每轮预算共同限制。
        // 两者共用 Written/NoData/Blocked/Failed 处理，背压仍重试当前 slot。
        const std::size_t burst = fixed_rate ? 1U : std::min<std::size_t>(
            metadata->o_queue, kMaximumMessagesPerInstancePerRun);
        for (std::size_t index = 0U;
             index < burst && written < kMaximumTopicMessagesPerRun;
            ++index) {
            const TopicResult result = append_topic(
                topic_index, instance, now_us, sampling.kind);
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

void SdLogWriter::initialize_active_generations() noexcept
{
    /* Definitions 期间发布的数据不在“完整回放”保证范围内。进入 Active 的
     * 瞬间把所有 source-rate 订阅游标对齐到当前 newest，之后每个 generation
     * 才逐项排空；这样不会把仅存于八槽历史窗口的启动旧样本冒充 Active 数据。 */
    for (std::size_t topic_index = 0U; topic_index < ORB_TOPICS_COUNT;
         ++topic_index) {
        const generated::SamplingPolicy sampling =
            generated::sampling_policy(topic_index, profile_);
        if (sampling.kind != generated::SamplingKind::SourceRate) {
            continue;
        }
        const orb_metadata *metadata = orb_get_topics()[topic_index];
        for (std::uint8_t instance = 0U; instance < metadata->max_instances;
             ++instance) {
            const std::size_t slot =
                topic_index * uORB::kMaximumInstances + instance;
            (void)uORB::orb_copy_latest(
                metadata, instance, topic_generations_[slot],
                message_buffer_ + sizeof(ulog_message_data_s));
        }
    }
}

bool SdLogWriter::flush_stop_topics(std::uint64_t now_us) noexcept
{
    /* actuator_armed/vehicle_control_mode/vehicle_status 的 flush 标志来自生成
     * Topic 合同。先逐 generation 写这些锁定边沿状态，再处理参数、文本和普通
     * 扫描；若 Ring 已满，write_active_message 会留下真实 dropout，而不会在
     * producer 停止后从非 storage 线程直接操作 FatFs。 */
    for (std::size_t topic_index = 0U; topic_index < ORB_TOPICS_COUNT;
         ++topic_index) {
        const auto &policy = generated::topic_policy(topic_index);
        const generated::SamplingPolicy sampling =
            generated::sampling_policy(topic_index, profile_);
        if (!policy.flush_on_stop ||
            sampling.kind == generated::SamplingKind::Excluded) {
            continue;
        }
        const orb_metadata *metadata = orb_get_topics()[topic_index];
        for (std::uint8_t instance = 0U; instance < metadata->max_instances;
             ++instance) {
            /* 停止合同要求的是“尚未记录的最新状态”，不是在关文件前追赶整段
             * 历史队列。保留 source-rate 的 generation gap 检测，但直接复制
             * newest，确保 disarm/status 边沿优先于普通 Topic 落盘。 */
            const TopicResult result = append_topic(
                topic_index, instance, now_us,
                generated::SamplingKind::SourceRate, true);
            if (result == TopicResult::Blocked) {
                return false;
            }
            if (result == TopicResult::Failed) {
                fail_stream("stop-edge Topic");
                return false;
            }
        }
    }
    (void)process_changed_parameters();
    (void)append_text_records(now_us);
    return true;
}

bool SdLogWriter::gps_candidate(const sensor_gps_s &gps,
                                std::uint64_t now_us,
                                std::uint64_t &boot_utc_us) const noexcept
{
    boot_utc_us = 0U;
    if (gps.timestamp == 0U || gps.time_utc_usec < kUtc2020Us ||
        gps.time_utc_usec > kMavlinkMaximumUtcUs) {
        return false;
    }

    std::uint64_t measurement_monotonic_us = gps.timestamp;
    if (gps.timestamp_time_relative >= 0) {
        const std::uint64_t relative = static_cast<std::uint64_t>(
            gps.timestamp_time_relative);
        if (measurement_monotonic_us >
            std::numeric_limits<std::uint64_t>::max() - relative) {
            return false;
        }
        measurement_monotonic_us += relative;
    } else {
        const std::uint64_t relative = static_cast<std::uint64_t>(
            -static_cast<std::int64_t>(gps.timestamp_time_relative));
        if (measurement_monotonic_us < relative) {
            return false;
        }
        measurement_monotonic_us -= relative;
    }
    if (measurement_monotonic_us > now_us ||
        now_us - measurement_monotonic_us > kUtcSampleMaximumAgeUs ||
        gps.time_utc_usec < measurement_monotonic_us) {
        return false;
    }

    /* GPS 给出的 UTC 对应 measurement_monotonic_us，因此启动映射公式为
     * boot_utc_us = time_utc_usec - (timestamp + timestamp_time_relative)。
     * 全程使用 us；任何加减溢出、2020 年前值或 MAVLink uint32 秒上限外值
     * 都只丢弃候选，不停止 ULog。 */
    boot_utc_us = gps.time_utc_usec - measurement_monotonic_us;
    if (boot_utc_us > std::numeric_limits<std::uint64_t>::max() - now_us) {
        return false;
    }
    const std::uint64_t current_utc_us = boot_utc_us + now_us;
    if (current_utc_us < kUtc2020Us ||
        current_utc_us > kMavlinkMaximumUtcUs) {
        return false;
    }

    if (writer_.ready()) {
        const dima::platform::LogSessionContext context =
            writer_.session_context();
        if (context.start_monotonic_us == 0U ||
            context.start_monotonic_us > now_us ||
            boot_utc_us > std::numeric_limits<std::uint64_t>::max() -
                              context.start_monotonic_us) {
            return false;
        }
        const std::uint64_t start_utc_us =
            boot_utc_us + context.start_monotonic_us;
        if (start_utc_us < kUtc2020Us ||
            start_utc_us > kMavlinkMaximumUtcUs) {
            return false;
        }
    }
    return true;
}

void SdLogWriter::publish_time_reference(std::uint64_t boot_utc_us) noexcept
{
    confirmed_boot_utc_us_ = boot_utc_us;
    utc_confirmed_ = true;
    pending_utc_candidate_ = false;
    dima::platform::LogTimeReference reference{};
    reference.boot_utc_us = boot_utc_us;
    reference.valid = true;
    /* 这里只向 SPSC writer 发布 RAM 快照；真正的 meta.bin 追加固定由
     * wq:storage 调用 LogFileStore::update_log_time()。 */
    writer_.update_time_reference(reference);
}

void SdLogWriter::process_gps_time(std::uint64_t now_us) noexcept
{
    for (std::size_t count = 0U; count < 4U && gps_subscription_.update();
         ++count) {
        std::uint64_t candidate = 0U;
        if (!gps_candidate(gps_subscription_.get(), now_us, candidate)) {
            continue;
        }

        if (utc_confirmed_) {
            const std::uint64_t difference =
                candidate >= confirmed_boot_utc_us_
                    ? candidate - confirmed_boot_utc_us_
                    : confirmed_boot_utc_us_ - candidate;
            if (difference > kUtcJumpToleranceUs &&
                (last_utc_jump_warning_us_ == 0U ||
                 now_us < last_utc_jump_warning_us_ ||
                 now_us - last_utc_jump_warning_us_ >=
                     kUtcWarningIntervalUs)) {
                /* 已确认后超过 2 s 的 GPS 跳变不能改写既有文件日期，否则同一
                 * 会话会出现两套 UTC；仅限频告警，日志继续使用首次确认映射。 */
                PX4_WARN("ULog ignored GPS UTC jump (%llu us)",
                         static_cast<unsigned long long>(difference));
                last_utc_jump_warning_us_ = now_us;
            }
            continue;
        }

        if (!pending_utc_candidate_) {
            pending_boot_utc_us_ = candidate;
            pending_utc_candidate_ = true;
            continue;
        }
        const std::uint64_t difference =
            candidate >= pending_boot_utc_us_
                ? candidate - pending_boot_utc_us_
                : pending_boot_utc_us_ - candidate;
        if (difference <= kUtcConfirmationToleranceUs) {
            publish_time_reference(candidate);
        } else {
            /* 不连续候选重新作为第一点；必须再有下一份一致样本才能确认。 */
            pending_boot_utc_us_ = candidate;
        }
    }
}

void SdLogWriter::Run()
{
    if (!running_ || stream_failed_ || !recording_intent_) {
        return;
    }

    const std::uint64_t now = hrt_absolute_time();
    process_gps_time(now);
    if (!writer_.ready()) {
        return;
    }

    const std::uint32_t generation = writer_.session_generation();
    if (generation == 0U) {
        return;
    }
    if (generation != writer_session_generation_) {
        reset_session(generation, writer_.session_context());
    }

    if (phase_ != SessionPhase::Active) {
        advance_definition_phase();
        return;
    }

    if (!write_late_boot_time()) {
        note_dropout(now);
        return;
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
