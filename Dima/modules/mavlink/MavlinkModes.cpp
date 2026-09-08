// SPDX-License-Identifier: BSD-3-Clause
// PX4 v1.17 Standard Modes 的固定目录适配；wire 字段来自官方 common.xml。
#include "MavlinkService.hpp"
#include "api/Time.hpp"

#include <cmath>
#include <cstring>

namespace dima::modules::mavlink {
namespace modes = dima::generated::mavlink_streams;

std::uint8_t MavlinkService::request_available_modes(float index) noexcept
{
    // common.xml：0 请求全部，1..N 请求目录中的一项。先检查有限性、整数性
    // 与范围，再收窄，不能让负数/小数经 round 或 uint8 转换变成有效索引。
    if (!std::isfinite(index) || index < 0.0F ||
        index > static_cast<float>(modes::kModeCount) || std::trunc(index) != index) {
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
    }
    if (available_modes_next_ != 0U) {
        // 单个有界回复事务未排空时不能覆盖其剩余目录；发送端可按标准 ACK 重试。
        return vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
    }
    const auto requested = static_cast<std::uint8_t>(index);
    available_modes_next_ = requested == 0U ? 1U : requested;
    available_modes_end_ = requested == 0U
        ? static_cast<std::uint8_t>(modes::kModeCount) : requested;
    return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
}

void MavlinkService::stream_available_modes() noexcept
{
    if (available_modes_next_ == 0U) return;
    const auto &mode = modes::kModes[available_modes_next_ - 1U];
    mavlink_available_modes_t available{};
    available.number_modes = static_cast<std::uint8_t>(modes::kModeCount);
    available.mode_index = available_modes_next_;
    available.standard_mode = mode.standard_mode;
    available.custom_mode = mode.custom_mode;
    available.properties = mode.properties;
    if (mode.standard_mode == MAV_STANDARD_MODE_NON_STANDARD) {
        // 名称含终止符的长度由生成合同按 mavgen 字段容量静态核对；标准模式
        // 留空名称，由 QGC 使用官方语义。外部校准名称不冒充 Mission 或 Hold。
        std::strncpy(available.mode_name, mode.name, sizeof(available.mode_name) - 1U);
    }
    mavlink_message_t message{};
    mavlink_msg_available_modes_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                      &message, &available);
    // 每轮只发一项，保留 ACK/HEARTBEAT 优先级；失败保留索引，断链统一清除。
    if (send_message(message)) {
        available_modes_next_ = available_modes_next_ == available_modes_end_
            ? 0U : static_cast<std::uint8_t>(available_modes_next_ + 1U);
    }
}

bool MavlinkService::current_mode_snapshot(mavlink_current_mode_t &mode) noexcept
{
    (void)vehicle_status_subscription_.update();
    const auto &status = vehicle_status_subscription_.get();
    // 高优先级 Commander 可以在上层记录调度时间后发布；复制后重新取时，
    // 避免把合法的新状态当成来自未来，反复推迟模式变化通知。
    const std::uint64_t now = hrt_absolute_time();
    if (status.timestamp == 0U || status.timestamp > now ||
        now - status.timestamp > 1000000ULL) return false;

    // 从同一 Commander 快照分别映射“实际模式”和“用户意图”。安全 Hold 会保留
    // Mission 意图，不能把 intended_custom_mode 固定为零或直接复制实际模式。
    const auto *current = modes::find_mode(status.nav_state);
    mode = {};
    mode.standard_mode = current != nullptr ? current->standard_mode
        : static_cast<std::uint8_t>(MAV_STANDARD_MODE_NON_STANDARD);
    mode.custom_mode = current != nullptr ? current->custom_mode : 0U;
    mode.intended_custom_mode = modes::custom_mode_for_nav_state(status.nav_state_user_intention);
    return true;
}

bool MavlinkService::current_mode_changed() noexcept
{
    mavlink_current_mode_t current{};
    return current_mode_snapshot(current) &&
        (!have_current_mode_tx_ || current.custom_mode != last_current_mode_.custom_mode ||
         current.intended_custom_mode != last_current_mode_.intended_custom_mode ||
         current.standard_mode != last_current_mode_.standard_mode);
}

bool MavlinkService::send_current_mode() noexcept
{
    mavlink_current_mode_t current{};
    if (!current_mode_snapshot(current)) return false;
    mavlink_message_t message{};
    mavlink_msg_current_mode_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   &message, &current);
    if (!send_message(message)) return false;
    last_current_mode_ = current;
    have_current_mode_tx_ = true;
    return true;
}

} // namespace dima::modules::mavlink
