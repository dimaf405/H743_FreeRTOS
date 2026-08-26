#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"

#include <cmath>
#include <limits>

namespace dima::modules::mavlink {

void MavlinkService::update_rc_input() noexcept
{
    if (input_rc_subscription_.update()) {
        latest_input_rc_ = input_rc_subscription_.get();
        have_input_rc_ = true;
    }
}

bool MavlinkService::refresh_protocol_parameters() noexcept
{
    // RC 可见性超时与系统 ID 作为同一协议快照校验；本产品身份固定为生成合同中的 1。
    float timeout_s = 0.0F;
    std::int32_t system_id = 0;
    rc_loss_timeout_valid_ =
        rc_loss_timeout_handle_ != PARAM_INVALID &&
        param_get(rc_loss_timeout_handle_, &timeout_s) == 0 &&
        std::isfinite(timeout_s) && timeout_s >= 0.1F && timeout_s <= 35.0F &&
        mav_system_id_handle_ != PARAM_INVALID &&
        param_get(mav_system_id_handle_, &system_id) == 0 && system_id == 1;
    if (rc_loss_timeout_valid_) {
        rc_loss_timeout_s_ = timeout_s;
    }
    return rc_loss_timeout_valid_;
}

bool MavlinkService::rc_sample_streamable(std::uint64_t now) const noexcept
{
    /* Failsafe/lost 只禁止控制，不能隐藏仍然新鲜的原始通道；否则 QGC
     * 在诊断或校准时会把“信号不允许控制”误判为“接收机完全无数据”。 */
    if (!have_input_rc_ || latest_input_rc_.channel_count == 0U ||
        latest_input_rc_.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
        return false;
    }

    if (!rc_loss_timeout_valid_) {
        return false;
    }

    const std::uint64_t sample_time =
        latest_input_rc_.timestamp_last_signal != 0U
            ? latest_input_rc_.timestamp_last_signal
            : latest_input_rc_.timestamp;
    if (sample_time == 0U || sample_time > now) {
        return false;
    }

    // 新鲜度基于接收机最后真实信号时间，不使用 MAVLink 本轮发送时间。
    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        static_cast<double>(rc_loss_timeout_s_) * 1000000.0);
    return timeout_us > 0U && now - sample_time <= timeout_us;
}

bool MavlinkService::send_rc_channels(std::uint64_t now) noexcept
{
    const std::uint8_t channel_count = latest_input_rc_.channel_count;
    const auto value = [this, channel_count](std::size_t index) {
        if (index < channel_count) {
            return latest_input_rc_.values[index];
        }
        // RC_CHANNELS 规定 0xFFFF 表示该通道未使用/不可用。
        return std::numeric_limits<std::uint16_t>::max();
    };

    mavlink_rc_channels_t channels{};
    const std::uint64_t sample_time = latest_input_rc_.timestamp != 0U
        ? latest_input_rc_.timestamp : now;
    channels.time_boot_ms = static_cast<std::uint32_t>(sample_time / 1000U);
    channels.chan1_raw = value(0U);
    channels.chan2_raw = value(1U);
    channels.chan3_raw = value(2U);
    channels.chan4_raw = value(3U);
    channels.chan5_raw = value(4U);
    channels.chan6_raw = value(5U);
    channels.chan7_raw = value(6U);
    channels.chan8_raw = value(7U);
    channels.chan9_raw = value(8U);
    channels.chan10_raw = value(9U);
    channels.chan11_raw = value(10U);
    channels.chan12_raw = value(11U);
    channels.chan13_raw = value(12U);
    channels.chan14_raw = value(13U);
    channels.chan15_raw = value(14U);
    channels.chan16_raw = value(15U);
    channels.chan17_raw = value(16U);
    channels.chan18_raw = value(17U);
    channels.chancount = channel_count;
    // RSSI 超出 0..100 时发送 UINT8_MAX，区别于真实的 0% 信号强度。
    channels.rssi = latest_input_rc_.rssi >= 0 &&
                            latest_input_rc_.rssi <= input_rc_s::RSSI_MAX
        ? static_cast<std::uint8_t>(latest_input_rc_.rssi)
        : std::numeric_limits<std::uint8_t>::max();

    mavlink_message_t message{};
    mavlink_msg_rc_channels_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID,
                                   &message, &channels);
    return send_message(message);
}

} // namespace dima::modules::mavlink
