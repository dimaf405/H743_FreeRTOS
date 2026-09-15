#include "MavlinkService.hpp"

#include "actuator_output_status.hpp"
#include "actuator_motors.hpp"
#include "manual_control_setpoint.hpp"
#include "rc_channels.hpp"
#include "rover_control_status.hpp"
#include "rover_motion_request.hpp"
#include "api/Time.hpp"
#include "logging/logging.hpp"

#include <limits>
#include <cmath>

namespace dima::modules::mavlink {
namespace {

std::uint32_t age_ms(std::uint64_t sample, std::uint64_t now) noexcept
{
    if (sample == 0U || sample > now) return UINT32_MAX;
    const auto elapsed = (now - sample) / 1000ULL;
    return elapsed > UINT32_MAX ? UINT32_MAX : static_cast<std::uint32_t>(elapsed);
}

const char *stick_sector(float throttle, float steering) noexcept
{
    // 只描述归一化杆位，不代表测得车体运动，也不参与混控或方向门控。
    if (!std::isfinite(throttle) || !std::isfinite(steering)) return "NA";
    const bool forward = throttle > 1.0e-6F, backward = throttle < -1.0e-6F;
    const bool right = steering > 1.0e-6F, left = steering < -1.0e-6F;
    if (forward) return right ? "FR" : (left ? "FL" : "F");
    if (backward) return right ? "BR" : (left ? "BL" : "B");
    return right ? "R" : (left ? "L" : "N");
}

} // namespace

void MavlinkService::report_drive_diagnostics() noexcept
{
    const auto before_copy = hrt_absolute_time();
    if (last_drive_diagnostic_us_ != 0U && before_copy >= last_drive_diagnostic_us_ &&
        before_copy - last_drive_diagnostic_us_ < 1000000ULL) return;
    last_drive_diagnostic_us_ = before_copy;

    // 只在低优先级通信队列降采样最新已有 Topic，不向实时混控/PWM 队列添加
    // 格式化工作，也不注册控制回调或更改输出。局部 generation=0 只用于快照读取。
    vehicle_status_s status{};
    std::uint64_t generation = 0U;
    if (!uORB::orb_copy_latest(ORB_ID(vehicle_status), 0U, generation, &status) ||
        status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
        status.nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL) return;
    manual_control_setpoint_s input{};
    rover_control_status_s control{};
    actuator_output_status_s output{};
    actuator_motors_s motors{};
    input_rc_s raw{};
    rc_channels_s channels{};
    generation = 0U;
    const bool have_input = uORB::orb_copy_latest(ORB_ID(manual_control_setpoint), 0U, generation, &input);
    generation = 0U;
    const bool have_control = uORB::orb_copy_latest(ORB_ID(rover_control_status), 0U, generation, &control) &&
        control.source == rover_motion_request_s::SOURCE_MANUAL;
    generation = 0U;
    const bool have_output = uORB::orb_copy_latest(ORB_ID(actuator_output_status), 0U, generation, &output);
    generation = 0U;
    const bool have_motors = uORB::orb_copy_latest(ORB_ID(actuator_motors), 0U, generation, &motors);
    generation = 0U;
    const bool have_raw = uORB::orb_copy_latest(ORB_ID(input_rc), 0U, generation, &raw);
    generation = 0U;
    const bool have_channels = uORB::orb_copy_latest(ORB_ID(rc_channels), 0U, generation, &channels);
    const auto now = hrt_absolute_time();
    if (age_ms(status.timestamp, now) > 250U) return;
    const auto input_age = age_ms(input.timestamp_sample, now);
    const auto control_age = age_ms(control.timestamp, now);
    const auto output_age = age_ms(output.timestamp, now);
    // 控制状态 valid 还要求估计器测量；Manual 无定位也能运行，不能因此隐藏
    // 实际电机命令。用同拍的有限命令证明输出路径，估计器有效性单独打印。
    const bool motor_valid = have_motors && std::isfinite(motors.control[0]) && std::isfinite(motors.control[1]);
    const bool drive_valid = have_control && motor_valid && std::isfinite(control.longitudinal) &&
        std::isfinite(control.steering) && control.timestamp == motors.timestamp;
    // 跨队列快照可能属于不同 RC 样本；明确标记，不把同次打印伪装成同拍链路。
    // rover_control_status.timestamp_sample 是估计器样本，不能冒充 RC 样本。
    // RC 因果链取 actuator_motors.timestamp_sample；控制诊断按发布时刻配对。
    const bool same_sample = have_input && drive_valid && have_output && input.valid &&
        input.timestamp_sample != 0U && input.timestamp_sample == motors.timestamp_sample &&
        motors.timestamp_sample == output.timestamp_sample && control.timestamp == motors.timestamp &&
        input_age <= 100U && control_age <= 100U && output_age <= 100U;
    const float unknown = std::numeric_limits<float>::quiet_NaN();
    const float right = motor_valid ? motors.control[0] : unknown;
    const float left = motor_valid ? motors.control[1] : unknown;
    if (++drive_diagnostic_sequence_ == 0U) ++drive_diagnostic_sequence_;
    const auto sequence = static_cast<unsigned long>(drive_diagnostic_sequence_);
    const float throttle = have_input && input.valid ? input.throttle : unknown;
    const float yaw = have_input && input.valid ? input.yaw : unknown;
    const float requested_throttle = drive_valid ? control.longitudinal : unknown;
    const float requested_yaw = drive_valid ? control.steering : unknown;
    const int throttle_channel = have_channels ? channels.function[rc_channels_s::FUNCTION_THROTTLE] : -1;
    const int yaw_channel = have_channels ? channels.function[rc_channels_s::FUNCTION_YAW] : -1;
    const auto valid_channel = [&](int channel) {
        return have_raw && channel >= 0 && channel < raw.channel_count &&
            channel < channels.channel_count && channel < input_rc_s::RC_INPUT_MAX_CHANNELS;
    };
    const bool throttle_channel_valid = valid_channel(throttle_channel);
    const bool yaw_channel_valid = valid_channel(yaw_channel);
    const auto raw_sample = raw.timestamp_last_signal != 0U ? raw.timestamp_last_signal : raw.timestamp;
    // 原始值和规范化输入可能跨样本；匹配失败仍显示原始值，但 raw_match=0。
    const bool raw_match = have_input && input.valid && throttle_channel_valid && yaw_channel_valid &&
        !raw.rc_lost && !raw.rc_failsafe && !channels.signal_lost && input_age <= 100U &&
        raw_sample == input.timestamp_sample && channels.timestamp_last_valid == input.timestamp_sample;

    // mavlink_log.text 仅 127 B（含 NUL），不是格式化器的 256 B。分成四条
    // 有同一 n 的短记录，保留完整 PWM/限制标志；不扩大共享消息容量或实时负担。
    PX4_INFO_RAW("[drive in] n=%lu T/Y=%.6f/%.6f req=%.6f/%.6f v=%u/%u dir=%s",
        sequence, static_cast<double>(throttle), static_cast<double>(yaw),
        static_cast<double>(requested_throttle), static_cast<double>(requested_yaw),
        static_cast<unsigned>(have_input && input.valid), static_cast<unsigned>(drive_valid),
        stick_sector(requested_throttle, requested_yaw));
    // ack/PWM 为后端确认接受的命令，仍不表示引脚波形或轮速实测。
    PX4_INFO_RAW("[drive out] n=%lu cmdR/L=%.3f/%.3f ackR/L=%.3f/%.3f mapR/L=%02x/%02x",
        sequence, static_cast<double>(right), static_cast<double>(left),
        static_cast<double>(have_output ? output.applied_right : unknown),
        static_cast<double>(have_output ? output.applied_left : unknown),
        static_cast<unsigned>(output.right_output_mask), static_cast<unsigned>(output.left_output_mask));
    PX4_INFO_RAW("[drive pwm] n=%lu us=%u,%u,%u,%u,%u,%u st=%u mix/slew/hold/ramp=%u/%u/%u/%u cfg=%lu",
        sequence, static_cast<unsigned>(output.pwm_us[0]), static_cast<unsigned>(output.pwm_us[1]),
        static_cast<unsigned>(output.pwm_us[2]), static_cast<unsigned>(output.pwm_us[3]),
        static_cast<unsigned>(output.pwm_us[4]), static_cast<unsigned>(output.pwm_us[5]),
        static_cast<unsigned>(output.state), static_cast<unsigned>(control.mixing_limited),
        static_cast<unsigned>(control.motor_slew_active), static_cast<unsigned>(control.reversal_held),
        static_cast<unsigned>(control.arm_ramp_active), static_cast<unsigned long>(control.parameter_update_instance));
    PX4_INFO_RAW("[drive src] n=%lu RC%u/%u=%u/%u raw_match=%u ages=%lu/%lu/%lu same=%u est=%u",
        sequence, throttle_channel_valid ? static_cast<unsigned>(throttle_channel + 1) : 0U,
        yaw_channel_valid ? static_cast<unsigned>(yaw_channel + 1) : 0U,
        throttle_channel_valid ? static_cast<unsigned>(raw.values[throttle_channel]) : 0U,
        yaw_channel_valid ? static_cast<unsigned>(raw.values[yaw_channel]) : 0U,
        static_cast<unsigned>(raw_match), static_cast<unsigned long>(input_age),
        static_cast<unsigned long>(control_age), static_cast<unsigned long>(output_age),
        static_cast<unsigned>(same_sample), static_cast<unsigned>(have_control && control.valid));
}

} // namespace dima::modules::mavlink
