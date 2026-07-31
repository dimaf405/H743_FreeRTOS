/****************************************************************************
 * PX4-Autopilot v1.17.0 SbusRc receive flow adapted to Dima FreeRTOS.
 ****************************************************************************/
#include "Dima/modules/rc/SbusRc.hpp"

#include "Dima/middleware/logging/logging.hpp"
#include "Dima/platform/freertos/hrt.hpp"

#include <cmath>

namespace dima::modules::rc {

SbusRc::SbusRc(dima::rc::SbusBackend &backend) noexcept
    : px4::ScheduledWorkItem("sbus_rc", px4::wq_configurations::io), backend_(backend)
{
}

bool SbusRc::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) return true;
    (void)rc_port_.update();
    (void)rc_protocol_.update();
    (void)sbus_inverted_.update();
    if (rc_protocol_.get() != 2 || rc_port_.get() <= 0 ||
        !backend_.configure(rc_port_.get(), sbus_inverted_.get())) {
        ++stats_.start_failures;
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("SBUS configuration invalid");
        return false;
    }
    parser_.reset();
    timestamp_last_signal_us_ = 0U;
    backend_started_ = false;
    signal_locked_ = false;
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("SBUS port=%ld inverted=%d", static_cast<long>(rc_port_.get()), sbus_inverted_.get() ? 1 : 0);
    if (!ScheduleNow()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    return true;
}

void SbusRc::stop()
{
    ScheduleClear();
    backend_.stop();
    backend_started_ = false;
    signal_locked_ = false;
    parser_.reset();
    timestamp_last_signal_us_ = 0U;
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
}

dima::middleware::lifecycle::ModuleState SbusRc::state() const { return state_; }

void SbusRc::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    if (!backend_started_) {
        if (!backend_.start(*this)) {
            ++stats_.start_failures;
            schedule_retry();
            return;
        }
        backend_started_ = true;
    }
    if (!backend_.service() || !backend_.running()) {
        ++stats_.service_failures;
        backend_.stop();
        backend_started_ = false;
        signal_locked_ = false;
        schedule_retry();
        return;
    }

    std::uint8_t buffer[kReadBufferSize]{};
    bool received = false;
    for (;;) {
        const std::size_t count = backend_.read(buffer, sizeof(buffer));
        if (count == 0U) break;
        received = true;
        const std::uint64_t now_us = hrt_absolute_time();
        dima::rc::SbusParser::Frame frame{};
        if (parser_.parse(now_us, buffer, count, frame)) {
            timestamp_last_signal_us_ = now_us;
            if (!signal_locked_) {
                PX4_INFO("SBUS signal locked channels=%u", frame.channel_count);
                signal_locked_ = true;
            }
            publish(frame, now_us);
        }
    }
    if (received) ++stats_.read_wakeups;
}

void SbusRc::schedule_retry() noexcept
{
    if (!ScheduleDelayed(kRetryDelayUs)) state_ = dima::middleware::lifecycle::ModuleState::Error;
}

void SbusRc::publish(const dima::rc::SbusParser::Frame &frame,
                     std::uint64_t now_us) noexcept
{
    input_rc_s message{};
    message.timestamp = now_us;
    message.timestamp_last_signal = timestamp_last_signal_us_;
    message.channel_count = frame.channel_count;
    message.rssi = -1;
    message.rc_failsafe = frame.failsafe;
    message.rc_lost = frame.frame_lost;
    message.rc_lost_frame_count = static_cast<std::uint16_t>(parser_.stats().frame_lost_flags);
    message.rc_total_frame_count = static_cast<std::uint16_t>(parser_.stats().valid_frames);
    message.rc_ppm_frame_length = 0U;
    message.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
    message.link_quality = -1;
    message.rssi_dbm = NAN;
    for (std::size_t i = 0U; i < frame.channel_count; ++i) message.values[i] = frame.values[i];
    if (frame.failsafe) PX4_WARN("SBUS receiver failsafe");
    (void)input_rc_pub_.publish(message);
    ++stats_.publications;
}

} // namespace dima::modules::rc
