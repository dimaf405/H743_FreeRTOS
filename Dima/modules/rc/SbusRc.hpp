/****************************************************************************
 * PX4-Autopilot v1.17.0 SbusRc receive flow adapted to Dima FreeRTOS.
 ****************************************************************************/
#pragma once

#include "rc/sbus.hpp"
#include "rc/sbus_backend.hpp"
#include "input_rc.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "perf/perf_counter.h"
#include "uorb/Publication.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::rc {

class SbusRc final : public dima::middleware::lifecycle::ModuleBase,
                     public px4::ScheduledWorkItem {
public:
    struct Stats {
        std::uint32_t start_failures{0U};
        std::uint32_t service_failures{0U};
        std::uint32_t publications{0U};
        std::uint32_t read_wakeups{0U};
    };

    explicit SbusRc(dima::rc::SbusBackend &backend) noexcept;
    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    const Stats &stats() const noexcept { return stats_; }
    const dima::rc::SbusParser::Stats &parser_stats() const noexcept { return parser_.stats(); }

private:
    static constexpr std::uint32_t kRetryDelayUs = 100000U;
    static constexpr std::size_t kReadBufferSize = 64U;
    void Run() override;
    void schedule_retry() noexcept;
    void allocate_perf_counters() noexcept;
    void free_perf_counters() noexcept;
    void publish(const dima::rc::SbusParser::Frame &frame,
                 std::uint64_t now_us) noexcept;

    dima::rc::SbusBackend &backend_;
    dima::rc::SbusParser parser_{};
    uORB::Publication<input_rc_s> input_rc_pub_{ORB_ID(input_rc)};
    px4::ParamInt<px4::params::RC_PORT_CONFIG> rc_port_{};
    px4::ParamInt<px4::params::RC_INPUT_PROTO> rc_protocol_{};
    px4::ParamBool<px4::params::DIMA_SBUS_INV> sbus_inverted_{};
    dima::middleware::lifecycle::ModuleState state_{dima::middleware::lifecycle::ModuleState::Stopped};
    std::uint64_t timestamp_last_signal_us_{0U};
    bool backend_started_{false};
    bool signal_locked_{false};
    bool signal_seen_{false};
    bool failsafe_active_{false};
    bool backend_fault_reported_{false};
    std::uint32_t last_invalid_frames_{0U};
    std::uint32_t last_uart_errors_{0U};
    perf_counter_t byte_count_{nullptr};
    perf_counter_t frame_count_{nullptr};
    perf_counter_t invalid_frame_count_{nullptr};
    perf_counter_t lost_frame_count_{nullptr};
    perf_counter_t uart_error_count_{nullptr};
    perf_counter_t publish_interval_{nullptr};
    Stats stats_{};
};

} // namespace dima::modules::rc
