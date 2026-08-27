/****************************************************************************
 * PX4-Autopilot v1.17.0 SbusRc receive flow adapted to the Dima platform.
 ****************************************************************************/
#pragma once

#include "SbusProtocol.hpp"
#include "input_rc.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "perf/perf_counter.h"
#include "api/Serial.hpp"
#include "serial/SerialPortAssignments.hpp"
#include "uORB/Publication.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::drivers::rc {

// SBUS 模块拥有“串口分配 -> TimestampedSerialInput -> SbusParser -> input_rc”
// 链路。它只发布原始 RC 输入；通道映射、手动控制和 COM_RC_LOSS_T 的上层
// 持续失联判定分别由 RCUpdate/RcManualInput 拥有。
class SbusRc final : public dima::middleware::lifecycle::ModuleBase,
                     public px4::ScheduledWorkItem {
public:
    struct Stats {
        std::uint32_t start_failures{0U};
        std::uint32_t service_failures{0U};
        std::uint32_t publications{0U};
        std::uint32_t read_wakeups{0U};
    };

    SbusRc(dima::platform::TimestampedSerialInput &backend,
           dima::lib::serial::SerialPortAssignments
               &serial_assignments) noexcept;
    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    const Stats &stats() const noexcept { return stats_; }
    const dima::protocols::sbus::SbusParser::Stats &parser_stats() const noexcept
    {
        return parser_.stats();
    }

private:
    // 硬故障 100 ms 重试；严格限定的 PE/NE/FE 瞬态线路错误 1 ms 重启 DMA。
    // 恢复后必须连续 3 个健康帧才重新锁定，前两帧只用于协议重新同步。
    static constexpr std::uint32_t kRetryDelayUs = 100000U;
    static constexpr std::uint32_t kLineErrorRetryDelayUs = 1000U;
    static constexpr std::uint8_t kRequiredLockFrames = 3U;
    static constexpr std::size_t kReadBufferSize = 64U;
    void Run() override;
    static void notify_from_isr(void *context) noexcept;
    void reset_runtime_state() noexcept;
    void schedule_retry(std::uint32_t delay_us = kRetryDelayUs) noexcept;
    bool schedule_signal_timeout() noexcept;
    void fail_scheduling(const char *reason) noexcept;
    bool publish_backend_loss(std::uint64_t now) noexcept;
    void invalidate_parameters() noexcept;
    void allocate_perf_counters() noexcept;
    void free_perf_counters() noexcept;
    void publish(const dima::protocols::sbus::SbusParser::Frame &frame,
                 std::uint64_t frame_arrival_us) noexcept;

    // backend_ 独占所分配 UART，并逐字节提供线速倒推时间戳；ISR 仅唤醒 io
    // WorkQueue，解析、错误分级和 uORB 发布全部在任务上下文完成。
    dima::platform::TimestampedSerialInput &backend_;
    dima::lib::serial::SerialPortAssignments &serial_assignments_;
    dima::protocols::sbus::SbusParser parser_{};
    uORB::Publication<input_rc_s> input_rc_pub_{ORB_ID(input_rc)};
    px4::ParamInt<px4::params::RC_INPUT_PROTO> rc_protocol_{};
    px4::ParamFloat<px4::params::COM_RC_LOSS_T> rc_loss_timeout_{};
    dima::middleware::lifecycle::ModuleState state_{dima::middleware::lifecycle::ModuleState::Stopped};
    // timestamp_last_signal_us_ 记录最近完整协议帧；signal_locked_ 只有连续三帧
    // 健康后为真。接收机显式 failsafe 不等待三帧，必须立即发布给安全链。
    std::uint64_t timestamp_last_signal_us_{0U};
    std::uint64_t signal_loss_timeout_us_{500000U};
    bool backend_started_{false};
    bool signal_locked_{false};
    bool signal_seen_{false};
    std::uint8_t consecutive_healthy_frames_{0U};
    bool failsafe_active_{false};
    bool backend_fault_reported_{false};
    bool backend_line_error_reported_{false};
    std::uint32_t last_invalid_frames_{0U};
    std::uint32_t last_backend_faults_{0U};
    perf_counter_t byte_count_{nullptr};
    perf_counter_t frame_count_{nullptr};
    perf_counter_t invalid_frame_count_{nullptr};
    perf_counter_t lost_frame_count_{nullptr};
    perf_counter_t uart_error_count_{nullptr};
    perf_counter_t publish_interval_{nullptr};
    Stats stats_{};
};

} // namespace dima::drivers::rc
