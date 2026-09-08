#pragma once

#include "actuator_armed.hpp"
#include "input_rc.hpp"
#include "mavlink_log.hpp"
#include "lifecycle/module_base.hpp"
#include "logging/logging.hpp"
#include "uORB/Publication.hpp"
#include "uORB/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"
#include "SdLogWriter.hpp"

#include <cstdint>

namespace dima::modules::logging {

/**
 * LogService — structured log producer.
 *
 * Responsibilities (USB transport is owned by MavlinkService):
 *   - Writes Event/SBUS debug records into the structured log path.
 *   - Owns the structured sink of the logging middleware and
 *     publishes every formatted record as a mavlink_log uORB message,
 *     which MavlinkService converts to MAVLink STATUSTEXT.
 */
class LogService final : public dima::middleware::lifecycle::ModuleBase,
                         public px4::ScheduledWorkItem {
public:
    explicit LogService(dima::platform::LogFileStore &log_files) noexcept;

    bool initialize() noexcept;
    void shutdown() noexcept;
    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

protected:
    void Run() override;

private:
    static bool structured_sink(void *context, dima::logging::Level level,
                                const char *text, std::size_t length) noexcept;

    void enqueue_sbus_data(std::uint64_t now_us) noexcept;
    void load_logger_configuration(
        SdLogWriter::Configuration &configuration) noexcept;
    std::int32_t load_integer_parameter(
        const generated::IntegerParameterContract &contract) noexcept;
    void apply_initial_mode() noexcept;
    void process_armed_state(bool armed) noexcept;
    void set_sd_recording_intent(bool enabled) noexcept;
    void reset_debug_state() noexcept;

    static constexpr uint32_t kFlushIntervalUs = 20000U;
    static uORB::Publication<mavlink_log_s> mavlink_log_publication_;
    uORB::SubscriptionData<input_rc_s> input_rc_subscription_{
        ORB_ID(input_rc)};
    uORB::SubscriptionCallbackWorkItem actuator_armed_subscription_{
        ORB_ID(actuator_armed), *this};
    SdLogWriter sd_writer_;
    std::uint64_t last_sbus_sample_timestamp_us_{0U};
    std::uint64_t last_sbus_output_time_us_{0U};
    bool sbus_sample_pending_{false};
    generated::LogMode log_mode_{generated::LogMode::ArmedSessions};
    bool armed_state_{false};
    bool sd_recording_intent_{false};
    bool sd_writer_available_{false};
    bool armed_callback_registered_{false};
    bool initialized_{false};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::modules::logging
