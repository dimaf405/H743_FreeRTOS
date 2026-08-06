#pragma once

#include "input_rc.hpp"
#include "lifecycle/module_base.hpp"
#include "platform/api/Platform.hpp"
#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstdint>

namespace dima::modules::logging {

class LogService final : public dima::middleware::lifecycle::ModuleBase,
                         public px4::ScheduledWorkItem {
public:
    explicit LogService(dima::platform::Console &console) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

protected:
    void Run() override;

private:
    void enqueue_sbus_data(std::uint64_t now_us) noexcept;
    void reset_debug_state() noexcept;

    dima::platform::Console &console_;
    static constexpr uint32_t kFlushIntervalUs = 20000U;
    uORB::SubscriptionData<input_rc_s> input_rc_subscription_{
        ORB_ID(input_rc)};
    std::uint64_t last_sbus_sample_timestamp_us_{0U};
    std::uint64_t last_sbus_output_time_us_{0U};
    bool sbus_sample_pending_{false};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::modules::logging
