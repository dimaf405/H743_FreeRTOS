/****************************************************************************
 * PX4-Autopilot v1.17.0 ManualControl RC subset adapted to Dima FreeRTOS.
 ****************************************************************************/
#pragma once

#include "Dima/messages/action_request.hpp"
#include "Dima/messages/manual_control_setpoint.hpp"
#include "Dima/messages/manual_control_switches.hpp"
#include "Dima/messages/rc_channels.hpp"
#include "Dima/middleware/lifecycle/module_base.hpp"
#include "Dima/middleware/uorb/Publication.hpp"
#include "Dima/middleware/uorb/SubscriptionData.hpp"
#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"

#include <cstdint>

namespace dima::modules::rc {

class ManualControl final : public dima::middleware::lifecycle::ModuleBase,
                            public px4::ScheduledWorkItem {
public:
    ManualControl() noexcept;
    ~ManualControl() override;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    void Run() override;
    void process_rc_channels(const rc_channels_s &channels) noexcept;
    void process_switches(const manual_control_switches_s &switches) noexcept;
    void publish_action(std::uint8_t action) noexcept;
    void reset_switch_baseline() noexcept;

    static bool mapped_channel(const rc_channels_s &channels,
                               std::uint8_t function,
                               float &value) noexcept;

    uORB::SubscriptionData<rc_channels_s> rc_channels_subscription_{ORB_ID(rc_channels)};
    uORB::SubscriptionData<manual_control_switches_s> switches_subscription_{
        ORB_ID(manual_control_switches)};
    uORB::Publication<manual_control_setpoint_s> setpoint_publication_{
        ORB_ID(manual_control_setpoint)};
    uORB::Publication<action_request_s> action_request_publication_{
        ORB_ID(action_request)};

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    manual_control_switches_s previous_switches_{};
    bool rc_signal_available_{false};
    bool switches_initialized_{false};
    bool lost_invalid_published_{false};
};

} // namespace dima::modules::rc
