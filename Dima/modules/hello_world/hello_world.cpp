#include "hello_world.hpp"

#include "platform/api/Platform.hpp"

#include <stdio.h>

#if !defined(APP_HELLO_WORLD_INTERVAL_MS)
#error "APP_HELLO_WORLD_INTERVAL_MS must be supplied by the build"
#elif APP_HELLO_WORLD_INTERVAL_MS == 0
#error "APP_HELLO_WORLD_INTERVAL_MS must be nonzero"
#elif APP_HELLO_WORLD_INTERVAL_MS > 4294967
#error "APP_HELLO_WORLD_INTERVAL_MS must not exceed 4294967"
#endif

namespace dima::modules::hello_world {

HelloWorld::HelloWorld() noexcept
    : px4::ScheduledWorkItem("hello_world", px4::wq_configurations::lp_default),
      heartbeat_publication_(ORB_ID(app_heartbeat))
{
}

bool HelloWorld::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    sequence_ = 0U;

    const bool scheduled = ScheduleOnInterval(
        static_cast<uint32_t>(APP_HELLO_WORLD_INTERVAL_MS) * 1000U);
    if (!scheduled) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void HelloWorld::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    sequence_ = 0U;
}

dima::middleware::lifecycle::ModuleState HelloWorld::state() const
{
    return state_;
}

void HelloWorld::Run()
{
    (void)printf("Hello World\r\n");
    (void)fflush(stdout);
    const uint64_t timestamp_us = dima::platform::platform_time_us();

    const app_heartbeat_s heartbeat{timestamp_us, ++sequence_};
    (void)heartbeat_publication_.publish(heartbeat);
}

} // namespace dima::modules::hello_world
