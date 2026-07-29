#include "App/features/hello_world/hello_world.hpp"

#include "App/runtime/time/platform_time.hpp"

#include <stdio.h>

#if !defined(APP_HELLO_WORLD_INTERVAL_MS)
#error "APP_HELLO_WORLD_INTERVAL_MS must be supplied by the build"
#elif APP_HELLO_WORLD_INTERVAL_MS == 0
#error "APP_HELLO_WORLD_INTERVAL_MS must be nonzero"
#elif APP_HELLO_WORLD_INTERVAL_MS > 0x7FFFFFFF
#error "APP_HELLO_WORLD_INTERVAL_MS must not exceed 0x7fffffff"
#endif

namespace app::features::hello_world {

#if defined(APP_HOST_TEST)
HelloWorld::HelloWorld(
    runtime::scheduling::WorkQueue &queue,
    runtime::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
    HostDependencies dependencies)
    : ScheduledWorkItem(queue), heartbeat_publication_(heartbeat_topic),
      dependencies_(dependencies)
{
}
#else
HelloWorld::HelloWorld(runtime::scheduling::WorkQueue &queue)
    : ScheduledWorkItem(queue), heartbeat_publication_(ORB_ID(app_heartbeat))
{
}
#endif

bool HelloWorld::start()
{
    if (state_ == runtime::lifecycle::ModuleState::Running) {
        return true;
    }

    if (!ScheduleOnInterval(APP_HELLO_WORLD_INTERVAL_MS)) {
        state_ = runtime::lifecycle::ModuleState::Error;
        return false;
    }

    state_ = runtime::lifecycle::ModuleState::Running;
    return true;
}

void HelloWorld::stop()
{
    ScheduleClear();
    state_ = runtime::lifecycle::ModuleState::Stopped;
}

runtime::lifecycle::ModuleState HelloWorld::state() const
{
    return state_;
}

void HelloWorld::Run()
{
#if defined(APP_HOST_TEST)
    (void)dependencies_.printf_line(dependencies_.context,
                                    "Hello World\r\n");
    (void)dependencies_.fflush_stdout(dependencies_.context);
    const uint64_t timestamp_us = dependencies_.time_us(dependencies_.context);
#else
    (void)printf("Hello World\r\n");
    (void)fflush(stdout);
    const uint64_t timestamp_us = runtime::time::platform_time_us();
#endif

    const app_heartbeat_s heartbeat{timestamp_us, ++sequence_};
    (void)heartbeat_publication_.publish(heartbeat);
}

#if defined(APP_HOST_TEST)
void HelloWorld::RunForTest()
{
    Run();
}
#endif

} // namespace app::features::hello_world
