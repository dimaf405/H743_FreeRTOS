#pragma once

#include "App/messages/app_heartbeat.hpp"
#include "App/runtime/lifecycle/module_base.hpp"
#include "App/runtime/scheduling/scheduled_work_item.hpp"

#if defined(APP_HOST_TEST)
#include "App/runtime/messaging/topic.hpp"
#else
#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/uorb/Publication.hpp"
#endif

#include <stdint.h>

namespace app::features::hello_world {

#if defined(APP_HOST_TEST)
struct HostDependencies {
    void *context;
    int (*printf_line)(void *context, const char *line);
    int (*fflush_stdout)(void *context);
    uint64_t (*time_us)(void *context);
};
#endif

class HelloWorld final : public runtime::lifecycle::ModuleBase,
                         public runtime::scheduling::ScheduledWorkItem {
public:
#if defined(APP_HOST_TEST)
    HelloWorld(runtime::scheduling::WorkQueue &queue,
               runtime::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
               HostDependencies dependencies);
#else
    explicit HelloWorld(runtime::scheduling::WorkQueue &queue);
#endif
    ~HelloWorld() = default;

    bool start() override;
    void stop() override;
    runtime::lifecycle::ModuleState state() const override;

#if defined(APP_HOST_TEST)
    void RunForTest();
#endif

protected:
    void Run() override;

private:
#if defined(APP_HOST_TEST)
    runtime::messaging::Publication<app_heartbeat_s> heartbeat_publication_;
#else
    uORB::Publication<app_heartbeat_s> heartbeat_publication_;
#endif
    runtime::lifecycle::ModuleState state_{
        runtime::lifecycle::ModuleState::Stopped};
    uint32_t sequence_{0U};
#if defined(APP_HOST_TEST)
    HostDependencies dependencies_;
#endif
};

} // namespace app::features::hello_world
