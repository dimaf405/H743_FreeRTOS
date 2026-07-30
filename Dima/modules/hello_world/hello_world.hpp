#pragma once

#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/lifecycle/module_base.hpp"

#if defined(APP_HOST_TEST)
#include "Dima/middleware/messaging/topic.hpp"
#include "Dima/middleware/scheduling/scheduled_work_item.hpp"
#else
#include "Dima/middleware/uorb/Publication.hpp"
#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"
#endif

#include <stdint.h>

namespace dima::modules::hello_world {

#if defined(APP_HOST_TEST)
struct HostDependencies {
    void *context;
    int (*printf_line)(void *context, const char *line);
    int (*fflush_stdout)(void *context);
    uint64_t (*time_us)(void *context);
};
#endif

class HelloWorld final : public dima::middleware::lifecycle::ModuleBase,
#if defined(APP_HOST_TEST)
                         public dima::middleware::scheduling::ScheduledWorkItem {
#else
                         public px4::ScheduledWorkItem {
#endif
public:
#if defined(APP_HOST_TEST)
    HelloWorld(dima::middleware::scheduling::WorkQueue &queue,
               dima::middleware::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
               HostDependencies dependencies);
#else
    HelloWorld() noexcept;
#endif
    ~HelloWorld() = default;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

#if defined(APP_HOST_TEST)
    void RunForTest();
#endif

protected:
    void Run() override;

private:
#if defined(APP_HOST_TEST)
    dima::middleware::messaging::Publication<app_heartbeat_s> heartbeat_publication_;
#else
    uORB::Publication<app_heartbeat_s> heartbeat_publication_;
#endif
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    uint32_t sequence_{0U};
#if defined(APP_HOST_TEST)
    HostDependencies dependencies_;
#endif
};

} // namespace dima::modules::hello_world
