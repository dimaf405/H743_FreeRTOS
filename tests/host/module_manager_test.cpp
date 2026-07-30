#include "test_framework.hpp"

#include "Dima/middleware/lifecycle/module_base.hpp"
#include "Dima/middleware/lifecycle/module_manager.hpp"

#include <array>
#include <cstdint>

namespace {

using dima::middleware::lifecycle::ModuleBase;
using dima::middleware::lifecycle::ModuleManager;
using dima::middleware::lifecycle::ModuleState;

class FakeModule final : public ModuleBase {
public:
    bool start_result{true};
    unsigned start_calls{0};
    unsigned stop_calls{0};
    ModuleState current_state{ModuleState::Stopped};

    bool start() override
    {
        ++start_calls;
        if (start_result) {
            current_state = ModuleState::Running;
        }
        return start_result;
    }
    void stop() override
    {
        ++stop_calls;
        current_state = ModuleState::Stopped;
    }
    ModuleState state() const override { return current_state; }
};

} // namespace

HOST_TEST(module_manager_rejects_duplicate_registration_and_controls_module_lifecycle)
{
    ModuleManager manager;
    FakeModule module;
    CHECK(manager.register_module(module));
    CHECK(!manager.register_module(module));
    CHECK_EQ(manager.status(module), ModuleState::Stopped);

    CHECK(manager.start(module));
    CHECK_EQ(module.start_calls, 1U);
    CHECK_EQ(manager.status(module), ModuleState::Running);

    CHECK(manager.stop(module));
    CHECK_EQ(module.stop_calls, 1U);
    CHECK_EQ(manager.status(module), ModuleState::Stopped);
}

HOST_TEST(module_manager_accepts_exactly_sixteen_modules_without_touching_the_seventeenth)
{
    struct GuardedManager {
        std::uint32_t before{0xA5A5A5A5U};
        ModuleManager manager;
        std::uint32_t after{0x5A5A5A5AU};
    } guarded_manager;
    ModuleManager &manager = guarded_manager.manager;
    std::array<FakeModule, 17> modules{};
    for (unsigned index = 0; index < ModuleManager::kMaxModules; ++index) {
        CHECK(manager.register_module(modules[index]));
    }

    CHECK_EQ(ModuleManager::kMaxModules, 16U);
    CHECK(!manager.register_module(modules[16]));
    CHECK_EQ(modules[16].start_calls, 0U);
    CHECK(!manager.start(modules[16]));
    CHECK_EQ(modules[16].start_calls, 0U);

    for (unsigned index = 0; index < ModuleManager::kMaxModules; ++index) {
        CHECK(manager.start(modules[index]));
        CHECK_EQ(manager.status(modules[index]), ModuleState::Running);
    }
    CHECK_EQ(guarded_manager.before, 0xA5A5A5A5U);
    CHECK_EQ(guarded_manager.after, 0x5A5A5A5AU);
}

HOST_TEST(module_manager_rejects_unregistered_stop_without_calling_the_module)
{
    ModuleManager manager;
    FakeModule module;

    CHECK(!manager.stop(module));
    CHECK_EQ(module.stop_calls, 0U);
}

HOST_TEST(module_manager_propagates_a_registered_modules_failed_start)
{
    ModuleManager manager;
    FakeModule module;
    module.start_result = false;

    CHECK(manager.register_module(module));
    CHECK(!manager.start(module));
    CHECK_EQ(module.start_calls, 1U);
    CHECK_EQ(manager.status(module), ModuleState::Stopped);
}
