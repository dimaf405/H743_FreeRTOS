#include "platform_composition.h"

#include "boot_diagnostics.h"
#include "platform/api/Platform.hpp"
#include "platform/freertos/Backend.hpp"
#include "platform/stm32h7/Backend.hpp"
#include "usb_console/UsbConsole.hpp"

namespace {

class BoardStartupDiagnostics final
    : public dima::platform::StartupDiagnostics {
public:
    void set_stage(dima::platform::StartupStage stage) noexcept override
    {
        dima_boot_stage_set(static_cast<std::uint32_t>(stage));
    }

    [[noreturn]] void panic(dima::platform::FailureKind failure,
                            std::uint32_t detail,
                            std::uint32_t auxiliary) noexcept override
    {
        dima_boot_diagnostics_panic(static_cast<std::uint32_t>(failure),
                                    detail, auxiliary);
    }
};

static_assert(static_cast<std::uint32_t>(
                  dima::platform::StartupStage::ApplicationTaskEnter) ==
              DIMA_BOOT_STAGE_APP_TASK_ENTER);
static_assert(static_cast<std::uint32_t>(
                  dima::platform::StartupStage::ApplicationRunning) ==
              DIMA_BOOT_STAGE_APPLICATION_RUNNING);
static_assert(static_cast<std::uint32_t>(
                  dima::platform::StartupStage::ApplicationFailed) ==
              DIMA_BOOT_STAGE_APPLICATION_FAILED);
static_assert(static_cast<std::uint32_t>(
                  dima::platform::StartupStage::MotorOutputStart) ==
              DIMA_BOOT_STAGE_MOTOR_OUTPUT_START);
static_assert(static_cast<std::uint32_t>(
                  dima::platform::FailureKind::ErrorHandler) ==
              DIMA_BOOT_FAILURE_ERROR_HANDLER);

BoardStartupDiagnostics &startup_diagnostics() noexcept
{
    static BoardStartupDiagnostics instance;
    return instance;
}

} // namespace

extern "C" bool dima_platform_early_init(void)
{
    using namespace dima::platform;

    if (services_installed()) {
        return true;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_HEAP_INIT);
    if (!freertos::initialize()) {
        return false;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_HRT_INIT);
    if (!stm32h7::clock_initialize() || !stm32h7::flash_initialize()) {
        return false;
    }

    auto &critical = freertos::critical_section();
    static ArmedFlashCoordinator armed_flash{critical};
    auto &console = dima::adapters::usb_console(
        freertos::synchronization(), freertos::task_runtime(),
        freertos::execution_context(), stm32h7::clock(),
        stm32h7::usb_console_transport());
    auto &boot_control = stm32h7::boot_control(
        freertos::flash_transactions(), armed_flash);

    static Services services{
        stm32h7::clock(),
        freertos::execution_context(),
        critical,
        freertos::synchronization(),
        freertos::task_runtime(),
        freertos::heap(),
        freertos::flash_transactions(),
        stm32h7::parameter_partition(),
        armed_flash,
        console,
        boot_control,
        startup_diagnostics(),
        stm32h7::dma_memory(),
        stm32h7::sbus_input(),
        stm32h7::sensor_interrupts(),
        &stm32h7::actuator_pwm(),
    };

    if (!install_services(services)) {
        return false;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_PLATFORM_READY);
    return true;
}
