#include "platform_composition.h"

#include "boot_diagnostics.h"
#include "api/Boot.hpp"
#include "api/Flash.hpp"
#include "api/Services.hpp"
#include "freertos/Backend.hpp"
#include "stm32h7/HardwareServices.hpp"
#include "usb_console/UsbConsole.hpp"

namespace {

/* 将平台无关的启动阶段/失败类型映射到板级 NOLOAD 诊断 ABI。下面的静态断言
 * 固定两个枚举域的数值合同，防止重排枚举后历史诊断被错误解码。 */
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

    /* 该入口允许幂等调用；首次调用严格按“RTOS 基础设施 -> 时钟/Flash ->
     * 依赖对象 -> Services 发布”构造，任何一步失败都不会发布半成品服务表。 */
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

    /* Services 保存引用，故所有被引用后端都必须具有静态生命周期。组合根在此
     * 集中表达资源所有权，业务模块只能经角色接口取用串口、SPI、CAN 和 PWM。 */
    static Services services{
        stm32h7::clock(),
        freertos::execution_context(),
        critical,
        freertos::synchronization(),
        freertos::task_runtime(),
        freertos::heap(),
        freertos::flash_transactions(),
        stm32h7::parameter_partition(),
        freertos::atomic_file_store(),
        freertos::log_file_store(),
        armed_flash,
        console,
        boot_control,
        startup_diagnostics(),
        stm32h7::dma_memory(),
        stm32h7::independent_watchdog(),
        stm32h7::serial_ports(),
        stm32h7::async_serial_port(),
        stm32h7::timestamped_serial_input(),
        stm32h7::interrupt_sources(),
        stm32h7::spi4(),
        stm32h7::can1(),
        &stm32h7::actuator_pwm(),
    };

    if (!install_services(services)) {
        return false;
    }

    dima_boot_stage_set(DIMA_BOOT_STAGE_PLATFORM_READY);
    return true;
}

/* 硬件板版本只在 PCB 修订变化时递增，不随固件版本或功能开关变化。 */
static constexpr std::uint32_t kBoardVersion = 1;

namespace dima::platform {

uint64_t board_hardware_uid() noexcept
{
    return stm32h7::board_hardware_uid();
}

uint32_t board_version() noexcept
{
    return kBoardVersion;
}

}  // namespace dima::platform
