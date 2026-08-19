#pragma once

#include <cstdint>

namespace dima::platform {

enum class BootConfirmResult : int {
    FlashError = -1,
    Ok = 0,
    AlreadyConfirmed = 1,
    NotTestImage = 2,
    Deferred = 3,
};

class BootControl {
public:
    virtual ~BootControl() = default;
    virtual BootConfirmResult confirm_running_image() noexcept = 0;
    /** Normal MCU reset (boots the confirmed application). */
    [[noreturn]] virtual void reboot() noexcept = 0;
    [[noreturn]] virtual void reboot_to_recovery() noexcept = 0;
};

enum class StartupStage : std::uint32_t {
    ApplicationTaskEnter = 0x0600U,
    UsbInit = 0x0610U,
    UsbReady = 0x061FU,
    WorkQueueInit = 0x0620U,
    UorbInit = 0x0630U,
    ParameterInit = 0x0640U,
    ModuleRegister = 0x0650U,
    ApplicationInitialized = 0x06FFU,
    BootHealthStart = 0x0700U,
    HelloStart = 0x0710U,
    ParameterStart = 0x0720U,
    LogStart = 0x0730U,
    MotorOutputStart = 0x0738U,
    CommanderStart = 0x0740U,
    MavlinkStart = 0x0745U,
    RcStart = 0x0750U,
    ApplicationRunning = 0x07FFU,
    ApplicationFailed = 0x0F00U,
};

enum class FailureKind : std::uint32_t {
    ErrorHandler = 6U,
};

class StartupDiagnostics {
public:
    virtual ~StartupDiagnostics() = default;
    virtual void set_stage(StartupStage stage) noexcept = 0;
    [[noreturn]] virtual void panic(FailureKind failure,
                                    std::uint32_t detail,
                                    std::uint32_t auxiliary) noexcept = 0;
};

class IndependentWatchdog {
public:
    virtual ~IndependentWatchdog() = default;
    virtual bool start(std::uint32_t timeout_ms) noexcept = 0;
    virtual void feed() noexcept = 0;
    virtual bool active() const noexcept = 0;
};

} // namespace dima::platform
