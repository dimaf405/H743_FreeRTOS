#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

using TimeUs = std::uint64_t;
using TimeMs = std::uint64_t;

struct Timeout {
    std::uint64_t microseconds{0U};
    bool infinite{false};

    static constexpr Timeout no_wait() noexcept { return {}; }
    static constexpr Timeout forever() noexcept { return {0U, true}; }
    static constexpr Timeout from_us(std::uint64_t value) noexcept
    {
        return {value, false};
    }
    static constexpr Timeout from_ms(std::uint64_t value) noexcept
    {
        return value > UINT64_MAX / 1000U
                   ? Timeout{UINT64_MAX, false}
                   : Timeout{value * 1000U, false};
    }
};

struct OpaqueHandle {
    std::uintptr_t value{0U};

    constexpr explicit operator bool() const noexcept { return value != 0U; }
};

constexpr bool operator==(OpaqueHandle left, OpaqueHandle right) noexcept
{
    return left.value == right.value;
}

constexpr bool operator!=(OpaqueHandle left, OpaqueHandle right) noexcept
{
    return !(left == right);
}

using MutexHandle = OpaqueHandle;
using SignalHandle = OpaqueHandle;
using TaskHandle = OpaqueHandle;

struct CriticalToken {
    std::uintptr_t state{0U};
    bool from_interrupt{false};
};

class MonotonicClock {
public:
    virtual ~MonotonicClock() = default;
    virtual bool initialized() const noexcept = 0;
    virtual TimeUs now_us() const noexcept = 0;
    TimeMs now_ms() const noexcept { return now_us() / 1000U; }
};

class ExecutionContext {
public:
    virtual ~ExecutionContext() = default;
    virtual bool in_interrupt() const noexcept = 0;
    virtual bool scheduler_running() const noexcept = 0;
    virtual bool in_realtime_task() const noexcept = 0;
};

class CriticalSection {
public:
    virtual ~CriticalSection() = default;
    virtual CriticalToken enter() noexcept = 0;
    virtual void leave(CriticalToken token) noexcept = 0;
};

class CriticalGuard final {
public:
    CriticalGuard() noexcept;
    explicit CriticalGuard(CriticalSection &section) noexcept;
    ~CriticalGuard();

    CriticalGuard(const CriticalGuard &) = delete;
    CriticalGuard &operator=(const CriticalGuard &) = delete;

private:
    CriticalSection *section_{nullptr};
    CriticalToken token_{};
};

enum class MutexKind : std::uint8_t {
    Normal,
    Recursive,
};

class Synchronization {
public:
    virtual ~Synchronization() = default;
    virtual MutexHandle create_mutex(MutexKind kind) noexcept = 0;
    virtual void destroy_mutex(MutexHandle handle) noexcept = 0;
    virtual bool lock(MutexHandle handle, Timeout timeout) noexcept = 0;
    virtual void unlock(MutexHandle handle) noexcept = 0;

    virtual SignalHandle create_signal() noexcept = 0;
    virtual void destroy_signal(SignalHandle handle) noexcept = 0;
    virtual bool wait(SignalHandle handle, Timeout timeout) noexcept = 0;
    virtual void notify(SignalHandle handle) noexcept = 0;
    virtual void notify_from_isr(SignalHandle handle) noexcept = 0;
};

class Mutex final {
public:
    Mutex() noexcept = default;
    ~Mutex();

    bool initialize(Synchronization &synchronization,
                    MutexKind kind = MutexKind::Normal) noexcept;
    void reset() noexcept;
    bool lock(Timeout timeout = Timeout::forever()) noexcept;
    void unlock() noexcept;
    bool valid() const noexcept { return static_cast<bool>(handle_); }

    Mutex(const Mutex &) = delete;
    Mutex &operator=(const Mutex &) = delete;

private:
    Synchronization *synchronization_{nullptr};
    MutexHandle handle_{};
};

class RecursiveMutex final {
public:
    RecursiveMutex() noexcept = default;
    ~RecursiveMutex();

    bool initialize(Synchronization &synchronization) noexcept;
    void reset() noexcept;
    bool lock(Timeout timeout = Timeout::forever()) noexcept;
    void unlock() noexcept;
    bool valid() const noexcept { return mutex_.valid(); }

    RecursiveMutex(const RecursiveMutex &) = delete;
    RecursiveMutex &operator=(const RecursiveMutex &) = delete;

private:
    Mutex mutex_{};
};

class MutexGuard final {
public:
    explicit MutexGuard(Mutex &mutex,
                        Timeout timeout = Timeout::forever()) noexcept;
    explicit MutexGuard(RecursiveMutex &mutex,
                        Timeout timeout = Timeout::forever()) noexcept;
    ~MutexGuard();

    explicit operator bool() const noexcept { return locked_; }
    MutexGuard(const MutexGuard &) = delete;
    MutexGuard &operator=(const MutexGuard &) = delete;

private:
    Mutex *mutex_{nullptr};
    RecursiveMutex *recursive_{nullptr};
    bool locked_{false};
};

class Signal final {
public:
    Signal() noexcept = default;
    ~Signal();

    bool initialize(Synchronization &synchronization) noexcept;
    void reset() noexcept;
    bool wait(Timeout timeout = Timeout::forever()) noexcept;
    void notify() noexcept;
    void notify_from_isr() noexcept;
    bool valid() const noexcept { return static_cast<bool>(handle_); }

    Signal(const Signal &) = delete;
    Signal &operator=(const Signal &) = delete;

private:
    Synchronization *synchronization_{nullptr};
    SignalHandle handle_{};
};

using TaskEntry = void (*)(void *argument);

struct TaskConfig {
    const char *name{nullptr};
    std::uint8_t priority{0U};
    std::uint32_t stack_bytes{0U};
    bool realtime{false};
};

class TaskRuntime {
public:
    virtual ~TaskRuntime() = default;
    virtual TaskHandle create(const TaskConfig &config, TaskEntry entry,
                              void *argument) noexcept = 0;
    virtual bool destroy(TaskHandle handle) noexcept = 0;
    virtual TaskHandle current() const noexcept = 0;
    virtual void suspend_current() noexcept = 0;
    virtual void delay(Timeout duration) noexcept = 0;
};

enum class AllocationDomain : std::uint8_t {
    Startup,
    Service,
    RealtimeForbidden,
};

struct HeapStats {
    std::size_t total_bytes{0U};
    std::size_t free_bytes{0U};
    std::size_t minimum_ever_free_bytes{0U};
    std::size_t largest_free_block{0U};
    std::uint32_t allocation_failures{0U};
};

class Heap {
public:
    virtual ~Heap() = default;
    virtual bool initialize() noexcept = 0;
    virtual void *allocate(std::size_t size,
                           AllocationDomain domain) noexcept = 0;
    virtual void deallocate(void *pointer) noexcept = 0;
    virtual HeapStats stats() const noexcept = 0;
    virtual std::size_t alignment() const noexcept = 0;
    virtual void record_failure() noexcept = 0;
};

class FlashTransactionManager {
public:
    virtual ~FlashTransactionManager() = default;
    virtual bool acquire(Timeout timeout) noexcept = 0;
    virtual void release() noexcept = 0;
};

class FlashTransaction final {
public:
    FlashTransaction(FlashTransactionManager &manager,
                     Timeout timeout) noexcept;
    ~FlashTransaction();

    explicit operator bool() const noexcept { return acquired_; }
    FlashTransaction(const FlashTransaction &) = delete;
    FlashTransaction &operator=(const FlashTransaction &) = delete;

private:
    FlashTransactionManager *manager_{nullptr};
    bool acquired_{false};
};

class FlashPartition {
public:
    virtual ~FlashPartition() = default;
    virtual std::uintptr_t base() const noexcept = 0;
    virtual std::size_t size() const noexcept = 0;
    virtual std::size_t program_size() const noexcept = 0;
    virtual bool read(std::size_t offset, void *destination,
                      std::size_t length) noexcept = 0;
    virtual bool program(std::size_t offset, const void *source,
                         std::size_t length) noexcept = 0;
    virtual bool erase() noexcept = 0;
};

class ArmedFlashCoordinator final {
public:
    explicit ArmedFlashCoordinator(CriticalSection &critical) noexcept;

    bool try_arm() noexcept;
    void disarm() noexcept;
    bool begin_flash() noexcept;
    void end_flash() noexcept;
    bool armed() const noexcept;
    bool flash_busy() const noexcept;

    ArmedFlashCoordinator(const ArmedFlashCoordinator &) = delete;
    ArmedFlashCoordinator &operator=(const ArmedFlashCoordinator &) = delete;

private:
    enum class State : std::uint8_t { Idle, Armed, FlashBusy };
    CriticalSection &critical_;
    State state_{State::Idle};
};

class FlashWriteLease final {
public:
    explicit FlashWriteLease(ArmedFlashCoordinator &coordinator) noexcept;
    ~FlashWriteLease();

    explicit operator bool() const noexcept { return acquired_; }
    FlashWriteLease(const FlashWriteLease &) = delete;
    FlashWriteLease &operator=(const FlashWriteLease &) = delete;

private:
    ArmedFlashCoordinator *coordinator_{nullptr};
    bool acquired_{false};
};

enum class ConsoleTransmitResult : std::uint8_t {
    Accepted,
    Busy,
    Failed,
};

class ConsoleTransport {
public:
    virtual ~ConsoleTransport() = default;
    virtual bool initialize() noexcept = 0;
    virtual void service() noexcept = 0;
    virtual bool ready() const noexcept = 0;
    virtual ConsoleTransmitResult transmit(const std::uint8_t *data,
                                             std::size_t length) noexcept = 0;
};

class Console {
public:
    virtual ~Console() = default;
    virtual bool initialize() noexcept = 0;
    virtual bool shutdown() noexcept = 0;
    virtual void service() noexcept = 0;
    virtual bool ready() const noexcept = 0;
    virtual int write(const std::uint8_t *data, std::size_t length,
                      std::uint32_t timeout_ms) noexcept = 0;
    virtual std::size_t read(std::uint8_t *data,
                             std::size_t capacity) noexcept = 0;
    virtual bool read_byte(std::uint8_t &byte) noexcept = 0;
    virtual std::size_t available() const noexcept = 0;
    virtual std::uint32_t overflow_count() const noexcept = 0;
    virtual void receive_from_isr(const std::uint8_t *data,
                                  std::size_t length) noexcept = 0;
    virtual void transmit_complete_from_isr() noexcept = 0;
    virtual void transport_connected_from_isr() noexcept = 0;
    virtual void transport_disconnected_from_isr() noexcept = 0;
};

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

struct IsrCallback {
    void (*function)(void *context) noexcept{nullptr};
    void *context{nullptr};

    void invoke() const noexcept
    {
        if (function != nullptr) {
            function(context);
        }
    }
};

enum class DmaDirection : std::uint8_t {
    PeripheralToMemory,
    MemoryToPeripheral,
    Bidirectional,
};

struct DmaBufferView {
    std::uint8_t *data{nullptr};
    std::size_t size{0U};
    std::uintptr_t token{0U};

    explicit operator bool() const noexcept
    {
        return data != nullptr && size != 0U && token != 0U;
    }
};

class DmaMemory {
public:
    virtual ~DmaMemory() = default;
    virtual DmaBufferView view(void *buffer, std::size_t length) noexcept = 0;
    virtual bool valid(const DmaBufferView &view) const noexcept = 0;
    virtual DmaBufferView acquire_bounce(const void *source,
                                         std::size_t length,
                                         DmaDirection direction) noexcept = 0;
    virtual void release_bounce(DmaBufferView &view, void *destination,
                                DmaDirection direction) noexcept = 0;
};

struct SbusInputStats {
    std::uint32_t received_bytes{0U};
    std::uint32_t overwritten_bytes{0U};
    std::uint32_t receive_errors{0U};
    std::uint32_t recovery_failures{0U};
};

class SbusInput {
public:
    virtual ~SbusInput() = default;
    virtual bool configure(std::int32_t port, bool inverted) noexcept = 0;
    virtual bool start(IsrCallback notification) noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual std::size_t read(std::uint8_t *destination,
                             std::uint64_t *arrival_timestamps_us,
                             std::size_t capacity) noexcept = 0;
    virtual bool service() noexcept = 0;
    virtual bool running() const noexcept = 0;
    virtual SbusInputStats stats() const noexcept = 0;
};

enum Icm42688InterruptMask : std::uint32_t {
    Icm42688InterruptNone = 0U,
    Icm42688InterruptInt1 = 1U << 0U,
    Icm42688InterruptInt2 = 1U << 1U,
};

struct Icm42688InterruptSnapshot {
    std::uint32_t pending_mask{0U};
    std::uint32_t int1_count{0U};
    std::uint32_t int2_count{0U};
    std::uint64_t int1_timestamp_us{0U};
    std::uint64_t int2_timestamp_us{0U};
};

class SensorInterrupts {
public:
    virtual ~SensorInterrupts() = default;
    virtual bool register_icm42688(IsrCallback notification) noexcept = 0;
    virtual void unregister_icm42688() noexcept = 0;
    virtual Icm42688InterruptSnapshot consume_icm42688() noexcept = 0;
};

constexpr std::size_t kActuatorPwmChannelCount = 6U;

enum class ActuatorPwmResult : std::uint8_t {
    Applied,
    Retry,
    Fault,
};

struct ActuatorPwmFrame {
    std::uint16_t pulse_us[kActuatorPwmChannelCount]{};
    std::uint8_t enabled_mask{0U};
};

class ActuatorPwm {
public:
    virtual ~ActuatorPwm() = default;
    virtual ActuatorPwmResult start() noexcept = 0;
    virtual ActuatorPwmResult stop() noexcept = 0;
    virtual ActuatorPwmResult write(const ActuatorPwmFrame &frame) noexcept = 0;
    virtual bool started() const noexcept = 0;
};

struct Services {
    MonotonicClock &clock;
    ExecutionContext &execution;
    CriticalSection &critical;
    Synchronization &synchronization;
    TaskRuntime &tasks;
    Heap &heap;
    FlashTransactionManager &flash_transactions;
    FlashPartition &parameter_partition;
    ArmedFlashCoordinator &armed_flash;
    Console &console;
    BootControl &boot_control;
    StartupDiagnostics &diagnostics;
    DmaMemory &dma;
    SbusInput &sbus;
    SensorInterrupts &sensor_interrupts;
    ActuatorPwm *actuator_pwm;
};

bool install_services(Services &services) noexcept;
bool services_installed() noexcept;
Services *try_services() noexcept;
Services &services() noexcept;

bool in_interrupt_context() noexcept;
bool in_realtime_context() noexcept;
TimeUs platform_time_us() noexcept;
TimeMs platform_time_ms() noexcept;
void *allocate(std::size_t size, AllocationDomain domain) noexcept;
void deallocate(void *pointer) noexcept;
HeapStats heap_stats() noexcept;

} // namespace dima::platform
