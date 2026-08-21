#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

class CriticalSection;

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
    bool begin_maintenance() noexcept;
    void end_maintenance() noexcept;
    bool armed() const noexcept;

    ArmedFlashCoordinator(const ArmedFlashCoordinator &) = delete;
    ArmedFlashCoordinator &operator=(const ArmedFlashCoordinator &) = delete;

private:
    CriticalSection &critical_;
    bool armed_{false};
    bool flash_busy_{false};
    bool maintenance_busy_{false};
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

} // namespace dima::platform
