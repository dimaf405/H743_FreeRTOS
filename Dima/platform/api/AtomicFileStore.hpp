#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

/**
 * 原子文件域决定同一代际角色对应的物理文件组。
 * Parameters 与 Mission 共享唯一 FatFs owner，但绝不共用文件名或数据格式。
 */
enum class AtomicFileDomain : std::uint8_t {
    Parameters,
    Mission,
};

enum class AtomicFile : std::uint8_t {
    Primary,
    Backup,
    Temporary,
};

class AtomicFileStore {
public:
    virtual ~AtomicFileStore() = default;

    virtual int initialize() noexcept = 0;
    virtual int begin_write(AtomicFileDomain domain, AtomicFile file,
                            const std::uint8_t *data,
                            std::size_t size) noexcept = 0;
    virtual int continue_write() noexcept = 0;
    virtual int read(AtomicFileDomain domain, AtomicFile file,
                     std::uint8_t *destination,
                     std::size_t capacity,
                     std::size_t &output_size) noexcept = 0;
    virtual int begin_verify(AtomicFileDomain domain, AtomicFile file,
                             const std::uint8_t *expected,
                             std::size_t size) noexcept = 0;
    virtual int continue_verify() noexcept = 0;
    virtual void cancel_operation() noexcept = 0;
    virtual int rename(AtomicFileDomain domain, AtomicFile from,
                       AtomicFile to) noexcept = 0;
    virtual int erase(AtomicFileDomain domain, AtomicFile file) noexcept = 0;
};

} // namespace dima::platform
