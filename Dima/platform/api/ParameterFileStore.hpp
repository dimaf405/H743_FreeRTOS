#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

enum class ParameterFile : std::uint8_t {
    Primary,
    Backup,
    Temporary,
};

class ParameterFileStore {
public:
    virtual ~ParameterFileStore() = default;

    virtual int initialize() noexcept = 0;
    virtual int begin_write(ParameterFile file, const std::uint8_t *data,
                            std::size_t size) noexcept = 0;
    virtual int continue_write() noexcept = 0;
    virtual int read(ParameterFile file, std::uint8_t *destination,
                     std::size_t capacity,
                     std::size_t &output_size) noexcept = 0;
    virtual int begin_verify(ParameterFile file,
                             const std::uint8_t *expected,
                             std::size_t size) noexcept = 0;
    virtual int continue_verify() noexcept = 0;
    virtual void cancel_operation() noexcept = 0;
    virtual int rename(ParameterFile from, ParameterFile to) noexcept = 0;
    virtual int erase(ParameterFile file) noexcept = 0;
};

} // namespace dima::platform
