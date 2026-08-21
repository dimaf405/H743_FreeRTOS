#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {
class ParameterFileStore;
class Synchronization;
}

namespace dima {

using FileStorageValidator = int (*)(const std::uint8_t *data,
                                     std::size_t size,
                                     void *context) noexcept;

int file_storage_initialize(platform::ParameterFileStore &store,
                            platform::Synchronization &synchronization,
                            bool &available) noexcept;
int file_storage_poll(bool &available) noexcept;
int file_storage_begin_save(const std::uint8_t *data,
                            std::size_t size) noexcept;
int file_storage_continue_save() noexcept;
void file_storage_cancel_save() noexcept;
int file_storage_load(std::uint8_t *destination, std::size_t capacity,
                      std::size_t &output_size,
                      FileStorageValidator validator,
                      void *validator_context) noexcept;

} // namespace dima
