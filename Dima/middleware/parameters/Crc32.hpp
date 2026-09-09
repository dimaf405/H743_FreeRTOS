#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::parameters {

std::uint32_t crc32_update(std::uint32_t crc,
                                  const std::uint8_t *data,
                                  std::size_t size) noexcept;

std::uint32_t crc32(const std::uint8_t *data,
                           std::size_t size) noexcept;

} // namespace dima::parameters
