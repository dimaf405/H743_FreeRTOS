#pragma once

#include <cstdint>

namespace dima::platform {

/** 组合根在 Services 安装完成后提供稳定的板级身份。 */
std::uint64_t board_hardware_uid() noexcept;
std::uint32_t board_version() noexcept;

} // namespace dima::platform
