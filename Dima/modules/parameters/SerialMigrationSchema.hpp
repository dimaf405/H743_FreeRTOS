#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::modules::parameters {

inline constexpr std::size_t kSchema1SerialCount = 7U;
inline constexpr std::int32_t kSchema1BaudDefaults[kSchema1SerialCount]{
    0, 115200, 0, 115200, 0, 57600, 921600,
};
inline constexpr std::int32_t kSchema1FunctionDefaults[kSchema1SerialCount]{
    0, 0, 1, 0, 0, 0, 0,
};

/* Schema v1 的 SERIAL1..7 是功能顺序，不能按数字复制到 v2。
 * 该表按物理 UART 身份迁移，避免升级后把 RC 接到错误端口。 */
inline constexpr std::int32_t kSchema1ToDirectSerial[kSchema1SerialCount]{
    2, 4, 6, 8, 3, 7, 1,
};

} // namespace dima::modules::parameters
