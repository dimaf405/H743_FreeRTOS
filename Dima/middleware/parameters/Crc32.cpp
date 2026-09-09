#include "Crc32.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::parameters {

std::uint32_t crc32_update(std::uint32_t crc,
                                  const std::uint8_t *data,
                                  std::size_t size) noexcept
{
    while (size-- > 0U) {
        crc ^= *data++;
        for (std::uint32_t bit = 0U; bit < 8U; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return crc;
}

std::uint32_t crc32(const std::uint8_t *data,
                           std::size_t size) noexcept
{
    return crc32_update(UINT32_MAX, data, size) ^ UINT32_MAX;
}

} // namespace dima::parameters
