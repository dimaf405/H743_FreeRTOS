#include "Format.hpp"

#include <cstdint>

#define NANOPRINTF_USE_FIELD_WIDTH_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_PRECISION_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FLOAT_HEX_FORMAT_SPECIFIER 0
#define NANOPRINTF_USE_FLOAT_SCI_FORMAT_SPECIFIER 0
#define NANOPRINTF_USE_FLOAT_SHORTEST_FORMAT_SPECIFIER 1
#define NANOPRINTF_USE_SMALL_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_LARGE_FORMAT_SPECIFIERS 1
#define NANOPRINTF_USE_FIXED_WIDTH_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_BINARY_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_WRITEBACK_FORMAT_SPECIFIERS 0
#define NANOPRINTF_USE_ALT_FORM_FLAG 0
#define NANOPRINTF_USE_FLOAT_SINGLE_PRECISION 0
#define NANOPRINTF_USE_DIVISION_FREE_CONVERSION 0
#define NANOPRINTF_CONVERSION_BUFFER_SIZE 64
#define NANOPRINTF_CONVERSION_FLOAT_TYPE std::uint64_t
#define NANOPRINTF_IMPLEMENTATION
#include "nanoprintf.h"

namespace dima::format {

int vformat_to(char *buffer, std::size_t capacity, const char *format,
               va_list arguments) noexcept
{
    if (format == nullptr || (buffer == nullptr && capacity != 0U)) {
        return -1;
    }
    return npf_vsnprintf(buffer, capacity, format, arguments);
}

int format_to(char *buffer, std::size_t capacity, const char *format,
              ...) noexcept
{
    va_list arguments;
    va_start(arguments, format);
    const int result = vformat_to(buffer, capacity, format, arguments);
    va_end(arguments);
    return result;
}

} // namespace dima::format
