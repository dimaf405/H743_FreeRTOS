#pragma once

#include <cstdarg>
#include <cstddef>

namespace dima::format {

// The firmware configuration covers the linked format set: strings, standard
// signed/unsigned integers (including long long), field width, precision and
// floating-point %f/%g. It intentionally excludes %n, binary, alternate form,
// small/C23 modifiers and direct %a/%e conversions.

#if defined(__GNUC__)
#define DIMA_FORMAT_ATTRIBUTE(format_index, arguments_index) \
    __attribute__((format(printf, format_index, arguments_index)))
#else
#define DIMA_FORMAT_ATTRIBUTE(format_index, arguments_index)
#endif

int vformat_to(char *buffer, std::size_t capacity, const char *format,
               va_list arguments) noexcept DIMA_FORMAT_ATTRIBUTE(3, 0);

int format_to(char *buffer, std::size_t capacity, const char *format,
              ...) noexcept DIMA_FORMAT_ATTRIBUTE(3, 4);

#undef DIMA_FORMAT_ATTRIBUTE

} // namespace dima::format
