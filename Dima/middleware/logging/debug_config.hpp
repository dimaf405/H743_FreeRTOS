#pragma once

#include <cstdint>

namespace dima::logging {

enum class Level : std::uint8_t {
    Debug = 0U,
    Info,
    Warning,
    Error,
    Panic,
    Off,
};

enum class Source : std::uint8_t {
    System = 0U,
    Sbus,
    Icm42688,
};

struct SourcePolicy {
    Level minimum_level;
    bool data_to_usb;
    std::uint32_t data_period_ms;
};

namespace config {

inline constexpr Level kUsbMinimumLevel = Level::Debug;
inline constexpr SourcePolicy kSystem{Level::Info, false, 0U};
inline constexpr SourcePolicy kSbus{Level::Error, true, 100U};
inline constexpr SourcePolicy kIcm42688{Level::Off, false, 100U};

constexpr const SourcePolicy &policy(Source source) noexcept
{
    switch (source) {
    case Source::Sbus:
        return kSbus;
    case Source::Icm42688:
        return kIcm42688;
    case Source::System:
    default:
        return kSystem;
    }
}

constexpr bool enabled(Source source, Level level) noexcept
{
    return level != Level::Off &&
           static_cast<std::uint8_t>(level) >=
               static_cast<std::uint8_t>(kUsbMinimumLevel) &&
           static_cast<std::uint8_t>(level) >=
               static_cast<std::uint8_t>(policy(source).minimum_level);
}

static_assert(!kSbus.data_to_usb || kSbus.data_period_ms >= 20U,
              "SBUS USB data output must not exceed the log service rate");
static_assert(!kSbus.data_to_usb ||
                  static_cast<std::uint8_t>(kUsbMinimumLevel) <=
                      static_cast<std::uint8_t>(Level::Debug),
              "SBUS data output requires USB Debug level");

} // namespace config
} // namespace dima::logging
