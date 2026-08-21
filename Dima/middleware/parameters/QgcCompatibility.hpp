#pragma once

#include <cstdint>
#include <cstring>

namespace dima::parameters {

struct QgcFixedInt32Parameter {
    const char *name;
    std::int32_t value;
};

inline constexpr QgcFixedInt32Parameter kQgcFixedInt32Parameters[]{
    {"SYS_AUTOSTART", 50000},
    {"SYS_AUTOCONFIG", 0},
    {"MAV_SYS_ID", 1},
    {"CAL_GYRO0_ID", 0},
    {"CAL_ACC0_ID", 0},
    {"CAL_MAG0_ID", 0},
    {"CAL_MAG1_ID", 0},
    {"CAL_MAG2_ID", 0},
    {"NAV_RCL_ACT", 6},
    {"NAV_DLL_ACT", 0},
    {"COM_LOW_BAT_ACT", 0},
};

inline const QgcFixedInt32Parameter *qgc_fixed_int32_parameter(
    const char *name) noexcept
{
    if (name == nullptr) {
        return nullptr;
    }
    for (const QgcFixedInt32Parameter &parameter :
         kQgcFixedInt32Parameters) {
        if (std::strcmp(name, parameter.name) == 0) {
            return &parameter;
        }
    }
    return nullptr;
}

} // namespace dima::parameters
