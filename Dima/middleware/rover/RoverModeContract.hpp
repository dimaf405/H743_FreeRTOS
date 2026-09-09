#pragma once

#include "vehicle_status.hpp"
#include "vehicle_control_mode.hpp"

#include <cstdint>

namespace dima::middleware::rover::mode_contract {

// 模式编号的唯一数值权威是生成的 vehicle_status_s；本文件只集中派生 Rover
// 产品支持掩码，Commander、RC 和差速控制不得各自复制另一份数值列表。
inline constexpr std::uint32_t kManualMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
inline constexpr std::uint32_t kMissionMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
inline constexpr std::uint32_t kLoiterMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
inline constexpr std::uint32_t kTerminationMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;
inline constexpr std::uint32_t kAutoCalibrationMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;

inline constexpr std::uint32_t kImplementedMask =
    kManualMask | kMissionMask | kLoiterMask | kTerminationMask |
    kAutoCalibrationMask;
inline constexpr std::uint32_t kUserSettableMask =
    kManualMask | kAutoCalibrationMask;

bool supported(std::uint8_t state) noexcept;

bool rc_slot_supported(std::int32_t state) noexcept;

bool auto_calibration(std::uint8_t state) noexcept;

// 控制层与执行器层共用逐字段精确模式投影，禁止相同模式在两层采用不同标志。
// Armed、快照一致性与失鲜门控仍由各消费者独立检查。
bool manual_projection(const vehicle_control_mode_s &control) noexcept;

bool navigation_projection(const vehicle_control_mode_s &control) noexcept;

bool calibration_projection_common(const vehicle_control_mode_s &control) noexcept;

bool calibration_open_loop_projection(const vehicle_control_mode_s &control) noexcept;

bool calibration_closed_loop_projection(const vehicle_control_mode_s &control) noexcept;

bool calibration_projection(const vehicle_control_mode_s &control) noexcept;

} // namespace dima::middleware::rover::mode_contract
