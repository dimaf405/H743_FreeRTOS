#include "MissionRepository.hpp"

#include "mavlink/MavlinkBridge.h"

#include <cmath>

namespace dima::modules::mission {
namespace {

constexpr std::int32_t kLatitudeMaximumE7 = 900000000;
constexpr std::int32_t kLongitudeMaximumE7 = 1800000000;

} // namespace

bool MissionRepository::valid_item(const MissionItem &item) noexcept
{
    // 仓库只保存规范化后的两种 _INT frame，坐标固定为 1e-7 deg；MAVLink
    // 接收边界允许 QGC 的非 _INT 别名，但必须先完成单位不变的 frame 收敛。
    // 高度仅为协议回读字段，Rover 控制不会使用，但仍要求有限值，避免损坏
    // 文件把 NaN 带入 QGC。
    return item.sequence < kMissionCapacity &&
           (item.frame == MAV_FRAME_GLOBAL_INT ||
            item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) &&
           item.latitude_e7 >= -kLatitudeMaximumE7 &&
           item.latitude_e7 <= kLatitudeMaximumE7 &&
           item.longitude_e7 >= -kLongitudeMaximumE7 &&
           item.longitude_e7 <= kLongitudeMaximumE7 &&
           std::isfinite(item.altitude_m) &&
           std::isfinite(item.acceptance_radius_m) &&
           item.acceptance_radius_m >= 0.0F;
}

bool MissionRepository::begin_staging(
    std::uint16_t expected_count) noexcept
{
    if (expected_count > kMissionCapacity || staging_open_) {
        return false;
    }
    staging_ = {};
    expected_staging_count_ = expected_count;
    staging_open_ = true;
    return true;
}

bool MissionRepository::stage_item(const MissionItem &item) noexcept
{
    if (!staging_open_ || staging_.count >= expected_staging_count_ ||
        item.sequence != staging_.count || !valid_item(item)) {
        return false;
    }
    staging_.items[staging_.count] = item;
    ++staging_.count;
    return true;
}

bool MissionRepository::staging_complete() const noexcept
{
    return staging_open_ && staging_.count == expected_staging_count_;
}

bool MissionRepository::copy_staging(MissionPlan &destination) const noexcept
{
    if (!staging_complete()) {
        return false;
    }
    destination = staging_;
    destination.current = 0U;
    destination.mission_id = 0U;
    return true;
}

void MissionRepository::abort_staging() noexcept
{
    staging_ = {};
    expected_staging_count_ = 0U;
    staging_open_ = false;
}

bool MissionRepository::activate(const MissionPlan &committed) noexcept
{
    if (committed.count > kMissionCapacity ||
        (committed.count > 0U && committed.mission_id == 0U)) {
        return false;
    }
    for (std::uint16_t sequence = 0U; sequence < committed.count;
         ++sequence) {
        if (committed.items[sequence].sequence != sequence ||
            !valid_item(committed.items[sequence])) {
            return false;
        }
    }

    // active 的切换是一次完整结构体赋值，只能由 MissionService 持锁调用；
    // 调用点位于 Mission State commit 成功之后，因此失败上传永远不会覆盖旧 bank。
    active_ = committed;
    active_.current = active_.count == 0U
                          ? 0U
                          : (active_.current < active_.count
                                 ? active_.current
                                 : 0U);
    abort_staging();
    return true;
}

void MissionRepository::clear_active() noexcept
{
    active_ = {};
}

bool MissionRepository::set_current(std::uint16_t sequence) noexcept
{
    if (sequence >= active_.count) {
        return false;
    }
    active_.current = sequence;
    return true;
}

bool MissionRepository::active_item(std::uint16_t sequence,
                                    MissionItem &item) const noexcept
{
    if (sequence >= active_.count) {
        return false;
    }
    item = active_.items[sequence];
    return true;
}

void MissionRepository::active_plan(MissionPlan &plan) const noexcept
{
    plan = active_;
}

} // namespace dima::modules::mission
