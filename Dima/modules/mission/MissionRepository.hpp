#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace dima::modules::mission {

constexpr std::size_t kMissionCapacity = 64U;

struct MissionItem {
    // 任务仓库只允许 GLOBAL_INT/GLOBAL_RELATIVE_ALT_INT；经纬度单位为 1e-7 deg。
    std::uint16_t sequence{0U};
    std::int32_t latitude_e7{0};
    std::int32_t longitude_e7{0};
    float altitude_m{0.0F};
    float acceptance_radius_m{0.0F};
    // frame 是持久化/回读所用的规范化 MAV_FRAME_*_INT 枚举值。
    std::uint8_t frame{0U};
};

struct MissionPlan {
    std::array<MissionItem, kMissionCapacity> items{};
    std::uint16_t count{0U};
    std::uint16_t current{0U};
    std::uint32_t mission_id{0U};
};

/**
 * 固定容量任务仓库。
 *
 * staging 只接收一个严格连续的上传事务；active 只保存已经通过完整格式校验且
 * PX4 Mission State 提交成功的任务。该类型不负责线程同步，跨 WorkQueue 的互斥由
 * MissionService 统一提供，避免仓库内部引入 RTOS 依赖。
 */
class MissionRepository {
public:
    bool begin_staging(std::uint16_t expected_count) noexcept;
    bool stage_item(const MissionItem &item) noexcept;
    bool staging_complete() const noexcept;
    bool copy_staging(MissionPlan &destination) const noexcept;
    void abort_staging() noexcept;

    bool activate(const MissionPlan &committed) noexcept;
    void clear_active() noexcept;
    bool set_current(std::uint16_t sequence) noexcept;
    bool active_item(std::uint16_t sequence,
                     MissionItem &item) const noexcept;
    void active_plan(MissionPlan &plan) const noexcept;

    std::uint16_t active_count() const noexcept { return active_.count; }
    std::uint16_t current() const noexcept { return active_.current; }
    std::uint32_t mission_id() const noexcept { return active_.mission_id; }
    bool committed() const noexcept
    {
        return active_.count > 0U && active_.mission_id != 0U;
    }

private:
    static bool valid_item(const MissionItem &item) noexcept;

    MissionPlan staging_{};
    MissionPlan active_{};
    std::uint16_t expected_staging_count_{0U};
    bool staging_open_{false};
};

} // namespace dima::modules::mission
