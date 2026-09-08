#pragma once

#include "api/Flash.hpp"
#include "parameter_update.hpp"
#include "parameters/param.h"
#include "uORB/SubscriptionData.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::rover::modes {

// 固定容量的阶段事务；参数标识由调用方使用生成的 dima::params 指定。
// 生命周期是 Candidate -> Applying -> Saving -> Done；失败经 Rollback，
// 只有旧值已重新应用并保存才释放 Arm 互锁。无运行期动态分配。
class CalibrationParameters final {
public:
    enum class Phase : std::uint8_t { Idle, Applying, Provisional, Rollback, Saving, Done, Failed, Fault };
    explicit CalibrationParameters(dima::platform::ArmedFlashCoordinator &armed) noexcept : armed_(armed) {}
    bool prepare() noexcept;
    bool add_float(dima::params parameter, float value) noexcept;
    bool add_int(dima::params parameter, std::int32_t value) noexcept;
    bool apply(std::uint64_t now, std::uint32_t expected_set_count, bool provisional = false) noexcept;
    bool refine(std::uint64_t now, std::uint32_t expected_set_count, const float (&offsets)[3]) noexcept;
    bool replace_provisional(std::uint64_t now, std::uint32_t expected_set_count,
                             const float *values, std::size_t count) noexcept;
    bool finalize_provisional(std::uint64_t now, std::uint32_t expected_set_count) noexcept;
    bool revise_float(dima::params parameter, float value) noexcept;
    bool apply_revisions(std::uint64_t now, std::uint32_t expected_set_count) noexcept;
    void poll(bool frontend_confirmed, bool validated, std::uint64_t now) noexcept;
    void cancel(std::uint64_t now) noexcept;
    Phase phase() const noexcept { return phase_; }
    bool active() const noexcept;
    bool rolling_back() const noexcept { return rollback_; }
    bool generation_valid() const noexcept { return generation_valid_; }
    std::uint32_t generation() const noexcept { return generation_; }
    std::uint64_t applied_at() const noexcept { return applied_at_; }
    std::uint32_t set_count_snapshot() const noexcept { return set_count_; }
    float expected_float(dima::params parameter) const noexcept;
    float original_float(dima::params parameter) const noexcept;
    std::int32_t expected_int(dima::params parameter) const noexcept;

private:
    struct Entry {
        param_t handle{PARAM_INVALID}; param_type_t type{}; param_value_u old{}; param_value_u next{};
        float revised{}; bool revision_pending{};
    };
    bool add(dima::params parameter, param_value_u value, param_type_t type) noexcept;
    bool matches(bool original) const noexcept;
    bool write(bool original) noexcept;
    bool apply_candidate(std::uint64_t now, bool provisional) noexcept;
    void notify(std::uint64_t now) noexcept;
    void release() noexcept;

    dima::platform::ArmedFlashCoordinator &armed_;
    uORB::SubscriptionData<parameter_update_s> update_{ORB_ID(parameter_update)};
    // 关联整定保留同一份最初快照；容量有界，不在运行期扩容或建立第二份注册表。
    Entry entries_[32]{};
    std::size_t count_{0U};
    Phase phase_{Phase::Idle};
    std::uint64_t deadline_{0U};
    std::uint64_t applied_at_{0U};
    std::uint32_t generation_{0U};
    std::uint32_t set_count_{0U};
    bool generation_valid_{false};
    bool held_{false};
    bool storage_paused_{false};
    bool rollback_{false};
    bool cancel_pending_{false};
    bool provisional_{false};
};

} // namespace dima::rover::modes
