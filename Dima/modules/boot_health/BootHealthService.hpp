#pragma once

#include "actuator_armed.hpp"
#include "actuator_output_status.hpp"
#include "lifecycle/module_base.hpp"
#include "maintenance/RuntimeMaintenanceCoordinator.hpp"
#include "api/Boot.hpp"
#include "api/Execution.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"

#include "uORB/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstdint>

namespace dima::modules::boot_health {

// BootHealth 独立核对参数核心、Commander 同拍安全快照、MotorOutput 物理输出快照
// 与维护互斥状态。它只发布“本轮可喂狗”的 generation，并在连续安全窗口后
// 确认 MCUboot test image；真正的 IWDG feed owner 仍是 appMain。
class BootHealthService final : public dima::middleware::lifecycle::ModuleBase,
                                public px4::ScheduledWorkItem {
public:
    BootHealthService(dima::platform::BootControl &boot_control,
                      dima::platform::MonotonicClock &clock,
                      dima::middleware::maintenance::
                          RuntimeMaintenanceCoordinator &maintenance) noexcept;
    ~BootHealthService() = default;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    // 单调非零的一次性健康票据；读取方必须证明比上一代更新后才可喂狗。
    std::uint32_t health_generation() const noexcept;

protected:
    void Run() override;

private:
    static constexpr std::uint32_t kCheckIntervalMs = 100U;
    // 镜像确认要求 5 s 连续健康；Topic 鲜度门限均短于应用 IWDG 期限。
    static constexpr std::uint64_t kStableWindowMs = 5000ULL;
    static constexpr std::uint64_t kSafetyTopicTimeoutUs = 750000ULL;
    static constexpr std::uint64_t kOutputStatusTimeoutUs = 250000ULL;
    // Armed 发布到 PWM 激活之间允许一个 250 ms 的明确过渡窗口。
    static constexpr std::uint64_t kActuatorArmTransitionUs = 250000ULL;

    dima::platform::BootControl &boot_control_;
    dima::platform::MonotonicClock &clock_;
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance_;
    uORB::SubscriptionData<actuator_armed_s> actuator_armed_subscription_{
        ORB_ID(actuator_armed)};
    uORB::SubscriptionData<vehicle_control_mode_s>
        vehicle_control_mode_subscription_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::SubscriptionData<actuator_output_status_s>
        actuator_output_status_subscription_{ORB_ID(actuator_output_status)};
    std::uint64_t stable_window_start_ms_{0U};
    std::uint64_t last_safety_timestamp_us_{0U};
    std::uint32_t last_output_sequence_{0U};
    std::uint32_t health_generation_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    // observed 位禁止把启动前缓存的静态 Topic 当作本次 Runtime 活性证明。
    bool safety_snapshot_observed_{false};
    bool output_snapshot_observed_{false};
    bool stable_window_active_{false};
    bool confirmation_attempted_{false};

    bool update_safety_health(std::uint64_t now_us) noexcept;
    bool update_output_health(std::uint64_t now_us) noexcept;
    bool safety_topics_consistent(std::uint64_t now_us) const noexcept;
    bool output_status_runtime_healthy(std::uint64_t now_us) const noexcept;
    bool output_status_confirmation_safe() const noexcept;
    bool output_mapping_consistent(
        const actuator_output_status_s &output) const noexcept;
    bool output_mapping_valid(
        const actuator_output_status_s &output) const noexcept;
    bool output_frame_valid(
        const actuator_output_status_s &output) const noexcept;
    bool confirmation_state_safe() const noexcept;
    void reset_stable_window(std::uint64_t now_ms) noexcept;
};

} // namespace dima::modules::boot_health
