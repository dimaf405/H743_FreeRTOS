#pragma once

#include "dronecan/DroneCanNode.hpp"
#include "dronecan/TransferIdTracker.hpp"
#include "lifecycle/module_base.hpp"
#include "parameter_update.hpp"
#include "parameters/param.h"
#include "api/Can.hpp"
#include "api/Flash.hpp"
#include "maintenance/RuntimeMaintenanceCoordinator.hpp"
#include "sensor_mag.hpp"
#include "uORB/Publication.hpp"
#include "uORB/uORB.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::parameters {
class FlashFS;
}

namespace dima::drivers::magnetometer {

// DroneCAN 磁力计后端只拥有 CAN 节点、源选择、Mag/Mag2 解码和原始 sensor_mag
// 发布；旋转、校准与输出限频属于独立 VehicleMagnetometer 前端，不能在此重复。
class DroneCanMag2 final : public dima::middleware::lifecycle::ModuleBase,
                           public px4::ScheduledWorkItem {
public:
    struct Stats {
        std::uint32_t accepted_transfers{0U};
        std::uint32_t rejected_sources{0U};
        std::uint32_t duplicate_transfers{0U};
        std::uint32_t stale_transfers{0U};
        std::uint32_t decode_errors{0U};
        std::uint32_t protocol_errors{0U};
        std::uint32_t publications{0U};
        std::uint32_t publication_failures{0U};
        std::uint32_t source_timeouts{0U};
        std::uint32_t node_status_transfers{0U};
        std::uint32_t node_info_responses{0U};
        std::uint32_t transport_failures{0U};
        std::uint32_t parameter_failures{0U};
        std::uint32_t allocation_requests{0U};
        std::uint32_t allocation_successes{0U};
        std::uint32_t allocation_malformed{0U};
        std::uint32_t allocation_timeouts{0U};
        std::uint32_t allocation_storage_failures{0U};
        std::uint32_t discovered_nodes{0U};
    };

    DroneCanMag2(dima::platform::CanTransport &transport,
                 dima::platform::ArmedFlashCoordinator &armed,
                 dima::middleware::maintenance::
                     RuntimeMaintenanceCoordinator &maintenance,
                 dima::parameters::FlashFS &allocation_storage) noexcept;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    const Stats &stats() const noexcept { return stats_; }

    static constexpr std::uint32_t make_device_id(
        std::uint8_t source_node_id) noexcept
    {
        // device_id 使用生成合同中的固定 PX4 位布局，禁止在驱动里手拼 bus/
        // devtype 位；source node-ID 是同类远端传感器的唯一实例标识。
        return dima::protocols::dronecan::generated::
            make_magnetometer_device_id(source_node_id);
    }

private:
    struct Configuration {
        // 参数层只保存业务值；允许值域和模式语义由生成的参数合同验证。
        bool enabled{false};
        bool automatic_allocation{false};
        std::uint32_t bitrate{500000U};
        std::uint8_t local_node_id{0U};
        std::uint8_t magnetic_node_id{0U};
    };

    enum class ReconfigurePhase : std::uint8_t {
        // 运行期总线重配必须经历 Idle -> WaitForDisarm -> WaitForPermit，
        // 从而与武装和其他 Flash/维护操作互斥；启动期尚未开放武装，可直接应用。
        Idle,
        WaitForDisarm,
        WaitForPermit,
    };

    // 启用时 2 ms 服务 CAN/libcanard，禁用时降为 100 ms 参数轮询；连续 500 ms
    // 无目标源磁场广播才报告一次超时，并把 NodeStatus health 降为 WARNING。
    static constexpr std::uint32_t kPollIntervalUs = 2000U;
    static constexpr std::uint32_t kDisabledPollIntervalUs = 100000U;
    static constexpr std::uint32_t kTransportRetryUs = 100000U;
    static constexpr std::uint64_t kSourceTimeoutUs = 500000ULL;

    void Run() override;
    void bind_parameters() noexcept;
    bool load_configuration(Configuration &configuration) noexcept;
    bool apply_configuration(const Configuration &configuration,
                             std::uint64_t now) noexcept;
    bool apply_configuration_transaction(
        const Configuration &configuration, std::uint64_t now) noexcept;
    void process_reconfiguration(std::uint64_t now) noexcept;
    void cancel_maintenance() noexcept;
    bool start_protocol(std::uint64_t now) noexcept;
    void stop_protocol() noexcept;
    void reset_source_state(std::uint64_t now) noexcept;
    void process_periodic(std::uint64_t now) noexcept;
    void handle_magnetic_field(
        dima::protocols::dronecan::DroneCanNode::Transfer &transfer) noexcept;
    void handle_allocation_event(
        const dima::protocols::dronecan::DroneCanNode::
            AllocationEvent &event) noexcept;
    bool should_accept_broadcast(std::uint64_t &signature,
                                 std::uint16_t data_type_id,
                                 std::uint8_t source_node_id) const noexcept;
    void synchronize_protocol_stats() noexcept;
    static bool same_transport_configuration(
        const Configuration &lhs, const Configuration &rhs) noexcept;
    static bool same_configuration(
        const Configuration &lhs, const Configuration &rhs) noexcept;
    dima::platform::CanTransport &transport_;
    dima::protocols::dronecan::DroneCanNode protocol_node_;
    dima::platform::ArmedFlashCoordinator &armed_;
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance_;
    dima::parameters::FlashFS &allocation_storage_;
    uORB::Subscription parameter_update_subscription_{
        ORB_ID(parameter_update)};
    uORB::Publication<sensor_mag_s> sensor_mag_publication_{
        ORB_ID(sensor_mag)};
    // configuration_ 是已应用值，pending_configuration_ 是等待维护许可的候选；
    // 事务失败回滚前者，不能让参数更新留下半切换的 transport。
    Configuration configuration_{};
    Configuration pending_configuration_{};
    dima::protocols::dronecan::TransferIdTracker magnetic_transfer_ids_{};
    dima::protocols::dronecan::DroneCanNode::Stats protocol_stats_snapshot_{};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    std::uint64_t start_time_us_{0U};
    std::uint64_t last_magnetic_time_us_{0U};
    std::uint64_t next_transport_retry_us_{0U};
    std::uint64_t next_reconfigure_retry_us_{0U};
    std::uint64_t next_allocation_error_log_us_{0U};
    // 第一个被接受的合法源锁定 active_source/device；超时只标记离线，不自动
    // 漂移到另一节点。显式参数重配/协议重启才清除绑定。
    std::uint32_t active_device_id_{0U};
    std::uint8_t active_source_node_id_{0U};
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator::Ticket
        maintenance_ticket_{0U};
    ReconfigurePhase reconfigure_phase_{ReconfigurePhase::Idle};
    bool protocol_started_{false};
    bool startup_configuration_pending_{false};
    bool configuration_pending_{false};
    bool source_online_{false};
    bool source_timeout_reported_{false};
    bool maintenance_interlock_acquired_{false};
    bool manual_mode_warning_reported_{false};
    bool protocol_start_log_reported_{false};
    bool allocation_ready_log_reported_{false};
    bool anonymous_request_log_reported_{false};
    bool source_absence_log_reported_{false};
    bool source_detection_log_reported_{false};
    Stats stats_{};
};

} // namespace dima::drivers::magnetometer
