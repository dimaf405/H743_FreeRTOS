#define MODULE_NAME "dronecan_mag2"
#include "DroneCanMag2.hpp"

#include "logging/logging.hpp"
#include "api/BoardIdentity.hpp"
#include "parameters/flashfs.h"

#include <algorithm>
#include <cerrno>

namespace dima::drivers::magnetometer {
namespace {

using dima::protocols::dronecan::DroneCanNode;
namespace contract = dima::protocols::dronecan::generated;

constexpr char kNodeName[] = "com.dima.h743.rover";

enum class OperatingMode : std::int32_t {
    Disabled = 0,
    Manual = 1,
    Automatic = 2,
};

constexpr bool operating_mode_supported(std::int32_t raw) noexcept
{
    // 数值与 module_dronecan.yaml 的枚举一致，但这里表达的是实际运行分支，
    // 不是第二份参数定义：Manual 使用静态节点，Automatic 额外启动 DNA 服务。
    return raw == static_cast<std::int32_t>(OperatingMode::Disabled) ||
           raw == static_cast<std::int32_t>(OperatingMode::Manual) ||
           raw == static_cast<std::int32_t>(OperatingMode::Automatic);
}

dima::parameters::flash_file_token_t allocation_storage_token() noexcept
{
    // FlashFS token 来自生成合同，确保动态分配表的持久化键与架构/生成门禁一致。
    dima::parameters::flash_file_token_t token{};
    std::copy_n(contract::kAllocationStorageToken,
                sizeof(token.bytes), token.bytes);
    return token;
}

} // namespace

bool DroneCanMag2::bind_parameters() noexcept
{
    const bool bound = enable_parameter_.bind() &&
                       bitrate_parameter_.bind() &&
                       local_node_parameter_.bind() &&
                       magnetic_node_parameter_.bind();
    if (!bound) {
        invalidate_parameters();
    }
    return bound;
}

bool DroneCanMag2::update_parameters() noexcept
{
    // 任一缓存曾失效时重新执行完整 bind，避免把新旧参数值拼成一个总线配置。
    if (!enable_parameter_.bound() || !bitrate_parameter_.bound() ||
        !local_node_parameter_.bound() || !magnetic_node_parameter_.bound()) {
        return bind_parameters();
    }

    bool updated = enable_parameter_.update();
    updated = bitrate_parameter_.update() && updated;
    updated = local_node_parameter_.update() && updated;
    updated = magnetic_node_parameter_.update() && updated;
    if (!updated) {
        invalidate_parameters();
    }
    return updated;
}

void DroneCanMag2::invalidate_parameters() noexcept
{
    enable_parameter_.invalidate();
    bitrate_parameter_.invalidate();
    local_node_parameter_.invalidate();
    magnetic_node_parameter_.invalidate();
}

bool DroneCanMag2::load_configuration(
    Configuration &configuration) noexcept
{
    const std::int32_t enable = enable_parameter_.get();
    const std::int32_t bitrate = bitrate_parameter_.get();
    const std::int32_t local_node = local_node_parameter_.get();
    const std::int32_t magnetic_node = magnetic_node_parameter_.get();

    // 节点上限来自 DSDL 协议生成合同；bitrate 的具体可实现集合由 STM32 FDCAN
    // 后端统一校验，本层只阻止负值在转成 uint32_t 后伪装成巨大合法速率。
    if (!operating_mode_supported(enable) || bitrate <= 0 ||
        local_node <= 0 ||
        local_node > static_cast<std::int32_t>(contract::kMaximumNodeId) ||
        magnetic_node < 0 ||
        magnetic_node > static_cast<std::int32_t>(contract::kMaximumNodeId)) {
        return false;
    }

    const auto mode = static_cast<OperatingMode>(enable);
    configuration.enabled = mode != OperatingMode::Disabled;
    configuration.automatic_allocation =
        mode == OperatingMode::Automatic;
    configuration.bitrate = static_cast<std::uint32_t>(bitrate);
    configuration.local_node_id = static_cast<std::uint8_t>(local_node);
    configuration.magnetic_node_id =
        static_cast<std::uint8_t>(magnetic_node);
    return true;
}

bool DroneCanMag2::same_transport_configuration(
    const Configuration &lhs, const Configuration &rhs) noexcept
{
    return lhs.enabled == rhs.enabled && lhs.bitrate == rhs.bitrate &&
           lhs.automatic_allocation == rhs.automatic_allocation &&
           lhs.local_node_id == rhs.local_node_id &&
           lhs.magnetic_node_id == rhs.magnetic_node_id;
}

bool DroneCanMag2::same_configuration(
    const Configuration &lhs, const Configuration &rhs) noexcept
{
    return same_transport_configuration(lhs, rhs);
}

bool DroneCanMag2::apply_configuration(
    const Configuration &configuration, std::uint64_t now) noexcept
{
    // enabled/bitrate/分配模式/本地或磁力计节点任一变化都要求完整协议重启；
    // 相同配置只清 transfer-ID tracker，不制造无意义 transport 抖动。
    const bool restart =
        !same_transport_configuration(configuration_, configuration);
    if (restart) {
        protocol_start_log_reported_ = false;
        allocation_ready_log_reported_ = false;
        anonymous_request_log_reported_ = false;
        source_absence_log_reported_ = false;
        source_detection_log_reported_ = false;
    }
    magnetic_transfer_ids_.reset();
    configuration_ = configuration;
    if (!restart && protocol_started_ == configuration.enabled) {
        return true;
    }
    stop_protocol();
    return !configuration_.enabled || start_protocol(now);
}

bool DroneCanMag2::apply_configuration_transaction(
    const Configuration &configuration, std::uint64_t now) noexcept
{
    // 运行期应用失败立即恢复先前完整配置；回滚也失败时保持后台重试并报告
    // transport failure，不能把“参数已写”误报为“硬件配置已生效”。
    const Configuration previous = configuration_;
    if (apply_configuration(configuration, now)) return true;

    ++stats_.parameter_failures;
    stop_protocol();
    if (!apply_configuration(previous, now)) {
        ++stats_.transport_failures;
        PX4_ERR("DroneCAN rollback failed; retrying previous configuration");
    }
    return false;
}

void DroneCanMag2::cancel_maintenance() noexcept
{
    if (maintenance_ticket_ != 0U) {
        maintenance_.cancel(maintenance_ticket_);
    }
    maintenance_ticket_ = 0U;
    if (maintenance_interlock_acquired_) {
        armed_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    reconfigure_phase_ = ReconfigurePhase::Idle;
}

void DroneCanMag2::process_reconfiguration(std::uint64_t now) noexcept
{
    // 启动配置在系统尚未开放武装前直接应用；运行期变化必须等待 disarm，取得
    // ArmedFlashCoordinator 排他锁和 RuntimeMaintenance permit 后才重启 CAN。
    if (startup_configuration_pending_) {
        if (now < next_reconfigure_retry_us_) return;
        if (apply_configuration(pending_configuration_, now)) {
            startup_configuration_pending_ = false;
            configuration_pending_ = false;
        } else {
            ++stats_.parameter_failures;
            next_reconfigure_retry_us_ = now + kTransportRetryUs;
        }
        return;
    }

    if (!configuration_pending_ || now < next_reconfigure_retry_us_) return;
    if (reconfigure_phase_ == ReconfigurePhase::Idle) {
        reconfigure_phase_ = ReconfigurePhase::WaitForDisarm;
    }
    if (reconfigure_phase_ == ReconfigurePhase::WaitForDisarm) {
        if (armed_.armed()) return;
        if (!armed_.begin_maintenance()) {
            next_reconfigure_retry_us_ = now + kTransportRetryUs;
            return;
        }
        maintenance_interlock_acquired_ = true;
        maintenance_ticket_ = maintenance_.request(now);
        if (maintenance_ticket_ == 0U) {
            armed_.end_maintenance();
            maintenance_interlock_acquired_ = false;
            next_reconfigure_retry_us_ = now + kTransportRetryUs;
            return;
        }
        reconfigure_phase_ = ReconfigurePhase::WaitForPermit;
        return;
    }

    const auto permit = maintenance_.permit(maintenance_ticket_, now);
    if (permit == dima::middleware::maintenance::
                      RuntimeMaintenanceCoordinator::Permit::Waiting) {
        return;
    }
    if (permit == dima::middleware::maintenance::
                      RuntimeMaintenanceCoordinator::Permit::Denied) {
        cancel_maintenance();
        next_reconfigure_retry_us_ = now + 1000000ULL;
        return;
    }

    // permit 后再次检查 armed 并续报进度，封住“等待期间重新武装”的竞态。
    const bool progress = !armed_.armed() && maintenance_.report_progress(
        maintenance_ticket_, 1U, now);
    const bool applied = progress && apply_configuration_transaction(
                                         pending_configuration_, now);
    if (applied) {
        maintenance_.complete(maintenance_ticket_);
        configuration_pending_ = false;
    } else {
        maintenance_.cancel(maintenance_ticket_);
        next_reconfigure_retry_us_ = now + 1000000ULL;
    }
    maintenance_ticket_ = 0U;
    if (maintenance_interlock_acquired_) {
        armed_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    reconfigure_phase_ = ReconfigurePhase::Idle;
}

bool DroneCanMag2::start_protocol(std::uint64_t now) noexcept
{
    magnetic_transfer_ids_.reset();
    DroneCanNode::Configuration node_configuration{};
    node_configuration.bitrate = configuration_.bitrate;
    node_configuration.local_node_id = configuration_.local_node_id;
    node_configuration.automatic_allocation =
        configuration_.automatic_allocation;
    node_configuration.identity.name = kNodeName;
    node_configuration.identity.software_major = 1U;
    node_configuration.identity.software_minor = 0U;
    const std::uint32_t board_version = dima::platform::board_version();
    node_configuration.identity.hardware_major =
        static_cast<std::uint8_t>((board_version >> 8U) & 0xFFU);
    node_configuration.identity.hardware_minor =
        static_cast<std::uint8_t>(board_version & 0xFFU);
    const std::uint64_t uid = dima::platform::board_hardware_uid();
    // DroneCAN UID 共 16 B：低 8 B 为板级硬件 UID；高 8 B 由 UID、板版本和
    // 固定域分离常量异或得到，使同一板身份稳定且不需要手写设备序列号。
    const std::uint64_t uid_extension =
        uid ^ (static_cast<std::uint64_t>(board_version) << 32U) ^
        0xD1A743C04E414E31ULL;
    for (std::size_t index = 0U; index < 8U; ++index) {
        node_configuration.identity.unique_id[index] =
            static_cast<std::uint8_t>(uid >> (index * 8U));
        node_configuration.identity.unique_id[index + 8U] =
            static_cast<std::uint8_t>(uid_extension >> (index * 8U));
    }
    // 集中式分配表通过 FlashFS 固定 token 异步保存；read 必须精确返回完整
    // 生成镜像大小，短读按 -EILSEQ 拒绝，不能接受半份映像。
    node_configuration.allocation_storage.context = &allocation_storage_;
    node_configuration.allocation_storage.load = +[](
        void *context, std::uint8_t *data,
        std::size_t capacity) noexcept -> int {
        auto *const storage = static_cast<dima::parameters::FlashFS *>(context);
        if (storage == nullptr) return -EINVAL;
        const int result = storage->read_entry(
            allocation_storage_token(), data, capacity);
        if (result < 0) return result;
        return static_cast<std::size_t>(result) == capacity ? 0 : -EILSEQ;
    };
    node_configuration.allocation_storage.begin_save = +[](
        void *context, const std::uint8_t *data,
        std::size_t size) noexcept -> int {
        auto *const storage = static_cast<dima::parameters::FlashFS *>(context);
        return storage == nullptr
                   ? -EINVAL
                   : storage->begin_write_entry(
                         allocation_storage_token(), data, size);
    };
    node_configuration.allocation_storage.continue_save = +[](
        void *context) noexcept -> int {
        auto *const storage = static_cast<dima::parameters::FlashFS *>(context);
        return storage == nullptr ? -EINVAL : storage->continue_operation();
    };
    node_configuration.allocation_storage.cancel_save = +[](
        void *context) noexcept {
        auto *const storage = static_cast<dima::parameters::FlashFS *>(context);
        if (storage != nullptr) storage->cancel_operation();
    };

    // C ABI 回调只把 context 转回当前对象；广播在回调内同步解码，因为
    // DroneCanNode 返回后会释放 libcanard payload。
    DroneCanNode::Callbacks callbacks{};
    callbacks.context = this;
    callbacks.accept_broadcast = +[](
        void *context, std::uint64_t &signature,
        std::uint16_t data_type_id,
        std::uint8_t source_node_id) noexcept -> bool {
        const auto *const self = static_cast<const DroneCanMag2 *>(context);
        return self != nullptr && self->should_accept_broadcast(
            signature, data_type_id, source_node_id);
    };
    callbacks.receive_broadcast = +[](
        void *context, DroneCanNode::Transfer &transfer) noexcept {
        auto *const self = static_cast<DroneCanMag2 *>(context);
        if (self != nullptr) self->handle_magnetic_field(transfer);
    };
    callbacks.allocation_event = +[](
        void *context, const DroneCanNode::AllocationEvent &event) noexcept {
        auto *const self = static_cast<DroneCanMag2 *>(context);
        if (self != nullptr) self->handle_allocation_event(event);
    };

    protocol_stats_snapshot_ = {};
    if (!protocol_node_.start(node_configuration, callbacks, now)) {
        synchronize_protocol_stats();
        next_transport_retry_us_ = now + kTransportRetryUs;
        return false;
    }
    protocol_started_ = true;
    next_transport_retry_us_ = 0U;
    reset_source_state(now);
    if (!protocol_start_log_reported_) {
        protocol_start_log_reported_ = true;
        PX4_INFO("FDCAN1 DroneCAN bitrate=%lu node=%u mag_node=%u dna=%u",
                 static_cast<unsigned long>(configuration_.bitrate),
                 configuration_.local_node_id,
                 configuration_.magnetic_node_id,
                 configuration_.automatic_allocation ? 1U : 0U);
    }
    if (!configuration_.automatic_allocation &&
        !manual_mode_warning_reported_) {
        PX4_WARN("DroneCAN DNA disabled; anonymous nodes require UAVCAN1_ENABLE=2");
        manual_mode_warning_reported_ = true;
    }
    return true;
}

void DroneCanMag2::stop_protocol() noexcept
{
    // 关闭前先同步最后一批节点统计，再释放 transport 和所有源/transfer-ID 状态。
    synchronize_protocol_stats();
    protocol_node_.stop();
    magnetic_transfer_ids_.reset();
    protocol_started_ = false;
    source_online_ = false;
    source_timeout_reported_ = false;
    active_device_id_ = 0U;
    active_source_node_id_ = 0U;
}

} // namespace dima::drivers::magnetometer
