#define MODULE_NAME "sensor_cal"
#include "SensorCalibration.hpp"

#include "logging/logging.hpp"
#include "parameters/param.h"
#include "api/Time.hpp"
#include "SensorRotation.hpp"
#include "imu/VehicleImu.hpp"
#include "magnetometer/VehicleMagnetometer.hpp"

#include <algorithm>
#include <cmath>

namespace dima::modules::sensors {

bool SensorCalibration::fresh(std::uint64_t now, std::uint64_t timestamp,
                              std::uint64_t maximum_age) noexcept
{
    // 拒绝 0、未来时间和超过最大年龄的样本，避免无符号下溢。
    return timestamp != 0U && timestamp <= now &&
           now - timestamp <= maximum_age;
}

SensorCalibration::SensorCalibration(
    dima::platform::ArmedFlashCoordinator &armed,
    VehicleImu &vehicle_imu_frontend,
    VehicleMagnetometer &vehicle_magnetometer_frontend) noexcept
    /* PX4 runs calibration in Commander's non-realtime worker.  Dima's
     * logging layer deliberately rejects formatting from realtime queues, so
     * placing this transaction on wq:sensors would discard every QGC [cal]
     * STATUSTEXT token even when the RAW logging API is used. */
    // QGC 校准依赖格式化后的精确 [cal] token；实时 sensors 队列会按日志合同
    // 拒绝格式化，因此整个长事务放在 lp_default，而原始传感器前端仍保持实时。
    : px4::ScheduledWorkItem("sensor_cal", px4::wq_configurations::lp_default),
      armed_(armed), vehicle_imu_frontend_(vehicle_imu_frontend),
      vehicle_magnetometer_frontend_(vehicle_magnetometer_frontend)
{
}

SensorCalibration::~SensorCalibration() { stop(); }

bool SensorCalibration::start() noexcept
{
    if (module_state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!param_is_ready() || !ScheduleEnable() ||
        !status_publication_.advertise()) {
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    // Commander is the sole vehicle_command owner. This worker only consumes
    // the generated internal calibration request topic.
    reset_runtime_state();
    if (!calibration_request_subscription_.registerCallback()) {
        ScheduleCancelAndDrain();
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    module_state_ = dima::middleware::lifecycle::ModuleState::Running;
    const std::uint64_t now = hrt_absolute_time();
    if (!publish_status(now, true) ||
        !ScheduleOnInterval(kRunIntervalUs, kRunIntervalUs)) {
        calibration_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    PX4_INFO("PX4/QGC gyro, accel and magnetometer calibration ready");
    return true;
}

void SensorCalibration::stop() noexcept
{
    if (module_state_ == dima::middleware::lifecycle::ModuleState::Stopped) {
        return;
    }
    // 停止中的活跃事务必须先尝试恢复原参数；即使恢复失败也明确记录，不能把
    // 新的半应用校准值静默当作已完成结果。
    if (type_ != Type::None && !restore_parameters()) {
        PX4_ERR("sensor calibration parameter rollback failed during stop");
    }
    type_ = Type::None;
    phase_ = Phase::Idle;
    progress_ = 0U;
    (void)publish_status(hrt_absolute_time(), true);
    release_interlock();
    calibration_request_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    reset_runtime_state();
    module_state_ = dima::middleware::lifecycle::ModuleState::Stopped;
}

dima::middleware::lifecycle::ModuleState SensorCalibration::state()
    const noexcept
{
    return module_state_;
}

void SensorCalibration::reset_runtime_state() noexcept
{
    release_interlock();
    type_ = Type::None;
    apply_type_ = Type::None;
    phase_ = Phase::Idle;
    rollback_outcome_ = RollbackOutcome::None;
    gyro_stats_.reset();
    for (auto &side : accel_sides_) {
        side.stats.reset();
        side.done = false;
    }
    reset_accel_candidate();
    reset_mag_accumulator();
    clear_parameter_snapshot();
    clear_parameter_expectation();
    started_us_ = 0U;
    last_sample_us_ = 0U;
    apply_deadline_us_ = 0U;
    commit_time_us_ = 0U;
    parameter_notification_time_us_ = 0U;
    parameter_update_baseline_timestamp_us_ = 0U;
    last_status_us_ = 0U;
    device_id_ = 0U;
    const float identity[9]{
        1.0F, 0.0F, 0.0F,
        0.0F, 1.0F, 0.0F,
        0.0F, 0.0F, 1.0F};
    for (std::size_t index = 0U; index < 9U; ++index) {
        board_rotation_matrix_[index] = identity[index];
    }
    progress_ = 0U;
    required_parameter_update_instance_ = 0U;
    parameter_update_baseline_instance_ = 0U;
    previous_mag_calibration_count_ = 0U;
    required_parameter_update_valid_ = false;
    rollback_terminal_sent_ = false;
}

void SensorCalibration::update_inputs() noexcept
{
    if (actuator_armed_subscription_.update()) {
        actuator_armed_ = actuator_armed_subscription_.get();
    }
    if (vehicle_status_subscription_.update()) {
        vehicle_status_ = vehicle_status_subscription_.get();
    }
    if (parameter_update_subscription_.update()) {
        parameter_update_ = parameter_update_subscription_.get();
    }
    if (sensor_accel_subscription_.update()) {
        sensor_accel_ = sensor_accel_subscription_.get();
    }
    if (sensor_gyro_subscription_.update()) {
        sensor_gyro_ = sensor_gyro_subscription_.get();
    }
    if (sensor_mag_subscription_.update()) {
        sensor_mag_ = sensor_mag_subscription_.get();
    }
    if (vehicle_magnetometer_subscription_.update()) {
        vehicle_magnetometer_ = vehicle_magnetometer_subscription_.get();
    }
}

void SensorCalibration::process_requests(std::uint64_t now) noexcept
{
    sensor_calibration_request_s request{};
    while (calibration_request_subscription_.copy(&request)) {
        if (!fresh(now, request.timestamp, kRequestFreshnessUs)) {
            report_start_failure(Type::None,
                                 "stale calibration worker request");
            continue;
        }

        if (request.request ==
                sensor_calibration_request_s::REQUEST_CANCEL) {
            if (type_ != Type::None) cancel();
            continue;
        }

        Type requested = Type::None;
        if (request.request ==
                sensor_calibration_request_s::REQUEST_GYRO) {
            requested = Type::Gyro;
        } else if (request.request ==
                   sensor_calibration_request_s::REQUEST_MAG) {
            requested = Type::Mag;
        } else if (request.request ==
                   sensor_calibration_request_s::REQUEST_ACCEL) {
            requested = Type::Accel;
        }

        if (requested == Type::None) {
            if (type_ == Type::None) {
                report_start_failure(Type::None,
                                     "unsupported calibration worker request");
            }
            continue;
        }
        if (type_ != Type::None) {
            report_start_failure(requested,
                                 "another sensor calibration is active");
            continue;
        }
        if (vehicle_status_.rc_calibration_in_progress) {
            report_start_failure(requested,
                                 "radio calibration is already active");
            continue;
        }
        const char *failure_reason = nullptr;
        if (!begin(requested, now, failure_reason)) {
            report_start_failure(requested, failure_reason);
            continue;
        }

        // Commander already sent the single PX4-style COMMAND_ACK. This
        // low-priority worker owns only progress and terminal [cal] tokens.
        PX4_INFO_RAW("[cal] calibration started: 2 %s", type_name(type_));
        PX4_INFO_RAW("[cal] progress <0>");
        if (type_ == Type::Accel || type_ == Type::Mag) {
            PX4_INFO_RAW("[cal] pending: back front left right up down");
            PX4_INFO_RAW("[cal] hold vehicle still on a pending side");
        }
    }
}

bool SensorCalibration::begin(Type type, std::uint64_t now,
                              const char *&failure_reason) noexcept
{
    failure_reason = nullptr;
    // 开始条件同时要求有效 disarmed 状态、目标传感器和所需辅助 IMU 都在
    // 500 ms freshness 内、device_id 非零且数值有限。
    const bool disarmed = actuator_armed_.timestamp != 0U &&
        !actuator_armed_.armed && !actuator_armed_.kill &&
        !actuator_armed_.termination && !actuator_armed_.lockdown;
    const bool gyro_ready = sensor_gyro_.device_id != 0U && fresh(
        now, sensor_gyro_.timestamp, kSensorFreshnessUs) &&
        algorithms::finite(sensor_gyro_.x, sensor_gyro_.y, sensor_gyro_.z);
    const bool accel_ready = sensor_accel_.device_id != 0U && fresh(
        now, sensor_accel_.timestamp, kSensorFreshnessUs) &&
        algorithms::finite(sensor_accel_.x, sensor_accel_.y, sensor_accel_.z);
    const bool mag_ready = sensor_mag_.device_id != 0U && fresh(
        now, sensor_mag_.timestamp, kSensorFreshnessUs) &&
        algorithms::finite(sensor_mag_.x, sensor_mag_.y, sensor_mag_.z);
    const bool source_ready = type == Type::Gyro ? gyro_ready
        : type == Type::Accel ? accel_ready && gyro_ready
                              : mag_ready && accel_ready && gyro_ready;
    if (!disarmed) {
        failure_reason = "vehicle must be safely disarmed";
        return false;
    }
    if (!source_ready) {
        failure_reason = "fresh sensor data is unavailable";
        return false;
    }
    if (type == Type::Accel || type == Type::Mag) {
        std::int32_t rotation = 0;
        if (param_get(param_handle(dima::params::SENS_BOARD_ROT),
                      &rotation) != 0 ||
            !dima::lib::sensors::make_rotation_matrix(
                rotation, board_rotation_matrix_)) {
            failure_reason = "board rotation parameter is invalid";
            return false;
        }
    }
    // 获取全局 maintenance interlock 后直到成功应用或完整回滚才释放，确保采样/
    // 参数切换期间不能武装，也不能与其他 Flash/维护事务并发。
    if (!armed_.begin_maintenance()) {
        failure_reason = "maintenance interlock is busy";
        return false;
    }

    interlock_held_ = true;
    type_ = type;
    apply_type_ = Type::None;
    rollback_outcome_ = RollbackOutcome::None;
    phase_ = type == Type::Gyro ? Phase::CollectGyro
           : type == Type::Accel ? Phase::CollectAccel
                                 : Phase::CollectMag;
    started_us_ = now;
    last_sample_us_ = 0U;
    apply_deadline_us_ = 0U;
    commit_time_us_ = 0U;
    progress_ = 0U;
    last_status_us_ = 0U;
    gyro_stats_.reset();
    for (auto &side : accel_sides_) {
        side.stats.reset();
        side.done = false;
    }
    reset_accel_candidate();
    reset_mag_accumulator();
    clear_parameter_snapshot();
    clear_parameter_expectation();
    parameter_notification_time_us_ = 0U;
    required_parameter_update_instance_ = 0U;
    required_parameter_update_valid_ = false;
    rollback_terminal_sent_ = false;
    previous_mag_calibration_count_ =
        vehicle_magnetometer_.calibration_count;
    device_id_ = type == Type::Gyro ? sensor_gyro_.device_id
               : type == Type::Accel ? sensor_accel_.device_id
                                     : sensor_mag_.device_id;
    mag_accel_device_id_ = type == Type::Mag
        ? sensor_accel_.device_id : 0U;
    mag_gyro_device_id_ = type == Type::Mag
        ? sensor_gyro_.device_id : 0U;
    if (!publish_status(now, true)) {
        release_interlock();
        type_ = Type::None;
        phase_ = Phase::Idle;
        failure_reason = "calibration status publication failed";
        return false;
    }
    return true;
}

void SensorCalibration::report_start_failure(Type type,
                                             const char *reason) noexcept
{
    const char *const name = type_name(type);
    PX4_ERR("%s calibration rejected: %s", name,
            reason != nullptr ? reason : "unavailable");
    /* QGC does not wait for the start COMMAND_ACK. This exact PX4 token is
     * therefore required to terminate its pending calibration workflow. Keep
     * it last because QGC disconnects its calibration log on this token. */
    // QGC 不以启动 ACK 结束等待；最后一条必须是精确 calibration failed token，
    // 因为收到它后 QGC 会立即断开校准日志监听。
    px4_log_raw(_PX4_LOG_LEVEL_ERROR,
                "[cal] calibration failed: %s", name);
}

const char *SensorCalibration::type_name(Type type) noexcept
{
    switch (type) {
    case Type::Gyro: return "gyro";
    case Type::Accel: return "accel";
    case Type::Mag: return "mag";
    default: return "sensor";
    }
}

bool SensorCalibration::publish_status(std::uint64_t now,
                                       bool force) noexcept
{
    // 周期状态最多 2 Hz；阶段/进度变化使用 force 立即发布，方便 Commander 在
    // STATUSTEXT 丢失时仍能观察校准占用和禁武装状态。
    if (!force && last_status_us_ != 0U && now >= last_status_us_ &&
        now - last_status_us_ < kStatusIntervalUs) {
        return true;
    }
    sensor_calibration_status_s status{};
    status.timestamp = now;
    status.type = static_cast<std::uint8_t>(type_);
    status.progress = progress_;
    status.active = type_ != Type::None;
    if (!status_publication_.publish(status)) return false;
    last_status_us_ = now;
    return true;
}

void SensorCalibration::update_progress(std::uint8_t progress,
                                        std::uint64_t now) noexcept
{
    // 进度单调递增并钳位 100，防止换面/重采样让 QGC 进度条倒退。
    progress = std::min<std::uint8_t>(progress, 100U);
    if (progress <= progress_) return;
    progress_ = progress;
    PX4_INFO_RAW("[cal] progress <%u>", progress_);
    (void)publish_status(now, true);
}

void SensorCalibration::release_interlock() noexcept
{
    if (interlock_held_) {
        armed_.end_maintenance();
        interlock_held_ = false;
    }
}

void SensorCalibration::cancel() noexcept
{
    // 取消与失败共用参数回滚握手；若尚未写参数可同步结束，否则转 WaitForRollback。
    if (type_ == Type::None) return;
    if (phase_ == Phase::WaitForRollback) return;
    if (begin_rollback(RollbackOutcome::Cancelled)) return;
    finish_rollback();
}

void SensorCalibration::fail(const char *reason) noexcept
{
    if (type_ == Type::None) return;
    if (phase_ == Phase::WaitForRollback) return;
    const char *const name = type_name(type_);
    PX4_ERR("%s calibration failure: %s", name,
            reason != nullptr ? reason : "unknown");
    if (begin_rollback(RollbackOutcome::Failed)) return;
    finish_rollback();
}

void SensorCalibration::finish_success() noexcept
{
    if (type_ == Type::None) return;
    const char *const name = type_name(type_);
    // 只有前端确认目标 generation、device ID 和校正值完全匹配后才到达这里；
    // 随后发送 done token、释放 interlock，并把状态恢复 Idle。
    clear_parameter_snapshot();
    clear_parameter_expectation();
    update_progress(100U, hrt_absolute_time());
    PX4_INFO_RAW("[cal] calibration done: %s", name);
    release_interlock();
    type_ = Type::None;
    apply_type_ = Type::None;
    rollback_outcome_ = RollbackOutcome::None;
    phase_ = Phase::Idle;
    progress_ = 0U;
    rollback_terminal_sent_ = false;
    (void)publish_status(hrt_absolute_time(), true);
}

void SensorCalibration::Run()
{
    if (module_state_ !=
        dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    update_inputs();
    const std::uint64_t now = hrt_absolute_time();
    process_requests(now);

    // 事务期间每轮重新检查 armed/kill/termination/lockdown；安全状态任一变化都
    // 立即走失败回滚，不能仅依赖 begin 时的一次 disarmed 快照。
    if (type_ != Type::None) {
        if (phase_ == Phase::WaitForRollback) {
            process_wait_for_rollback(now);
        } else if (actuator_armed_.armed || actuator_armed_.kill ||
            actuator_armed_.termination || actuator_armed_.lockdown) {
            fail("safety state changed");
        } else {
            switch (phase_) {
            case Phase::CollectGyro: process_gyro(now); break;
            case Phase::CollectAccel: process_accel(now); break;
            case Phase::CollectMag: process_mag(now); break;
            case Phase::WaitForApply: process_wait_for_apply(now); break;
            case Phase::WaitForRollback: break;
            case Phase::Idle: break;
            }
        }
    }

    if (!publish_status(now, false) ||
        !ScheduleOnInterval(kRunIntervalUs)) {
        fail("status publication or scheduling failed");
        if (type_ != Type::None && !rollback_terminal_sent_) {
            PX4_ERR("%s calibration cannot continue; arming remains "
                    "inhibited", type_name(type_));
            px4_log_raw(_PX4_LOG_LEVEL_ERROR,
                        "[cal] calibration failed: %s", type_name(type_));
            rollback_outcome_ = RollbackOutcome::Failed;
            rollback_terminal_sent_ = true;
        }
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        calibration_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
    }
}

} // namespace dima::modules::sensors
