#define MODULE_NAME "sensor_cal"
#include "SensorCalibration.hpp"

#include "imu/VehicleImu.hpp"
#include "magnetometer/VehicleMagnetometer.hpp"

#include "logging/logging.hpp"
#include "parameters/param.h"
#include "api/Time.hpp"

#include <cmath>
#include <limits>

namespace dima::modules::sensors {
namespace {

bool set_float(param_t parameter, double value) noexcept
{
    // 拟合使用 double，参数 ABI 为 float；写入前同时验证有限性和 float 范围，
    // 再用 no_notification 组成一个参数事务，最后只发一次 parameter_update。
    if (!std::isfinite(value) ||
        value < -static_cast<double>(std::numeric_limits<float>::max()) ||
        value > static_cast<double>(std::numeric_limits<float>::max())) {
        return false;
    }
    const float converted = static_cast<float>(value);
    return param_set_no_notification(parameter, &converted) == 0;
}

bool set_int(param_t parameter, std::uint32_t value) noexcept
{
    if (value == 0U || value > static_cast<std::uint32_t>(INT32_MAX)) {
        return false;
    }
    const std::int32_t converted = static_cast<std::int32_t>(value);
    return param_set_no_notification(parameter, &converted) == 0;
}

} // namespace

void SensorCalibration::clear_parameter_snapshot() noexcept
{
    parameter_snapshot_ = ParameterSnapshot{};
}

void SensorCalibration::clear_parameter_expectation() noexcept
{
    parameter_expectation_ = ParameterSnapshot{};
}

void SensorCalibration::notify_parameter_changes() noexcept
{
    // 先记录通知前的 timestamp/instance 基线和本地通知时刻，再广播；后续必须
    // 捕获“更新于本次通知之后”的 generation，不能误认历史 parameter_update。
    parameter_update_baseline_timestamp_us_ = parameter_update_.timestamp;
    parameter_update_baseline_instance_ = parameter_update_.instance;
    parameter_notification_time_us_ = hrt_absolute_time();
    required_parameter_update_instance_ = 0U;
    required_parameter_update_valid_ = false;
    param_notify_changes();
}

bool SensorCalibration::capture_required_parameter_update() noexcept
{
    if (required_parameter_update_valid_) return true;
    // instance 可能回绕，因此“新 generation”以 timestamp 变新或 instance 变化
    // 识别，同时要求消息时间不早于本地 param_notify_changes 调用时刻。
    const bool newer_than_baseline =
        parameter_update_.timestamp > parameter_update_baseline_timestamp_us_ ||
        parameter_update_.instance != parameter_update_baseline_instance_;
    if (!newer_than_baseline || parameter_notification_time_us_ == 0U ||
        parameter_update_.timestamp < parameter_notification_time_us_) {
        return false;
    }
    required_parameter_update_instance_ = parameter_update_.instance;
    required_parameter_update_valid_ = true;
    return true;
}

bool SensorCalibration::restore_parameters() noexcept
{
    if (!parameter_snapshot_.valid) return true;
    // 回滚也建立 expectation，并在同一 AtomicTransaction 中恢复 ID 与全部值；
    // 只有所有 no_notification 写入成功才通知前端并清 snapshot。
    parameter_expectation_ = parameter_snapshot_;

    param_t id = PARAM_INVALID;
    param_t values[6]{
        PARAM_INVALID, PARAM_INVALID, PARAM_INVALID,
        PARAM_INVALID, PARAM_INVALID, PARAM_INVALID};
    if (parameter_snapshot_.type == Type::Gyro) {
        id = param_handle(px4::params::CAL_GYRO0_ID);
        values[0] = param_handle(px4::params::CAL_GYRO0_XOFF);
        values[1] = param_handle(px4::params::CAL_GYRO0_YOFF);
        values[2] = param_handle(px4::params::CAL_GYRO0_ZOFF);
    } else if (parameter_snapshot_.type == Type::Accel) {
        id = param_handle(px4::params::CAL_ACC0_ID);
        values[0] = param_handle(px4::params::CAL_ACC0_XOFF);
        values[1] = param_handle(px4::params::CAL_ACC0_YOFF);
        values[2] = param_handle(px4::params::CAL_ACC0_ZOFF);
        values[3] = param_handle(px4::params::CAL_ACC0_XSCALE);
        values[4] = param_handle(px4::params::CAL_ACC0_YSCALE);
        values[5] = param_handle(px4::params::CAL_ACC0_ZSCALE);
    } else if (parameter_snapshot_.type == Type::Mag) {
        id = param_handle(px4::params::CAL_MAG0_ID);
        values[0] = param_handle(px4::params::CAL_MAG0_XOFF);
        values[1] = param_handle(px4::params::CAL_MAG0_YOFF);
        values[2] = param_handle(px4::params::CAL_MAG0_ZOFF);
        values[3] = param_handle(px4::params::CAL_MAG0_XSCALE);
        values[4] = param_handle(px4::params::CAL_MAG0_YSCALE);
        values[5] = param_handle(px4::params::CAL_MAG0_ZSCALE);
    } else {
        return false;
    }

    bool restored = true;
    px4::AtomicTransaction transaction;
    if (param_set_no_notification(id, &parameter_snapshot_.id) != 0) {
        restored = false;
    }
    for (std::size_t index = 0U;
         index < parameter_snapshot_.value_count; ++index) {
        if (values[index] == PARAM_INVALID ||
            param_set_no_notification(
                values[index], &parameter_snapshot_.values[index]) != 0) {
            restored = false;
        }
    }
    if (restored) {
        notify_parameter_changes();
        clear_parameter_snapshot();
    }
    return restored;
}

void SensorCalibration::begin_wait_for_apply(std::uint64_t now) noexcept
{
    apply_type_ = type_;
    phase_ = Phase::WaitForApply;
    commit_time_us_ = now;
    apply_deadline_us_ = now + kApplyTimeoutUs;
    update_progress(95U, now);
    /* VehicleImu and VehicleMagnetometer apply corrections directly while
     * disarmed, as PX4 frontends do. Keep the calibration arming interlock
     * until the selected frontend confirms this parameter generation. PX4
     * v1.17 gyro/accel/mag routines use ParametersSave(..., true) followed by
     * param_notify_changes(); they do not block CAL_QGC_DONE_MSG on a
     * param_save_default(true). Dima likewise leaves physical persistence to
     * the existing autosave path after the runtime correction handshake. */
    // 这里确认的是“运行时前端已应用”，不是“SD/Flash 已物理持久化”。物理保存
    // 仍由现有 autosave 路径异步完成，QGC done token 不等待 param_save_default。
}

void SensorCalibration::process_wait_for_apply(std::uint64_t now) noexcept
{
    // 成功条件同时包含：捕获本次 parameter_update generation、对应前端标记已
    // 应用、device_id/校正值精确匹配、新鲜输出可见；不能仅以 param_set 成功结束。
    bool applied = false;
    const bool update_captured = capture_required_parameter_update();
    const bool imu_update_applied = update_captured &&
        vehicle_imu_frontend_.calibration_parameter_update_applied(
            required_parameter_update_instance_);
    if (apply_type_ == Type::Gyro) {
        const float values[3]{parameter_expectation_.values[0],
                              parameter_expectation_.values[1],
                              parameter_expectation_.values[2]};
        applied = sensor_gyro_.device_id == device_id_ &&
            fresh(now, sensor_gyro_.timestamp, kSensorFreshnessUs) &&
            parameter_expectation_.valid &&
            parameter_expectation_.type == Type::Gyro &&
            imu_update_applied &&
            vehicle_imu_frontend_.gyro_calibration_matches(
                parameter_expectation_.id, values);
    } else if (apply_type_ == Type::Accel) {
        applied = sensor_accel_.device_id == device_id_ &&
            fresh(now, sensor_accel_.timestamp, kSensorFreshnessUs) &&
            parameter_expectation_.valid &&
            parameter_expectation_.type == Type::Accel &&
            imu_update_applied &&
            vehicle_imu_frontend_.accel_calibration_matches(
                parameter_expectation_.id,
                parameter_expectation_.values);
    } else if (apply_type_ == Type::Mag) {
        const bool mag_update_applied = update_captured &&
            vehicle_magnetometer_frontend_.
                calibration_parameter_update_applied(
                    required_parameter_update_instance_);
        // 磁前端还必须产生 calibration_count 变化；若计数已饱和，则以提交后的
        // 新鲜输出时间证明新配置已走过前端应用路径。
        applied = vehicle_magnetometer_.device_id == device_id_ &&
            fresh(now, vehicle_magnetometer_.timestamp,
                   kSensorFreshnessUs) &&
            parameter_expectation_.valid &&
            parameter_expectation_.type == Type::Mag &&
            mag_update_applied &&
            vehicle_magnetometer_frontend_.mag_calibration_matches(
                parameter_expectation_.id,
                parameter_expectation_.values) &&
            (vehicle_magnetometer_.calibration_count !=
                 previous_mag_calibration_count_ ||
             (previous_mag_calibration_count_ == UINT8_MAX &&
              vehicle_magnetometer_.timestamp >= commit_time_us_));
    }
    if (applied) {
        finish_success();
    } else if (now >= apply_deadline_us_) {
        fail("calibration parameters were not applied by the sensor frontend");
    }
}

bool SensorCalibration::begin_rollback(RollbackOutcome outcome) noexcept
{
    // 若事务尚未提交参数，restore 后可同步结束；若已有 snapshot，则进入
    // WaitForRollback，并保持同一 arming interlock 直到前端确认旧校正生效。
    rollback_outcome_ = outcome;
    rollback_terminal_sent_ = false;
    const bool wait_for_frontend = parameter_snapshot_.valid &&
        (parameter_snapshot_.type == Type::Gyro ||
         parameter_snapshot_.type == Type::Accel ||
         parameter_snapshot_.type == Type::Mag);
    if (!restore_parameters()) {
        latch_rollback_failure();
        return true;
    }
    if (!wait_for_frontend) return false;

    apply_type_ = parameter_expectation_.type;
    phase_ = Phase::WaitForRollback;
    const std::uint64_t now = hrt_absolute_time();
    apply_deadline_us_ = now > UINT64_MAX - kApplyTimeoutUs
        ? UINT64_MAX : now + kApplyTimeoutUs;
    /* Keep the same arming interlock across rollback. Both sensor frontends
     * apply their restored correction directly while disarmed and acknowledge
     * the exact parameter_update generation below. */
    (void)publish_status(now, true);
    return true;
}

void SensorCalibration::finish_rollback() noexcept
{
    if (type_ == Type::None) return;
    if (!rollback_terminal_sent_) {
        if (rollback_outcome_ == RollbackOutcome::Cancelled) {
            px4_log_raw(_PX4_LOG_LEVEL_ERROR,
                        "[cal] calibration cancelled");
        } else {
            /* QGC disconnects its calibration listener on this token. */
            px4_log_raw(_PX4_LOG_LEVEL_ERROR,
                        "[cal] calibration failed: %s", type_name(type_));
        }
    }
    clear_parameter_snapshot();
    clear_parameter_expectation();
    release_interlock();
    type_ = Type::None;
    apply_type_ = Type::None;
    rollback_outcome_ = RollbackOutcome::None;
    phase_ = Phase::Idle;
    progress_ = 0U;
    apply_deadline_us_ = 0U;
    rollback_terminal_sent_ = false;
    (void)publish_status(hrt_absolute_time(), true);
}

void SensorCalibration::latch_rollback_failure() noexcept
{
    // 参数无法恢复时故意停留在 WaitForRollback 且不释放 interlock；这是一种
    // fail-closed 安全锁存，需要外部修复参数/重启，而不是继续允许武装。
    PX4_ERR("%s calibration parameter rollback failed; arming remains "
            "inhibited", type_name(type_));
    px4_log_raw(_PX4_LOG_LEVEL_ERROR,
                "[cal] calibration failed: %s", type_name(type_));
    rollback_outcome_ = RollbackOutcome::Failed;
    rollback_terminal_sent_ = true;
    apply_type_ = type_;
    phase_ = Phase::WaitForRollback;
    apply_deadline_us_ = 0U;
    clear_parameter_expectation();
    parameter_notification_time_us_ = 0U;
    required_parameter_update_valid_ = false;
    (void)publish_status(hrt_absolute_time(), true);
}

void SensorCalibration::process_wait_for_rollback(
    std::uint64_t now) noexcept
{
    bool applied = false;
    const bool update_captured = capture_required_parameter_update();
    const bool imu_update_applied = update_captured &&
        vehicle_imu_frontend_.calibration_parameter_update_applied(
            required_parameter_update_instance_);
    if (apply_type_ == Type::Gyro) {
        const float values[3]{parameter_expectation_.values[0],
                              parameter_expectation_.values[1],
                              parameter_expectation_.values[2]};
        applied = parameter_expectation_.valid && imu_update_applied &&
            vehicle_imu_frontend_.gyro_calibration_matches(
                parameter_expectation_.id, values);
    } else if (apply_type_ == Type::Accel) {
        applied = parameter_expectation_.valid && imu_update_applied &&
            vehicle_imu_frontend_.accel_calibration_matches(
                parameter_expectation_.id,
                parameter_expectation_.values);
    } else if (apply_type_ == Type::Mag) {
        const bool mag_update_applied = update_captured &&
            vehicle_magnetometer_frontend_.
                calibration_parameter_update_applied(
                    required_parameter_update_instance_);
        applied = vehicle_magnetometer_.device_id == device_id_ &&
            fresh(now, vehicle_magnetometer_.timestamp,
                  kSensorFreshnessUs) &&
            parameter_expectation_.valid &&
            parameter_expectation_.type == Type::Mag &&
            mag_update_applied &&
            vehicle_magnetometer_frontend_.mag_calibration_matches(
                parameter_expectation_.id,
                parameter_expectation_.values);
    }

    // 回滚的前端确认与正向应用使用相同 generation 和数值匹配合同。
    if (applied) {
        finish_rollback();
        return;
    }
    if (apply_deadline_us_ != 0U && now >= apply_deadline_us_ &&
        !rollback_terminal_sent_) {
        PX4_ERR("%s calibration rollback was not applied; arming remains "
                "inhibited", type_name(type_));
        px4_log_raw(_PX4_LOG_LEVEL_ERROR,
                    "[cal] calibration failed: %s", type_name(type_));
        rollback_outcome_ = RollbackOutcome::Failed;
        rollback_terminal_sent_ = true;
        apply_deadline_us_ = 0U;
        (void)publish_status(now, true);
    }
}

bool SensorCalibration::commit_gyro(
    const algorithms::Vector3d &offset, std::uint32_t device_id) noexcept
{
    // commit 先读出完整旧值形成 snapshot，再原子写入 ID+offset。任一写失败
    // 立即就地恢复旧值且不发送通知，避免前端看到半套校准。
    clear_parameter_snapshot();
    clear_parameter_expectation();
    const param_t id = param_handle(px4::params::CAL_GYRO0_ID);
    const param_t values[3]{
        param_handle(px4::params::CAL_GYRO0_XOFF),
        param_handle(px4::params::CAL_GYRO0_YOFF),
        param_handle(px4::params::CAL_GYRO0_ZOFF)};
    std::int32_t old_id{};
    float old[3]{};
    px4::AtomicTransaction transaction;
    if (param_get(id, &old_id) != 0 || param_get(values[0], &old[0]) != 0 ||
        param_get(values[1], &old[1]) != 0 ||
        param_get(values[2], &old[2]) != 0) return false;
    parameter_snapshot_.type = Type::Gyro;
    parameter_snapshot_.id = old_id;
    parameter_snapshot_.value_count = 3U;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        parameter_snapshot_.values[axis] = old[axis];
    }
    parameter_snapshot_.valid = true;
    const bool written = set_int(id, device_id) &&
        set_float(values[0], offset.x) && set_float(values[1], offset.y) &&
        set_float(values[2], offset.z);
    if (!written) {
        (void)param_set_no_notification(id, &old_id);
        for (std::size_t axis = 0U; axis < 3U; ++axis) {
            (void)param_set_no_notification(values[axis], &old[axis]);
        }
        return false;
    }
    parameter_expectation_.type = Type::Gyro;
    parameter_expectation_.id = static_cast<std::int32_t>(device_id);
    parameter_expectation_.value_count = 3U;
    parameter_expectation_.values[0] = static_cast<float>(offset.x);
    parameter_expectation_.values[1] = static_cast<float>(offset.y);
    parameter_expectation_.values[2] = static_cast<float>(offset.z);
    parameter_expectation_.valid = true;
    notify_parameter_changes();
    return true;
}

bool SensorCalibration::commit_accel(
    const algorithms::Vector3d &offset,
    const algorithms::Vector3d &scale,
    std::uint32_t device_id) noexcept
{
    // 加速度事务固定为 ID + 三轴 offset + 三轴 diagonal scale，共七个参数。
    clear_parameter_snapshot();
    clear_parameter_expectation();
    const param_t id = param_handle(px4::params::CAL_ACC0_ID);
    const param_t values[6]{
        param_handle(px4::params::CAL_ACC0_XOFF),
        param_handle(px4::params::CAL_ACC0_YOFF),
        param_handle(px4::params::CAL_ACC0_ZOFF),
        param_handle(px4::params::CAL_ACC0_XSCALE),
        param_handle(px4::params::CAL_ACC0_YSCALE),
        param_handle(px4::params::CAL_ACC0_ZSCALE)};
    const double next[6]{offset.x, offset.y, offset.z,
                         scale.x, scale.y, scale.z};
    std::int32_t old_id{};
    float old[6]{};
    px4::AtomicTransaction transaction;
    if (param_get(id, &old_id) != 0) return false;
    for (std::size_t index = 0U; index < 6U; ++index) {
        if (param_get(values[index], &old[index]) != 0) return false;
    }
    parameter_snapshot_.type = Type::Accel;
    parameter_snapshot_.id = old_id;
    parameter_snapshot_.value_count = 6U;
    for (std::size_t index = 0U; index < 6U; ++index) {
        parameter_snapshot_.values[index] = old[index];
    }
    parameter_snapshot_.valid = true;
    bool written = set_int(id, device_id);
    for (std::size_t index = 0U; index < 6U; ++index) {
        written = written && set_float(values[index], next[index]);
    }
    if (!written) {
        (void)param_set_no_notification(id, &old_id);
        for (std::size_t index = 0U; index < 6U; ++index) {
            (void)param_set_no_notification(values[index], &old[index]);
        }
        return false;
    }
    parameter_expectation_.type = Type::Accel;
    parameter_expectation_.id = static_cast<std::int32_t>(device_id);
    parameter_expectation_.value_count = 6U;
    for (std::size_t index = 0U; index < 6U; ++index) {
        parameter_expectation_.values[index] =
            static_cast<float>(next[index]);
    }
    parameter_expectation_.valid = true;
    notify_parameter_changes();
    return true;
}

bool SensorCalibration::commit_mag(
    const algorithms::Vector3d &offset,
    const algorithms::Vector3d &scale,
    std::uint32_t device_id) noexcept
{
    /* Capture the frontend generation immediately before this parameter
     * transaction. A long rotation calibration must not use the count that
     * happened to be current when sampling began. */
    // 磁校准可能持续数分钟，必须在提交前一刻读取 calibration_count 基线，
    // 不能沿用采样开始时可能已经过期的计数。
    previous_mag_calibration_count_ =
        vehicle_magnetometer_.calibration_count;
    clear_parameter_snapshot();
    clear_parameter_expectation();
    const param_t id = param_handle(px4::params::CAL_MAG0_ID);
    const param_t values[6]{
        param_handle(px4::params::CAL_MAG0_XOFF),
        param_handle(px4::params::CAL_MAG0_YOFF),
        param_handle(px4::params::CAL_MAG0_ZOFF),
        param_handle(px4::params::CAL_MAG0_XSCALE),
        param_handle(px4::params::CAL_MAG0_YSCALE),
        param_handle(px4::params::CAL_MAG0_ZSCALE)};
    const double next[6]{offset.x, offset.y, offset.z,
                         scale.x, scale.y, scale.z};
    std::int32_t old_id{};
    float old[6]{};
    px4::AtomicTransaction transaction;
    if (param_get(id, &old_id) != 0) return false;
    for (std::size_t index = 0U; index < 6U; ++index) {
        if (param_get(values[index], &old[index]) != 0) return false;
    }
    parameter_snapshot_.type = Type::Mag;
    parameter_snapshot_.id = old_id;
    parameter_snapshot_.value_count = 6U;
    for (std::size_t index = 0U; index < 6U; ++index) {
        parameter_snapshot_.values[index] = old[index];
    }
    parameter_snapshot_.valid = true;
    bool written = set_int(id, device_id);
    for (std::size_t index = 0U; index < 6U; ++index) {
        written = written && set_float(values[index], next[index]);
    }
    if (!written) {
        (void)param_set_no_notification(id, &old_id);
        for (std::size_t index = 0U; index < 6U; ++index) {
            (void)param_set_no_notification(values[index], &old[index]);
        }
        return false;
    }
    parameter_expectation_.type = Type::Mag;
    parameter_expectation_.id = static_cast<std::int32_t>(device_id);
    parameter_expectation_.value_count = 6U;
    for (std::size_t index = 0U; index < 6U; ++index) {
        parameter_expectation_.values[index] =
            static_cast<float>(next[index]);
    }
    parameter_expectation_.valid = true;
    notify_parameter_changes();
    return true;
}

} // namespace dima::modules::sensors
