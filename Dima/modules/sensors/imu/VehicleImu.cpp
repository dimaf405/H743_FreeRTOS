/****************************************************************************
 * PX4-Autopilot v1.17.0 VehicleIMU single-device front end adapted for Dima.
 * Upstream: src/modules/sensors/vehicle_imu
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#define MODULE_NAME "vehicle_imu"
#include "VehicleImu.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <cmath>
#include <limits>

namespace dima::modules::sensors {
namespace algorithms = vehicle_imu_algorithms;
namespace validation = dima::lib::sensors::validation;
namespace {

template<typename Sample>
validation::ImuSample validation_sample(const Sample &sample) noexcept
{
    // 将 accel/gyro 同构字段投影到统一校验视图，不复制或修改原始消息。
    validation::ImuSample result{};
    result.timestamp = sample.timestamp;
    result.timestamp_sample = sample.timestamp_sample;
    result.device_id = sample.device_id;
    result.value[0] = sample.x;
    result.value[1] = sample.y;
    result.value[2] = sample.z;
    result.temperature_c = sample.temperature;
    result.error_count = sample.error_count;
    result.clip_counter[0] = sample.clip_counter[0];
    result.clip_counter[1] = sample.clip_counter[1];
    result.clip_counter[2] = sample.clip_counter[2];
    result.samples = sample.samples;
    return result;
}

std::uint32_t saturating_sum(std::uint32_t left,
                             std::uint32_t right) noexcept
{
    return right > UINT32_MAX - left ? UINT32_MAX : left + right;
}

void saturating_add(std::uint32_t &value,
                    std::uint32_t increment) noexcept
{
    value = saturating_sum(value, increment);
}

float vector_norm(const algorithms::Vector3 &value) noexcept
{
    return std::sqrt(value.x * value.x + value.y * value.y +
                     value.z * value.z);
}

bool finite_message_vector(const float (&value)[3]) noexcept
{
    return std::isfinite(value[0]) && std::isfinite(value[1]) &&
           std::isfinite(value[2]);
}

template<std::size_t Count>
bool commit_calibration_parameters(
    param_t id_handle, const param_t (&value_handles)[Count],
    std::uint32_t device_id, const float (&next_values)[Count]) noexcept
{
    if (device_id == 0U ||
        device_id > static_cast<std::uint32_t>(INT32_MAX)) {
        return false;
    }
    for (std::size_t index = 0U; index < Count; ++index) {
        if (!std::isfinite(next_values[index])) {
            return false;
        }
    }

    std::int32_t old_id{};
    float old_values[Count]{};
    px4::AtomicTransaction transaction;
    if (param_get(id_handle, &old_id) != 0) {
        return false;
    }
    for (std::size_t index = 0U; index < Count; ++index) {
        if (param_get(value_handles[index], &old_values[index]) != 0) {
            return false;
        }
    }

    const std::int32_t next_id = static_cast<std::int32_t>(device_id);
    bool written = param_set_no_notification(id_handle, &next_id) == 0;
    for (std::size_t index = 0U; index < Count; ++index) {
        written =
            param_set_no_notification(value_handles[index],
                                      &next_values[index]) == 0 &&
            written;
    }
    if (!written) {
        // ID 和各轴 offset/scale 是一个不可拆分的校准事务；任一写失败都在同一
        // 参数锁内恢复整组旧值，绝不通知前端观察半套坐标校正。
        bool restored = param_set_no_notification(id_handle, &old_id) == 0;
        for (std::size_t index = 0U; index < Count; ++index) {
            restored =
                param_set_no_notification(value_handles[index],
                                          &old_values[index]) == 0 &&
                restored;
        }
        if (!restored) {
            PX4_ERR("IMU autocal parameter rollback failed");
        }
        return false;
    }

    // 这里只更新参数内存并发送一次 generation；Flash/SD 物理保存仍由现有
    // Parameter autosave 负责，实时传感器前端不直接执行阻塞持久化。
    param_notify_changes();
    return true;
}

void update_moments(algorithms::Vector3 &mean,
                    algorithms::Vector3 &m2,
                    std::uint32_t &count,
                    const algorithms::Vector3 &value) noexcept
{
    // Welford：mean_n=mean+delta/n，M2+=delta*(x-mean_n)；计数饱和后保持窗口。
    if (count == UINT32_MAX) {
        return;
    }
    ++count;
    const float divisor = static_cast<float>(count);
    const algorithms::Vector3 delta{
        value.x - mean.x, value.y - mean.y, value.z - mean.z};
    mean.x += delta.x / divisor;
    mean.y += delta.y / divisor;
    mean.z += delta.z / divisor;
    m2.x += delta.x * (value.x - mean.x);
    m2.y += delta.y * (value.y - mean.y);
    m2.z += delta.z * (value.z - mean.z);
}

float update_rate(std::uint32_t updates, std::uint64_t first_us,
                  std::uint64_t last_us) noexcept
{
    if (updates < 2U || last_us <= first_us) {
        return 0.0F;
    }
    // 更新频率=(updates-1)*1e6/(last-first)，因为 N 个时间点只有 N-1 个间隔。
    return static_cast<float>(updates - 1U) * 1000000.0F /
           static_cast<float>(last_us - first_us);
}

float raw_rate(std::uint32_t samples, std::uint8_t first_samples,
               std::uint64_t first_us, std::uint64_t last_us) noexcept
{
    if (samples <= first_samples || last_us <= first_us) {
        return 0.0F;
    }
    return static_cast<float>(samples - first_samples) * 1000000.0F /
           static_cast<float>(last_us - first_us);
}

float variance(float m2, std::uint32_t count) noexcept
{
    return count > 1U ? m2 / static_cast<float>(count - 1U) : 0.0F;
}

} // namespace

VehicleImu::VehicleImu(
    dima::platform::ArmedFlashCoordinator &armed) noexcept
    : px4::ScheduledWorkItem("vehicle_imu", px4::wq_configurations::sensors),
      armed_(armed),
      parameter_commit_worker_(*this)
{
}

VehicleImu::ParameterCommitWorkItem::ParameterCommitWorkItem(
    VehicleImu &owner) noexcept
    : px4::ScheduledWorkItem("imu_param_commit",
                             px4::wq_configurations::lp_default),
      owner_(owner)
{
}

void VehicleImu::ParameterCommitWorkItem::Run()
{
    owner_.run_parameter_commit();
}

bool VehicleImu::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    reset_parameter_commit();
    if (!ScheduleEnable() || !parameter_commit_worker_.ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        parameter_commit_worker_.ScheduleCancelAndDrain();
        return false;
    }

    stats_ = Stats{};
    accel_device_id_ = 0U;
    gyro_device_id_ = 0U;
    accel_validator_.reset();
    gyro_validator_.reset();
    accel_validation_error_count_ = 0U;
    gyro_validation_error_count_ = 0U;
    latest_accel_error_count_ = 0U;
    latest_gyro_error_count_ = 0U;
    __atomic_store_n(&applied_parameter_update_instance_, 0U,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&applied_parameter_update_valid_, false,
                     __ATOMIC_RELEASE);
    last_status_publish_us_ = 0U;
    validation_fault_active_ = false;
    clipping_fault_active_ = false;
    status_dirty_ = false;
    accel_clipping_total_[0] = 0U;
    accel_clipping_total_[1] = 0U;
    accel_clipping_total_[2] = 0U;
    gyro_clipping_total_[0] = 0U;
    gyro_clipping_total_[1] = 0U;
    gyro_clipping_total_[2] = 0U;
    accel_vibration_metric_ = 0.0F;
    gyro_vibration_metric_ = 0.0F;
    previous_status_accel_ = {};
    previous_status_gyro_ = {};
    have_previous_status_accel_ = false;
    have_previous_status_gyro_ = false;
    reset_status_window();
    clear_pending_configuration();
    clear_learned_calibrations();
    autocal_last_bias_check_us_ = 0U;
    autocal_quiet_until_us_ = 0U;
    autocal_retry_after_us_ = 0U;
    reset_integrators(true);
    // 参数、初始配置和两个发布端均成功后才注册回调；任一步失败完整回滚，
    // 不让部分启动的实时前端消费传感器数据。
    Configuration initial_configuration{};
    if (!bind_parameters() ||
        read_configuration(false, initial_configuration) !=
            ConfigurationReadResult::Valid ||
        !vehicle_imu_pub_.advertise() ||
        !vehicle_imu_status_pub_.advertise()) {
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        parameter_commit_worker_.ScheduleCancelAndDrain();
        PX4_ERR("parameter binding or uORB advertisement failed");
        return false;
    }
    apply_configuration(initial_configuration);
    if (!gyro_sub_.registerCallback()) {
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        parameter_commit_worker_.ScheduleCancelAndDrain();
        PX4_ERR("gyro callback registration failed");
        return false;
    }
    if (!parameter_update_sub_.registerCallback()) {
        gyro_sub_.unregisterCallback();
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        parameter_commit_worker_.ScheduleCancelAndDrain();
        PX4_ERR("parameter callback registration failed");
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleNow()) {
        parameter_update_sub_.unregisterCallback();
        gyro_sub_.unregisterCallback();
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        parameter_commit_worker_.ScheduleCancelAndDrain();
        return false;
    }
    return true;
}

void VehicleImu::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    parameter_update_sub_.unregisterCallback();
    gyro_sub_.unregisterCallback();
    // 先排空 realtime producer，再排空 lp_default consumer，保证参数句柄失效前
    // 不再有提交器读取快照或执行事务。
    ScheduleCancelAndDrain();
    parameter_commit_worker_.ScheduleCancelAndDrain();
    reset_parameter_commit();
    clear_pending_configuration();
    clear_learned_calibrations();
    invalidate_parameters();
    reset_integrators(true);
    accel_validator_.reset();
    gyro_validator_.reset();
    reset_status_window();
    stats_ = Stats{};
    accel_device_id_ = 0U;
    gyro_device_id_ = 0U;
}

dima::middleware::lifecycle::ModuleState VehicleImu::state() const
{
    return state_;
}

bool VehicleImu::calibration_parameter_update_applied(
    std::uint32_t required_instance) const noexcept
{
    if (!__atomic_load_n(&applied_parameter_update_valid_,
                         __ATOMIC_ACQUIRE)) {
        return false;
    }
    const std::uint32_t applied = __atomic_load_n(
        &applied_parameter_update_instance_, __ATOMIC_ACQUIRE);
    // 用有符号差比较 32-bit generation，允许 UINT32 回绕；在半空间内 applied
    // 等于或晚于 required 即满足校准握手。
    return static_cast<std::int32_t>(applied - required_instance) >= 0;
}

bool VehicleImu::accel_calibration_matches(
    std::int32_t configured_device_id,
    const float (&values)[6]) const noexcept
{
    const auto &active = active_configuration_.accel;
    if (active.configured_device_id != configured_device_id) return false;
    const bool saved_calibration_applied =
        algorithms::calibration_id_matches(configured_device_id,
                                           accel_device_id_);
    if (!saved_calibration_applied) {
        return active.enabled && algorithms::calibration_is_identity(active);
    }
    return active.enabled && active.offset.x == values[0] &&
           active.offset.y == values[1] && active.offset.z == values[2] &&
           active.scale.x == values[3] && active.scale.y == values[4] &&
           active.scale.z == values[5];
}

bool VehicleImu::gyro_calibration_matches(
    std::int32_t configured_device_id,
    const float (&values)[3]) const noexcept
{
    const auto &active = active_configuration_.gyro;
    if (active.configured_device_id != configured_device_id) return false;
    const bool saved_calibration_applied =
        algorithms::calibration_id_matches(configured_device_id,
                                           gyro_device_id_);
    if (!saved_calibration_applied) {
        return active.enabled && algorithms::calibration_is_identity(active);
    }
    return active.enabled && active.offset.x == values[0] &&
           active.offset.y == values[1] && active.offset.z == values[2];
}

bool VehicleImu::bind_parameters() noexcept
{
    return board_rotation_.bind() && integration_rate_.bind() &&
           imu_autocal_.bind() &&
           clipping_notifications_.bind() &&
           accel_id_.bind() &&
           accel_x_offset_.bind() && accel_y_offset_.bind() &&
           accel_z_offset_.bind() && accel_x_scale_.bind() &&
           accel_y_scale_.bind() && accel_z_scale_.bind() &&
           gyro_id_.bind() && gyro_x_offset_.bind() &&
           gyro_y_offset_.bind() && gyro_z_offset_.bind();
}

void VehicleImu::invalidate_parameters() noexcept
{
    board_rotation_.invalidate();
    integration_rate_.invalidate();
    imu_autocal_.invalidate();
    clipping_notifications_.invalidate();
    accel_id_.invalidate();
    accel_x_offset_.invalidate();
    accel_y_offset_.invalidate();
    accel_z_offset_.invalidate();
    accel_x_scale_.invalidate();
    accel_y_scale_.invalidate();
    accel_z_scale_.invalidate();
    gyro_id_.invalidate();
    gyro_x_offset_.invalidate();
    gyro_y_offset_.invalidate();
    gyro_z_offset_.invalidate();
    active_configuration_ = Configuration{};
    pending_configuration_ = Configuration{};
}

bool VehicleImu::refresh_parameter_cache() noexcept
{
    bool refreshed = board_rotation_.update();
    refreshed = integration_rate_.update() && refreshed;
    refreshed = imu_autocal_.update() && refreshed;
    refreshed = clipping_notifications_.update() && refreshed;
    refreshed = accel_id_.update() && refreshed;
    refreshed = accel_x_offset_.update() && refreshed;
    refreshed = accel_y_offset_.update() && refreshed;
    refreshed = accel_z_offset_.update() && refreshed;
    refreshed = accel_x_scale_.update() && refreshed;
    refreshed = accel_y_scale_.update() && refreshed;
    refreshed = accel_z_scale_.update() && refreshed;
    refreshed = gyro_id_.update() && refreshed;
    refreshed = gyro_x_offset_.update() && refreshed;
    refreshed = gyro_y_offset_.update() && refreshed;
    refreshed = gyro_z_offset_.update() && refreshed;
    return refreshed;
}

VehicleImu::ConfigurationReadResult VehicleImu::read_configuration(
    bool refresh, Configuration &configuration) noexcept
{
    if (refresh && !refresh_parameter_cache()) {
        return ConfigurationReadResult::ReadError;
    }

    // 先构造完整 candidate 并验证旋转、积分频率、clipping 开关及校准范围；
    // 全部合法后才交给 staged/active 状态，避免逐参数半应用。
    Configuration candidate{};
    candidate.rotation = board_rotation_.get();
    if (!algorithms::make_rotation_matrix(
            candidate.rotation, candidate.rotation_matrix)) {
        return ConfigurationReadResult::Invalid;
    }
    candidate.integration_rate_hz = integration_rate_.get();
    if (!algorithms::supported_integration_rate(
            candidate.integration_rate_hz)) {
        PX4_WARN("unsupported IMU_INTEG_RATE=%ld",
                 static_cast<long>(candidate.integration_rate_hz));
        return ConfigurationReadResult::Invalid;
    }
    const std::int32_t clipping_notifications =
        clipping_notifications_.get();
    if (clipping_notifications != 0 && clipping_notifications != 1) {
        PX4_WARN("unsupported SENS_IMU_CLPNOTI=%ld",
                 static_cast<long>(clipping_notifications));
        return ConfigurationReadResult::Invalid;
    }
    candidate.clipping_notifications = clipping_notifications != 0;
    const std::int32_t imu_autocal = imu_autocal_.get();
    if (imu_autocal != 0 && imu_autocal != 1) {
        PX4_WARN("unsupported SENS_IMU_AUTOCAL=%ld",
                 static_cast<long>(imu_autocal));
        return ConfigurationReadResult::Invalid;
    }

    candidate.accel.configured_device_id = accel_id_.get();
    if (candidate.accel.configured_device_id < 0) {
        ++stats_.calibration_rejections;
        candidate.accel.configured_device_id = 0;
        PX4_WARN("negative CAL_ACC0_ID ignored; using identity calibration");
    }
    candidate.accel.enabled = accel_device_id_ != 0U;
    bool accel_calibration_applied =
        algorithms::calibration_id_matches(
            candidate.accel.configured_device_id, accel_device_id_);
    if (accel_calibration_applied) {
        candidate.accel.offset = {
            accel_x_offset_.get(), accel_y_offset_.get(),
            accel_z_offset_.get()};
        candidate.accel.scale = {
            accel_x_scale_.get(), accel_y_scale_.get(),
            accel_z_scale_.get()};
        if (!algorithms::valid_accel_calibration(candidate.accel)) {
            ++stats_.calibration_rejections;
            candidate.accel.offset = {};
            candidate.accel.scale = {1.0F, 1.0F, 1.0F};
            accel_calibration_applied = false;
            PX4_WARN("invalid CAL_ACC0 values ignored; using identity calibration");
        }
    }

    candidate.gyro.configured_device_id = gyro_id_.get();
    if (candidate.gyro.configured_device_id < 0) {
        ++stats_.calibration_rejections;
        candidate.gyro.configured_device_id = 0;
        PX4_WARN("negative CAL_GYRO0_ID ignored; using identity calibration");
    }
    candidate.gyro.enabled = gyro_device_id_ != 0U;
    bool gyro_calibration_applied =
        algorithms::calibration_id_matches(
            candidate.gyro.configured_device_id, gyro_device_id_);
    if (gyro_calibration_applied) {
        candidate.gyro.offset = {
            gyro_x_offset_.get(), gyro_y_offset_.get(),
            gyro_z_offset_.get()};
        if (!algorithms::valid_gyro_calibration(candidate.gyro)) {
            ++stats_.calibration_rejections;
            candidate.gyro.offset = {};
            gyro_calibration_applied = false;
            PX4_WARN("invalid CAL_GYRO0 values ignored; using identity calibration");
        }
    }

    const bool accel_changed = !algorithms::calibration_equal(
        candidate.accel, active_configuration_.accel);
    const bool gyro_changed = !algorithms::calibration_equal(
        candidate.gyro, active_configuration_.gyro);
    if (accel_calibration_applied) {
        candidate.accel.count = accel_changed
                                    ? algorithms::next_calibration_count(
                                          active_configuration_.accel.count)
                                    : active_configuration_.accel.count;
    }
    if (gyro_calibration_applied) {
        candidate.gyro.count = gyro_changed
                                   ? algorithms::next_calibration_count(
                                         active_configuration_.gyro.count)
                                   : active_configuration_.gyro.count;
    }

    if (((candidate.accel.configured_device_id != 0 &&
          accel_device_id_ != 0U &&
          !accel_calibration_applied) ||
         (candidate.gyro.configured_device_id != 0 &&
          gyro_device_id_ != 0U &&
          !gyro_calibration_applied)) &&
        (accel_changed || gyro_changed)) {
        ++stats_.calibration_rejections;
        /* PX4 treats a missing calibration for the detected device as an
         * enabled identity calibration. Never apply another device's saved
         * offsets, but do not suppress the healthy raw/vehicle IMU path. */
        // CAL_*_ID 不匹配时绝不套用其他设备的 offset/scale，但使用 identity
        // correction 保持健康原始/vehicle_imu 数据链可见，供用户重新校准。
        PX4_WARN("saved IMU calibration unavailable; using identity calibration");
    }

    configuration = candidate;
    return ConfigurationReadResult::Valid;
}

void VehicleImu::process_parameter_update(std::uint32_t instance) noexcept
{
    // 相同配置只确认 generation；不同配置写入唯一 pending 槽，后续相同更新只
    // 刷新 instance。非法更新丢弃 pending 并保留 active 配置。
    Configuration candidate{};
    if (read_configuration(true, candidate) !=
        ConfigurationReadResult::Valid) {
        ++stats_.parameter_update_failures;
        clear_pending_configuration();
        PX4_WARN("invalid IMU parameter update; active configuration retained");
        return;
    }

    if (algorithms::configuration_equal(
            candidate, active_configuration_)) {
        if (configuration_pending_) {
            clear_pending_configuration();
        }
        mark_parameter_update_applied(instance);
        return;
    }
    if (configuration_pending_ && algorithms::configuration_equal(
                                      candidate,
                                      pending_configuration_)) {
        pending_configuration_instance_ = instance;
        return;
    }

    pending_configuration_ = candidate;
    pending_configuration_instance_ = instance;
    configuration_pending_ = true;
    ++stats_.configurations_staged;
}

void VehicleImu::service_pending_configuration() noexcept
{
    /* PX4 VehicleIMU applies correction parameters in the frontend while
     * disarmed. SensorCalibration already owns the global arming interlock
     * for its transaction; acquiring it again here would deadlock the CAL_*
     * applied handshake. */
    // 校准协调器已持有非重入 interlock；前端只检查 disarmed 后直接原子切换，
    // 再发布 applied generation，避免二次加锁造成 CAL_* 永久等待。
    if (!configuration_pending_ || armed_.armed()) return;

    const std::uint32_t applied_instance =
        pending_configuration_instance_;
    apply_configuration(pending_configuration_);
    mark_parameter_update_applied(applied_instance);
    clear_pending_configuration();
    ++stats_.configurations_applied;
}

void VehicleImu::apply_configuration(
    const Configuration &configuration) noexcept
{
    // 校正或积分周期变化后清空所有积分/状态统计窗口，不能跨配置边界拼接 delta。
    const bool accel_calibration_changed =
        !algorithms::calibration_equal(active_configuration_.accel,
                                       configuration.accel);
    const bool gyro_calibration_changed =
        !algorithms::calibration_equal(active_configuration_.gyro,
                                       configuration.gyro);
    if (accel_calibration_changed) {
        learned_accel_calibration_ = {};
    }
    if (gyro_calibration_changed) {
        learned_gyro_calibration_ = {};
    }
    if ((accel_calibration_changed || gyro_calibration_changed) &&
        state_ == dima::middleware::lifecycle::ModuleState::Running) {
        const std::uint64_t now_us = hrt_absolute_time();
        autocal_quiet_until_us_ =
            now_us > UINT64_MAX - kAutocalQuietPeriodUs
                ? UINT64_MAX
                : now_us + kAutocalQuietPeriodUs;
    }
    active_configuration_ = configuration;
    reset_integrators(true);
    reset_status_window();
    have_previous_status_accel_ = false;
    have_previous_status_gyro_ = false;
    accel_vibration_metric_ = 0.0F;
    gyro_vibration_metric_ = 0.0F;
    status_dirty_ = true;
}

void VehicleImu::mark_parameter_update_applied(
    std::uint32_t instance) noexcept
{
    __atomic_store_n(&applied_parameter_update_instance_, instance,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&applied_parameter_update_valid_, true,
                     __ATOMIC_RELEASE);
}

void VehicleImu::clear_pending_configuration() noexcept
{
    pending_configuration_ = Configuration{};
    pending_configuration_instance_ = 0U;
    configuration_pending_ = false;
}

void VehicleImu::clear_learned_calibrations() noexcept
{
    learned_accel_calibration_ = {};
    learned_gyro_calibration_ = {};
}

VehicleImu::Vector3 VehicleImu::bias_to_sensor_frame(
    const float (&bias)[3]) const noexcept
{
    // active rotation 是 sensor->body 的 row-major R。EKF bias 位于机体系，
    // 写回传感器 offset 必须先做 R^T*bias：sensor_j=sum_i R_ij*body_i。
    const float *const rotation = active_configuration_.rotation_matrix;
    return {
        rotation[0] * bias[0] + rotation[3] * bias[1] +
            rotation[6] * bias[2],
        rotation[1] * bias[0] + rotation[4] * bias[1] +
            rotation[7] * bias[2],
        rotation[2] * bias[0] + rotation[5] * bias[1] +
            rotation[8] * bias[2]};
}

void VehicleImu::capture_estimator_bias(std::uint64_t now_us) noexcept
{
    estimator_sensor_bias_s estimator_bias{};
    if (!estimator_sensor_bias_sub_.copy(&estimator_bias) ||
        estimator_bias.timestamp == 0U ||
        now_us < estimator_bias.timestamp ||
        now_us - estimator_bias.timestamp > kBiasFreshnessUs) {
        return;
    }

    if (estimator_bias.accel_bias_valid &&
        estimator_bias.accel_bias_stable &&
        estimator_bias.accel_device_id != 0U &&
        estimator_bias.accel_device_id == accel_device_id_ &&
        active_configuration_.accel.enabled &&
        finite_message_vector(estimator_bias.accel_bias) &&
        finite_message_vector(estimator_bias.accel_bias_variance)) {
        const Vector3 sensor_bias =
            bias_to_sensor_frame(estimator_bias.accel_bias);
        const auto &active = active_configuration_.accel;
        // corrected=(raw-offset)*scale，所以机体系 bias 写回传感器 offset 时：
        // offset_new=offset_old+(R^T*bias)./scale。符号和除 scale 次序与 PX4
        // Accelerometer::BiasCorrectedSensorOffset 完全一致。
        const Vector3 offset{
            active.offset.x + sensor_bias.x / active.scale.x,
            active.offset.y + sensor_bias.y / active.scale.y,
            active.offset.z + sensor_bias.z / active.scale.z};
        if (algorithms::finite_vector(offset)) {
            learned_accel_calibration_.offset = offset;
            learned_accel_calibration_.device_id = accel_device_id_;
            learned_accel_calibration_.valid = true;
        }
    }

    if (estimator_bias.gyro_bias_valid &&
        estimator_bias.gyro_bias_stable &&
        estimator_bias.gyro_device_id != 0U &&
        estimator_bias.gyro_device_id == gyro_device_id_ &&
        active_configuration_.gyro.enabled &&
        finite_message_vector(estimator_bias.gyro_bias) &&
        finite_message_vector(estimator_bias.gyro_bias_variance)) {
        const Vector3 sensor_bias =
            bias_to_sensor_frame(estimator_bias.gyro_bias);
        const auto &active = active_configuration_.gyro;
        // 陀螺校正没有 scale：offset_new=offset_old+R^T*bias。
        const Vector3 offset{active.offset.x + sensor_bias.x,
                             active.offset.y + sensor_bias.y,
                             active.offset.z + sensor_bias.z};
        if (algorithms::finite_vector(offset)) {
            learned_gyro_calibration_.offset = offset;
            learned_gyro_calibration_.device_id = gyro_device_id_;
            learned_gyro_calibration_.valid = true;
        }
    }
}

bool VehicleImu::save_accel_bias() noexcept
{
    if (!learned_accel_calibration_.valid) {
        return true;
    }
    if (learned_accel_calibration_.device_id != accel_device_id_ ||
        accel_device_id_ == 0U) {
        learned_accel_calibration_ = {};
        return true;
    }

    const auto &active = active_configuration_.accel;
    const bool calibrated = algorithms::calibration_id_matches(
        active.configured_device_id, accel_device_id_);
    const Vector3 change{
        learned_accel_calibration_.offset.x - active.offset.x,
        learned_accel_calibration_.offset.y - active.offset.y,
        learned_accel_calibration_.offset.z - active.offset.z};
    // PX4 仅在 offset 变化超过 0.05 m/s^2 或设备尚未校准时保存，避免稳定
    // 小噪声反复磨损参数存储；单实例无需多 EKF 方差加权。
    if (calibrated && vector_norm(change) <= 0.05F) {
        learned_accel_calibration_ = {};
        return true;
    }

    algorithms::Calibration next{active};
    next.offset = learned_accel_calibration_.offset;
    next.configured_device_id = static_cast<std::int32_t>(accel_device_id_);
    if (!algorithms::valid_accel_calibration(next)) {
        learned_accel_calibration_ = {};
        return true;
    }

    const float values[6]{next.offset.x, next.offset.y, next.offset.z,
                          next.scale.x, next.scale.y, next.scale.z};
    return queue_calibration_commit(CalibrationCommitKind::Accel,
                                    accel_device_id_, values, 6U);
}

bool VehicleImu::save_gyro_bias() noexcept
{
    if (!learned_gyro_calibration_.valid) {
        return true;
    }
    if (learned_gyro_calibration_.device_id != gyro_device_id_ ||
        gyro_device_id_ == 0U) {
        learned_gyro_calibration_ = {};
        return true;
    }

    const auto &active = active_configuration_.gyro;
    const bool calibrated = algorithms::calibration_id_matches(
        active.configured_device_id, gyro_device_id_);
    const Vector3 change{
        learned_gyro_calibration_.offset.x - active.offset.x,
        learned_gyro_calibration_.offset.y - active.offset.y,
        learned_gyro_calibration_.offset.z - active.offset.z};
    if (calibrated && vector_norm(change) <= 0.01F) {
        learned_gyro_calibration_ = {};
        return true;
    }

    algorithms::Calibration next{active};
    next.offset = learned_gyro_calibration_.offset;
    next.configured_device_id = static_cast<std::int32_t>(gyro_device_id_);
    if (!algorithms::valid_gyro_calibration(next)) {
        learned_gyro_calibration_ = {};
        return true;
    }

    const float values[3]{next.offset.x, next.offset.y, next.offset.z};
    return queue_calibration_commit(CalibrationCommitKind::Gyro,
                                    gyro_device_id_, values, 3U);
}

bool VehicleImu::parameter_commit_idle() const noexcept
{
    return __atomic_load_n(&parameter_commit_state_, __ATOMIC_ACQUIRE) ==
           static_cast<std::uint8_t>(ParameterCommitState::Idle);
}

bool VehicleImu::queue_calibration_commit(
    CalibrationCommitKind kind, std::uint32_t device_id,
    const float *values, std::size_t value_count) noexcept
{
    const std::size_t expected_count =
        kind == CalibrationCommitKind::Accel
            ? 6U
            : (kind == CalibrationCommitKind::Gyro ? 3U : 0U);
    if (!parameter_commit_idle() || device_id == 0U || values == nullptr ||
        value_count != expected_count) {
        return false;
    }

    CalibrationCommitRequest request{};
    request.kind = kind;
    request.device_id = device_id;
    for (std::size_t index = 0U; index < value_count; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
        request.values[index] = values[index];
    }

    // request 全量写完后再以 release 发布 Pending；lp_default 只有 acquire 成功
    // 后才复制该快照，因此不会读取 sensors 正在修改的 active configuration。
    parameter_commit_request_ = request;
    __atomic_store_n(
        &parameter_commit_state_,
        static_cast<std::uint8_t>(ParameterCommitState::Pending),
        __ATOMIC_RELEASE);
    if (!parameter_commit_worker_.ScheduleNow()) {
        parameter_commit_request_ = {};
        __atomic_store_n(
            &parameter_commit_state_,
            static_cast<std::uint8_t>(ParameterCommitState::Idle),
            __ATOMIC_RELEASE);
        return false;
    }
    return true;
}

void VehicleImu::run_parameter_commit() noexcept
{
    std::uint8_t expected =
        static_cast<std::uint8_t>(ParameterCommitState::Pending);
    if (!__atomic_compare_exchange_n(
            &parameter_commit_state_, &expected,
            static_cast<std::uint8_t>(ParameterCommitState::Running),
            false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
        return;
    }

    // 复制不可变请求后只访问参数 Core；本 worker 不读取 realtime owner 的
    // learned candidate 或 active configuration，参数更新并发到来不会撕裂事务。
    const CalibrationCommitRequest request{parameter_commit_request_};
    bool success = false;
    if (request.kind == CalibrationCommitKind::Accel) {
        const param_t handles[6]{
            param_handle(dima::params::CAL_ACC0_XOFF),
            param_handle(dima::params::CAL_ACC0_YOFF),
            param_handle(dima::params::CAL_ACC0_ZOFF),
            param_handle(dima::params::CAL_ACC0_XSCALE),
            param_handle(dima::params::CAL_ACC0_YSCALE),
            param_handle(dima::params::CAL_ACC0_ZSCALE)};
        success = commit_calibration_parameters(
            param_handle(dima::params::CAL_ACC0_ID), handles,
            request.device_id, request.values);
        if (success) {
            PX4_INFO("Accel %lu autocal offset committed [%.3f %.3f %.3f]",
                     static_cast<unsigned long>(request.device_id),
                     static_cast<double>(request.values[0]),
                     static_cast<double>(request.values[1]),
                     static_cast<double>(request.values[2]));
        }
    } else if (request.kind == CalibrationCommitKind::Gyro) {
        const param_t handles[3]{
            param_handle(dima::params::CAL_GYRO0_XOFF),
            param_handle(dima::params::CAL_GYRO0_YOFF),
            param_handle(dima::params::CAL_GYRO0_ZOFF)};
        const float values[3]{request.values[0], request.values[1],
                              request.values[2]};
        success = commit_calibration_parameters(
            param_handle(dima::params::CAL_GYRO0_ID), handles,
            request.device_id, values);
        if (success) {
            PX4_INFO("Gyro %lu autocal offset committed [%.3f %.3f %.3f]",
                     static_cast<unsigned long>(request.device_id),
                     static_cast<double>(request.values[0]),
                     static_cast<double>(request.values[1]),
                     static_cast<double>(request.values[2]));
        }
    }

    if (!success) {
        PX4_WARN("IMU autocal parameter transaction failed");
    }
    __atomic_store_n(
        &parameter_commit_state_,
        static_cast<std::uint8_t>(
            success ? ParameterCommitState::Succeeded
                    : ParameterCommitState::Failed),
        __ATOMIC_RELEASE);
    // 结果发布后主动唤醒 sensors 消费；若正在 shutdown，owner 已关闭调度，
    // ScheduleNow 会安全失败，stop 仍会在句柄失效前 drain 本 worker。
    (void)ScheduleNow();
}

void VehicleImu::consume_parameter_commit_result(
    std::uint64_t now_us) noexcept
{
    const auto commit_state = static_cast<ParameterCommitState>(
        __atomic_load_n(&parameter_commit_state_, __ATOMIC_ACQUIRE));
    if (commit_state != ParameterCommitState::Succeeded &&
        commit_state != ParameterCommitState::Failed) {
        return;
    }

    const CalibrationCommitKind kind = parameter_commit_request_.kind;
    if (commit_state == ParameterCommitState::Succeeded) {
        if (kind == CalibrationCommitKind::Accel) {
            learned_accel_calibration_ = {};
        } else if (kind == CalibrationCommitKind::Gyro) {
            learned_gyro_calibration_ = {};
        }
        // 成功后 30 s 内不重新采集估计 bias，避免 parameter_update 尚未被前端
        // 应用时又基于旧 offset 生成第二份候选。
        autocal_quiet_until_us_ =
            now_us > UINT64_MAX - kAutocalQuietPeriodUs
                ? UINT64_MAX
                : now_us + kAutocalQuietPeriodUs;
        autocal_retry_after_us_ = 0U;
    } else {
        // 失败保留 learned candidate，并限速到 1 Hz；参数服务短暂不可用不能让
        // 8 kHz IMU 输入把 lp_default 反复唤醒。
        autocal_retry_after_us_ =
            now_us > UINT64_MAX - kAutocalUpdateIntervalUs
                ? UINT64_MAX
                : now_us + kAutocalUpdateIntervalUs;
    }

    parameter_commit_request_ = {};
    __atomic_store_n(
        &parameter_commit_state_,
        static_cast<std::uint8_t>(ParameterCommitState::Idle),
        __ATOMIC_RELEASE);
}

void VehicleImu::reset_parameter_commit() noexcept
{
    parameter_commit_request_ = {};
    __atomic_store_n(
        &parameter_commit_state_,
        static_cast<std::uint8_t>(ParameterCommitState::Idle),
        __ATOMIC_RELEASE);
}

void VehicleImu::service_estimator_bias(
    std::uint64_t now_us, bool parameters_updated) noexcept
{
    consume_parameter_commit_result(now_us);
    if (!parameter_commit_idle()) {
        return;
    }
    if (!imu_autocal_.bound() || imu_autocal_.get() == 0) {
        clear_learned_calibrations();
        return;
    }
    if (parameters_updated || now_us < autocal_quiet_until_us_ ||
        now_us < autocal_retry_after_us_) {
        return;
    }

    const bool accel_calibrated = algorithms::calibration_id_matches(
        active_configuration_.accel.configured_device_id,
        accel_device_id_);
    const bool gyro_calibrated = algorithms::calibration_id_matches(
        active_configuration_.gyro.configured_device_id,
        gyro_device_id_);
    const bool capture_allowed =
        armed_.armed() || !accel_calibrated || !gyro_calibrated;
    const bool capture_due = autocal_last_bias_check_us_ == 0U ||
                             now_us < autocal_last_bias_check_us_ ||
                             now_us - autocal_last_bias_check_us_ >=
                                 kAutocalUpdateIntervalUs;
    if (capture_allowed && capture_due) {
        capture_estimator_bias(now_us);
        autocal_last_bias_check_us_ = now_us;
        return;
    }

    if (!armed_.armed()) {
        // 已武装阶段只缓存 EKF 稳定候选；解除武装后才原子更新参数。此路径不持有
        // ArmedFlash interlock，也不调用 param_save_default，Manual/解锁安全链不受
        // EKF 健康或持久化时延反向门控。
        const bool accel_saved = save_accel_bias();
        const bool gyro_saved =
            parameter_commit_idle() ? save_gyro_bias() : true;
        if (!accel_saved || !gyro_saved) {
            PX4_WARN("IMU autocal write rejected; candidate retained");
            autocal_retry_after_us_ =
                now_us > UINT64_MAX - kAutocalUpdateIntervalUs
                    ? UINT64_MAX
                    : now_us + kAutocalUpdateIntervalUs;
        } else {
            autocal_retry_after_us_ = 0U;
        }
    }
}

VehicleImu::Vector3 VehicleImu::correct_accel(
    const sensor_accel_s &sample) const noexcept
{
    return algorithms::correct_accel(
        {sample.x, sample.y, sample.z}, active_configuration_.accel,
        active_configuration_.rotation_matrix);
}

VehicleImu::Vector3 VehicleImu::correct_gyro(
    const sensor_gyro_s &sample) const noexcept
{
    return algorithms::correct_gyro(
        {sample.x, sample.y, sample.z}, active_configuration_.gyro,
        active_configuration_.rotation_matrix);
}

bool VehicleImu::select_accel_device(std::uint32_t device_id) noexcept
{
    if (device_id == 0U) return false;
    if (accel_device_id_ != 0U) return accel_device_id_ == device_id;

    accel_device_id_ = device_id;
    Configuration configuration{};
    if (read_configuration(false, configuration) !=
        ConfigurationReadResult::Valid) {
        accel_device_id_ = 0U;
        PX4_ERR("accel device selection rejected device_id=%lu",
                static_cast<unsigned long>(device_id));
        return false;
    }
    accel_validator_.reset();
    apply_configuration(configuration);
    ++stats_.accel_device_selections;
    PX4_INFO("accel selected device_id=%lu",
             static_cast<unsigned long>(device_id));
    return true;
}

bool VehicleImu::select_gyro_device(std::uint32_t device_id) noexcept
{
    if (device_id == 0U) return false;
    if (gyro_device_id_ != 0U) return gyro_device_id_ == device_id;

    gyro_device_id_ = device_id;
    Configuration configuration{};
    if (read_configuration(false, configuration) !=
        ConfigurationReadResult::Valid) {
        gyro_device_id_ = 0U;
        PX4_ERR("gyro device selection rejected device_id=%lu",
                static_cast<unsigned long>(device_id));
        return false;
    }
    gyro_validator_.reset();
    apply_configuration(configuration);
    ++stats_.gyro_device_selections;
    PX4_INFO("gyro selected device_id=%lu",
             static_cast<unsigned long>(device_id));
    return true;
}

bool VehicleImu::process_accel(const sensor_accel_s &sample) noexcept
{
    // 先做结构/device/timestamp/有限值校验，再更新流健康，最后才校正和积分；
    // 任何拒绝都会重置 accel+gyro 的共同积分窗口，防止发布不同时间段的组合量。
    (void)select_accel_device(sample.device_id);
    const validation::ImuSample input = validation_sample(sample);
    const validation::ImuValidationResult structure =
        validation::validate_imu_sample(input, accel_device_id_);
    if (!structure.valid()) {
        if (accel_validation_error_count_ != UINT32_MAX) {
            ++accel_validation_error_count_;
        }
        const std::uint32_t previous_error_count =
            latest_accel_error_count_;
        latest_accel_error_count_ = saturating_sum(
            sample.error_count, accel_validation_error_count_);
        accel_validator_.reject(
            (structure.failure_mask & validation::ImuFailureTimestamp) != 0U
                ? validation::StreamFailureTimestamp
                : validation::StreamFailureInvalidValue,
            0U);
        ++stats_.validation_rejections;
        status_dirty_ = status_dirty_ ||
                        latest_accel_error_count_ != previous_error_count;
        reset_integrators(true);
        return false;
    }
    const float values[3]{sample.x, sample.y, sample.z};
    const std::uint32_t previous_error_count = latest_accel_error_count_;
    latest_accel_error_count_ = saturating_sum(
        sample.error_count, accel_validation_error_count_);
    status_dirty_ = status_dirty_ ||
                    latest_accel_error_count_ != previous_error_count;
    if (!accel_validator_.put(sample.timestamp, values,
                              latest_accel_error_count_)) {
        ++stats_.validation_rejections;
        status_dirty_ = true;
        reset_integrators(true);
        return false;
    }
    const Vector3 corrected = correct_accel(sample);
    ++stats_.accel_updates;
    accumulate_accel_status(sample, corrected);
    if (!accel_validator_.evaluate(hrt_absolute_time()).healthy()) {
        reset_integrators(true);
        return false;
    }
    const algorithms::SampleTimeStep time_step =
        algorithms::classify_sample_time(
            last_accel_timestamp_us_, sample.timestamp_sample,
            kMaximumSampleGapUs);
    if (time_step.action == algorithms::SampleTimeAction::Prime) {
        last_accel_ = corrected;
        last_accel_timestamp_us_ = sample.timestamp_sample;
        return true;
    }
    if (time_step.action == algorithms::SampleTimeAction::Reset) {
        ++stats_.timestamp_gaps;
        reset_integrators(true);
        last_accel_ = corrected;
        last_accel_timestamp_us_ = sample.timestamp_sample;
        return true;
    }

    // delta_velocity 使用梯形积分，accel_integral_dt 以饱和加法累计微秒。
    const std::uint32_t dt_us = time_step.dt_us;
    const Vector3 delta =
        algorithms::trapezoid_delta(last_accel_, corrected, dt_us);
    accel_integral_ = algorithms::add(accel_integral_, delta);
    accel_integral_dt_us_ =
        dt_us > UINT32_MAX - accel_integral_dt_us_
            ? UINT32_MAX
            : accel_integral_dt_us_ + dt_us;
    latest_accel_update_us_ = dt_us;
    last_accel_ = corrected;
    last_accel_timestamp_us_ = sample.timestamp_sample;
    if (sample.clip_counter[0] != 0U) {
        accel_clipping_ |= vehicle_imu_s::CLIPPING_X;
    }
    if (sample.clip_counter[1] != 0U) {
        accel_clipping_ |= vehicle_imu_s::CLIPPING_Y;
    }
    if (sample.clip_counter[2] != 0U) {
        accel_clipping_ |= vehicle_imu_s::CLIPPING_Z;
    }
    return true;
}

bool VehicleImu::process_gyro(const sensor_gyro_s &sample) noexcept
{
    // gyro 路径与 accel 使用相同三层边界，并额外累计 coning correction。
    (void)select_gyro_device(sample.device_id);
    const validation::ImuSample input = validation_sample(sample);
    const validation::ImuValidationResult structure =
        validation::validate_imu_sample(input, gyro_device_id_);
    if (!structure.valid()) {
        if (gyro_validation_error_count_ != UINT32_MAX) {
            ++gyro_validation_error_count_;
        }
        const std::uint32_t previous_error_count =
            latest_gyro_error_count_;
        latest_gyro_error_count_ = saturating_sum(
            sample.error_count, gyro_validation_error_count_);
        gyro_validator_.reject(
            (structure.failure_mask & validation::ImuFailureTimestamp) != 0U
                ? validation::StreamFailureTimestamp
                : validation::StreamFailureInvalidValue,
            0U);
        ++stats_.validation_rejections;
        status_dirty_ = status_dirty_ ||
                        latest_gyro_error_count_ != previous_error_count;
        reset_integrators(true);
        return false;
    }
    const float values[3]{sample.x, sample.y, sample.z};
    const std::uint32_t previous_error_count = latest_gyro_error_count_;
    latest_gyro_error_count_ = saturating_sum(
        sample.error_count, gyro_validation_error_count_);
    status_dirty_ = status_dirty_ ||
                    latest_gyro_error_count_ != previous_error_count;
    if (!gyro_validator_.put(sample.timestamp, values,
                             latest_gyro_error_count_)) {
        ++stats_.validation_rejections;
        status_dirty_ = true;
        reset_integrators(true);
        return false;
    }
    const Vector3 corrected = correct_gyro(sample);
    ++stats_.gyro_updates;
    accumulate_gyro_status(sample, corrected);
    if (!gyro_validator_.evaluate(hrt_absolute_time()).healthy()) {
        reset_integrators(true);
        return false;
    }
    const algorithms::SampleTimeStep time_step =
        algorithms::classify_sample_time(
            last_gyro_timestamp_us_, sample.timestamp_sample,
            kMaximumSampleGapUs);
    if (time_step.action == algorithms::SampleTimeAction::Prime) {
        last_gyro_ = corrected;
        last_gyro_timestamp_us_ = sample.timestamp_sample;
        return true;
    }
    if (time_step.action == algorithms::SampleTimeAction::Reset) {
        ++stats_.timestamp_gaps;
        reset_integrators(true);
        last_gyro_ = corrected;
        last_gyro_timestamp_us_ = sample.timestamp_sample;
        return true;
    }

    const std::uint32_t dt_us = time_step.dt_us;
    const Vector3 delta =
        algorithms::trapezoid_delta(last_gyro_, corrected, dt_us);
    // delta_angle=梯形积分；coning 修正按相邻小转角叉积累计，最终发布时相加。
    const Vector3 coning_increment = algorithms::coning_increment(
        last_angle_integral_, last_delta_angle_, delta);
    coning_correction_ = algorithms::add(coning_correction_,
                                         coning_increment);
    const float dt_s = static_cast<float>(dt_us) * 1.0e-6F;
    coning_metric_accumulator_ += vector_norm(coning_increment) * dt_s;
    coning_metric_time_s_ += dt_s;
    last_delta_angle_ = delta;
    last_angle_integral_ = gyro_integral_;
    gyro_integral_ = algorithms::add(gyro_integral_, delta);
    gyro_integral_dt_us_ =
        dt_us > UINT32_MAX - gyro_integral_dt_us_
            ? UINT32_MAX
            : gyro_integral_dt_us_ + dt_us;
    latest_gyro_update_us_ = dt_us;
    last_gyro_ = corrected;
    last_gyro_timestamp_us_ = sample.timestamp_sample;
    if (sample.clip_counter[0] != 0U) {
        gyro_clipping_ |= vehicle_imu_s::CLIPPING_X;
    }
    if (sample.clip_counter[1] != 0U) {
        gyro_clipping_ |= vehicle_imu_s::CLIPPING_Y;
    }
    if (sample.clip_counter[2] != 0U) {
        gyro_clipping_ |= vehicle_imu_s::CLIPPING_Z;
    }
    return true;
}

void VehicleImu::accumulate_accel_status(
    const sensor_accel_s &sample, const Vector3 &value) noexcept
{
    if (accel_status_updates_ == 0U) {
        accel_status_first_us_ = sample.timestamp_sample;
        accel_status_first_samples_ = sample.samples;
    }
    accel_status_last_us_ = sample.timestamp_sample;
    if (accel_status_updates_ != UINT32_MAX) {
        ++accel_status_updates_;
    }
    saturating_add(accel_status_raw_samples_, sample.samples);
    update_moments(accel_status_moments_.mean,
                   accel_status_moments_.m2,
                   accel_status_moments_.count, value);

    // vibration_metric=0.99*old+0.01*|current-previous|，是样本差分 EWMA，
    // 不是频谱分析或板端机械振动结论。
    if (have_previous_status_accel_) {
        const Vector3 difference{
            value.x - previous_status_accel_.x,
            value.y - previous_status_accel_.y,
            value.z - previous_status_accel_.z};
        accel_vibration_metric_ = 0.99F * accel_vibration_metric_ +
                                   0.01F * vector_norm(difference);
    }
    previous_status_accel_ = value;
    have_previous_status_accel_ = true;

    accel_temperature_sum_ += sample.temperature;
    if (accel_temperature_count_ != UINT32_MAX) {
        ++accel_temperature_count_;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        saturating_add(accel_clipping_total_[axis],
                       sample.clip_counter[axis]);
        status_dirty_ = status_dirty_ || sample.clip_counter[axis] != 0U;
    }
}

void VehicleImu::accumulate_gyro_status(
    const sensor_gyro_s &sample, const Vector3 &value) noexcept
{
    if (gyro_status_updates_ == 0U) {
        gyro_status_first_us_ = sample.timestamp_sample;
        gyro_status_first_samples_ = sample.samples;
    }
    gyro_status_last_us_ = sample.timestamp_sample;
    if (gyro_status_updates_ != UINT32_MAX) {
        ++gyro_status_updates_;
    }
    saturating_add(gyro_status_raw_samples_, sample.samples);
    update_moments(gyro_status_moments_.mean,
                   gyro_status_moments_.m2,
                   gyro_status_moments_.count, value);

    if (have_previous_status_gyro_) {
        const Vector3 difference{
            value.x - previous_status_gyro_.x,
            value.y - previous_status_gyro_.y,
            value.z - previous_status_gyro_.z};
        gyro_vibration_metric_ = 0.99F * gyro_vibration_metric_ +
                                  0.01F * vector_norm(difference);
    }
    previous_status_gyro_ = value;
    have_previous_status_gyro_ = true;

    gyro_temperature_sum_ += sample.temperature;
    if (gyro_temperature_count_ != UINT32_MAX) {
        ++gyro_temperature_count_;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        saturating_add(gyro_clipping_total_[axis],
                       sample.clip_counter[axis]);
        status_dirty_ = status_dirty_ || sample.clip_counter[axis] != 0U;
    }
}

void VehicleImu::publish_status(std::uint64_t now_us, bool force) noexcept
{
    if (accel_status_moments_.count == 0U &&
        gyro_status_moments_.count == 0U) {
        return;
    }
    const bool clock_rollback = last_status_publish_us_ != 0U &&
                                now_us < last_status_publish_us_;
    const std::uint64_t elapsed = last_status_publish_us_ == 0U ||
                                  clock_rollback
                                      ? UINT64_MAX
                                      : now_us - last_status_publish_us_;
    // 正常 1 Hz 发布；错误/clip 使 status_dirty 后最短 100 ms 提前发布，兼顾
    // 故障可见性与 MAVLink/uORB 负载。
    const bool periodic = elapsed >= kStatusPublishIntervalUs;
    const bool early = status_dirty_ && elapsed >= kStatusMinimumIntervalUs;
    if (!force && !periodic && !early) {
        return;
    }

    vehicle_imu_status_s status{};
    status.timestamp = now_us;
    status.accel_device_id = accel_device_id_;
    status.gyro_device_id = gyro_device_id_;
    status.accel_error_count = latest_accel_error_count_;
    status.gyro_error_count = latest_gyro_error_count_;
    status.accel_rate_hz = update_rate(
        accel_status_updates_, accel_status_first_us_,
        accel_status_last_us_);
    status.gyro_rate_hz = update_rate(
        gyro_status_updates_, gyro_status_first_us_,
        gyro_status_last_us_);
    status.accel_raw_rate_hz = raw_rate(
        accel_status_raw_samples_, accel_status_first_samples_,
        accel_status_first_us_, accel_status_last_us_);
    status.gyro_raw_rate_hz = raw_rate(
        gyro_status_raw_samples_, gyro_status_first_samples_,
        gyro_status_first_us_, gyro_status_last_us_);
    status.accel_vibration_metric = accel_vibration_metric_;
    status.gyro_vibration_metric = gyro_vibration_metric_;
    // coning metric=累计 |coning_increment|*dt / 累计 dt，单位 rad；温度用窗口
    // 算术平均，无有效温度时输出 NaN。
    status.delta_angle_coning_metric = coning_metric_time_s_ > 0.0F
        ? coning_metric_accumulator_ / coning_metric_time_s_
        : 0.0F;

    const Vector3 &accel_mean = accel_status_moments_.mean;
    const Vector3 &gyro_mean = gyro_status_moments_.mean;
    const Vector3 &accel_m2 = accel_status_moments_.m2;
    const Vector3 &gyro_m2 = gyro_status_moments_.m2;
    status.mean_accel[0] = accel_mean.x;
    status.mean_accel[1] = accel_mean.y;
    status.mean_accel[2] = accel_mean.z;
    status.mean_gyro[0] = gyro_mean.x;
    status.mean_gyro[1] = gyro_mean.y;
    status.mean_gyro[2] = gyro_mean.z;
    status.var_accel[0] = variance(
        accel_m2.x, accel_status_moments_.count);
    status.var_accel[1] = variance(
        accel_m2.y, accel_status_moments_.count);
    status.var_accel[2] = variance(
        accel_m2.z, accel_status_moments_.count);
    status.var_gyro[0] = variance(
        gyro_m2.x, gyro_status_moments_.count);
    status.var_gyro[1] = variance(
        gyro_m2.y, gyro_status_moments_.count);
    status.var_gyro[2] = variance(
        gyro_m2.z, gyro_status_moments_.count);

    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    status.temperature_accel = accel_temperature_count_ == 0U
        ? unavailable
        : accel_temperature_sum_ /
              static_cast<float>(accel_temperature_count_);
    status.temperature_gyro = gyro_temperature_count_ == 0U
        ? unavailable
        : gyro_temperature_sum_ /
              static_cast<float>(gyro_temperature_count_);
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        status.accel_clipping[axis] = accel_clipping_total_[axis];
        status.gyro_clipping[axis] = gyro_clipping_total_[axis];
    }

    if (vehicle_imu_status_pub_.publish(status)) {
        ++stats_.status_publications;
        last_status_publish_us_ = now_us;
        status_dirty_ = false;
        reset_status_window();
    } else {
        ++stats_.status_publication_failures;
        status_dirty_ = true;
    }
}

void VehicleImu::reset_status_window() noexcept
{
    accel_status_moments_ = {};
    gyro_status_moments_ = {};
    accel_status_first_us_ = 0U;
    accel_status_last_us_ = 0U;
    gyro_status_first_us_ = 0U;
    gyro_status_last_us_ = 0U;
    accel_status_updates_ = 0U;
    gyro_status_updates_ = 0U;
    accel_status_raw_samples_ = 0U;
    gyro_status_raw_samples_ = 0U;
    accel_status_first_samples_ = 0U;
    gyro_status_first_samples_ = 0U;
    accel_temperature_sum_ = 0.0F;
    gyro_temperature_sum_ = 0.0F;
    accel_temperature_count_ = 0U;
    gyro_temperature_count_ = 0U;
    coning_metric_accumulator_ = 0.0F;
    coning_metric_time_s_ = 0.0F;
}

void VehicleImu::update_health_state(std::uint64_t now_us) noexcept
{
    const validation::StreamValidity accel =
        accel_validator_.evaluate(now_us);
    const validation::StreamValidity gyro =
        gyro_validator_.evaluate(now_us);
    // failure_mask 低 8 bit 为 accel 流，高 8 bit 为 gyro 流，bit16/17 表示
    // 对应设备前端未启用；只在健康->故障边沿记录一次。
    std::uint32_t failure_mask = accel.failure_mask |
                                 (gyro.failure_mask << 8U);
    if (!active_configuration_.accel.enabled) {
        failure_mask |= 1U << 16U;
    }
    if (!active_configuration_.gyro.enabled) {
        failure_mask |= 1U << 17U;
    }
    if (failure_mask == 0U) {
        validation_fault_active_ = false;
        return;
    }
    if (validation_fault_active_) {
        return;
    }
    validation_fault_active_ = true;
    PX4_ERR("IMU validation failed mask=0x%lx accel_err=%lu gyro_err=%lu",
            static_cast<unsigned long>(failure_mask),
            static_cast<unsigned long>(latest_accel_error_count_),
            static_cast<unsigned long>(latest_gyro_error_count_));
}

bool VehicleImu::publish_if_ready() noexcept
{
    // IMU_INTEG_RATE 只定义 vehicle_imu 的积分窗口，不改变 ICM42688P
    // 的 8 kHz ODR；例如 400 Hz 对应约 2500 us 的积分周期。
    const std::uint32_t interval_us = algorithms::integration_interval_us(
        active_configuration_.integration_rate_hz);
    if (!algorithms::integration_ready(
            accel_integral_dt_us_, latest_accel_update_us_, interval_us) ||
        !algorithms::integration_ready(
            gyro_integral_dt_us_, latest_gyro_update_us_, interval_us)) {
        return false;
    }

    const std::uint64_t now_us = hrt_absolute_time();
    const bool clipped = accel_clipping_ != 0U || gyro_clipping_ != 0U;
    if (active_configuration_.clipping_notifications && clipped &&
        !clipping_fault_active_) {
        PX4_WARN("IMU clipping accel=0x%02x gyro=0x%02x",
                 accel_clipping_, gyro_clipping_);
        clipping_fault_active_ = true;
        ++stats_.clipping_warnings;
    } else if (!clipped) {
        clipping_fault_active_ = false;
    }

    if (!active_configuration_.accel.enabled ||
        !active_configuration_.gyro.enabled ||
        !accel_validator_.evaluate(now_us).healthy() ||
        !gyro_validator_.evaluate(now_us).healthy()) {
        reset_integrators(true);
        return true;
    }

    // 输出 delta_angle=gyro_integral+coning_correction（rad），delta_velocity 为
    // accel_integral（m/s），各自 dt 保留真实积分微秒而非强制标称周期。
    const Vector3 delta_angle =
        algorithms::add(gyro_integral_, coning_correction_);
    vehicle_imu_s message{};
    message.timestamp = now_us;
    message.timestamp_sample = last_gyro_timestamp_us_;
    message.accel_device_id = accel_device_id_;
    message.gyro_device_id = gyro_device_id_;
    message.delta_angle[0] = delta_angle.x;
    message.delta_angle[1] = delta_angle.y;
    message.delta_angle[2] = delta_angle.z;
    message.delta_velocity[0] = accel_integral_.x;
    message.delta_velocity[1] = accel_integral_.y;
    message.delta_velocity[2] = accel_integral_.z;
    message.delta_angle_dt = gyro_integral_dt_us_;
    message.delta_velocity_dt = accel_integral_dt_us_;
    message.delta_angle_clipping = gyro_clipping_;
    message.delta_velocity_clipping = accel_clipping_;
    message.accel_calibration_count =
        active_configuration_.accel.count;
    message.gyro_calibration_count =
        active_configuration_.gyro.count;

    if (vehicle_imu_pub_.publish(message)) {
        ++stats_.publications;
    } else {
        ++stats_.publication_failures;
    }
    reset_integrators(false);
    return true;
}

void VehicleImu::reset_integrators(bool reset_last_samples) noexcept
{
    // 正常发布后保留 last sample 作为下一窗口梯形左端点；时间跳变、配置变化或
    // 校验失败时 reset_last_samples=true，连左端点一起清除。
    accel_integral_ = {};
    gyro_integral_ = {};
    coning_correction_ = {};
    last_angle_integral_ = {};
    accel_integral_dt_us_ = 0U;
    gyro_integral_dt_us_ = 0U;
    latest_accel_update_us_ = 0U;
    latest_gyro_update_us_ = 0U;
    accel_clipping_ = 0U;
    gyro_clipping_ = 0U;
    if (reset_last_samples) {
        last_accel_ = {};
        last_gyro_ = {};
        last_delta_angle_ = {};
        last_accel_timestamp_us_ = 0U;
        last_gyro_timestamp_us_ = 0U;
    }
}

void VehicleImu::fail_module(const char *reason) noexcept
{
    clear_pending_configuration();
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    PX4_ERR("%s", reason);
}

void VehicleImu::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s parameter_update{};
    const bool parameters_updated =
        parameter_update_sub_.copy(&parameter_update);
    if (parameters_updated) {
        process_parameter_update(parameter_update.instance);
    }
    service_pending_configuration();

    // 每轮最多消费 8 组 gyro/accel 更新，防止高频 FIFO 数据长期占满 sensors
    // WorkQueue；每组后尝试一次积分发布，20 ms 备份调度保证遗漏回调仍可推进。
    for (std::size_t update = 0U; update < kMaximumUpdatesPerRun; ++update) {
        bool progressed = false;
        sensor_gyro_s gyro{};
        if (gyro_sub_.copy(&gyro)) {
            progressed = true;
            (void)process_gyro(gyro);
        }
        sensor_accel_s accel{};
        if (accel_sub_.copy(&accel)) {
            progressed = true;
            (void)process_accel(accel);
        }
        (void)publish_if_ready();
        if (!progressed) {
            break;
        }
    }

    const std::uint64_t health_now_us = hrt_absolute_time();
    update_health_state(health_now_us);
    publish_status(health_now_us);
    service_estimator_bias(health_now_us, parameters_updated);

    if (!ScheduleDelayed(kBackupScheduleUs)) {
        fail_module("backup scheduling failed");
    }
}

} // namespace dima::modules::sensors
