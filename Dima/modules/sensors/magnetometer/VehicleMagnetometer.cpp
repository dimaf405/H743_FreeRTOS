/****************************************************************************
 * PX4-Autopilot v1.17.0 VehicleMagnetometer single-device frontend adapted
 * for Dima.
 * Upstream: src/modules/sensors/vehicle_magnetometer and
 *           src/lib/sensor_calibration/Magnetometer.cpp
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#define MODULE_NAME "vehicle_magnetometer"
#include "VehicleMagnetometer.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::modules::sensors {
namespace {

std::uint8_t next_calibration_count(std::uint8_t current) noexcept
{
    // calibration_count 表示当前设备校正发生变化，饱和到 255 而不回绕。
    return current == UINT8_MAX
        ? UINT8_MAX : static_cast<std::uint8_t>(current + 1U);
}

} // namespace

VehicleMagnetometer::VehicleMagnetometer(
    dima::platform::ArmedFlashCoordinator &armed) noexcept
    : px4::ScheduledWorkItem("vehicle_magnetometer",
                            px4::wq_configurations::sensors),
      armed_(armed)
{
}

bool VehicleMagnetometer::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    stats_ = Stats{};
    active_device_id_ = 0U;
    calibration_count_ = 0U;
    clear_pending_configuration();
    __atomic_store_n(&applied_parameter_update_instance_, 0U,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&applied_parameter_update_valid_, false,
                     __ATOMIC_RELEASE);
    invalid_saved_calibration_reported_ = false;
    reset_accumulator(true);

    // 完整参数配置与 publication advertise 成功后才注册 sensor/parameter 回调，
    // 避免半启动前端丢失或错误发布原始样本。
    Configuration initial{};
    if (!bind_parameters() || !read_configuration(false, initial) ||
        !vehicle_magnetometer_publication_.advertise()) {
        invalidate_parameters();
        ScheduleCancelAndDrain();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("parameter binding or uORB advertisement failed");
        return false;
    }
    apply_configuration(initial);

    if (!sensor_mag_subscription_.registerCallback()) {
        invalidate_parameters();
        ScheduleCancelAndDrain();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("sensor_mag callback registration failed");
        return false;
    }
    if (!parameter_update_subscription_.registerCallback()) {
        sensor_mag_subscription_.unregisterCallback();
        invalidate_parameters();
        ScheduleCancelAndDrain();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("parameter callback registration failed");
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleNow()) {
        parameter_update_subscription_.unregisterCallback();
        sensor_mag_subscription_.unregisterCallback();
        invalidate_parameters();
        ScheduleCancelAndDrain();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    return true;
}

void VehicleMagnetometer::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    parameter_update_subscription_.unregisterCallback();
    sensor_mag_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    invalidate_parameters();
    active_configuration_ = Configuration{};
    active_correction_ = Calibration{};
    active_device_id_ = 0U;
    calibration_count_ = 0U;
    clear_pending_configuration();
    __atomic_store_n(&applied_parameter_update_instance_, 0U,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&applied_parameter_update_valid_, false,
                     __ATOMIC_RELEASE);
    invalid_saved_calibration_reported_ = false;
    reset_accumulator(true);
}

dima::middleware::lifecycle::ModuleState VehicleMagnetometer::state() const
{
    return state_;
}

bool VehicleMagnetometer::calibration_parameter_update_applied(
    std::uint32_t required_instance) const noexcept
{
    if (!__atomic_load_n(&applied_parameter_update_valid_,
                         __ATOMIC_ACQUIRE)) {
        return false;
    }
    const std::uint32_t applied = __atomic_load_n(
        &applied_parameter_update_instance_, __ATOMIC_ACQUIRE);
    // 以 32-bit 模空间的有符号差比较 generation，兼容 instance 回绕。
    return static_cast<std::int32_t>(applied - required_instance) >= 0;
}

bool VehicleMagnetometer::mag_calibration_matches(
    std::int32_t configured_device_id,
    const float (&values)[6]) const noexcept
{
    const Calibration &configured = active_configuration_.calibration;
    if (configured.configured_device_id != configured_device_id) {
        return false;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        if (configured.offset[axis] != values[axis] ||
            configured.scale[axis] != values[axis + 3U]) {
            return false;
        }
    }

    const bool saved_calibration_applied = configured_device_id > 0 &&
        active_device_id_ != 0U &&
        static_cast<std::uint32_t>(configured_device_id) ==
            active_device_id_ &&
        valid_saved_calibration(configured);
    if (saved_calibration_applied) {
        Calibration expected = configured;
        expected.saved = true;
        return same_correction(active_correction_, expected);
    }
    return same_correction(active_correction_, Calibration{});
}

bool VehicleMagnetometer::bind_parameters() noexcept
{
    return publication_rate_.bind() && calibration_id_.bind() &&
           calibration_rotation_.bind() && x_offset_.bind() &&
           y_offset_.bind() && z_offset_.bind() && x_scale_.bind() &&
           y_scale_.bind() && z_scale_.bind();
}

void VehicleMagnetometer::invalidate_parameters() noexcept
{
    publication_rate_.invalidate();
    calibration_id_.invalidate();
    calibration_rotation_.invalidate();
    x_offset_.invalidate();
    y_offset_.invalidate();
    z_offset_.invalidate();
    x_scale_.invalidate();
    y_scale_.invalidate();
    z_scale_.invalidate();
}

bool VehicleMagnetometer::refresh_parameter_cache() noexcept
{
    bool refreshed = publication_rate_.update();
    refreshed = calibration_id_.update() && refreshed;
    refreshed = calibration_rotation_.update() && refreshed;
    refreshed = x_offset_.update() && refreshed;
    refreshed = y_offset_.update() && refreshed;
    refreshed = z_offset_.update() && refreshed;
    refreshed = x_scale_.update() && refreshed;
    refreshed = y_scale_.update() && refreshed;
    refreshed = z_scale_.update() && refreshed;
    return refreshed;
}

bool VehicleMagnetometer::read_configuration(
    bool refresh, Configuration &configuration) noexcept
{
    if (refresh && !refresh_parameter_cache()) return false;

    Configuration candidate{};
    candidate.publication_rate_hz = publication_rate_.get();
    candidate.calibration.configured_device_id = calibration_id_.get();
    candidate.calibration.rotation = calibration_rotation_.get();
    candidate.calibration.offset[0] = x_offset_.get();
    candidate.calibration.offset[1] = y_offset_.get();
    candidate.calibration.offset[2] = z_offset_.get();
    candidate.calibration.scale[0] = x_scale_.get();
    candidate.calibration.scale[1] = y_scale_.get();
    candidate.calibration.scale[2] = z_scale_.get();

    // SENS_MAG_RATE 只允许 1..200 Hz；CAL_MAG0_* 的完整有效性要结合实际
    // device_id 在 correction_for_device/configure_device 中判断。
    if (!std::isfinite(candidate.publication_rate_hz) ||
        candidate.publication_rate_hz < 1.0F ||
        candidate.publication_rate_hz > 200.0F ||
        candidate.calibration.configured_device_id < 0) {
        return false;
    }
    configuration = candidate;
    return true;
}

bool VehicleMagnetometer::valid_saved_calibration(
    const Calibration &calibration) noexcept
{
    // 保存校准必须有正 device ID、合法旋转、有限 offset 和 0.1..3.0 scale。
    if (calibration.configured_device_id <= 0 ||
        !dima::lib::sensors::valid_rotation(calibration.rotation)) {
        return false;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        if (!std::isfinite(calibration.offset[axis]) ||
            !std::isfinite(calibration.scale[axis]) ||
            calibration.scale[axis] < 0.1F ||
            calibration.scale[axis] > 3.0F) {
            return false;
        }
    }
    return true;
}

bool VehicleMagnetometer::same_correction(
    const Calibration &left, const Calibration &right) noexcept
{
    if (left.configured_device_id != right.configured_device_id ||
        left.rotation != right.rotation || left.saved != right.saved) {
        return false;
    }
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        if (left.offset[axis] != right.offset[axis] ||
            left.scale[axis] != right.scale[axis]) {
            return false;
        }
    }
    return true;
}

bool VehicleMagnetometer::same_configuration(
    const Configuration &left, const Configuration &right) noexcept
{
    return left.publication_rate_hz == right.publication_rate_hz &&
           same_correction(left.calibration, right.calibration);
}

VehicleMagnetometer::Calibration
VehicleMagnetometer::correction_for_device(
    const Configuration &configuration, std::uint32_t device_id) noexcept
{
    // 只有保存 ID 与当前 device_id 精确匹配且数值完整合法才选择保存校准；
    // 不匹配/无效统一退化为 identity，绝不套用另一磁力计的偏置。
    const Calibration &saved = configuration.calibration;
    if (device_id != 0U && saved.configured_device_id > 0 &&
        static_cast<std::uint32_t>(saved.configured_device_id) == device_id &&
        valid_saved_calibration(saved)) {
        Calibration selected = saved;
        selected.saved = true;
        return selected;
    }
    return Calibration{};
}

void VehicleMagnetometer::process_parameter_update(
    std::uint32_t instance) noexcept
{
    // 与 active 相同只确认 generation；不同值进入唯一 pending 槽，非法更新
    // 保留当前配置，避免逐参数半应用。
    Configuration candidate{};
    if (!read_configuration(true, candidate)) {
        ++stats_.parameter_update_failures;
        clear_pending_configuration();
        PX4_WARN("invalid magnetometer parameter update; active configuration retained");
        return;
    }
    if (same_configuration(candidate, active_configuration_)) {
        clear_pending_configuration();
        mark_parameter_update_applied(instance);
        return;
    }
    if (configuration_pending_ && same_configuration(
            candidate, pending_configuration_)) {
        pending_configuration_instance_ = instance;
        return;
    }
    pending_configuration_ = candidate;
    pending_configuration_instance_ = instance;
    configuration_pending_ = true;
}

void VehicleMagnetometer::service_pending_configuration() noexcept
{
    /* PX4 VehicleMagnetometer updates correction parameters while disarmed.
     * SensorCalibration already owns the arming interlock, so this frontend
     * must not attempt to acquire the same non-reentrant lock again. */
    // SensorCalibration 已持有全局锁，前端只检查 disarmed 并直接应用，再确认
    // parameter_update instance；重复获取非重入锁会使 QGC 校准永久等待。
    if (!configuration_pending_ || armed_.armed()) return;
    const std::uint32_t applied_instance =
        pending_configuration_instance_;
    apply_configuration(pending_configuration_);
    mark_parameter_update_applied(applied_instance);
    clear_pending_configuration();
}

void VehicleMagnetometer::apply_configuration(
    const Configuration &configuration) noexcept
{
    active_configuration_ = configuration;
    // 输出间隔=floor(1e6/rate_hz) us；设备硬件采样率不受此参数影响。
    publication_interval_us_ = static_cast<std::uint32_t>(
        1000000.0F / active_configuration_.publication_rate_hz);
    configure_device(active_device_id_);
}

void VehicleMagnetometer::mark_parameter_update_applied(
    std::uint32_t instance) noexcept
{
    __atomic_store_n(&applied_parameter_update_instance_, instance,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&applied_parameter_update_valid_, true,
                     __ATOMIC_RELEASE);
}

void VehicleMagnetometer::clear_pending_configuration() noexcept
{
    pending_configuration_ = Configuration{};
    pending_configuration_instance_ = 0U;
    configuration_pending_ = false;
}

void VehicleMagnetometer::configure_device(std::uint32_t device_id) noexcept
{
    // 设备或实际 correction 变化才重建旋转/清积分；相同配置不增加
    // calibration_count。新设备有保存校准时 count 从 1 开始。
    const bool device_changed = device_id != active_device_id_;
    const Calibration next = correction_for_device(
        active_configuration_, device_id);
    const bool correction_changed = device_changed ||
        !same_correction(next, active_correction_);
    if (!correction_changed) return;

    active_device_id_ = device_id;
    active_correction_ = next;
    invalid_saved_calibration_reported_ = false;
    if (next.saved) {
        // 理论上 valid_saved_calibration 已验证 rotation；若矩阵构造仍失败，
        // fail-safe 回 identity 并把 count 清零，不能发布半校正磁场。
        if (!dima::lib::sensors::make_rotation_matrix(
                next.rotation, rotation_matrix_)) {
            active_correction_ = Calibration{};
            (void)dima::lib::sensors::make_rotation_matrix(
                0, rotation_matrix_);
            calibration_count_ = 0U;
        } else {
            calibration_count_ = device_changed
                ? 1U : next_calibration_count(calibration_count_);
        }
    } else {
        (void)dima::lib::sensors::make_rotation_matrix(0, rotation_matrix_);
        calibration_count_ = 0U;
        const Calibration &configured = active_configuration_.calibration;
        if (device_id != 0U && configured.configured_device_id > 0 &&
            static_cast<std::uint32_t>(configured.configured_device_id) ==
                device_id && !valid_saved_calibration(configured)) {
            invalid_saved_calibration_reported_ = true;
            ++stats_.parameter_update_failures;
            PX4_WARN("invalid saved magnetometer calibration; using identity");
        }
    }
    ++stats_.calibration_changes;
    reset_accumulator(true);
}

void VehicleMagnetometer::reset_accumulator(
    bool reset_last_publication) noexcept
{
    sum_ga_[0] = 0.0;
    sum_ga_[1] = 0.0;
    sum_ga_[2] = 0.0;
    timestamp_sample_sum_us_ = 0U;
    last_sample_timestamp_us_ = 0U;
    sample_count_ = 0U;
    if (reset_last_publication) {
        last_publication_timestamp_us_ = 0U;
    }
}

bool VehicleMagnetometer::process_sample(
    const sensor_mag_s &sample) noexcept
{
    if (sample.timestamp == 0U || sample.timestamp_sample == 0U ||
        sample.device_id == 0U || !std::isfinite(sample.x) ||
        !std::isfinite(sample.y) || !std::isfinite(sample.z)) {
        ++stats_.invalid_samples;
        return false;
    }

    if (sample.device_id != active_device_id_) {
        configure_device(sample.device_id);
    }

    // 校正次序：corrected=(raw-offset)*scale，再用 row-major 矩阵 sensor->body；
    // 输入/累加/输出磁场单位均为 gauss。
    const dima::lib::sensors::Vector3 corrected{
        (sample.x - active_correction_.offset[0]) *
            active_correction_.scale[0],
        (sample.y - active_correction_.offset[1]) *
            active_correction_.scale[1],
        (sample.z - active_correction_.offset[2]) *
            active_correction_.scale[2],
    };
    const dima::lib::sensors::Vector3 rotated =
        dima::lib::sensors::rotate(rotation_matrix_, corrected);
    if (!std::isfinite(rotated.x) || !std::isfinite(rotated.y) ||
        !std::isfinite(rotated.z)) {
        ++stats_.invalid_samples;
        return false;
    }

    // 时间不递增、样本计数饱和或 timestamp 求和将溢出时丢弃当前累计窗口，
    // 但保留上次发布时间，避免时钟异常制造高频输出。
    if ((last_sample_timestamp_us_ != 0U &&
         sample.timestamp_sample <= last_sample_timestamp_us_) ||
        sample_count_ == UINT32_MAX ||
        sample.timestamp_sample >
            UINT64_MAX - timestamp_sample_sum_us_) {
        reset_accumulator(false);
    }
    last_sample_timestamp_us_ = sample.timestamp_sample;
    sum_ga_[0] += rotated.x;
    sum_ga_[1] += rotated.y;
    sum_ga_[2] += rotated.z;
    timestamp_sample_sum_us_ += sample.timestamp_sample;
    ++sample_count_;
    ++stats_.raw_updates;

    // 首样本立即发布；以后按 sample 时间跨过 publication_interval 才发布窗口
    // 均值，timestamp_sample=窗口所有样本时间戳的算术平均。
    const bool publication_due = last_publication_timestamp_us_ == 0U ||
        sample.timestamp_sample < last_publication_timestamp_us_ ||
        sample.timestamp_sample - last_publication_timestamp_us_ >=
            publication_interval_us_;
    if (!publication_due) return true;

    vehicle_magnetometer_s output{};
    output.timestamp = hrt_absolute_time();
    output.timestamp_sample = timestamp_sample_sum_us_ / sample_count_;
    output.device_id = active_device_id_;
    output.magnetometer_ga[0] = static_cast<float>(
        sum_ga_[0] / sample_count_);
    output.magnetometer_ga[1] = static_cast<float>(
        sum_ga_[1] / sample_count_);
    output.magnetometer_ga[2] = static_cast<float>(
        sum_ga_[2] / sample_count_);
    output.calibration_count = calibration_count_;
    const bool published = vehicle_magnetometer_publication_.publish(output);
    if (published) {
        last_publication_timestamp_us_ = output.timestamp_sample;
        ++stats_.publications;
    } else {
        ++stats_.publication_failures;
    }
    reset_accumulator(false);
    return published;
}

void VehicleMagnetometer::fail_module(const char *reason) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    PX4_ERR("%s", reason);
}

void VehicleMagnetometer::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;

    parameter_update_s parameter_update{};
    if (parameter_update_subscription_.copy(&parameter_update)) {
        process_parameter_update(parameter_update.instance);
    }
    service_pending_configuration();

    // 参数先 staged/apply，再处理最多四个原始样本，确保新 generation 不会在
    // 同一批次中部分使用旧校正、部分使用新校正。
    for (std::size_t update = 0U; update < kMaximumUpdatesPerRun; ++update) {
        sensor_mag_s sample{};
        if (!sensor_mag_subscription_.copy(&sample)) break;
        (void)process_sample(sample);
    }

    if (!ScheduleDelayed(kBackupScheduleUs)) {
        fail_module("backup scheduling failed");
    }
}

} // namespace dima::modules::sensors
