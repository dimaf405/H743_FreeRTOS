/****************************************************************************
 * PX4/QGC-compatible sensor calibration transaction for the Dima Rover.
 * Protocol reference: PX4 v1.17.0 commander calibration_messages.h and
 * calibration_routines.cpp @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "calibration/SensorCalibrationAlgorithms.hpp"

#include "actuator_armed.hpp"
#include "lifecycle/module_base.hpp"
#include "api/Flash.hpp"
#include "parameter_update.hpp"
#include "sensor_calibration_request.hpp"
#include "sensor_accel.hpp"
#include "sensor_calibration_status.hpp"
#include "sensor_gyro.hpp"
#include "sensor_mag.hpp"
#include "uORB/Publication.hpp"
#include "uORB/SubscriptionData.hpp"
#include "vehicle_magnetometer.hpp"
#include "vehicle_status.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::sensors {

namespace algorithms = dima::lib::sensors::calibration;

class VehicleImu;
class VehicleMagnetometer;

// PX4/QGC 兼容的单传感器校准事务协调器。它负责命令/STATUSTEXT 协议、采样
// 状态机、参数原子写入和前端应用握手；实际校正公式分别由 VehicleImu 与
// VehicleMagnetometer 应用。整个事务持有 arming maintenance interlock。
class SensorCalibration final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    explicit SensorCalibration(
        dima::platform::ArmedFlashCoordinator &armed,
        VehicleImu &vehicle_imu_frontend,
        VehicleMagnetometer &vehicle_magnetometer_frontend) noexcept;
    ~SensorCalibration() override;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    enum class Type : std::uint8_t {
        None = sensor_calibration_status_s::TYPE_NONE,
        Gyro = sensor_calibration_status_s::TYPE_GYRO,
        Accel = sensor_calibration_status_s::TYPE_ACCEL,
        Mag = sensor_calibration_status_s::TYPE_MAG,
    };

    enum class Phase : std::uint8_t {
        // Idle -> Collect* -> WaitForApply -> Idle；取消/失败时转入
        // WaitForRollback，只有传感器前端确认旧参数重新生效后才释放禁武装锁。
        Idle,
        CollectGyro,
        CollectAccel,
        CollectMag,
        WaitForApply,
        WaitForRollback,
    };

    enum class RollbackOutcome : std::uint8_t {
        None,
        Cancelled,
        Failed,
    };

    // 协调器运行在非实时低优先级队列，每 20 ms 推进；输入超过 500 ms 不再
    // 视为新鲜。参数提交后最多等待前端 20 s 确认同一 generation。
    static constexpr std::uint32_t kRunIntervalUs = 20000U;
    static constexpr std::uint64_t kSensorFreshnessUs = 500000ULL;
    static constexpr std::uint64_t kRequestFreshnessUs = 1000000ULL;
    static constexpr std::uint64_t kApplyTimeoutUs = 20000000ULL;
    static constexpr std::uint64_t kStatusIntervalUs = 500000ULL;
    // 陀螺收集 150 个静止样本；加速度每面 75 个；磁力计六面各 40 个、总计
    // 240 个空间去重点。样本数固定，运行期不分配容器。
    static constexpr std::uint32_t kGyroSamples = 150U;
    static constexpr std::uint32_t kAccelSideSamples = 75U;
    /* PX4 mag_calibration.cpp collects 240 points over six QGC-visible
     * orientations. Keep the same fixed single-magnetometer contract. */
    static constexpr std::uint32_t kMagOrientationSamples = 25U;
    static constexpr std::uint32_t kMagSideSamples = 40U;
    static constexpr std::uint32_t kMagMinimumSamples =
        kMagSideSamples * 6U;
    static constexpr std::uint64_t kAccelTimeoutUs = 120000000ULL;
    static constexpr std::uint64_t kMagSideCollectionUs = 7000000ULL;
    static constexpr std::uint64_t kMagRotationTimeoutUs = 35000000ULL;
    static constexpr std::uint64_t kMagTimeoutUs = 240000000ULL;
    // 每个磁力计面必须先有至少 0.5 rad 的有符号净旋转，再在 7 s 内收满样本；
    // 全流程上限 240 s。scale 与参数元数据/前端共同限制在 0.1..3.0。
    static constexpr double kMagMinimumRotationRad = 0.5;
    /* Match the shipped PX4 CAL_MAG0_*SCALE metadata and the
     * VehicleMagnetometer frontend's accepted single-device range. */
    static constexpr double kMagMinimumScale = 0.1;
    static constexpr double kMagMaximumScale = 3.0;

    struct AccelSide {
        algorithms::RunningStats3 stats{};
        bool done{false};
    };

    struct MagAccumulator {
        // 球拟合按法方程累积 A^T A 与 A^T b，A=[2x,2y,2z,1]，
        // b=x^2+y^2+z^2；同时保留 240 个点用于 PX4 风格空间去重。
        double normal[4][4]{};
        double rhs[4]{};
        double minimum[3]{};
        double maximum[3]{};
        float x[kMagMinimumSamples]{};
        float y[kMagMinimumSamples]{};
        float z[kMagMinimumSamples]{};
        std::uint32_t samples{0U};
    };

    struct ParameterSnapshot {
        // snapshot 是提交前可回滚值，expectation 是等待前端确认的目标值；
        // 两者均只缓存一个传感器的固定 ID+最多六个校准量。
        Type type{Type::None};
        std::int32_t id{0};
        float values[6]{};
        std::uint8_t value_count{0U};
        bool valid{false};
    };

    void Run() override;
    void reset_runtime_state() noexcept;
    void update_inputs() noexcept;
    void process_requests(std::uint64_t now) noexcept;
    bool begin(Type type, std::uint64_t now,
               const char *&failure_reason) noexcept;
    void cancel() noexcept;
    void fail(const char *reason) noexcept;
    void finish_success() noexcept;
    void release_interlock() noexcept;
    void report_start_failure(Type type, const char *reason) noexcept;
    bool publish_status(std::uint64_t now, bool force) noexcept;
    void update_progress(std::uint8_t progress, std::uint64_t now) noexcept;
    static const char *type_name(Type type) noexcept;
    static bool fresh(std::uint64_t now, std::uint64_t timestamp,
                      std::uint64_t maximum_age) noexcept;

    void process_gyro(std::uint64_t now) noexcept;
    void process_accel(std::uint64_t now) noexcept;
    void process_mag(std::uint64_t now) noexcept;
    void process_wait_for_apply(std::uint64_t now) noexcept;
    void process_wait_for_rollback(std::uint64_t now) noexcept;
    int classify_accel_side(const sensor_accel_s &sample) const noexcept;
    void reset_accel_candidate() noexcept;
    void reset_mag_accumulator() noexcept;
    bool add_mag_sample(const sensor_mag_s &sample) noexcept;

    bool commit_gyro(const algorithms::Vector3d &offset,
                     std::uint32_t device_id) noexcept;
    bool commit_accel(const algorithms::Vector3d &offset,
                      const algorithms::Vector3d &scale,
                      std::uint32_t device_id) noexcept;
    bool commit_mag(const algorithms::Vector3d &offset,
                    const algorithms::Vector3d &scale,
                    std::uint32_t device_id) noexcept;
    void begin_wait_for_apply(std::uint64_t now) noexcept;
    bool begin_rollback(RollbackOutcome outcome) noexcept;
    void finish_rollback() noexcept;
    void latch_rollback_failure() noexcept;
    void clear_parameter_snapshot() noexcept;
    void clear_parameter_expectation() noexcept;
    bool restore_parameters() noexcept;
    void notify_parameter_changes() noexcept;
    bool capture_required_parameter_update() noexcept;

    // SensorCalibration 持有全局 maintenance 锁；前端在 disarmed 时直接应用
    // 参数但不得再次获取同一非重入锁，否则校准握手会死锁。
    dima::platform::ArmedFlashCoordinator &armed_;
    VehicleImu &vehicle_imu_frontend_;
    VehicleMagnetometer &vehicle_magnetometer_frontend_;
    uORB::SubscriptionCallbackWorkItem calibration_request_subscription_{
        ORB_ID(sensor_calibration_request), *this};
    uORB::SubscriptionData<actuator_armed_s> actuator_armed_subscription_{
        ORB_ID(actuator_armed)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::SubscriptionData<parameter_update_s> parameter_update_subscription_{
        ORB_ID(parameter_update)};
    uORB::SubscriptionData<sensor_accel_s> sensor_accel_subscription_{
        ORB_ID(sensor_accel)};
    uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_subscription_{
        ORB_ID(sensor_gyro)};
    uORB::SubscriptionData<sensor_mag_s> sensor_mag_subscription_{
        ORB_ID(sensor_mag)};
    uORB::SubscriptionData<vehicle_magnetometer_s>
        vehicle_magnetometer_subscription_{ORB_ID(vehicle_magnetometer)};
    uORB::Publication<sensor_calibration_status_s> status_publication_{
        ORB_ID(sensor_calibration_status)};

    actuator_armed_s actuator_armed_{};
    vehicle_status_s vehicle_status_{};
    parameter_update_s parameter_update_{};
    sensor_accel_s sensor_accel_{};
    sensor_gyro_s sensor_gyro_{};
    sensor_mag_s sensor_mag_{};
    vehicle_magnetometer_s vehicle_magnetometer_{};
    algorithms::RunningStats3 gyro_stats_{};
    AccelSide accel_sides_[6]{};
    algorithms::RunningStats3 accel_candidate_stats_{};
    algorithms::RunningStats3 mag_orientation_stats_{};
    MagAccumulator mag_{};
    ParameterSnapshot parameter_snapshot_{};
    ParameterSnapshot parameter_expectation_{};

    Type type_{Type::None};
    Phase phase_{Phase::Idle};
    Type apply_type_{Type::None};
    RollbackOutcome rollback_outcome_{RollbackOutcome::None};
    dima::middleware::lifecycle::ModuleState module_state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    std::uint64_t started_us_{0U};
    std::uint64_t last_sample_us_{0U};
    std::uint64_t apply_deadline_us_{0U};
    std::uint64_t commit_time_us_{0U};
    std::uint64_t mag_last_accel_sample_us_{0U};
    std::uint64_t mag_last_gyro_sample_us_{0U};
    std::uint64_t mag_side_started_us_{0U};
    std::uint64_t mag_collection_started_us_{0U};
    std::uint64_t parameter_notification_time_us_{0U};
    std::uint64_t parameter_update_baseline_timestamp_us_{0U};
    std::uint64_t last_status_us_{0U};
    std::uint32_t device_id_{0U};
    std::uint32_t mag_accel_device_id_{0U};
    std::uint32_t mag_gyro_device_id_{0U};
    std::uint32_t mag_side_samples_{0U};
    float board_rotation_matrix_[9]{
        1.0F, 0.0F, 0.0F,
        0.0F, 1.0F, 0.0F,
        0.0F, 0.0F, 1.0F};
    std::uint8_t progress_{0U};
    std::uint32_t required_parameter_update_instance_{0U};
    std::uint32_t parameter_update_baseline_instance_{0U};
    std::uint8_t previous_mag_calibration_count_{0U};
    bool mag_sides_done_[6]{};
    // 逐轴积分 omega*dt，检测 max(|integral_axis|)>=0.5 rad；不能积分角速度模长，
    // 否则振动或往复转动会伪装成有效净旋转。
    double mag_rotation_integral_[3]{};
    int accel_candidate_{-1};
    int mag_orientation_candidate_{-1};
    int mag_active_side_{-1};
    int mag_reported_completed_side_{-1};
    bool interlock_held_{false};
    bool mag_rotation_detected_{false};
    bool required_parameter_update_valid_{false};
    bool rollback_terminal_sent_{false};
};

} // namespace dima::modules::sensors
