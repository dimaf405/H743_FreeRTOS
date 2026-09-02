#pragma once
/*
 * MavlinkService 是 USB CDC 的唯一 MAVLink RX/TX 所有者。
 *
 * 帧协议来自官方生成的裁剪方言；接收侧分派给 Commands/Parameters/Mission/Timesync，
 * 发送侧固定优先级为 COMMAND_ACK/HEARTBEAT -> 原始 RC -> Metadata FTP 待发响应
 * -> 传感器 -> 板载日志分片 -> 参数 -> STATUSTEXT。板载日志每轮限量发送，不能
 * 挤占心跳、命令确认和传感器通道；批准重启后优先送达 ACK，最迟在固定期限执行。
 * 模块运行在低优先级 WorkQueue，热路径只用定长缓冲和有界 USB 写，不动态分配。
 */

#include "input_rc.hpp"
#include "estimator_gps_status.hpp"
#include "estimator_status.hpp"
#include "mavlink_log.hpp"
#include "parameter_update.hpp"
#include "sensor_accel.hpp"
#include "sensor_gyro.hpp"
#include "sensor_mag.hpp"
#include "vehicle_command.hpp"
#include "vehicle_command_ack.hpp"
#include "sensor_gps.hpp"
#include "vehicle_imu.hpp"
#include "vehicle_imu_status.hpp"
#include "vehicle_attitude.hpp"
#include "vehicle_global_position.hpp"
#include "vehicle_local_position.hpp"
#include "vehicle_magnetometer.hpp"
#include "vehicle_odometry.hpp"
#include "mavlink/MavlinkBridge.h"
#include "mavlink_stream_contract.hpp"
#include "lifecycle/module_base.hpp"
#include "api/Boot.hpp"
#include "api/Console.hpp"
#include "api/LogFileStore.hpp"
#include "parameters/param.h"
#include "uORB/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include "HeartbeatPacer.hpp"
#include "MavlinkCommands.hpp"
#include "MavlinkIdentity.hpp"
#include "MavlinkLogHandler.hpp"
#include "MavlinkMission.hpp"
#include "MavlinkMetadataFtp.hpp"
#include "MavlinkParameters.hpp"
#include "MavlinkTimesync.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkService final : public dima::middleware::lifecycle::ModuleBase,
                             public px4::ScheduledWorkItem {
public:
    MavlinkService(dima::platform::Console &console,
                   dima::platform::BootControl &boot_control,
                   dima::modules::mission::MissionService &mission_service,
                   dima::platform::LogFileStore &log_files)
        noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    // 100 Hz 是协议调度节拍，不等于任一消息的发布频率；各流有独立间隔门。
    static constexpr std::uint32_t kRunIntervalUs = 10000U;
    static constexpr std::size_t kRxBatchBytes = 256U;
    static constexpr std::size_t kMaxStatusTextPerRun = 2U;
    static constexpr std::uint32_t kTxTimeoutMs = 5U;
    static constexpr std::uint64_t kRebootDeadlineUs = 400000ULL;
    static constexpr std::uint64_t kImuFreshnessUs = 200000ULL;
    static constexpr std::uint64_t kImuStatusFreshnessUs = 1500000ULL;
    static constexpr std::uint64_t kMagFreshnessUs = 500000ULL;
    static constexpr std::uint64_t kGpsFreshnessUs = 1000000ULL;
    static constexpr std::uint64_t kGpsStatusFreshnessUs = 1000000ULL;
    static constexpr std::uint64_t kEstimatorOutputFreshnessUs = 1000000ULL;
    /* COMMAND_ACK 重试槽上限（首次发送失败后最多再试 4 次）。 */
    static constexpr std::uint8_t kMaxAckRetries = 4U;

    // 协议处理器只通过这些 trampoline 回到唯一链路所有者，不能直接操作 USB。
    static bool send_frame(void *ctx, mavlink_message_t &msg) noexcept;
    static void send_frame_void(void *ctx, mavlink_message_t &msg) noexcept;
    static std::uint8_t request_message(void *ctx,
                                        std::uint16_t message_id) noexcept;
    static std::uint8_t set_message_interval(
        void *ctx, std::uint16_t message_id, float interval_us,
        float param3, float param4, float param7) noexcept;
    static std::uint8_t get_message_interval(
        void *ctx, std::uint16_t message_id) noexcept;
    static bool stream_due(std::uint64_t now, std::uint64_t last_tx,
                           std::int32_t interval_us) noexcept;

    void Run() override;
    void reset_runtime_state() noexcept;
    void reset_parser_state() noexcept;
    void discard_rx() noexcept;
    void drain_rx() noexcept;
    void dispatch(const mavlink_message_t &msg) noexcept;
    void handle_ping(const mavlink_message_t &msg) noexcept;
    void process_command_acks() noexcept;
    void send_command_ack(const mavlink_command_ack_t &ack,
                          bool reboot_ack) noexcept;
    void flush_pending_ack() noexcept;
    void maybe_perform_reboot(std::uint64_t now) noexcept;
    bool refresh_protocol_parameters() noexcept;
    void update_rc_input() noexcept;
    void update_sensor_topics() noexcept;
    void reset_sensor_streams() noexcept;
    void reset_sensor_link_state() noexcept;
    void reset_configured_streams() noexcept;
    void report_sensor_link_summary() noexcept;
    void stream_configured_messages(
        std::uint64_t now,
        dima::generated::mavlink_streams::TxStage stage) noexcept;
    bool send_contract_message(
        dima::generated::mavlink_streams::MessageHandler handler,
        std::uint64_t now, bool refresh_topics) noexcept;
    bool rc_sample_streamable(std::uint64_t now) const noexcept;
    bool send_rc_channels(std::uint64_t now) noexcept;
    bool send_highres_imu(std::uint64_t now) noexcept;
    bool send_scaled_imu(std::uint64_t now) noexcept;
    bool send_gps_raw_int(std::uint64_t now) noexcept;
    bool send_system_status(std::uint64_t now) noexcept;
    bool send_attitude(std::uint64_t now) noexcept;
    bool send_local_position_ned(std::uint64_t now) noexcept;
    bool send_global_position_int(std::uint64_t now) noexcept;
    bool send_estimator_status(std::uint64_t now) noexcept;
    void stream_statustext() noexcept;
    bool send_message(mavlink_message_t &msg,
                      std::uint32_t timeout_ms = kTxTimeoutMs) noexcept;
    bool send_autopilot_version() noexcept;
    bool send_protocol_version() noexcept;
    bool send_component_metadata() noexcept;
    bool send_component_information() noexcept;
    [[noreturn]] void perform_reboot() noexcept;

    dima::platform::Console &console_;
    dima::platform::BootControl &boot_control_;

    mavlink_message_t parse_message_{};
    mavlink_status_t parse_status_{};

    MavlinkIdentity identity_{};
    HeartbeatPacer heartbeat_pacer_{identity_};
    MavlinkParameters parameters_{&MavlinkService::send_frame, this};
    MavlinkTimesync timesync_{&MavlinkService::send_frame_void, this};
    MavlinkCommands commands_{&MavlinkService::request_message,
                              &MavlinkService::set_message_interval,
                              &MavlinkService::get_message_interval, this};
    MavlinkMission mission_;
    MavlinkLogHandler log_handler_;
    MavlinkMetadataFtp metadata_ftp_{&MavlinkService::send_frame, this};

    uORB::SubscriptionData<vehicle_command_ack_s>
        command_ack_subscription_{ORB_ID(vehicle_command_ack)};
    uORB::SubscriptionData<mavlink_log_s>
        mavlink_log_subscription_{ORB_ID(mavlink_log)};
    uORB::SubscriptionData<input_rc_s>
        input_rc_subscription_{ORB_ID(input_rc)};
    uORB::SubscriptionData<parameter_update_s>
        parameter_update_subscription_{ORB_ID(parameter_update)};
    uORB::SubscriptionData<sensor_accel_s>
        sensor_accel_subscription_{ORB_ID(sensor_accel)};
    uORB::SubscriptionData<sensor_gyro_s>
        sensor_gyro_subscription_{ORB_ID(sensor_gyro)};
    uORB::SubscriptionData<sensor_mag_s>
        sensor_mag_subscription_{ORB_ID(sensor_mag)};
    uORB::SubscriptionData<vehicle_imu_s>
        vehicle_imu_subscription_{ORB_ID(vehicle_imu)};
    uORB::SubscriptionData<vehicle_imu_status_s>
        vehicle_imu_status_subscription_{ORB_ID(vehicle_imu_status)};
    uORB::SubscriptionData<vehicle_magnetometer_s>
        vehicle_magnetometer_subscription_{ORB_ID(vehicle_magnetometer)};
    // vehicle_gps_position 只是 Topic alias，payload 类型遵循官方 sensor_gps_s。
    uORB::SubscriptionData<sensor_gps_s>
        vehicle_gps_subscription_{ORB_ID(vehicle_gps_position)};
    uORB::SubscriptionData<estimator_gps_status_s>
        estimator_gps_status_subscription_{ORB_ID(estimator_gps_status)};
    uORB::SubscriptionData<vehicle_attitude_s>
        vehicle_attitude_subscription_{ORB_ID(vehicle_attitude)};
    uORB::SubscriptionData<vehicle_local_position_s>
        vehicle_local_position_subscription_{ORB_ID(vehicle_local_position)};
    uORB::SubscriptionData<vehicle_global_position_s>
        vehicle_global_position_subscription_{ORB_ID(vehicle_global_position)};
    uORB::SubscriptionData<vehicle_odometry_s>
        vehicle_odometry_subscription_{ORB_ID(vehicle_odometry)};
    uORB::SubscriptionData<estimator_status_s>
        estimator_status_subscription_{ORB_ID(estimator_status)};

    std::uint8_t rx_buffer_[kRxBatchBytes]{};
    std::uint8_t tx_buffer_[MAVLINK_MAX_PACKET_LEN]{};
    /* COMMAND_ACK 发送失败单重试槽；pending 时不继续消费 uORB ACK，
     * 因此其深度 4 队列继续保持 FIFO。 */
    mavlink_command_ack_t pending_ack_{};
    bool pending_ack_valid_{false};
    bool pending_ack_is_reboot_{false};
    std::uint8_t ack_retry_{0U};

    std::uint16_t statustext_id_{0U};
    struct ConfiguredStreamState {
        std::int32_t interval_us{-1};
        std::uint64_t last_tx_us{0U};
    };
    std::array<ConfiguredStreamState,
               dima::generated::mavlink_streams::kServiceStreamCount>
        configured_streams_{};
    input_rc_s latest_input_rc_{};
    sensor_accel_s latest_sensor_accel_{};
    sensor_gyro_s latest_sensor_gyro_{};
    sensor_mag_s latest_sensor_mag_{};
    vehicle_imu_s latest_vehicle_imu_{};
    vehicle_imu_status_s latest_vehicle_imu_status_{};
    vehicle_magnetometer_s latest_vehicle_magnetometer_{};
    sensor_gps_s latest_vehicle_gps_{};
    estimator_gps_status_s latest_estimator_gps_status_{};
    vehicle_attitude_s latest_vehicle_attitude_{};
    vehicle_local_position_s latest_vehicle_local_position_{};
    vehicle_global_position_s latest_vehicle_global_position_{};
    vehicle_odometry_s latest_vehicle_odometry_{};
    estimator_status_s latest_estimator_status_{};
    param_t rc_loss_timeout_handle_{PARAM_INVALID};
    param_t mav_system_id_handle_{PARAM_INVALID};
    float rc_loss_timeout_s_{0.0F};
    std::uint64_t last_highres_imu_timestamp_us_{0U};
    std::uint64_t last_highres_mag_timestamp_us_{0U};
    std::uint64_t last_scaled_imu_timestamp_us_{0U};
    std::uint64_t last_scaled_mag_timestamp_us_{0U};
    bool was_link_ready_{false};
    bool transport_was_ready_{false};
    bool have_input_rc_{false};
    bool rc_stream_active_{false};
    bool accel_seen_{false};
    bool gyro_seen_{false};
    bool mag_seen_{false};
    bool gps_seen_{false};
    bool mag_health_known_{false};
    bool imu_streamable_{false};
    bool gps_streamable_{false};
    bool imu_healthy_{false};
    bool mag_healthy_{false};
    bool gps_healthy_{false};
    bool rc_loss_timeout_valid_{false};
    // 0=无请求，1=普通复位，3=MCUboot Recovery；仅来自 Commander 已批准 ACK。
    int reboot_mode_pending_{0};
    std::uint64_t reboot_deadline_us_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

}  // namespace dima::modules::mavlink
