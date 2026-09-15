#pragma once
/*
 * 一个 MavlinkService 在 lp_default 调度两个固定端点：USB Config/COMM_0、
 * UART Normal/COMM_1。每条链路独占 parser、序号、协议会话和发送 FIFO；
 * Commander 路由与单 reader 日志租约由 SharedState 仲裁。
 * ACK/心跳优先入队，已编码帧保持线序。USB 所有写共享每轮 5 ms 等待预算，
 * UART 异步提交；普通复位必须等待来源链路的物理完成证据。
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
#include "vehicle_status.hpp"
#include "mavlink/MavlinkBridge.h"
#include "mavlink_stream_contract.hpp"
#include "lifecycle/module_base.hpp"
#include "api/Boot.hpp"
#include "api/Console.hpp"
#include "api/LogFileStore.hpp"
#include "MavlinkTransport.hpp"
#include "MavlinkSharedState.hpp"
#include "serial/SerialPortAssignments.hpp"
#include <atomic>
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

class MavlinkService;

class MavlinkEndpoint final {
    friend class MavlinkService;
public:
    MavlinkEndpoint(MavlinkTransport &transport, MavlinkSharedState &shared,
                    std::uint8_t channel, dima::platform::BootControl &boot_control,
                    dima::modules::mission::MissionService &mission_service,
                    dima::platform::LogFileStore &log_files) noexcept;
    bool start() noexcept;
    void stop() noexcept;
    dima::middleware::lifecycle::ModuleState state() const noexcept;
    void Run(bool quiescing = false);
    void reset_link() noexcept;
    bool tx_drained() noexcept;
    bool queue_command_ack(const vehicle_command_ack_s &ack, std::uint32_t epoch) noexcept;

private:
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

    // 协议处理器只通过这些 trampoline 回到唯一链路所有者，不能直接操作 USB。
    static bool send_frame(void *ctx, mavlink_message_t &msg) noexcept;
    static bool send_log_batch(void *ctx, const std::uint8_t *data,
                               std::size_t length) noexcept;
    static void send_frame_void(void *ctx, mavlink_message_t &msg) noexcept;
    static std::uint8_t request_message(void *ctx,
                                        std::uint16_t message_id,
                                        float param2, float param3,
                                        float param4, float param5,
                                        float param6, float param7) noexcept;
    static std::uint8_t set_message_interval(
        void *ctx, std::uint16_t message_id, float interval_us,
        float param3, float param4, float param7) noexcept;
    static std::uint8_t get_message_interval(
        void *ctx, std::uint16_t message_id) noexcept;
    static bool stream_due(std::uint64_t now, std::uint64_t last_tx,
                           std::int32_t interval_us) noexcept;

    void reset_runtime_state() noexcept;
    void reset_parser_state() noexcept;
    void discard_rx() noexcept;
    void drain_rx() noexcept;
    bool dispatch(const mavlink_message_t &msg) noexcept;
    void handle_ping(const mavlink_message_t &msg) noexcept;
    void send_command_ack(const mavlink_command_ack_t &ack,
                          bool reboot_ack) noexcept;
    void maybe_perform_reboot(std::uint64_t now) noexcept;
    void cancel_reboot(const char *reason) noexcept;
    bool refresh_protocol_parameters() noexcept;
    void update_rc_input() noexcept;
    void update_sensor_topics() noexcept;
    void reset_sensor_streams() noexcept;
    void reset_sensor_link_state() noexcept;
    void reset_configured_streams() noexcept;
    void report_sensor_link_summary() noexcept;
    void report_imu_fault(std::uint64_t now) noexcept;
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
    std::uint8_t request_available_modes(float index) noexcept;
    void stream_available_modes() noexcept;
    bool current_mode_snapshot(mavlink_current_mode_t &mode) noexcept;
    bool current_mode_changed() noexcept;
    bool send_current_mode() noexcept;
    void stream_statustext() noexcept;
    bool send_message(mavlink_message_t &msg,
                      std::uint32_t timeout_ms = kTxTimeoutMs) noexcept;
    bool send_autopilot_version() noexcept;
    bool send_protocol_version() noexcept;
    bool send_component_metadata() noexcept;
    bool send_component_information() noexcept;
    [[noreturn]] void perform_reboot() noexcept;

    enum class TxClass : std::uint8_t { Reply, Stream, Bulk };
    struct TxFrame {
        std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
        std::uint16_t length{0U};
        TxClass kind{TxClass::Reply};
        bool reboot_ack{false};
    };
    static constexpr std::size_t kTxQueueCapacity = 8U;
    static std::uint8_t dispatch_command(void *, const vehicle_command_s &) noexcept;
    static void acknowledge_local(void *, const vehicle_command_ack_s &) noexcept;
    bool enqueue_frame(const std::uint8_t *, std::size_t, bool reboot_ack = false) noexcept;
    bool background_space() const noexcept;
    void flush_tx() noexcept;
    std::uint32_t usb_timeout_remaining() const noexcept;
    void update_rate_mult(std::uint64_t now) noexcept;
    std::int32_t default_interval(const dima::generated::mavlink_streams::MessageContract &) const noexcept;
    std::uint64_t drain_timeout_us() const noexcept;

    std::uint8_t channel_;
    MavlinkSharedState &shared_;
    MavlinkTransport &transport_;
    dima::platform::BootControl &boot_control_;

    mavlink_message_t parse_message_{};
    mavlink_status_t parse_status_{};

    MavlinkIdentity identity_{};
    HeartbeatPacer heartbeat_pacer_{identity_, channel_};
    MavlinkParameters parameters_{&MavlinkEndpoint::send_frame, this, channel_};
    MavlinkTimesync timesync_{&MavlinkEndpoint::send_frame_void, this, channel_};
    MavlinkCommands commands_{&MavlinkEndpoint::request_message,
                              &MavlinkEndpoint::set_message_interval,
                              &MavlinkEndpoint::get_message_interval,
                              &MavlinkEndpoint::dispatch_command,
                              &MavlinkEndpoint::acknowledge_local, this};
    MavlinkMission mission_;
    MavlinkLogHandler log_handler_;
    MavlinkMetadataFtp metadata_ftp_{&MavlinkEndpoint::send_frame, this, channel_};

    uORB::SubscriptionData<mavlink_log_s>
        mavlink_log_subscription_{ORB_ID(mavlink_log)};
    uORB::SubscriptionData<input_rc_s>
        input_rc_subscription_{ORB_ID(input_rc)};
    uORB::SubscriptionData<parameter_update_s>
        parameter_update_subscription_{ORB_ID(parameter_update)};
    uORB::Subscription
        sensor_accel_subscription_{ORB_ID(sensor_accel)};
    uORB::Subscription
        sensor_gyro_subscription_{ORB_ID(sensor_gyro)};
    uORB::Subscription
        sensor_mag_subscription_{ORB_ID(sensor_mag)};
    uORB::Subscription
        vehicle_imu_subscription_{ORB_ID(vehicle_imu)};
    uORB::Subscription
        vehicle_imu_status_subscription_{ORB_ID(vehicle_imu_status)};
    uORB::Subscription
        vehicle_magnetometer_subscription_{ORB_ID(vehicle_magnetometer)};
    // vehicle_gps_position 只是 Topic alias，payload 类型遵循官方 sensor_gps_s。
    uORB::Subscription
        vehicle_gps_subscription_{ORB_ID(vehicle_gps_position)};
    uORB::Subscription
        estimator_gps_status_subscription_{ORB_ID(estimator_gps_status)};
    uORB::Subscription
        vehicle_attitude_subscription_{ORB_ID(vehicle_attitude)};
    uORB::Subscription
        vehicle_local_position_subscription_{ORB_ID(vehicle_local_position)};
    uORB::Subscription
        vehicle_global_position_subscription_{ORB_ID(vehicle_global_position)};
    uORB::Subscription
        vehicle_odometry_subscription_{ORB_ID(vehicle_odometry)};
    uORB::Subscription
        estimator_status_subscription_{ORB_ID(estimator_status)};
    uORB::SubscriptionData<vehicle_status_s>
        vehicle_status_subscription_{ORB_ID(vehicle_status)};

    std::uint8_t rx_buffer_[kRxBatchBytes]{};
    std::uint8_t tx_buffer_[MAVLINK_MAX_PACKET_LEN]{};
    TxFrame tx_queue_[kTxQueueCapacity]{};
    std::uint8_t tx_head_{0U};
    std::uint8_t tx_count_{0U};
    TxClass tx_class_{TxClass::Reply};
    std::uint16_t rx_position_{0U};
    std::uint16_t rx_size_{0U};
    bool rx_message_pending_{false};
    // 完整通过 CRC/签名校验的接收帧计数；Auto 波特率扫描用它作为锁定证据。
    std::uint32_t parsed_frames_{0U};
    std::uint32_t epoch_{1U};
    std::uint32_t error_generation_{0U};
    std::uint64_t usb_deadline_us_{0U};
    std::uint64_t rate_window_us_{0U};
    std::uint32_t transaction_bytes_{0U};
    std::uint32_t stream_attempts_{0U};
    std::uint32_t stream_blocked_{0U};
    float rate_multiplier_{1.0F};
    param_t rate_handle_{PARAM_INVALID};
    bool wait_reboot_completion_{false};
    bool reboot_ack_completed_{false};
    bool reboot_save_requested_{false};
    std::uint32_t reboot_tx_completion_baseline_{0U};
    std::uint32_t reboot_tx_error_baseline_{0U};

    std::uint16_t statustext_id_{0U};
    std::uint16_t cpu_load_permille_{0U};
    // AVAILABLE_MODES 只冻结静态目录区间，每轮一帧；CURRENT_MODE 的提交快照
    // 仅在真实发送成功后推进，丢帧不会吞掉模式变化或用户意图变化。
    std::uint8_t available_modes_next_{0U};
    std::uint8_t available_modes_end_{0U};
    mavlink_current_mode_t last_current_mode_{};
    bool have_current_mode_tx_{false};
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
    bool imu_fault_reported_{false};
    bool mag_healthy_{false};
    bool gps_healthy_{false};
    bool rc_loss_timeout_valid_{false};
    // 0=无请求，1=普通复位，3=MCUboot Recovery；仅来自 Commander 已批准 ACK。
    int reboot_mode_pending_{0};
    std::uint64_t reboot_deadline_us_{0U};
    std::uint64_t reboot_save_deadline_us_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

// 对外只有一个生命周期和 WorkQueue；两条链路共享业务执行者，独立持有协议上下文。
class MavlinkService final : public dima::middleware::lifecycle::ModuleBase,
                             public px4::ScheduledWorkItem {
public:
    MavlinkService(dima::platform::Console &console,
                   dima::platform::BootControl &boot_control,
                   dima::modules::mission::MissionService &mission_service,
                   dima::platform::LogFileStore &log_files,
                   dima::platform::AsyncSerialPort &serial,
                   const dima::lib::serial::SerialPortAssignments &assignments) noexcept;
    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;
    bool prepare_serial_reconfigure() noexcept;
    void resume_serial() noexcept;
    bool serial_reconfigure_failed() const noexcept;
    bool apply_serial_configuration() noexcept;
private:
    void Run() override;
    static void notify_from_isr(void *context) noexcept;
    static bool deliver_ack(void *, std::uint8_t, std::uint32_t,
                            const vehicle_command_ack_s &) noexcept;
    void service_uart_scan(std::uint64_t now) noexcept;
    MavlinkSharedState shared_{};
    UsbMavlinkTransport usb_transport_;
    SerialMavlinkTransport uart_transport_;
    MavlinkEndpoint usb_;
    MavlinkEndpoint uart_;
    const dima::lib::serial::SerialPortAssignments &assignments_;
    std::atomic<bool> quiesce_requested_{false};
    std::atomic<bool> uart_quiesced_{false};
    std::atomic<bool> quiesce_failed_{false};
    std::uint64_t quiesce_deadline_us_{0U};
    std::uint32_t quiesce_error_generation_{0U};
    std::uint64_t retry_uart_after_us_{0U};
    /* Auto 波特率扫描状态：SERIALx_BAUD=0 时逐档尝试常见速率，窗口内收到
     * 足量完整 MAVLink 帧即锁定；锁定结果只保留在本 Runtime 的线配置中。 */
    std::uint8_t scan_index_{0U};
    std::uint64_t scan_window_started_us_{0U};
    std::uint32_t scan_frames_baseline_{0U};
    bool scan_locked_{false};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

}  // namespace dima::modules::mavlink
