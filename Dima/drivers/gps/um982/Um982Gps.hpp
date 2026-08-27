#pragma once

#include "GpsErrorCounter.hpp"
#include "Um982Protocol.hpp"
#include "validation/DataValidator.hpp"
#include "validation/SensorValidityAlgorithms.hpp"
#include "lifecycle/module_base.hpp"
#include "parameter_update.hpp"
#include "parameters/param.h"
#include "api/Execution.hpp"
#include "api/Flash.hpp"
#include "api/Serial.hpp"
#include "maintenance/RuntimeMaintenanceCoordinator.hpp"
#include "estimator_gps_status.hpp"
#include "sensor_gps.hpp"
#include "serial/SerialPortAssignments.hpp"
#include "uORB/Publication.hpp"
#include "uORB/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::drivers::gps {

// UM982 驱动同时承担三层职责：串口/协议接收、接收机配置收敛，以及将多种
// NMEA/Unicore 日志合成为统一 GPS 样本。所有工作都在 io WorkQueue 串行执行，
// UART ISR 只负责唤醒，不在中断中解析、发布或修改配置。
class Um982Gps final : public dima::middleware::lifecycle::ModuleBase,
                       public px4::ScheduledWorkItem {
public:
    // ReceiverStatus 描述外部接收机可见性，与 ModuleState（驱动任务生命周期）
    // 相互独立：驱动可以仍在 Running，但接收机处于 Offline 并执行退避重探测。
    enum class ReceiverStatus : std::uint8_t {
        Unassigned,
        Probing,
        Operational,
        Offline,
    };

    Um982Gps(
        dima::platform::AsyncSerialPort &uart,
        dima::platform::MonotonicClock &clock,
        dima::lib::serial::SerialPortAssignments
            &serial_assignments,
        dima::platform::ArmedFlashCoordinator &armed,
        dima::middleware::maintenance::RuntimeMaintenanceCoordinator
            &maintenance) noexcept;
    ~Um982Gps() override;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

protected:
    void Run() override;

private:
    // 配置状态机严格按以下方向推进：
    // WaitAssignment -> Detect -> ReadConfiguration ->
    // ApplyConfiguration -> SaveConfiguration -> WaitRestart -> Run。
    // 任意串口分配变化返回 WaitAssignment；接收超时返回 Detect；配置失败则保持
    // 数据接收并在 30 s 后重试，不能让配置写入失败拖垮 GPS 运行数据面。
    enum class Phase : std::uint8_t {
        WaitAssignment,
        Detect,
        ReadConfiguration,
        ApplyConfiguration,
        SaveConfiguration,
        WaitRestart,
        Run,
    };

    // 时间常量单位均为微秒。每个候选波特率观察 400 ms；运行期连续 1.3 s
    // 没有有效测量才判离线，避免把单个 10 Hz 日志抖动误判为接收机消失。
    static constexpr std::uint32_t kProbeTimeoutUs = 400000U;
    static constexpr std::uint32_t kReceiveTimeoutUs = 1300000U;
    /* Product UM982 logs run at 10 Hz. Five periods tolerate bounded work-queue
     * and serial jitter without retaining optional fields for a full timeout. */
    // 产品日志为 10 Hz，辅助字段最多复用 5 个周期（500 ms）；超过该窗口即用
    // NaN/无效标志表达“本周期不可用”，不能沿用过期精度、速度或双天线航向。
    static constexpr std::uint32_t kAuxiliaryFreshnessUs = 500000U;
    static constexpr std::uint32_t kReceiverStatusPublishIntervalUs =
        500000U;
    static constexpr std::uint32_t kConfigTimeoutUs = 750000U;
    static constexpr std::uint32_t kConfigProgressUs = 200000U;
    static constexpr std::uint32_t kConfigurationRetryUs = 30000000U;
    static constexpr std::uint32_t kMaintenanceRetryUs = 1000000U;
    static constexpr std::uint32_t kReceiverRestartWaitUs = 1000000U;
    static constexpr std::uint32_t kInitialBackoffUs = 500000U;
    static constexpr std::uint32_t kMaximumBackoffUs = 8000000U;
    static constexpr std::uint32_t kReceiveScheduleUs = 100000U;
    // 每次 Run 最多消费 2048 B；若仍有积压，主动延后 1 ms 让出 io 队列，
    // 避免持续串口流量饿死同一 WorkQueue 上的其他驱动。
    static constexpr std::size_t kRxReadBudgetBytes = 2048U;
    static constexpr std::uint32_t kRxYieldUs = 1000U;
    static constexpr std::uint64_t kValidationReportIntervalUs = 30000000ULL;
    static constexpr std::uint32_t kGpsDeviceBase = 0x554D9800U;

    // 接收数据面：ISR 唤醒 -> 有界 drain -> 协议帧合并 -> 结构/流健康校验 -> 发布。
    static void uart_notification(void *context) noexcept;
    void schedule(std::uint32_t delay_us = 0U) noexcept;
    bool drain_uart() noexcept;
    void clear_measurement_cache() noexcept;
    void handle_frame(const dima::protocols::um982::Um982Protocol::Frame &frame,
                      std::uint64_t arrival_us) noexcept;
    void publish_receiver_status(std::uint64_t now_us) noexcept;
    void publish_if_ready(std::uint64_t now_us) noexcept;
    bool publish_validated(sensor_gps_s &output,
                           std::uint64_t now_us) noexcept;
    void publish_solution_status(
        const sensor_gps_s &output,
        const dima::lib::sensors::validation::GpsSolutionStatus
            &solution) noexcept;
    void record_protocol_failure(
        const dima::protocols::um982::Um982Protocol::Frame &frame) noexcept;
    void update_uart_error_count() noexcept;
    void report_validation_failure(std::uint32_t structure_mask,
                                   std::uint32_t stream_mask) noexcept;
    bool read_yaw_offset(float &radians) const noexcept;
    bool refresh_yaw_offset() noexcept;
    bool assignment_changed() const noexcept;
    void transition(Phase phase, std::uint32_t delay_us = 0U) noexcept;
    void fail() noexcept;
    void run_assignment() noexcept;
    void run_detect(std::uint64_t now_us) noexcept;
    void run_normal(std::uint64_t now_us) noexcept;

    // 探测与配置控制面。配置消息名称、命令和周期只能来自生成的
    // Um982MessageContract.hpp；这里不得维护第二份手写消息列表。
    void build_scan_baudrates(std::uint32_t target) noexcept;
    bool start_scan_candidate() noexcept;
    bool send_command(const char *body) noexcept;
    void complete_probe() noexcept;
    void begin_configuration_read() noexcept;
    void run_configuration_read(std::uint64_t now_us) noexcept;
    void begin_configuration_apply() noexcept;
    void run_configuration_apply(std::uint64_t now_us) noexcept;
    void run_configuration_save(std::uint64_t now_us) noexcept;
    bool keep_configuration_alive(std::uint64_t now_us) noexcept;
    void release_configuration_maintenance(bool complete) noexcept;
    void defer_configuration(bool rescan, const char *reason) noexcept;
    bool identify_receiver_port() noexcept;
    void build_log_update_mask() noexcept;

    dima::platform::AsyncSerialPort &uart_;
    dima::platform::MonotonicClock &clock_;
    dima::lib::serial::SerialPortAssignments &serial_assignments_;
    dima::platform::ArmedFlashCoordinator &armed_;
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance_;
    uORB::SubscriptionData<parameter_update_s> parameter_subscription_{
        ORB_ID(parameter_update)};
    uORB::Publication<sensor_gps_s> sensor_gps_publication_{
        ORB_ID(sensor_gps)};
    /* vehicle_gps_position 是 SensorGps.msg 声明的 Topic alias，官方布局
     * 仍然是 sensor_gps_s，不为 alias 复制第二个消息结构。 */
    uORB::Publication<sensor_gps_s> vehicle_gps_publication_{
        ORB_ID(vehicle_gps_position)};
    uORB::Publication<estimator_gps_status_s> gps_status_publication_{
        ORB_ID(estimator_gps_status)};

    dima::protocols::um982::Um982Protocol protocol_{};
    // stream_validator_ 判断时间戳、超时和错误密度；solution_checker_ 判断当前
    // 解算质量。前者不健康时禁止发布，后者不通过时仍发布 NO_FIX/状态供上层观测。
    dima::lib::sensors::validation::DataValidator stream_validator_{
        kReceiveTimeoutUs, UINT32_MAX};
    dima::lib::sensors::validation::GpsSolutionChecker solution_checker_{};
    // 各协议日志以独立到达时间缓存。GGA 是 10 Hz 发布节拍，GST/GSA/RMC/
    // AGRICA/HEADING 仅在 freshness 窗口内参与同一输出样本。
    dima::protocols::um982::Um982Protocol::Gga gga_{};
    dima::protocols::um982::Um982Protocol::Gst gst_{};
    dima::protocols::um982::Um982Protocol::Gsa gsa_{};
    dima::protocols::um982::Um982Protocol::Rmc rmc_{};
    dima::protocols::um982::Um982Protocol::Agrica agrica_{};
    dima::protocols::um982::Um982Protocol::Heading heading_{};
    dima::protocols::um982::Um982Protocol::ConfigPort port_config_[3]{};
    dima::protocols::um982::Um982Protocol::UnilogList unilog_{};

    // 扫描表按“目标值、最近确认值、常见值”去重，固定容量避免运行期分配。
    std::uint32_t scan_baudrates_[8]{};
    std::uint32_t active_target_baudrate_{0U};
    std::uint32_t detected_baudrate_{0U};
    std::uint32_t last_confirmed_baudrate_{0U};
    std::uint32_t retry_backoff_us_{kInitialBackoffUs};
    std::uint64_t phase_started_us_{0U};
    std::uint64_t last_frame_arrival_us_{0U};
    std::uint64_t last_valid_data_arrival_us_{0U};
    std::uint64_t last_gga_arrival_us_{0U};
    std::uint64_t last_gst_arrival_us_{0U};
    std::uint64_t last_gsa_arrival_us_{0U};
    std::uint64_t last_rmc_arrival_us_{0U};
    std::uint64_t last_agrica_arrival_us_{0U};
    std::uint64_t last_heading_arrival_us_{0U};
    std::uint64_t last_receiver_status_publish_us_{0U};
    std::uint64_t configuration_retry_after_us_{0U};
    std::uint64_t maintenance_retry_after_us_{0U};
    std::uint64_t last_validation_report_us_{0U};
    std::uint32_t rx_budget_yields_{0U};
    std::uint32_t maintenance_progress_{0U};
    GpsErrorCounter gps_error_counter_{};
    // 协议语法计数仅用于离线摘要；可恢复的校验和/结构/超长帧不会注入
    // DataValidator 错误密度。timestamp/sample_structure 才属于发布数据健康层。
    std::uint32_t protocol_checksum_errors_{0U};
    std::uint32_t protocol_structure_errors_{0U};
    std::uint32_t protocol_overflow_errors_{0U};
    std::uint32_t timestamp_errors_{0U};
    std::uint32_t sample_structure_errors_{0U};
    param_t yaw_offset_handle_{PARAM_INVALID};
    float yaw_offset_rad_{0.0F};
    std::int32_t active_port_{0};
    // SAVECONFIG 会写接收机非易失存储，必须同时持有全局 maintenance ticket
    // 和 ArmedFlashCoordinator 排他锁；武装状态下绝不进入写配置阶段。
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator::Ticket
        maintenance_ticket_{0U};
    std::uint8_t scan_count_{0U};
    std::uint8_t scan_index_{0U};
    std::uint8_t config_mask_{0U};
    std::uint8_t selected_receiver_port_{0U};
    std::uint8_t port_probe_index_{0U};
    std::uint8_t log_update_mask_{0U};
    std::uint8_t configuration_read_step_{0U};
    std::uint8_t configuration_command_index_{0U};
    Phase phase_{Phase::WaitAssignment};
    ReceiverStatus receiver_status_{ReceiverStatus::Unassigned};
    dima::middleware::lifecycle::ModuleState module_state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool gga_new_{false};
    bool agrica_new_{false};
    bool heading_new_{false};
    bool version_seen_{false};
    bool unilog_seen_{false};
    bool configuration_complete_{false};
    bool candidate_active_{false};
    bool command_pending_{false};
    bool baud_change_complete_{false};
    bool maintenance_ready_{false};
    bool maintenance_interlock_acquired_{false};
    bool rx_schedule_suppressed_{false};
    bool validation_fault_active_{false};
};

} // namespace dima::drivers::gps
