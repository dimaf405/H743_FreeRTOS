#define MODULE_NAME "um982"
#include "Um982Gps.hpp"
#include "Um982MessageContract.hpp"
#include "Um982QgcLog.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::drivers::gps {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegreesToRadians = kPi / 180.0F;

float wrap_pi(float angle) noexcept
{
    // 将有限角归一化到 [-pi, pi]；NaN/Inf 统一输出 NaN，避免非法航向在
    // while 中无法收敛。输入角的单位必须是弧度。
    if (!std::isfinite(angle)) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    constexpr float kTwoPi = 2.0F * kPi;
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

bool sample_is_fresh(std::uint64_t now_us, std::uint64_t arrival_us,
                     std::uint64_t maximum_age_us) noexcept
{
    // 同时拒绝“从未到达”和时钟倒退，防止无符号减法下溢把旧样本判为新鲜。
    return arrival_us != 0U && now_us >= arrival_us &&
           now_us - arrival_us <= maximum_age_us;
}

void saturating_increment(std::uint32_t &counter) noexcept
{
    if (counter != std::numeric_limits<std::uint32_t>::max()) {
        ++counter;
    }
}

void saturating_increment(std::uint16_t &counter) noexcept
{
    if (counter != std::numeric_limits<std::uint16_t>::max()) {
        ++counter;
    }
}

std::uint8_t fix_type_from_gga(std::uint8_t quality,
                               std::uint8_t dimension) noexcept
{
    // GGA 质量码优先表达 RTK/差分类型；普通定位再结合 GSA 维度区分 2D/3D。
    switch (quality) {
    case 4U: return sensor_gps_s::FIX_TYPE_RTK_FIXED;
    case 5U: return sensor_gps_s::FIX_TYPE_RTK_FLOAT;
    case 2U: return sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL;
    default:
        return quality == 0U ? sensor_gps_s::FIX_TYPE_NONE
                             : (dimension >= 3U ? sensor_gps_s::FIX_TYPE_3D
                                                : sensor_gps_s::FIX_TYPE_2D);
    }
}

} // namespace

Um982Gps::Um982Gps(
    dima::platform::AsyncSerialPort &uart,
    dima::platform::MonotonicClock &clock,
    dima::lib::serial::SerialPortAssignments &serial_assignments,
    dima::platform::ArmedFlashCoordinator &armed,
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator
        &maintenance) noexcept
    : px4::ScheduledWorkItem("um982", px4::wq_configurations::io),
      uart_(uart), clock_(clock), serial_assignments_(serial_assignments),
      armed_(armed), maintenance_(maintenance)
{
}

Um982Gps::~Um982Gps() { stop(); }

bool Um982Gps::start() noexcept
{
    if (module_state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    // 参数句柄只在启动时解析；运行期 parameter_update 仅刷新已生成参数的值。
    yaw_offset_handle_ = param_handle(dima::params::GPS_YAW_OFFSET);
    param_set_used(yaw_offset_handle_);
    if (!refresh_yaw_offset() || !ScheduleEnable()) {
        module_state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    rx_budget_yields_ = 0U;
    last_validation_report_us_ = 0U;
    retry_backoff_us_ = kInitialBackoffUs;
    configuration_complete_ = false;
    configuration_retry_after_us_ = 0U;
    maintenance_retry_after_us_ = 0U;
    configuration_fault_active_ = false;
    configuration_persistence_pending_ = false;
    last_config_response_arrival_us_ = 0U;
    last_unilog_entry_arrival_us_ = 0U;
    config_response_progress_pending_ = false;
    unilog_entry_seen_ = false;
    unilog_entry_progress_pending_ = false;
    __atomic_store_n(&rx_schedule_suppressed_, false, __ATOMIC_RELEASE);
    clear_measurement_cache();
    (void)parameter_subscription_.update();
    module_state_ = dima::middleware::lifecycle::ModuleState::Running;
    receiver_status_ = ReceiverStatus::Unassigned;
    transition(Phase::WaitAssignment);
    return true;
}

void Um982Gps::stop() noexcept
{
    // 先禁止 ISR 再次排队，再等待 WorkQueue 回调退出，之后才释放 maintenance
    // 和 UART 所有权，保证析构过程中不会出现回调访问已清理状态。
    __atomic_store_n(&rx_schedule_suppressed_, true, __ATOMIC_RELEASE);
    ScheduleCancelAndDrain();
    if (maintenance_ticket_ != 0U) {
        maintenance_.cancel(maintenance_ticket_);
        maintenance_ticket_ = 0U;
    }
    if (maintenance_interlock_acquired_) {
        armed_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    maintenance_ready_ = false;
    candidate_active_ = false;
    command_pending_ = false;
    (void)uart_.stop();
    protocol_.reset();
    clear_measurement_cache();
    module_state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    receiver_status_ = ReceiverStatus::Unassigned;
    phase_ = Phase::WaitAssignment;
}

dima::middleware::lifecycle::ModuleState Um982Gps::state() const noexcept
{
    return module_state_;
}

void Um982Gps::uart_notification(void *context) noexcept
{
    // ISR 只发布一次“立即运行”请求；解析和 uORB 发布都留在任务上下文。
    if (context != nullptr) {
        auto *const self = static_cast<Um982Gps *>(context);
        if (!__atomic_load_n(&self->rx_schedule_suppressed_,
                             __ATOMIC_ACQUIRE)) {
            (void)self->ScheduleNowFromISR();
        }
    }
}

void Um982Gps::schedule(std::uint32_t delay_us) noexcept
{
    if (module_state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    if (delay_us == 0U) {
        (void)ScheduleNow();
    } else {
        (void)ScheduleDelayed(delay_us);
    }
}

bool Um982Gps::read_yaw_offset(float &radians) const noexcept
{
    float degrees = 0.0F;
    if (yaw_offset_handle_ == PARAM_INVALID ||
        param_get(yaw_offset_handle_, &degrees) != 0 ||
        !std::isfinite(degrees) || degrees < 0.0F || degrees > 360.0F) {
        return false;
    }
    // GPS_YAW_OFFSET 的参数层单位为 deg；输出合同为 rad，并归一化到 [-pi, pi]。
    radians = wrap_pi(degrees * kDegreesToRadians);
    return true;
}

bool Um982Gps::refresh_yaw_offset() noexcept
{
    return read_yaw_offset(yaw_offset_rad_);
}

void Um982Gps::transition(Phase phase, std::uint32_t delay_us) noexcept
{
    // 每次迁移同时重置阶段计时原点，所有阶段超时都相对该时间计算。
    phase_ = phase;
    phase_started_us_ = clock_.now_us();
    schedule(delay_us);
}

void Um982Gps::fail() noexcept
{
    // 数据面失败必须释放可能持有的写配置权限，关闭当前 UART 会话，然后从
    // Detect 重启。退避序列为 0.5、1、2、4、8 s，之后钳位在 8 s。
    if (maintenance_ticket_ != 0U) {
        maintenance_.cancel(maintenance_ticket_);
        maintenance_ticket_ = 0U;
    }
    if (maintenance_interlock_acquired_) {
        armed_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    maintenance_ready_ = false;
    candidate_active_ = false;
    command_pending_ = false;
    configuration_complete_ = false;
    const bool report_offline = receiver_status_ != ReceiverStatus::Offline;
    receiver_status_ = ReceiverStatus::Offline;
    if (report_offline) {
        update_uart_error_count();
        const auto stats = uart_.stats();
        UM982_QGC_ERR("GPS offline S%ld target=%lu b=%lu rx=%lu uart=%lu/%lu flags=0x%lx proto=%lu/%lu/%lu",
                static_cast<long>(active_port_),
                static_cast<unsigned long>(active_target_baudrate_),
                static_cast<unsigned long>(
                    uart_.line_configuration().baudrate),
                static_cast<unsigned long>(stats.received_bytes),
                static_cast<unsigned long>(
                    gps_error_counter_.uart_receive_errors()),
                static_cast<unsigned long>(
                    gps_error_counter_.uart_dropped_bytes()),
                static_cast<unsigned long>(stats.receive_error_flags),
                static_cast<unsigned long>(protocol_checksum_errors_),
                static_cast<unsigned long>(protocol_structure_errors_),
                static_cast<unsigned long>(protocol_overflow_errors_));
    }
    (void)uart_.stop();
    build_scan_baudrates(active_target_baudrate_);
    transition(Phase::Detect, retry_backoff_us_);
    retry_backoff_us_ = std::min(
        retry_backoff_us_ * 2U, kMaximumBackoffUs);
}

bool Um982Gps::drain_uart() noexcept
{
    std::uint8_t bytes[512]{};
    std::uint64_t arrival_us = 0U;
    std::size_t processed = 0U;
    dima::protocols::um982::Um982Protocol::Frame frame{};
    // 单次最多读取 2048 B，每块最多 512 B；arrival_us 是该 DMA 批次的时间戳。
    // 达到预算返回 true，让 Run 在 1 ms 后继续，形成有界协作式让步。
    while (processed < kRxReadBudgetBytes) {
        const std::size_t capacity = std::min(
            sizeof(bytes), kRxReadBudgetBytes - processed);
        const std::size_t count = uart_.read(bytes, capacity, arrival_us);
        if (count == 0U) break;
        processed += count;
        for (std::size_t index = 0U; index < count; ++index) {
            const bool frame_complete = protocol_.feed(bytes[index], frame);
            if (frame_complete) {
                handle_frame(frame, arrival_us == 0U ? clock_.now_us()
                                                     : arrival_us);
                frame = {};
            }
        }
    }
    return processed >= kRxReadBudgetBytes;
}

void Um982Gps::clear_measurement_cache() noexcept
{
    // 波特率/端口/时钟域变化后必须原子地丢弃整组融合缓存，不能让不同会话的
    // GGA 与 AGRICA/HEADING 拼成同一个输出样本。
    gga_ = {};
    gst_ = {};
    gsa_ = {};
    rmc_ = {};
    agrica_ = {};
    heading_ = {};
    last_frame_arrival_us_ = 0U;
    last_valid_data_arrival_us_ = 0U;
    last_gga_arrival_us_ = 0U;
    last_gst_arrival_us_ = 0U;
    last_gsa_arrival_us_ = 0U;
    last_rmc_arrival_us_ = 0U;
    last_agrica_arrival_us_ = 0U;
    last_heading_arrival_us_ = 0U;
    last_receiver_status_publish_us_ = 0U;
    /* UART statistics are boot-cumulative and include expected errors from
     * wrong-baud probe candidates. Only errors after this session starts may
     * affect the operational GPS stream validator. */
    // HAL 计数器自启动累计，错误波特率探测产生的线路错误是预期现象；这里建立
    // 新会话基线，运行期验证只接收此后新增的 UART 错误和丢字节。
    const auto uart_stats = uart_.stats();
    gps_error_counter_.begin_session(uart_stats.receive_errors,
                                     uart_stats.dropped_bytes);
    stream_validator_.reset();
    validation_fault_active_ = false;
    protocol_checksum_errors_ = 0U;
    protocol_structure_errors_ = 0U;
    protocol_overflow_errors_ = 0U;
    timestamp_errors_ = 0U;
    sample_structure_errors_ = 0U;
    gga_new_ = false;
    agrica_new_ = false;
    heading_new_ = false;
    reset_diagnostics();
}

void Um982Gps::reset_diagnostics() noexcept
{
    // 诊断窗口与当前 UART/波特率会话绑定；切换候选或时钟倒退时，旧计数不能
    // 混入新会话并伪造某类消息仍在持续到达。
    for (auto &count : diagnostic_frame_counts_) {
        count = 0U;
    }
    diagnostic_other_frames_ = 0U;
    diagnostic_window_started_us_ = 0U;
    diagnostic_fix_type_ = 0U;
    diagnostic_satellites_ = 0U;
}

void Um982Gps::record_diagnostic_frame(
    const dima::protocols::um982::Um982Protocol::Frame &frame,
    std::uint64_t arrival_us) noexcept
{
    namespace generated = dima::protocols::um982::generated;
    if (frame.message_contract_index < generated::kMessageContractCount) {
        saturating_increment(
            diagnostic_frame_counts_[frame.message_contract_index]);
        if (diagnostic_window_started_us_ == 0U) {
            diagnostic_window_started_us_ = arrival_us;
        }
    } else if (frame.kind ==
               dima::protocols::um982::Um982Protocol::Kind::Unknown) {
        // GSV 等合同外但校验合法的 UM982/NMEA 帧单独汇总，不把它们当错误，
        // 也不能让其掩盖六类产品合同消息中的缺流。
        saturating_increment(diagnostic_other_frames_);
    }
}

void Um982Gps::maybe_report_diagnostics(std::uint64_t now_us) noexcept
{
    namespace generated = dima::protocols::um982::generated;
    using Role = generated::MessageRole;
    if (receiver_status_ != ReceiverStatus::Operational ||
        diagnostic_window_started_us_ == 0U) {
        return;
    }
    if (now_us < diagnostic_window_started_us_) {
        reset_diagnostics();
        return;
    }
    const std::uint64_t elapsed_us =
        now_us - diagnostic_window_started_us_;
    if (elapsed_us < kDiagnosticReportIntervalUs) {
        return;
    }

    const auto role_count = [this](Role role) noexcept -> std::uint16_t {
        const std::size_t index = generated::find_message_role(role);
        return index < generated::kMessageContractCount
                   ? diagnostic_frame_counts_[index]
                   : 0U;
    };
    const auto display_count = [](std::uint32_t value) noexcept
        -> unsigned int {
        // 文本字段钳位到三位数，既覆盖正常 10 s/10 Hz 的约 100 帧，也保证
        // 整条状态不超过 mavlink_log 的 126 字节有效载荷。
        return static_cast<unsigned int>(
            std::min<std::uint32_t>(value, 999U));
    };
    const std::uint64_t rounded_seconds =
        (elapsed_us + 500000ULL) / 1000000ULL;
    const auto seconds = static_cast<unsigned long>(
        std::min<std::uint64_t>(rounded_seconds, 999ULL));
    // gga/agr/hdg/gst/gsa/rmc 是协议语义角色，实际名称及其位号仍由生成合同
    // 决定。10 s 窗口在 10 Hz 正常情况下各约为 100，某项为 0 可直接判缺流。
    UM982_QGC_INFO("GPS S%ld b=%lu cfg=%u fix=%u sat=%u dt=%lus "
             "gga=%u agr=%u hdg=%u gst=%u gsa=%u rmc=%u oth=%u "
             "e=%u/%u/%u",
             static_cast<long>(active_port_),
             static_cast<unsigned long>(detected_baudrate_),
             configuration_complete_ ? 1U : 0U,
             static_cast<unsigned int>(diagnostic_fix_type_),
             static_cast<unsigned int>(diagnostic_satellites_),
             seconds,
             display_count(role_count(Role::Gga)),
             display_count(role_count(Role::Agrica)),
             display_count(role_count(Role::Heading)),
             display_count(role_count(Role::Gst)),
             display_count(role_count(Role::Gsa)),
             display_count(role_count(Role::Rmc)),
             display_count(diagnostic_other_frames_),
             display_count(protocol_checksum_errors_),
             display_count(protocol_structure_errors_),
             display_count(protocol_overflow_errors_));

    for (auto &count : diagnostic_frame_counts_) {
        count = 0U;
    }
    diagnostic_other_frames_ = 0U;
    diagnostic_window_started_us_ = now_us;
}

void Um982Gps::handle_frame(
    const dima::protocols::um982::Um982Protocol::Frame &frame,
    std::uint64_t arrival_us) noexcept
{
    using Kind = dima::protocols::um982::Um982Protocol::Kind;
    if (frame.kind == Kind::BadChecksum ||
        frame.kind == Kind::BadStructure || frame.kind == Kind::Overflow) {
        record_protocol_failure(frame);
        return;
    }
    // 协议语法失败只进入有界计数；只有已解析测量才证明接收机数据面活跃。
    bool receiver_measurement = false;
    if (last_frame_arrival_us_ != 0U &&
        arrival_us < last_frame_arrival_us_) {
        clear_measurement_cache();
        saturating_increment(timestamp_errors_);
        gps_error_counter_.record();
        stream_validator_.reject(
            dima::lib::sensors::validation::StreamFailureTimestamp, 0U);
    }
    last_frame_arrival_us_ = arrival_us;
    record_diagnostic_frame(frame, arrival_us);
    // 配置响应更新控制面；六类测量按各自到达时间缓存。GGA 触发一次融合发布，
    // GST/GSA/RMC/AGRICA/HEADING 只作为有 freshness 上限的辅助来源。
    switch (frame.kind) {
    case Kind::Version:
        version_seen_ = frame.version.is_um982;
        break;
    case Kind::ConfigPort:
        if (frame.config_port.port >= 1U &&
            frame.config_port.port <=
                dima::protocols::um982::Um982Protocol::kReceiverPortCount) {
            const std::uint8_t port_bit = static_cast<std::uint8_t>(
                1U << (frame.config_port.port - 1U));
            const bool first_response = (config_mask_ & port_bit) == 0U;
            port_config_[frame.config_port.port - 1U] = frame.config_port;
            config_mask_ |= port_bit;
            const bool configuration_query =
                phase_ == Phase::ReadConfiguration ||
                phase_ == Phase::VerifyConfiguration;
            if (first_response && configuration_query) {
                // 同一端口的重复回显不能无限伪造进度；只有本轮首次出现的新端口
                // 才更新时间并在 Verify 中触发一次 maintenance 进度。
                last_config_response_arrival_us_ = arrival_us;
                config_response_progress_pending_ =
                    phase_ == Phase::VerifyConfiguration;
            }
        }
        break;
    case Kind::UnilogList:
        unilog_ = frame.unilog_list;
        unilog_seen_ = true;
        break;
    case Kind::UnilogListEntry:
        consume_unilog_list_entry(frame.unilog_list, arrival_us);
        break;
    case Kind::Gga:
        gga_ = frame.gga;
        last_gga_arrival_us_ = arrival_us;
        last_valid_data_arrival_us_ = arrival_us;
        gga_new_ = true;
        receiver_measurement = true;
        break;
    case Kind::Gst:
        gst_ = frame.gst;
        last_gst_arrival_us_ = arrival_us;
        break;
    case Kind::Gsa:
        gsa_ = frame.gsa;
        last_gsa_arrival_us_ = arrival_us;
        break;
    case Kind::Rmc:
        rmc_ = frame.rmc;
        last_rmc_arrival_us_ = arrival_us;
        last_valid_data_arrival_us_ = arrival_us;
        receiver_measurement = true;
        break;
    case Kind::Agrica:
        agrica_ = frame.agrica;
        last_agrica_arrival_us_ = arrival_us;
        last_valid_data_arrival_us_ = arrival_us;
        agrica_new_ = true;
        receiver_measurement = true;
        break;
    case Kind::Heading:
        heading_ = frame.heading;
        last_heading_arrival_us_ = arrival_us;
        last_valid_data_arrival_us_ = arrival_us;
        heading_new_ = true;
        receiver_measurement = true;
        break;
    default:
        break;
    }
    publish_if_ready(arrival_us);
    if (receiver_measurement &&
        !sample_is_fresh(arrival_us, last_gga_arrival_us_,
                         kAuxiliaryFreshnessUs)) {
        publish_receiver_status(arrival_us);
    }
}

void Um982Gps::publish_receiver_status(std::uint64_t now_us) noexcept
{
    // 没有新鲜 GGA 但仍收到其他测量时，以最多 2 Hz 发布 NO_FIX。位置、速度、
    // 精度和航向使用 NaN 表达“接收机在线但当前不可用”，不能伪造为数值 0。
    if (last_receiver_status_publish_us_ != 0U &&
        now_us >= last_receiver_status_publish_us_ &&
        now_us - last_receiver_status_publish_us_ <
            kReceiverStatusPublishIntervalUs) {
        return;
    }

    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    const double unavailable_double =
        std::numeric_limits<double>::quiet_NaN();
    sensor_gps_s output{};
    output.timestamp = now_us;
    output.timestamp_sample = now_us;
    output.device_id = kGpsDeviceBase |
        static_cast<std::uint32_t>(active_port_ & 0xFF);
    output.latitude_deg = unavailable_double;
    output.longitude_deg = unavailable_double;
    output.altitude_msl_m = unavailable_double;
    output.altitude_ellipsoid_m = unavailable_double;
    output.s_variance_m_s = unavailable;
    output.c_variance_rad = unavailable;
    output.fix_type = sensor_gps_s::FIX_TYPE_NONE;
    output.eph = unavailable;
    output.epv = unavailable;
    output.hdop = unavailable;
    output.vdop = unavailable;
    output.vel_m_s = unavailable;
    output.vel_n_m_s = unavailable;
    output.vel_e_m_s = unavailable;
    output.vel_d_m_s = unavailable;
    output.cog_rad = unavailable;
    output.vel_ned_valid = false;
    output.jamming_state = sensor_gps_s::JAMMING_STATE_UNKNOWN;
    output.spoofing_state = sensor_gps_s::SPOOFING_STATE_UNKNOWN;
    output.authentication_state =
        sensor_gps_s::AUTHENTICATION_STATE_UNKNOWN;
    output.system_error = sensor_gps_s::SYSTEM_ERROR_OK;
    output.heading = unavailable;
    output.heading_offset = yaw_offset_rad_;
    output.heading_accuracy = unavailable;
    output.rtcm_msg_used = sensor_gps_s::RTCM_MSG_USED_UNKNOWN;
    if (publish_validated(output, now_us)) {
        last_receiver_status_publish_us_ = now_us;
    }
}

void Um982Gps::publish_if_ready(std::uint64_t now_us) noexcept
{
    if (!gga_new_ || last_gga_arrival_us_ == 0U) {
        return;
    }

    // GGA 是 10 Hz 合成节拍；输出 sample 时间取 GGA 与新鲜 AGRICA 中较晚者。
    sensor_gps_s output{};
    output.timestamp = now_us;
    const bool agrica_fresh = sample_is_fresh(
        now_us, last_agrica_arrival_us_, kAuxiliaryFreshnessUs);
    output.timestamp_sample = agrica_fresh
        ? std::max(last_gga_arrival_us_, last_agrica_arrival_us_)
        : last_gga_arrival_us_;
    output.device_id = kGpsDeviceBase |
        static_cast<std::uint32_t>(active_port_ & 0xFF);
    output.latitude_deg = gga_.latitude_deg;
    output.longitude_deg = gga_.longitude_deg;
    output.altitude_msl_m = gga_.altitude_msl_m;
    // 椭球高 = MSL 海拔 + geoid separation，三者单位均为米。
    output.altitude_ellipsoid_m =
        static_cast<double>(gga_.altitude_msl_m + gga_.geoid_separation_m);
    const bool gsa_fresh = sample_is_fresh(
        now_us, last_gsa_arrival_us_, kAuxiliaryFreshnessUs);
    const bool gst_fresh = sample_is_fresh(
        now_us, last_gst_arrival_us_, kAuxiliaryFreshnessUs);
    const bool rmc_fresh = sample_is_fresh(
        now_us, last_rmc_arrival_us_, kAuxiliaryFreshnessUs);
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    output.fix_type = fix_type_from_gga(
        gga_.fix_quality, gsa_fresh ? gsa_.fix_dimension : 0U);
    // 水平 1-sigma 合成误差 eph = sqrt(sigma_lat^2 + sigma_lon^2)，单位 m；
    // 垂直误差直接使用 GST altitude_stddev，超龄时必须保持 NaN。
    output.eph = gst_fresh
                     ? std::sqrt(gst_.latitude_stddev_m *
                                     gst_.latitude_stddev_m +
                                 gst_.longitude_stddev_m *
                                     gst_.longitude_stddev_m)
                     : unavailable;
    output.epv = gst_fresh ? gst_.altitude_stddev_m : unavailable;
    output.hdop = gga_.hdop;
    output.vdop = gsa_fresh ? gsa_.vdop : unavailable;
    output.c_variance_rad = unavailable;
    output.vel_m_s = unavailable;
    output.vel_n_m_s = unavailable;
    output.vel_e_m_s = unavailable;
    output.vel_d_m_s = unavailable;
    output.s_variance_m_s = unavailable;
    output.cog_rad = unavailable;
    output.vel_ned_valid = false;
    if (agrica_fresh) {
        // AGRICA 使用 N/E/U，而消息合同使用 N/E/D，因此 vel_d=-vel_up。
        // 三轴速度精度取 RMS：sqrt((sigma_N^2+sigma_E^2+sigma_U^2)/3)。
        // 地面航迹以真北为 0，故 course=atan2(vel_E, vel_N)。
        output.vel_m_s = agrica_.speed_m_s;
        output.vel_n_m_s = agrica_.velocity_north_m_s;
        output.vel_e_m_s = agrica_.velocity_east_m_s;
        output.vel_d_m_s = -agrica_.velocity_up_m_s;
        output.s_variance_m_s = std::sqrt(
            (agrica_.velocity_north_stddev_m_s *
                 agrica_.velocity_north_stddev_m_s +
             agrica_.velocity_east_stddev_m_s *
                 agrica_.velocity_east_stddev_m_s +
             agrica_.velocity_up_stddev_m_s *
                 agrica_.velocity_up_stddev_m_s) / 3.0F);
        output.cog_rad = std::atan2(output.vel_e_m_s, output.vel_n_m_s);
        output.vel_ned_valid =
            output.fix_type >= sensor_gps_s::FIX_TYPE_3D;
    } else if (rmc_fresh && rmc_.valid &&
               std::isfinite(rmc_.ground_speed_m_s) &&
               std::isfinite(rmc_.course_deg)) {
        // AGRICA 不新鲜时仅用 RMC 地速/航迹补充水平分量；缺少垂直速度，
        // 因而不能把 vel_ned_valid 置真。
        const float course_rad = wrap_pi(
            rmc_.course_deg * kDegreesToRadians);
        output.vel_m_s = rmc_.ground_speed_m_s;
        output.vel_n_m_s = output.vel_m_s * std::cos(course_rad);
        output.vel_e_m_s = output.vel_m_s * std::sin(course_rad);
        output.cog_rad = course_rad;
    }
    output.satellites_used = gga_.satellites;
    output.time_utc_usec = rmc_fresh && rmc_.valid
        ? dima::protocols::um982::Um982Protocol::utc_usec(
              rmc_.date_ddmmyy, rmc_.utc_hhmmss_ms)
        : 0U;
    output.timestamp_time_relative = 0;
    output.jamming_state = sensor_gps_s::JAMMING_STATE_UNKNOWN;
    output.spoofing_state = sensor_gps_s::SPOOFING_STATE_UNKNOWN;
    output.authentication_state =
        sensor_gps_s::AUTHENTICATION_STATE_UNKNOWN;
    output.system_error = sensor_gps_s::SYSTEM_ERROR_OK;
    output.heading = unavailable;
    output.heading_offset = yaw_offset_rad_;
    output.heading_accuracy = unavailable;
    // UM982 双天线基线方向与机体期望航向相反，需要加 pi；再减安装偏置：
    // heading=wrap_pi(heading_deg*pi/180 + pi - yaw_offset)。
    if (sample_is_fresh(now_us, last_heading_arrival_us_,
                        kAuxiliaryFreshnessUs) &&
        heading_new_ && heading_.solution_computed &&
        heading_.baseline_m > 0.0F &&
        heading_.heading_stddev_deg > 0.0F) {
        output.heading = wrap_pi(
            heading_.heading_deg * kDegreesToRadians + kPi -
            yaw_offset_rad_);
        output.heading_accuracy =
            heading_.heading_stddev_deg * kDegreesToRadians;
    }
    output.rtcm_msg_used = sensor_gps_s::RTCM_MSG_USED_UNKNOWN;
    (void)publish_validated(output, now_us);
    gga_new_ = false;
    agrica_new_ = false;
    heading_new_ = false;
}

bool Um982Gps::publish_validated(sensor_gps_s &output,
                                 std::uint64_t now_us) noexcept
{
    // 校验分三层：结构层检查单个消息字段/单位；流层检查时间戳、超时和错误
    // 密度。两层失败才禁止原始 Topic；fix/卫星数/PDOP/精度以及静止漂移检查
    // 统一交给 EKF2 GnssChecks，避免驱动与估计器维护冲突的双健康状态机。
    namespace validation = dima::lib::sensors::validation;
    validation::GpsSample sample{};
    sample.timestamp = output.timestamp;
    sample.timestamp_sample = output.timestamp_sample;
    sample.device_id = output.device_id;
    sample.latitude_deg = output.latitude_deg;
    sample.longitude_deg = output.longitude_deg;
    sample.altitude_msl_m = output.altitude_msl_m;
    sample.altitude_ellipsoid_m = output.altitude_ellipsoid_m;
    sample.eph = output.eph;
    sample.epv = output.epv;
    sample.hdop = output.hdop;
    sample.vdop = output.vdop;
    sample.speed_accuracy_m_s = output.s_variance_m_s;
    sample.velocity_m_s = output.vel_m_s;
    sample.velocity_n_m_s = output.vel_n_m_s;
    sample.velocity_e_m_s = output.vel_e_m_s;
    sample.velocity_d_m_s = output.vel_d_m_s;
    sample.course_rad = output.cog_rad;
    sample.heading_rad = output.heading;
    sample.heading_offset_rad = output.heading_offset;
    sample.heading_accuracy_rad = output.heading_accuracy;
    sample.fix_type = output.fix_type;
    sample.satellites_used = output.satellites_used;
    sample.spoofing_state = output.spoofing_state;
    sample.authentication_state = output.authentication_state;
    sample.velocity_ned_valid = output.vel_ned_valid;

    const validation::GpsStructureResult structure =
        validation::validate_gps_structure(sample);
    if (!structure.valid()) {
        saturating_increment(sample_structure_errors_);
        gps_error_counter_.record();
        stream_validator_.reject(
            (structure.failure_mask & validation::GpsFailureTimestamp) != 0U
                ? validation::StreamFailureTimestamp
                : validation::StreamFailureInvalidValue,
            0U);
        report_validation_failure(
            structure.failure_mask,
            stream_validator_.evaluate(now_us).failure_mask);
        return false;
    }

    update_uart_error_count();
    const float validator_values[3]{
        static_cast<float>(output.fix_type),
        static_cast<float>(output.satellites_used),
        std::isfinite(output.hdop) ? output.hdop : 0.0F};
    if (!stream_validator_.put(output.timestamp, validator_values,
                               gps_error_counter_.total())) {
        saturating_increment(timestamp_errors_);
        report_validation_failure(
            validation::GpsFailureTimestamp,
            stream_validator_.evaluate(now_us).failure_mask);
        return false;
    }
    const validation::StreamValidity stream =
        stream_validator_.evaluate(now_us);
    if (!stream.healthy()) {
        report_validation_failure(validation::GpsFailureNone,
                                  stream.failure_mask);
        return false;
    }
    validation_fault_active_ = false;
    diagnostic_fix_type_ = output.fix_type;
    diagnostic_satellites_ = output.satellites_used;

    (void)sensor_gps_publication_.publish(output);
    (void)vehicle_gps_publication_.publish(output);
    return true;
}

void Um982Gps::record_protocol_failure(
    const dima::protocols::um982::Um982Protocol::Frame &frame) noexcept
{
    using Kind = dima::protocols::um982::Um982Protocol::Kind;
    switch (frame.kind) {
    case Kind::BadChecksum:
        saturating_increment(protocol_checksum_errors_);
        break;
    case Kind::BadStructure:
        saturating_increment(protocol_structure_errors_);
        break;
    case Kind::Overflow:
        saturating_increment(protocol_overflow_errors_);
        break;
    default:
        return;
    }
    /* PX4-GPSDrivers silently drops recoverable NMEA/Unicore syntax failures.
     * Keep bounded counters for the offline edge summary, but do not poison
     * DataValidator error density or publish per-frame QGC errors. */
    // 可恢复语法错误只在离线边沿汇总，不进入 DataValidator density，也不逐帧
    // 向 QGC 报错；否则少量线路噪声会被错误放大成持续 GPS 健康故障。
}

void Um982Gps::update_uart_error_count() noexcept
{
    const auto stats = uart_.stats();
    gps_error_counter_.update_uart(stats.receive_errors,
                                   stats.dropped_bytes);
}

void Um982Gps::report_validation_failure(
    std::uint32_t structure_mask, std::uint32_t stream_mask) noexcept
{
    // structure_mask 描述当前样本字段，stream_mask 描述跨样本时间/错误密度；
    // 两者属于不同语义层，日志必须并列保留，不能只看其中一个推断硬件故障。
    if (validation_fault_active_) {
        return;
    }
    validation_fault_active_ = true;
    const std::uint64_t now_us = clock_.now_us();
    if (last_validation_report_us_ != 0U &&
        now_us >= last_validation_report_us_ &&
        now_us - last_validation_report_us_ < kValidationReportIntervalUs) {
        return;
    }
    last_validation_report_us_ = now_us;
    UM982_QGC_ERR("GPS invalid structure=0x%lx stream=0x%lx errors=%lu density=%lu",
            static_cast<unsigned long>(structure_mask),
            static_cast<unsigned long>(stream_mask),
            static_cast<unsigned long>(gps_error_counter_.total()),
            static_cast<unsigned long>(stream_validator_.error_density()));
}

void Um982Gps::Run()
{
    if (module_state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    // 每轮先排空有界 UART 数据，再处理参数与状态机。达到 2048 B 预算时临时
    // 屏蔽 ISR 重复唤醒并延后 1 ms，防止 ScheduleNow 风暴抵消主动让步。
    __atomic_store_n(&rx_schedule_suppressed_, false, __ATOMIC_RELEASE);
    if (drain_uart()) {
        ++rx_budget_yields_;
        __atomic_store_n(&rx_schedule_suppressed_, true, __ATOMIC_RELEASE);
        schedule(kRxYieldUs);
        return;
    }
    if (parameter_subscription_.update()) {
        float next_yaw_offset = 0.0F;
        if (read_yaw_offset(next_yaw_offset) &&
            next_yaw_offset != yaw_offset_rad_) {
            yaw_offset_rad_ = next_yaw_offset;
        }
    }
    const std::uint64_t now_us = clock_.now_us();
    // 串口分配是运行期可变资源合同；变化时必须先释放 maintenance/UART，清除
    // 协议半帧和融合缓存，再回到 WaitAssignment 重新建立唯一所有权。
    if (phase_ != Phase::WaitAssignment && assignment_changed()) {
        if (maintenance_ticket_ != 0U) {
            maintenance_.cancel(maintenance_ticket_);
            maintenance_ticket_ = 0U;
        }
        if (maintenance_interlock_acquired_) {
            armed_.end_maintenance();
            maintenance_interlock_acquired_ = false;
        }
        maintenance_ready_ = false;
        candidate_active_ = false;
        command_pending_ = false;
        (void)uart_.stop();
        protocol_.reset();
        clear_measurement_cache();
        configuration_complete_ = false;
        transition(Phase::WaitAssignment);
        return;
    }

    maybe_report_diagnostics(now_us);
    // 单一 switch 是配置状态机的唯一调度入口，阶段函数只负责本阶段推进。
    switch (phase_) {
    case Phase::WaitAssignment:
        run_assignment();
        return;

    case Phase::Detect:
        run_detect(now_us);
        return;

    case Phase::ReadConfiguration:
        run_configuration_read(now_us, false);
        return;

    case Phase::ApplyConfiguration:
        run_configuration_apply(now_us);
        return;

    case Phase::VerifyConfiguration:
        run_configuration_read(now_us, true);
        return;

    case Phase::SaveConfiguration:
        run_configuration_save(now_us);
        return;

    case Phase::Run:
        run_normal(now_us);
        return;
    }
}

} // namespace dima::drivers::gps
