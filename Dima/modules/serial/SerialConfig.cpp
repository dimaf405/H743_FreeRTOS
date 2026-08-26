#define MODULE_NAME "serial"
#include "SerialConfig.hpp"
#include "Um982MessageContract.hpp"

#include "logging/logging.hpp"

namespace dima::modules::serial {
namespace {

constexpr dima::platform::SerialLineConfiguration
normal_line_configuration(std::uint32_t baudrate) noexcept
{
    // 普通遥测/GPS 端口统一使用 8N1；SBUS 的 100000 8E2 反相配置由专用驱动持有。
    dima::platform::SerialLineConfiguration configuration{};
    configuration.baudrate = baudrate;
    configuration.data_bits = 8U;
    configuration.parity = dima::platform::SerialParity::None;
    configuration.stop_bits = dima::platform::SerialStopBits::One;
    configuration.rx_pull = dima::platform::SerialRxPull::Preserve;
    configuration.rx_enabled = true;
    configuration.tx_enabled = true;
    return configuration;
}

bool gps_protocol_supported(std::int32_t protocol) noexcept
{
    return protocol == 0 || protocol == 6;
}

} // namespace

SerialConfig::SerialConfig(dima::platform::SerialPorts &backend) noexcept
    : backend_(backend)
{
}

bool SerialConfig::bind_parameters() noexcept
{
    // 绑定顺序及成员集合来自生成宏，避免参数注册表与板级端口数量发生漂移。
    bool bound = gps1_config_.bind() && gps1_protocol_.bind();
#define DIMA_BIND_SERIAL_PARAMETERS(index, baud, function) \
    bound = serial##index##_baud_.bind() && \
            serial##index##_function_.bind() && bound;
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_BIND_SERIAL_PARAMETERS)
#undef DIMA_BIND_SERIAL_PARAMETERS
    return bound;
}

void SerialConfig::invalidate_parameters() noexcept
{
    gps1_config_.invalidate();
    gps1_protocol_.invalidate();
#define DIMA_INVALIDATE_SERIAL_PARAMETERS(index, baud, function) \
    serial##index##_baud_.invalidate(); \
    serial##index##_function_.invalidate();
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_INVALIDATE_SERIAL_PARAMETERS)
#undef DIMA_INVALIDATE_SERIAL_PARAMETERS
}

bool SerialConfig::read_configuration(
    Configuration &configuration) const noexcept
{
    // 先构造候选快照并验证唯一所有权，校验完成以前不修改当前生效配置。
    configuration = {};
    bool configuration_valid = true;
    unsigned rc_owner_count = 0U;
    unsigned legacy_gps_owner_count = 0U;
    std::int32_t legacy_gps_port = 0;
#define DIMA_READ_SERIAL_PARAMETERS(index, baud, function) \
    do { \
        const std::int32_t baud_value = serial##index##_baud_.get(); \
        const std::int32_t function_value = serial##index##_function_.get(); \
        if (baud_value < 0 || \
            !dima::board::serial_baud_supported( \
                static_cast<std::uint32_t>(baud_value)) || \
            !dima::board::serial_function_supported(function_value)) { \
            configuration_valid = false; \
        } else { \
            configuration.baudrate[index - 1U] = \
                static_cast<std::uint32_t>(baud_value); \
        } \
        if (function_value == dima::board::kSerialFunctionRcInput) { \
            ++rc_owner_count; \
            configuration.rc_input_port = index; \
        } else if (function_value == dima::board::kSerialFunctionGps) { \
            ++legacy_gps_owner_count; \
            legacy_gps_port = index; \
        } \
    } while (false);
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_READ_SERIAL_PARAMETERS)
#undef DIMA_READ_SERIAL_PARAMETERS

    // GPS_1_CONFIG 是当前主合同；仅当其为 0 时接受唯一旧式
    // SERIALx_FUNCTION=GPS。两者同时存在但指向不同端口时必须拒绝。
    const std::int32_t configured_gps_port = gps1_config_.get();
    const std::int32_t configured_protocol = gps1_protocol_.get();
    if (configured_gps_port < 0 ||
        configured_gps_port > static_cast<std::int32_t>(kPortCount) ||
        !gps_protocol_supported(configured_protocol) ||
        legacy_gps_owner_count > 1U ||
        (configured_gps_port != 0 && legacy_gps_port != 0 &&
         configured_gps_port != legacy_gps_port)) {
        configuration_valid = false;
    }
    configuration.gps_port = configured_gps_port != 0
        ? configured_gps_port : legacy_gps_port;
    if (configuration.gps_port > 0 &&
        configuration.gps_port <= static_cast<std::int32_t>(kPortCount)) {
        // UM982 运行波特率属于生成消息合同，不能由本模块另写常量或沿用旧参数值。
        configuration.baudrate[
            static_cast<std::size_t>(configuration.gps_port - 1)] =
                dima::protocols::um982::generated::kTargetBaudrate;
    }
    configuration.gps_target_baudrate = configuration.gps_port == 0
        ? 0U : dima::protocols::um982::generated::kTargetBaudrate;
    configuration.gps_protocol = configured_protocol;

    if (!configuration_valid || rc_owner_count > 1U) {
        return false;
    }
    return configuration.rc_input_port == 0 ||
           configuration.rc_input_port != configuration.gps_port;
}

bool SerialConfig::apply_baudrates(const std::uint32_t *baudrates) noexcept
{
    if (baudrates == nullptr) return false;
    // 遍历集合仍由板级生成清单提供；返回值汇总所有端口，防止半套配置被误判成功。
    bool configured = true;
#define DIMA_APPLY_SERIAL_BAUD(index, baud, function) \
    if (baudrates[index - 1U] != 0U) { \
        configured = backend_.configure_line( \
            index, normal_line_configuration(baudrates[index - 1U])) && \
            configured; \
    }
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_APPLY_SERIAL_BAUD)
#undef DIMA_APPLY_SERIAL_BAUD
    return configured;
}

void SerialConfig::commit_configuration(
    const Configuration &configuration) noexcept
{
    rc_input_port_ = configuration.rc_input_port;
    gps_port_ = configuration.gps_port;
    gps_target_baudrate_ = configuration.gps_target_baudrate;
    for (std::size_t index = 0U; index < kPortCount; ++index) {
        applied_baudrates_[index] = configuration.baudrate[index];
    }
}

bool SerialConfig::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }

    rc_input_port_ = 0;
    gps_port_ = 0;
    gps_target_baudrate_ = 0U;
    if (!bind_parameters()) {
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("board serial parameters unavailable");
        return false;
    }

    Configuration configuration{};
    if (!read_configuration(configuration)) {
        rc_input_port_ = 0;
        gps_port_ = 0;
        gps_target_baudrate_ = 0U;
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("invalid serial configuration");
        return false;
    }

    if (!apply_baudrates(configuration.baudrate)) {
        (void)backend_.reset_configuration();
        rc_input_port_ = 0;
        gps_port_ = 0;
        gps_target_baudrate_ = 0U;
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("normal serial configuration failed");
        return false;
    }

    commit_configuration(configuration);
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("configured SERIAL1..SERIAL8 rc_port=%ld gps_port=%ld gps_baud=%lu protocol=%ld",
             static_cast<long>(rc_input_port_),
             static_cast<long>(gps_port_),
             static_cast<unsigned long>(gps_target_baudrate_),
             static_cast<long>(configuration.gps_protocol));
    return true;
}

bool SerialConfig::reconfigure() noexcept
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return false;
    }
    Configuration configuration{};
    if (!read_configuration(configuration)) {
        PX4_ERR("pending serial configuration invalid; retaining active ports");
        return false;
    }
    if (!apply_baudrates(configuration.baudrate)) {
        // 新配置任一端口应用失败时回滚全部旧波特率；所有权字段只有成功后才提交。
        const bool restored = apply_baudrates(applied_baudrates_);
        if (!restored) {
            PX4_ERR("serial baud rollback failed");
        }
        return false;
    }
    commit_configuration(configuration);
    PX4_INFO("reconfigured SERIAL1..SERIAL8 rc_port=%ld gps_port=%ld gps_baud=%lu protocol=%ld",
             static_cast<long>(rc_input_port_),
             static_cast<long>(gps_port_),
             static_cast<unsigned long>(gps_target_baudrate_),
             static_cast<long>(configuration.gps_protocol));
    return true;
}

bool SerialConfig::pending_configuration_valid() const noexcept
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return false;
    }
    Configuration configuration{};
    return read_configuration(configuration);
}

void SerialConfig::stop() noexcept
{
    invalidate_parameters();
    rc_input_port_ = 0;
    gps_port_ = 0;
    gps_target_baudrate_ = 0U;
    for (std::uint32_t &baudrate : applied_baudrates_) {
        baudrate = 0U;
    }
    state_ = backend_.reset_configuration()
                 ? dima::middleware::lifecycle::ModuleState::Stopped
                 : dima::middleware::lifecycle::ModuleState::Error;
}

dima::middleware::lifecycle::ModuleState SerialConfig::state() const noexcept
{
    return state_;
}

std::int32_t SerialConfig::rc_input_port() const noexcept
{
    return state_ == dima::middleware::lifecycle::ModuleState::Running
               ? rc_input_port_
               : 0;
}

std::int32_t SerialConfig::gps_port() const noexcept
{
    return state_ == dima::middleware::lifecycle::ModuleState::Running
               ? gps_port_
               : 0;
}

std::uint32_t SerialConfig::gps_target_baudrate() const noexcept
{
    return state_ == dima::middleware::lifecycle::ModuleState::Running
               ? gps_target_baudrate_
               : 0U;
}

std::uint64_t SerialConfig::configuration_signature() const noexcept
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return 0U;
    }
    // FNV-1a 64 位签名按小端字节追加：hash = (hash XOR byte) * 1099511628211。
    // 它只用于检测配置代际变化，不承担密码学完整性校验。
    std::uint64_t hash = 14695981039346656037ULL;
    const auto append = [&hash](std::int32_t value) noexcept {
        const std::uint32_t bits = static_cast<std::uint32_t>(value);
        for (unsigned shift = 0U; shift < 32U; shift += 8U) {
            hash ^= static_cast<std::uint8_t>(bits >> shift);
            hash *= 1099511628211ULL;
        }
    };
#define DIMA_HASH_SERIAL(index, baud, function) \
    do { \
        append(serial##index##_baud_.get()); \
        append(serial##index##_function_.get()); \
    } while (false);
    DIMA_BOARD_SERIAL_PARAMETER_LIST(DIMA_HASH_SERIAL)
#undef DIMA_HASH_SERIAL
    append(gps1_config_.get());
    append(gps1_protocol_.get());
    return hash;
}

} // namespace dima::modules::serial
