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
    for (ParameterBinding &binding : serial_parameters_) {
        binding = {};
    }
    if (!gps1_protocol_.bind()) {
        return false;
    }

    // Dima 生成参数目录是运行时唯一目录；按 SERIALx 命名规则发现参数，
    // 从而新增或删除 YAML 条目时无需同步修改 C++ 参数成员或名称数组。
    for (unsigned index = 0U; index < param_count(); ++index) {
        const param_t handle = param_for_index(index);
        const dima::lib::serial::SerialParameterIdentity identity =
            dima::lib::serial::identify_serial_parameter(param_name(handle));
        if (identity.kind ==
                dima::lib::serial::SerialParameterKind::None) {
            continue;
        }
        if (identity.port == 0U || identity.port > kPortCount ||
            handle == PARAM_INVALID || param_type(handle) != PARAM_TYPE_INT32) {
            return false;
        }

        ParameterBinding &binding = serial_parameters_[identity.port - 1U];
        param_t &destination = identity.kind ==
                dima::lib::serial::SerialParameterKind::Baud
            ? binding.baud : binding.function;
        if (destination != PARAM_INVALID) {
            return false;
        }
        destination = handle;
    }

    bool found_serial_port = false;
    for (const ParameterBinding &binding : serial_parameters_) {
        const bool has_baud = binding.baud != PARAM_INVALID;
        const bool has_function = binding.function != PARAM_INVALID;
        if (has_baud != has_function) {
            return false;
        }
        if (has_baud) {
            param_set_used(binding.baud);
            param_set_used(binding.function);
            found_serial_port = true;
        }
    }
    return found_serial_port;
}

void SerialConfig::invalidate_parameters() noexcept
{
    gps1_protocol_.invalidate();
    for (ParameterBinding &binding : serial_parameters_) {
        binding = {};
    }
}

bool SerialConfig::read_configuration(
    Configuration &configuration) const noexcept
{
    // 先构造候选快照并验证唯一所有权，校验完成以前不修改当前生效配置。
    configuration = {};
    bool configuration_valid = true;
    unsigned sbus_owner_count = 0U;
    unsigned gps_owner_count = 0U;
    for (std::size_t index = 0U; index < kPortCount; ++index) {
        const ParameterBinding &binding = serial_parameters_[index];
        if (binding.baud == PARAM_INVALID &&
            binding.function == PARAM_INVALID) {
            continue;
        }

        std::int32_t baud_value = 0;
        std::int32_t function_value = 0;
        if (binding.baud == PARAM_INVALID ||
            binding.function == PARAM_INVALID ||
            param_get(binding.baud, &baud_value) != 0 ||
            param_get(binding.function, &function_value) != 0 ||
            baud_value < 0 ||
            !dima::lib::serial::serial_function_supported(function_value)) {
            configuration_valid = false;
            continue;
        }

        const std::int32_t port = static_cast<std::int32_t>(index + 1U);
        configuration.baudrate[index] =
            static_cast<std::uint32_t>(baud_value);
        if (function_value == dima::lib::serial::kSerialFunctionSbus) {
            ++sbus_owner_count;
            configuration.rc_input_port = port;
        } else if (function_value ==
                   dima::lib::serial::kSerialFunctionGps) {
            ++gps_owner_count;
            configuration.gps_port = port;
        }
    }

    // GPS 端口只由 SERIALx_FUNCTION=GPS 决定；唯一性校验避免两个驱动争用 UART。
    std::int32_t configured_protocol = 0;
    if (param_get(gps1_protocol_.handle(), &configured_protocol) != 0 ||
        !gps_protocol_supported(configured_protocol) || gps_owner_count > 1U) {
        configuration_valid = false;
    }
    if (configuration.gps_port > 0) {
        // UM982 运行波特率属于生成消息合同，不能由本模块另写常量或沿用旧参数值。
        configuration.baudrate[
            static_cast<std::size_t>(configuration.gps_port - 1)] =
                dima::protocols::um982::generated::kTargetBaudrate;
    }
    configuration.gps_target_baudrate = configuration.gps_port == 0
        ? 0U : dima::protocols::um982::generated::kTargetBaudrate;
    configuration.gps_protocol = configured_protocol;

    if (!configuration_valid || sbus_owner_count > 1U) {
        return false;
    }
    return configuration.rc_input_port == 0 ||
           configuration.rc_input_port != configuration.gps_port;
}

bool SerialConfig::apply_baudrates(const std::uint32_t *baudrates) noexcept
{
    if (baudrates == nullptr) return false;
    // 只遍历 Dima 生成参数目录实际发现的成对参数；稀疏的 SERIAL5 槽不会触发
    // UART 配置。返回值汇总全部物理端口，防止半套配置被误判成功。
    bool configured = true;
    for (std::size_t index = 0U; index < kPortCount; ++index) {
        if (serial_parameters_[index].baud == PARAM_INVALID ||
            baudrates[index] == 0U) {
            continue;
        }
        configured = backend_.configure_line(
            static_cast<std::int32_t>(index + 1U),
            normal_line_configuration(baudrates[index])) && configured;
    }
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
    PX4_INFO("configured physical serial ports rc_port=%ld gps_port=%ld gps_baud=%lu protocol=%ld",
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
    PX4_INFO("reconfigured physical serial ports rc_port=%ld gps_port=%ld gps_baud=%lu protocol=%ld",
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
    for (const ParameterBinding &binding : serial_parameters_) {
        if (binding.baud == PARAM_INVALID &&
            binding.function == PARAM_INVALID) {
            continue;
        }
        std::int32_t baud = 0;
        std::int32_t function = 0;
        if (binding.baud == PARAM_INVALID ||
            binding.function == PARAM_INVALID ||
            param_get(binding.baud, &baud) != 0 ||
            param_get(binding.function, &function) != 0) {
            return 0U;
        }
        append(baud);
        append(function);
    }
    std::int32_t gps_protocol = 0;
    if (param_get(gps1_protocol_.handle(), &gps_protocol) != 0) {
        return 0U;
    }
    append(gps_protocol);
    return hash;
}

} // namespace dima::modules::serial
