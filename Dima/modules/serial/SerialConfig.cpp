#define MODULE_NAME "serial"
#include "SerialConfig.hpp"
#include "Um982MessageContract.hpp"

#include "logging/logging.hpp"
#include "parameters/atomic_transaction.h"

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
    for (ParameterBinding &binding : serial_parameters_) {
        binding = {};
    }
}

bool SerialConfig::read_configuration(
    Configuration &configuration) const noexcept
{
    // 先构造候选快照并验证唯一所有权，校验完成以前不修改当前生效配置。
    px4::AtomicTransaction transaction;
    configuration = {};
    bool configuration_valid = true;
    unsigned sbus_owner_count = 0U;
    unsigned gps_owner_count = 0U;
    unsigned telemetry_owner_count = 0U;
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

        configuration.requested_baudrate[index] = baud_value;
        configuration.function[index] = function_value;
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
        } else if (function_value == dima::lib::serial::kSerialFunctionMavlink) {
            // 数传线路由独立端点应用 8N1 线格式，失败不能中断启动时的
            // USB/GPS/SBUS。BAUD=0 (Auto) 是合法候选：波特率由 MAVLink 服务
            // 自动探测；显式值直接使用。通用路径不得触碰 owner 端口的线路。
            configuration.baudrate[index] = 0U;
            ++telemetry_owner_count;
            configuration.telemetry_port = port;
            configuration.telemetry_baudrate = static_cast<std::uint32_t>(baud_value);
        }
    }

    // 数传错误只撤销其端口选择；启动时保留 USB/GPS/SBUS。热重配入口会拒绝
    // 同一无效候选，不把已运行的有效配置替换掉。
    configuration.telemetry_valid = configuration.telemetry_valid && telemetry_owner_count <= 1U;
    if (!configuration.telemetry_valid) {
        configuration.telemetry_port = 0;
        configuration.telemetry_baudrate = 0U;
    }

    // 本产品只有 NMEA/UM982 一个 GPS 实现；原 Auto/6 两项不选择不同驱动。
    // 取消重复协议开关，仍由 SERIALx_FUNCTION=GPS 唯一选端口并拒绝所有权冲突。
    if (gps_owner_count > 1U) {
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
    previous_configuration_ = active_configuration_;
    active_configuration_ = configuration;
    telemetry_port_ = configuration.telemetry_port;
    telemetry_baudrate_ = configuration.telemetry_baudrate;
    telemetry_valid_ = configuration.telemetry_valid;
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
    PX4_INFO("configured physical serial ports rc_port=%ld gps_port=%ld gps_baud=%lu GPS=NMEA/UM982",
             static_cast<long>(rc_input_port_),
             static_cast<long>(gps_port_),
             static_cast<unsigned long>(gps_target_baudrate_));
    return true;
}

bool SerialConfig::reconfigure() noexcept
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return false;
    }
    Configuration configuration{};
    if (!read_configuration(configuration) || !configuration.telemetry_valid) {
        PX4_ERR("pending serial configuration invalid; retaining active ports");
        return false;
    }
    /* 运行期重配只在 RC 链与 GPS 已停止、且持有维护许可时被调用。SBUS 停止后
     * timestamped 端点仍保留“正常 UART”接管快照（normal_configuration_valid_），
     * 不释放会使所有 configure_line 被端点所有权门禁拒绝，导致应用与回滚同时
     * 失败并陷入重试循环。SBUS 重启时 SbusRc::start() 会重新 configure 并重建
     * 快照，因此此处释放不改变端点合同。 */
    if (!backend_.reset_configuration()) {
        PX4_ERR("serial endpoint snapshot release failed; retaining active ports");
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
    PX4_INFO("reconfigured physical serial ports rc_port=%ld gps_port=%ld gps_baud=%lu GPS=NMEA/UM982",
             static_cast<long>(rc_input_port_),
             static_cast<long>(gps_port_),
             static_cast<unsigned long>(gps_target_baudrate_));
    return true;
}

bool SerialConfig::rollback_configuration() noexcept
{
    const Configuration previous = previous_configuration_;
    if (!apply_baudrates(previous.baudrate)) return false;
    {
        // 恢复硬件后，以原子事务恢复参数核心中的旧串口快照。只回滚仍等于本次
        // 失败候选的配置；USB 在维护期间提交的更新不能被旧事务覆盖。
        px4::AtomicTransaction transaction;
        Configuration pending{};
        if (read_configuration(pending)) {
            bool unchanged = true;
            for (std::size_t index = 0U; index < kPortCount; ++index) {
                unchanged = unchanged && pending.function[index] == active_configuration_.function[index] &&
                    pending.requested_baudrate[index] == active_configuration_.requested_baudrate[index];
            }
            if (unchanged) {
                for (std::size_t index = 0U; index < kPortCount; ++index) {
                    const auto &binding = serial_parameters_[index];
                    if (binding.baud == PARAM_INVALID) continue;
                    if (param_set_no_notification(binding.baud, &previous.requested_baudrate[index]) != 0 ||
                        param_set_no_notification(binding.function, &previous.function[index]) != 0) return false;
                }
                param_notify_changes();
            }
        }
    }
    commit_configuration(previous);
    return true;
}

bool SerialConfig::pending_configuration_valid() const noexcept
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return false;
    }
    Configuration configuration{};
    return read_configuration(configuration) && configuration.telemetry_valid;
}

void SerialConfig::stop() noexcept
{
    invalidate_parameters();
    telemetry_port_ = 0;
    telemetry_baudrate_ = 0U;
    telemetry_valid_ = true;
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

std::int32_t SerialConfig::telemetry_port() const noexcept
{
    return state_ == dima::middleware::lifecycle::ModuleState::Running ? telemetry_port_ : 0;
}

std::uint32_t SerialConfig::telemetry_baudrate() const noexcept
{
    return telemetry_port() != 0 ? telemetry_baudrate_ : 0U;
}

bool SerialConfig::telemetry_configuration_valid() const noexcept
{
    return telemetry_valid_;
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
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return 0U;
    Configuration pending{};
    return read_configuration(pending) ? signature(pending) : 0U;
}

std::uint64_t SerialConfig::applied_configuration_signature() const noexcept
{
    // 应用期间 USB 仍能写下一代候选；只记录已生效快照，不能误吞后来的参数更新。
    return signature(active_configuration_);
}

std::uint64_t SerialConfig::signature(const Configuration &configuration) const noexcept
{
    // FNV-1a：hash = (hash XOR byte) * 1099511628211，仅用于配置代际比较。
    std::uint64_t hash = 14695981039346656037ULL;
    const auto append = [&hash](std::int32_t value) noexcept {
        const auto bits = static_cast<std::uint32_t>(value);
        for (unsigned shift = 0U; shift < 32U; shift += 8U) {
            hash ^= static_cast<std::uint8_t>(bits >> shift);
            hash *= 1099511628211ULL;
        }
    };
    for (std::size_t index = 0U; index < kPortCount; ++index) {
        if (serial_parameters_[index].baud == PARAM_INVALID) continue;
        append(configuration.requested_baudrate[index]);
        append(configuration.function[index]);
    }
    return hash;
}

} // namespace dima::modules::serial
