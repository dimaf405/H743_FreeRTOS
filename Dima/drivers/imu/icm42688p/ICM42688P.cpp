/****************************************************************************
 * PX4-Autopilot v1.17.0 ICM42688P state/configuration flow adapted for Dima.
 * Upstream: src/drivers/imu/invensense/icm42688p
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#define MODULE_NAME "icm42688p"
#include "ICM42688P.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>

namespace dima::drivers::imu {
namespace {

using namespace icm42688p::registers;

constexpr std::uint8_t kFifoWatermarkLow = static_cast<std::uint8_t>(
    (kWatermarkSamples * sizeof(FifoPacket)) & 0xFFU);
constexpr std::uint8_t kFifoWatermarkHigh = static_cast<std::uint8_t>(
    ((kWatermarkSamples * sizeof(FifoPacket)) >> 8U) & 0x0FU);

/* 水位字节数=10*20=200，低 8 位写 FIFO_CONFIG2，高 4 位写 FIFO_CONFIG3；
 * 公式保留为编译期表达式，包大小变化时寄存器值同步更新。 */

std::uint32_t elapsed_delay(std::uint64_t now, std::uint64_t deadline) noexcept
{
    if (deadline <= now) {
        return 1U;
    }
    const std::uint64_t delay = deadline - now;
    return delay > UINT32_MAX ? UINT32_MAX
                              : static_cast<std::uint32_t>(delay);
}

} // namespace

ICM42688P::ICM42688P(dima::platform::SpiDevice &spi,
                     dima::platform::InterruptSources &interrupts) noexcept
    : px4::ScheduledWorkItem("icm42688p", px4::wq_configurations::sensors),
      spi_(spi), interrupts_(interrupts)
{
    fifo_transmit_[0] = static_cast<std::uint8_t>(bank0::INT_STATUS | kRead);
}

void ICM42688P::notify_from_isr(void *context) noexcept
{
    if (context != nullptr) {
        (void)static_cast<ICM42688P *>(context)->ScheduleNowFromISR();
    }
}

bool ICM42688P::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    /* 获取顺序：启用 WorkItem -> SPI 能力/配置 -> WHO_AM_I -> EXTI -> 四个 uORB
     * publisher -> 首次调度。任一步失败按已取得资源逆序释放，不发布半运行模块。 */
    reset_runtime_state();
    const dima::platform::SpiConfiguration configuration{
        kMaximumSpiFrequencyHz, dima::platform::SpiMode::Mode0};
    if (spi_.maximum_dma_transfer_size() < kFifoTransferBytes) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("SPI4 DMA capacity=%u bytes, FIFO transfer requires=%u",
                static_cast<unsigned>(spi_.maximum_dma_transfer_size()),
                static_cast<unsigned>(kFifoTransferBytes));
        return false;
    }
    if (!spi_.configure(configuration)) {
        const auto bus = spi_.stats();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("SPI4 Mode0 config failed req=%lu kernel=%lu actual=%lu err=0x%lx",
                static_cast<unsigned long>(bus.requested_frequency_hz),
                static_cast<unsigned long>(bus.kernel_frequency_hz),
                static_cast<unsigned long>(bus.configured_frequency_hz),
                static_cast<unsigned long>(bus.last_error_flags));
        return false;
    }

    if (!probe()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        spi_.shutdown();
        ScheduleCancelAndDrain();
        return false;
    }

    const dima::platform::IsrCallback notification{
        &ICM42688P::notify_from_isr, this};
    if (!interrupts_.register_sources(notification)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        spi_.shutdown();
        ScheduleCancelAndDrain();
        PX4_ERR("INT1 registration failed");
        return false;
    }

    if (!accel_pub_.advertise() || !gyro_pub_.advertise() ||
        !accel_fifo_pub_.advertise() || !gyro_fifo_pub_.advertise()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        interrupts_.unregister_sources();
        spi_.shutdown();
        ScheduleCancelAndDrain();
        PX4_ERR("uORB advertisement failed");
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleNow()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        interrupts_.unregister_sources();
        spi_.shutdown();
        ScheduleCancelAndDrain();
        return false;
    }
    return true;
}

void ICM42688P::stop()
{
    /* 先撤销模块运行态与中断源，再 drain WorkItem；随后 abort DMA/关闭 SPI，
     * 防止回调调度已析构或已重置的驱动状态。 */
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    interrupts_.unregister_sources();
    ScheduleCancelAndDrain();
    (void)spi_.abort_transfer();
    spi_.shutdown();
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState ICM42688P::state() const
{
    return state_;
}

bool ICM42688P::schedule_now() noexcept
{
    if (ScheduleNow()) {
        return true;
    }
    fail_module("immediate scheduling failed");
    return false;
}

bool ICM42688P::schedule_delayed(std::uint32_t delay_us) noexcept
{
    if (ScheduleDelayed(delay_us)) {
        return true;
    }
    fail_module("delayed scheduling failed");
    return false;
}

void ICM42688P::fail_module(const char *reason) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    PX4_ERR("%s", reason);
}

void ICM42688P::reset_runtime_state() noexcept
{
    driver_state_ = DriverState::Reset;
    selected_bank_ = Bank::Bank0;
    configuration_index_ = 0U;
    verification_index_ = 0U;
    runtime_check_index_ = 0U;
    consecutive_failures_ = 0U;
    last_int1_count_ = 0U;
    last_int2_count_ = 0U;
    reset_timestamp_us_ = 0U;
    dma_start_us_ = 0U;
    dma_sample_timestamp_us_ = 0U;
    pending_sample_timestamp_us_ = 0U;
    last_fifo_request_us_ = 0U;
    last_register_check_us_ = 0U;
    suppressed_restart_logs_ = 0U;
    healthy_publications_after_fault_ = 0U;
    restart_fault_active_ = false;
    dma_active_ = false;
    have_last_accel_ = false;
    have_last_gyro_ = false;
    std::fill(std::begin(last_accel_), std::end(last_accel_), 0.0F);
    std::fill(std::begin(last_gyro_), std::end(last_gyro_), 0.0F);
    temperature_c_ = 0.0F;
    std::memset(fifo_receive_, 0, sizeof(fifo_receive_));
    stats_ = Stats{};
}

void ICM42688P::log_restart_diagnostics(RestartReason reason) noexcept
{
    if (restart_fault_active_) {
        healthy_publications_after_fault_ = 0U;
        if (suppressed_restart_logs_ != UINT32_MAX) {
            ++suppressed_restart_logs_;
        }
        return;
    }
    restart_fault_active_ = true;
    healthy_publications_after_fault_ = 0U;

    const char *reason_name = "unknown";
    switch (reason) {
    case RestartReason::ResetOrWhoAmI: reason_name = "reset_whoami"; break;
    case RestartReason::Configure: reason_name = "configure"; break;
    case RestartReason::Verify: reason_name = "verify"; break;
    case RestartReason::FifoReset: reason_name = "fifo_reset"; break;
    case RestartReason::DmaTimeout: reason_name = "dma_timeout"; break;
    case RestartReason::FifoTransfer: reason_name = "fifo_transfer"; break;
    case RestartReason::DmaStart: reason_name = "dma_start"; break;
    case RestartReason::RegisterCheck: reason_name = "register_check"; break;
    case RestartReason::None: break;
    }

    const auto bus = spi_.stats();
    PX4_WARN("IMU restart %s count=%lu skip=%lu reg=%lu xfer=%lu ovf=%lu invalid=%lu dma_to=%lu spi=%lu flags=0x%lx",
             reason_name,
             static_cast<unsigned long>(stats_.restarts),
             static_cast<unsigned long>(suppressed_restart_logs_),
             static_cast<unsigned long>(stats_.register_failures),
             static_cast<unsigned long>(stats_.transfer_failures),
             static_cast<unsigned long>(stats_.fifo_overflows),
             static_cast<unsigned long>(stats_.fifo_invalid),
             static_cast<unsigned long>(stats_.dma_timeouts),
             static_cast<unsigned long>(bus.errors),
             static_cast<unsigned long>(bus.last_error_flags));
    suppressed_restart_logs_ = 0U;
}

void ICM42688P::restart_driver(RestartReason reason,
                               std::uint32_t delay_us) noexcept
{
    /* 重启只重置芯片/驱动子状态，不撤销模块/uORB/EXTI 所有权；先 abort 在途 DMA，
     * 清时间戳，再回到 Reset 并延迟，避免立即重试形成总线风暴。 */
    (void)spi_.abort_transfer();
    dma_active_ = false;
    pending_sample_timestamp_us_ = 0U;
    ++stats_.restarts;
    log_restart_diagnostics(reason);
    driver_state_ = DriverState::Reset;
    selected_bank_ = Bank::Bank0;
    configuration_index_ = 0U;
    verification_index_ = 0U;
    runtime_check_index_ = 0U;
    consecutive_failures_ = 0U;
    (void)schedule_delayed(delay_us);
}

bool ICM42688P::probe() noexcept
{
    std::uint8_t who_am_i = 0U;
    bool register_read = false;
    for (std::uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        selected_bank_ = Bank::Bank0;
        if (select_bank(Bank::Bank0, true) &&
            read_register(Bank::Bank0, bank0::WHO_AM_I, who_am_i)) {
            register_read = true;
            if (who_am_i == kWhoAmI) {
                return true;
            }
        }
    }

    ++stats_.probe_failures;
    const auto bus = spi_.stats();
    PX4_ERR("ICM42688P probe failed read=%u WHOAMI=0x%02x expected=0x%02x spi_err=0x%lx",
            register_read ? 1U : 0U, who_am_i, kWhoAmI,
            static_cast<unsigned long>(bus.last_error_flags));
    return false;
}

const ICM42688P::RegisterConfig &ICM42688P::configuration(
    std::size_t index) noexcept
{
    /* 该表是芯片寄存器合同，不是参数/消息列表：配置 8 kHz accel+gyro、20-bit
     * FIFO、200 B watermark/INT1、timestamp 与抗混叠滤波，并在 verify 阶段回读。 */
    static constexpr RegisterConfig configs[kConfigurationCount]{
        {Bank::Bank0, bank0::INT_CONFIG,
         static_cast<std::uint8_t>(bits::INT1_MODE_PULSED |
                                   bits::INT1_PUSH_PULL |
                                   bits::INT1_ACTIVE_HIGH), 0U},
        {Bank::Bank0, bank0::FIFO_CONFIG, bits::FIFO_STOP_ON_FULL, 0U},
        {Bank::Bank0, bank0::INTF_CONFIG0,
         static_cast<std::uint8_t>(bits::FIFO_COUNT_BIG_ENDIAN |
                                   bits::SENSOR_DATA_BIG_ENDIAN |
                                   bits::DISABLE_I2C),
         static_cast<std::uint8_t>(bit(7U) | bit(6U))},
        {Bank::Bank0, bank0::INTF_CONFIG1, bits::DISABLE_AFSR_SET,
         bits::DISABLE_AFSR_CLEAR},
        {Bank::Bank0, bank0::PWR_MGMT0,
         static_cast<std::uint8_t>(bits::GYRO_LOW_NOISE |
                                   bits::ACCEL_LOW_NOISE), 0U},
        {Bank::Bank0, bank0::GYRO_CONFIG0, bits::ODR_8KHZ_SET,
         static_cast<std::uint8_t>(0xE0U | bits::ODR_8KHZ_CLEAR)},
        {Bank::Bank0, bank0::ACCEL_CONFIG0, bits::ODR_8KHZ_SET,
         static_cast<std::uint8_t>(0xE0U | bits::ODR_8KHZ_CLEAR)},
        {Bank::Bank0, bank0::GYRO_CONFIG1, 0U,
         bits::GYRO_UI_FILTER_ORDER},
        {Bank::Bank0, bank0::GYRO_ACCEL_CONFIG0, 0U,
         static_cast<std::uint8_t>(bits::ACCEL_UI_FILTER_BW |
                                   bits::GYRO_UI_FILTER_BW)},
        {Bank::Bank0, bank0::ACCEL_CONFIG1, 0U,
         bits::ACCEL_UI_FILTER_ORDER},
        {Bank::Bank0, bank0::TMST_CONFIG, bits::TIMESTAMP_ENABLE,
         bits::TIMESTAMP_FSYNC},
        {Bank::Bank0, bank0::FIFO_CONFIG1, bits::FIFO_ENABLE,
         static_cast<std::uint8_t>(bits::FIFO_TIMESTAMP_FSYNC | bit(6U))},
        {Bank::Bank0, bank0::FIFO_CONFIG2, kFifoWatermarkLow,
         static_cast<std::uint8_t>(~kFifoWatermarkLow)},
        {Bank::Bank0, bank0::FIFO_CONFIG3, kFifoWatermarkHigh,
         static_cast<std::uint8_t>(0x0FU & ~kFifoWatermarkHigh)},
        {Bank::Bank0, bank0::INT_CONFIG0, bits::CLEAR_FIFO_INT_ON_READ,
         bit(2U)},
        {Bank::Bank0, bank0::INT_CONFIG1, 0U, bits::INT_ASYNC_RESET},
        {Bank::Bank0, bank0::INT_SOURCE0,
         bits::FIFO_THRESHOLD_TO_INT1,
         0x7BU},

        {Bank::Bank1, bank1::GYRO_CONFIG_STATIC2, 0U,
         static_cast<std::uint8_t>(bits::GYRO_NOTCH_DISABLE |
                                   bits::GYRO_AAF_DISABLE)},
        {Bank::Bank1, bank1::GYRO_CONFIG_STATIC3,
         bits::AAF_DELTA_585_SET, bits::AAF_DELTA_585_CLEAR},
        {Bank::Bank1, bank1::GYRO_CONFIG_STATIC4,
         bits::AAF_DELTA_SQUARED_LSB_SET,
         bits::AAF_DELTA_SQUARED_LSB_CLEAR},
        {Bank::Bank1, bank1::GYRO_CONFIG_STATIC5,
         bits::AAF_BITSHIFT_SET,
         static_cast<std::uint8_t>(bits::AAF_BITSHIFT_CLEAR |
                                   bits::AAF_DELTA_SQUARED_MSB_CLEAR)},

        {Bank::Bank2, bank2::ACCEL_CONFIG_STATIC2,
         bits::ACCEL_AAF_DELTA_585_SET,
         bits::ACCEL_AAF_DELTA_585_CLEAR},
        {Bank::Bank2, bank2::ACCEL_CONFIG_STATIC3,
         bits::AAF_DELTA_SQUARED_LSB_SET,
         bits::AAF_DELTA_SQUARED_LSB_CLEAR},
        {Bank::Bank2, bank2::ACCEL_CONFIG_STATIC4,
         bits::AAF_BITSHIFT_SET,
         static_cast<std::uint8_t>(bits::AAF_BITSHIFT_CLEAR |
                                   bits::AAF_DELTA_SQUARED_MSB_CLEAR)},
    };
    return configs[index < kConfigurationCount ? index : 0U];
}

bool ICM42688P::select_bank(Bank bank, bool force) noexcept
{
    if (!force && selected_bank_ == bank) {
        return true;
    }
    const std::uint8_t transmit[2]{bank0::REG_BANK_SEL,
                                   static_cast<std::uint8_t>(bank)};
    std::uint8_t receive[2]{};
    if (!spi_.transfer(transmit, receive, sizeof(transmit),
                       dima::platform::Timeout::from_us(
                           kRegisterTimeoutUs))) {
        ++stats_.transfer_failures;
        return false;
    }
    selected_bank_ = bank;
    return true;
}

bool ICM42688P::read_register(Bank bank, std::uint8_t reg,
                              std::uint8_t &value) noexcept
{
    if (!select_bank(bank)) {
        return false;
    }
    const std::uint8_t transmit[2]{static_cast<std::uint8_t>(reg | kRead),
                                   0U};
    std::uint8_t receive[2]{};
    if (!spi_.transfer(transmit, receive, sizeof(transmit),
                       dima::platform::Timeout::from_us(
                           kRegisterTimeoutUs))) {
        ++stats_.transfer_failures;
        return false;
    }
    value = receive[1];
    return true;
}

bool ICM42688P::write_register(Bank bank, std::uint8_t reg,
                               std::uint8_t value) noexcept
{
    if (!select_bank(bank)) {
        return false;
    }
    const std::uint8_t transmit[2]{reg, value};
    std::uint8_t receive[2]{};
    if (!spi_.transfer(transmit, receive, sizeof(transmit),
                       dima::platform::Timeout::from_us(
                           kRegisterTimeoutUs))) {
        ++stats_.transfer_failures;
        return false;
    }
    return true;
}

bool ICM42688P::apply_register(const RegisterConfig &config) noexcept
{
    std::uint8_t current = 0U;
    if (!read_register(config.bank, config.reg, current)) {
        return false;
    }
    /* 读改写只改变声明位，避免覆盖芯片保留位或复位默认的未管理功能。 */
    const std::uint8_t desired = static_cast<std::uint8_t>(
        (current & static_cast<std::uint8_t>(~config.clear_bits)) |
        config.set_bits);
    return desired == current ||
           write_register(config.bank, config.reg, desired);
}

bool ICM42688P::verify_register(const RegisterConfig &config) noexcept
{
    std::uint8_t value = 0U;
    if (!read_register(config.bank, config.reg, value)) {
        return false;
    }
    return (value & config.set_bits) == config.set_bits &&
           (value & config.clear_bits) == 0U;
}

bool ICM42688P::flush_fifo() noexcept
{
    std::uint8_t value = 0U;
    if (!read_register(Bank::Bank0, bank0::SIGNAL_PATH_RESET, value)) {
        return false;
    }
    value = static_cast<std::uint8_t>(value | bits::FIFO_FLUSH);
    return write_register(Bank::Bank0, bank0::SIGNAL_PATH_RESET, value);
}

void ICM42688P::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    const std::uint64_t now_us = hrt_absolute_time();
    /* 每次 Run 只推进一个状态/寄存器/批次，所有重试通过 WorkQueue 延后；传感器
     * 队列不会被 reset/config 的长同步循环独占。 */
    switch (driver_state_) {
    case DriverState::Reset:
        run_reset(now_us);
        break;
    case DriverState::WaitReset:
        run_wait_reset(now_us);
        break;
    case DriverState::StartupDelay:
        configuration_index_ = 0U;
        verification_index_ = 0U;
        driver_state_ = DriverState::Configure;
        (void)schedule_now();
        break;
    case DriverState::Configure:
        run_configure();
        break;
    case DriverState::Verify:
        run_verify();
        break;
    case DriverState::FifoReset:
        if (!flush_fifo()) {
            ++stats_.register_failures;
            restart_driver(RestartReason::FifoReset,
                           kConfigurationRetryUs);
            return;
        }
        {
            const auto snapshot = interrupts_.consume();
            last_int1_count_ = snapshot.count[0];
            last_int2_count_ = snapshot.count[1];
        }
        driver_state_ = DriverState::Running;
        last_fifo_request_us_ = now_us;
        last_register_check_us_ = now_us;
        consecutive_failures_ = 0U;
        (void)schedule_delayed(1000U);
        break;
    case DriverState::Running:
        run_fifo(now_us);
        break;
    }
}

void ICM42688P::run_reset(std::uint64_t now_us) noexcept
{
    (void)spi_.abort_transfer();
    dma_active_ = false;
    selected_bank_ = Bank::Bank0;
    if (!select_bank(Bank::Bank0, true) ||
        !write_register(Bank::Bank0, bank0::DEVICE_CONFIG,
                        bits::SOFT_RESET)) {
        ++stats_.register_failures;
        (void)schedule_delayed(kResetRetryUs);
        return;
    }
    ++stats_.resets;
    reset_timestamp_us_ = now_us;
    selected_bank_ = Bank::Bank0;
    driver_state_ = DriverState::WaitReset;
    (void)schedule_delayed(1000U);
}

void ICM42688P::run_wait_reset(std::uint64_t now_us) noexcept
{
    std::uint8_t who_am_i = 0U;
    std::uint8_t device_config = 0U;
    std::uint8_t interrupt_status = 0U;
    const bool reset_complete =
        read_register(Bank::Bank0, bank0::WHO_AM_I, who_am_i) &&
        read_register(Bank::Bank0, bank0::DEVICE_CONFIG, device_config) &&
        read_register(Bank::Bank0, bank0::INT_STATUS, interrupt_status) &&
        who_am_i == kWhoAmI && device_config == 0U &&
        (interrupt_status & bits::RESET_DONE_INT) != 0U;

    if (reset_complete &&
        write_register(Bank::Bank0, bank0::PWR_MGMT0,
                       static_cast<std::uint8_t>(bits::GYRO_LOW_NOISE |
                                                 bits::ACCEL_LOW_NOISE))) {
        consecutive_failures_ = 0U;
        driver_state_ = DriverState::StartupDelay;
        (void)schedule_delayed(30000U);
        return;
    }

    ++consecutive_failures_;
    if (now_us - reset_timestamp_us_ >= kResetTimeoutUs) {
        PX4_ERR("reset/WHOAMI failed");
        restart_driver(RestartReason::ResetOrWhoAmI, kResetRetryUs);
    } else {
        (void)schedule_delayed(kConfigurationRetryUs);
    }
}

void ICM42688P::run_configure() noexcept
{
    if (configuration_index_ >= kConfigurationCount) {
        driver_state_ = DriverState::Verify;
        verification_index_ = 0U;
        (void)schedule_now();
        return;
    }

    if (apply_register(configuration(configuration_index_))) {
        ++configuration_index_;
        consecutive_failures_ = 0U;
        (void)schedule_now();
        return;
    }

    ++stats_.register_failures;
    if (++consecutive_failures_ > 10U) {
        restart_driver(RestartReason::Configure, kResetRetryUs);
    } else {
        (void)schedule_delayed(kConfigurationRetryUs);
    }
}

void ICM42688P::run_verify() noexcept
{
    if (verification_index_ >= kConfigurationCount) {
        driver_state_ = DriverState::FifoReset;
        (void)schedule_now();
        return;
    }

    if (verify_register(configuration(verification_index_))) {
        ++verification_index_;
        consecutive_failures_ = 0U;
        (void)schedule_now();
        return;
    }

    ++stats_.register_failures;
    if (++consecutive_failures_ > 10U) {
        restart_driver(RestartReason::Verify, kResetRetryUs);
    } else {
        (void)schedule_delayed(kConfigurationRetryUs);
    }
}

void ICM42688P::run_fifo(std::uint64_t now_us) noexcept
{
    /* EXTI count 为累计值，用无符号差得到本周期边沿；pending 的 INT1 时间作为
     * 本批最后样本近似时刻。没有 pending 时 2.5 ms watchdog 仍触发 FIFO 读取。 */
    const auto interrupt = interrupts_.consume();
    stats_.int1_edges += interrupt.count[0] - last_int1_count_;
    stats_.int2_edges += interrupt.count[1] - last_int2_count_;
    last_int1_count_ = interrupt.count[0];
    last_int2_count_ = interrupt.count[1];
    if ((interrupt.pending_mask & dima::platform::InterruptSource1) !=
        0U) {
        pending_sample_timestamp_us_ = interrupt.timestamp_us[0];
    }

    /* DMA 完成由 ISR 只调度 WorkItem，此处 finish 归还缓冲并解析。5 ms 超时后
     * abort；连续失败>10 才整驱动 reset，单次瞬态只退避重试。 */
    if (dma_active_) {
        const dima::platform::SpiTransferResult result =
            spi_.finish_transfer();
        if (result == dima::platform::SpiTransferResult::Pending) {
            if (now_us - dma_start_us_ >= kDmaTimeoutUs) {
                (void)spi_.abort_transfer();
                dma_active_ = false;
                ++stats_.dma_timeouts;
                ++stats_.transfer_failures;
                if (++consecutive_failures_ > 10U) {
                    restart_driver(RestartReason::DmaTimeout,
                                   kResetRetryUs);
                } else {
                    (void)schedule_delayed(kConfigurationRetryUs);
                }
            } else {
                (void)schedule_delayed(elapsed_delay(
                    now_us, dma_start_us_ + kDmaTimeoutUs));
            }
            return;
        }

        dma_active_ = false;
        if (result != dima::platform::SpiTransferResult::Complete ||
            !process_fifo_transfer()) {
            ++stats_.transfer_failures;
            if (++consecutive_failures_ > 10U) {
                restart_driver(RestartReason::FifoTransfer,
                               kResetRetryUs);
                return;
            }
        } else if (consecutive_failures_ > 0U) {
            --consecutive_failures_;
        }
    }

    /* 运行期每 100 ms 轮查一个配置寄存器，24 项约 2.4 s 覆盖一轮；任一漂移
     * 立即重启，避免传感器静默回到错误 ODR/量程。 */
    if (now_us - last_register_check_us_ >= kRegisterCheckUs) {
        if (!verify_register(configuration(runtime_check_index_))) {
            ++stats_.register_failures;
            restart_driver(RestartReason::RegisterCheck,
                           kConfigurationRetryUs);
            return;
        }
        runtime_check_index_ =
            (runtime_check_index_ + 1U) % kConfigurationCount;
        last_register_check_us_ = now_us;
    }

    if (pending_sample_timestamp_us_ != 0U ||
        now_us - last_fifo_request_us_ >= kWatchdogUs) {
        const std::uint64_t timestamp = pending_sample_timestamp_us_ != 0U
                                            ? pending_sample_timestamp_us_
                                            : now_us;
        pending_sample_timestamp_us_ = 0U;
        if (!start_fifo_transfer(timestamp)) {
            ++stats_.transfer_failures;
            if (++consecutive_failures_ > 10U) {
                restart_driver(RestartReason::DmaStart,
                               kResetRetryUs);
            } else {
                (void)schedule_delayed(kConfigurationRetryUs);
            }
        }
        return;
    }

    const std::uint64_t fifo_deadline = last_fifo_request_us_ + kWatchdogUs;
    const std::uint64_t register_deadline =
        last_register_check_us_ + kRegisterCheckUs;
    (void)schedule_delayed(elapsed_delay(
        now_us, std::min(fifo_deadline, register_deadline)));
}

bool ICM42688P::start_fifo_transfer(
    std::uint64_t timestamp_sample) noexcept
{
    if (!select_bank(Bank::Bank0)) {
        return false;
    }
    std::memset(fifo_receive_, 0, sizeof(fifo_receive_));
    const dima::platform::IsrCallback completion{
        &ICM42688P::notify_from_isr, this};
    /* 先发布 active/start/sample 时间并预排超时 Run，再启动 SPI DMA；启动失败时
     * 清掉超时调度和 active，完成中断再快也能观察完整状态。 */
    dma_active_ = true;
    dma_start_us_ = hrt_absolute_time();
    dma_sample_timestamp_us_ = timestamp_sample;
    last_fifo_request_us_ = dma_start_us_;
    if (!schedule_delayed(kDmaTimeoutUs)) {
        dma_active_ = false;
        return false;
    }
    if (!spi_.start_transfer(fifo_transmit_, fifo_receive_,
                             sizeof(fifo_transmit_), completion)) {
        ScheduleClear();
        dma_active_ = false;
        return false;
    }
    return true;
}

} // namespace dima::drivers::imu
