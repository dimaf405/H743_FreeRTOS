/****************************************************************************
 * PX4-Autopilot v1.17.0 ICM42688P flow adapted to the Dima platform.
 * Upstream: src/drivers/imu/invensense/icm42688p
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "SensorDeviceContract.hpp"
#include "ICM42688PRegisters.hpp"

#include "lifecycle/module_base.hpp"
#include "api/SensorInterrupts.hpp"
#include "api/Spi.hpp"
#include "sensor_accel.hpp"
#include "sensor_accel_fifo.hpp"
#include "sensor_gyro.hpp"
#include "sensor_gyro_fifo.hpp"
#include "uORB/Publication.hpp"
#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::drivers::imu {

class ICM42688P final : public dima::middleware::lifecycle::ModuleBase,
                        public px4::ScheduledWorkItem {
public:
    static constexpr std::uint32_t kDeviceId =
        dima::generated::sensor_devices::kIcm42688pDeviceId;

    struct Stats {
        std::uint32_t resets{0U};
        std::uint32_t restarts{0U};
        std::uint32_t probe_failures{0U};
        std::uint32_t register_failures{0U};
        std::uint32_t transfer_failures{0U};
        std::uint32_t fifo_empty{0U};
        std::uint32_t fifo_overflows{0U};
        std::uint32_t fifo_invalid{0U};
        std::uint32_t dma_timeouts{0U};
        std::uint32_t int1_edges{0U};
        std::uint32_t int2_edges{0U};
        std::uint32_t publications{0U};
        std::uint32_t publication_failures{0U};
    };

    ICM42688P(dima::platform::SpiDevice &spi,
              dima::platform::InterruptSources &interrupts) noexcept;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

    const Stats &stats() const noexcept { return stats_; }

private:
    using Bank = icm42688p::registers::Bank;
    using FifoPacket = icm42688p::registers::FifoPacket;

    static constexpr std::uint32_t kRegisterTimeoutUs = 2000U;
    static constexpr std::uint32_t kMaximumSpiFrequencyHz = 24000000U;
    static constexpr std::uint32_t kResetRetryUs = 100000U;
    static constexpr std::uint32_t kResetTimeoutUs = 1000000U;
    static constexpr std::uint32_t kConfigurationRetryUs = 10000U;
    static constexpr std::uint32_t kDmaTimeoutUs = 5000U;
    /* 8 kHz 下 10 个样本每 1.25 ms 达到 FIFO 水位；看门狗取两倍水位周期 2.5 ms，
     * INT1 延迟/漏失时主动轮询，不允许退化为约 20 ms 的发布空洞。 */
    static constexpr std::uint32_t kWatchdogUs = 2500U;
    static constexpr std::uint32_t kRegisterCheckUs = 100000U;
    static constexpr std::uint32_t kRestartLogRecoveryPublications = 800U;
    static constexpr std::size_t kConfigurationCount = 24U;
    static constexpr std::size_t kFifoTransferBytes =
        4U + icm42688p::registers::kWatermarkSamples * sizeof(FifoPacket);

    enum class DriverState : std::uint8_t {
        /* Reset -> WaitReset -> StartupDelay -> Configure(逐寄存器) -> Verify
         * -> FifoReset -> Running；运行期 DMA/寄存器错误超过门限回到 Reset。 */
        Reset = 0U,
        WaitReset,
        StartupDelay,
        Configure,
        Verify,
        FifoReset,
        Running,
    };

    enum class RestartReason : std::uint8_t {
        None = 0U,
        ResetOrWhoAmI,
        Configure,
        Verify,
        FifoReset,
        DmaTimeout,
        FifoTransfer,
        DmaStart,
        RegisterCheck,
    };

    struct RegisterConfig {
        /* 期望寄存器变换：desired=(current & ~clear_bits) | set_bits；验证要求
         * set_bits 全为 1、clear_bits 全为 0，未声明位保持芯片原值。 */
        Bank bank;
        std::uint8_t reg;
        std::uint8_t set_bits;
        std::uint8_t clear_bits;
    };

    void Run() override;
    static void notify_from_isr(void *context) noexcept;
    bool schedule_now() noexcept;
    bool schedule_delayed(std::uint32_t delay_us) noexcept;
    void fail_module(const char *reason) noexcept;
    void reset_runtime_state() noexcept;
    void restart_driver(RestartReason reason,
                        std::uint32_t delay_us) noexcept;
    void log_restart_diagnostics(RestartReason reason) noexcept;
    bool probe() noexcept;

    bool select_bank(Bank bank, bool force = false) noexcept;
    bool read_register(Bank bank, std::uint8_t reg,
                       std::uint8_t &value) noexcept;
    bool write_register(Bank bank, std::uint8_t reg,
                        std::uint8_t value) noexcept;
    bool apply_register(const RegisterConfig &config) noexcept;
    bool verify_register(const RegisterConfig &config) noexcept;
    static const RegisterConfig &configuration(std::size_t index) noexcept;
    bool flush_fifo() noexcept;

    void run_reset(std::uint64_t now_us) noexcept;
    void run_wait_reset(std::uint64_t now_us) noexcept;
    void run_configure() noexcept;
    void run_verify() noexcept;
    void run_fifo(std::uint64_t now_us) noexcept;
    bool start_fifo_transfer(std::uint64_t timestamp_sample) noexcept;
    bool process_fifo_transfer() noexcept;
    bool process_fifo(std::uint64_t timestamp_sample,
                      const FifoPacket *packets,
                      std::size_t samples) noexcept;
    std::uint32_t sensor_error_count() const noexcept;

    dima::platform::SpiDevice &spi_;
    dima::platform::InterruptSources &interrupts_;
    uORB::Publication<sensor_accel_s> accel_pub_{ORB_ID(sensor_accel)};
    uORB::Publication<sensor_gyro_s> gyro_pub_{ORB_ID(sensor_gyro)};
    uORB::Publication<sensor_accel_fifo_s> accel_fifo_pub_{
        ORB_ID(sensor_accel_fifo)};
    uORB::Publication<sensor_gyro_fifo_s> gyro_fifo_pub_{
        ORB_ID(sensor_gyro_fifo)};

    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    DriverState driver_state_{DriverState::Reset};
    Bank selected_bank_{Bank::Bank0};
    std::size_t configuration_index_{0U};
    std::size_t verification_index_{0U};
    std::size_t runtime_check_index_{0U};
    std::uint32_t consecutive_failures_{0U};
    std::uint32_t last_int1_count_{0U};
    std::uint32_t last_int2_count_{0U};
    std::uint64_t reset_timestamp_us_{0U};
    std::uint64_t dma_start_us_{0U};
    std::uint64_t dma_sample_timestamp_us_{0U};
    std::uint64_t pending_sample_timestamp_us_{0U};
    std::uint64_t last_fifo_request_us_{0U};
    std::uint64_t last_register_check_us_{0U};
    std::uint32_t suppressed_restart_logs_{0U};
    std::uint32_t healthy_publications_after_fault_{0U};
    bool restart_fault_active_{false};
    bool dma_active_{false};
    bool have_last_accel_{false};
    bool have_last_gyro_{false};
    float last_accel_[3]{};
    float last_gyro_[3]{};
    float temperature_c_{0.0F};
    std::uint8_t fifo_transmit_[kFifoTransferBytes]{};
    std::uint8_t fifo_receive_[kFifoTransferBytes]{};
    Stats stats_{};
};

} // namespace dima::drivers::imu
