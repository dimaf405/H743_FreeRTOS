/****************************************************************************
 * PX4-Autopilot v1.17.0 ICM42688P high-resolution FIFO flow adapted for Dima.
 * Upstream: src/drivers/imu/invensense/icm42688p
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "ICM42688P.hpp"
#include "ICM42688PFifoAlgorithms.hpp"

#include "api/Time.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace dima::drivers::imu {
namespace {

using namespace icm42688p::registers;

// 驱动私有批次工作区，只承载普通 sensor_accel/sensor_gyro 所需的整数样本与
// 缩放因子；容量由既有单次 FIFO 读取上限决定，不再借用未消费的 uORB 消息。
struct SensorBatch {
    std::int16_t x[kWatermarkSamples]{};
    std::int16_t y[kWatermarkSamples]{};
    std::int16_t z[kWatermarkSamples]{};
    float scale{};
};

std::int16_t negate_saturated(std::int16_t value) noexcept
{
    /* INT16_MIN 取负不可表示，钳到 INT16_MAX，避免二补码溢出仍为负值。 */
    return value == std::numeric_limits<std::int16_t>::min()
               ? std::numeric_limits<std::int16_t>::max()
               : static_cast<std::int16_t>(-value);
}

std::uint8_t clipping(const std::int16_t *samples,
                      std::size_t count) noexcept
{
    std::uint8_t result = 0U;
    for (std::size_t index = 0U; index < count; ++index) {
        if (samples[index] <= std::numeric_limits<std::int16_t>::min() + 1 ||
            samples[index] >= std::numeric_limits<std::int16_t>::max() - 1) {
            ++result;
        }
    }
    return result;
}

float batch_average(const std::int16_t *samples, std::size_t count,
                    float scale, float &last, bool have_last) noexcept
{
    if (count == 0U) {
        return 0.0F;
    }
    if (!have_last) {
        float total = 0.0F;
        for (std::size_t index = 0U; index < count; ++index) {
            total += static_cast<float>(samples[index]) * scale;
        }
        last = static_cast<float>(samples[count - 1U]) * scale;
        return total / static_cast<float>(count);
    }

    /* 后续批次用跨批梯形端点：0.5*(previous_last+current_last)+本批除末项之和，
     * 再除以 N；用于低成本近似积分窗口平均并保持批次边界连续。 */
    const float current_last =
        static_cast<float>(samples[count - 1U]) * scale;
    float total = 0.5F * (last + current_last);
    for (std::size_t index = 0U; index + 1U < count; ++index) {
        total += static_cast<float>(samples[index]) * scale;
    }
    last = current_last;
    return total / static_cast<float>(count);
}

} // namespace

bool ICM42688P::process_fifo_transfer() noexcept
{
    const std::uint8_t interrupt_status = fifo_receive_[1];
    const std::uint16_t fifo_count = combine_u16(
        fifo_receive_[2], fifo_receive_[3]);
    /* DMA 前 4 B 为 INT_STATUS+FIFO_COUNT，后续为最多 10 个 20 B 包；硬件溢出
     * 立即 flush，has_more 则设置 pending 让同一 WorkQueue 尽快续读。 */
    const icm42688p::fifo::BatchPlan plan =
        icm42688p::fifo::plan_batch(interrupt_status, fifo_count);
    if (plan.status == icm42688p::fifo::BatchStatus::Overflow) {
        ++stats_.fifo_overflows;
        (void)flush_fifo();
        return false;
    }
    if (plan.status == icm42688p::fifo::BatchStatus::Empty) {
        ++stats_.fifo_empty;
        return false;
    }
    if (plan.status != icm42688p::fifo::BatchStatus::Ready) {
        ++stats_.fifo_invalid;
        return false;
    }
    if (plan.has_more) {
        pending_sample_timestamp_us_ = hrt_absolute_time();
    }

    const auto *packets = reinterpret_cast<const FifoPacket *>(
        &fifo_receive_[4]);
    return process_fifo(dma_sample_timestamp_us_, packets, plan.samples);
}

bool ICM42688P::process_fifo(std::uint64_t timestamp_sample,
                             const FifoPacket *packets,
                             std::size_t samples) noexcept
{
    if (packets == nullptr || samples == 0U ||
        samples > icm42688p::registers::kWatermarkSamples) {
        ++stats_.fifo_invalid;
        return false;
    }

    std::size_t valid_samples = 0U;
    float temperature_sum = 0.0F;
    std::int16_t temperatures[icm42688p::registers::kWatermarkSamples]{};
    bool accel_wide = false;
    bool gyro_wide = false;
    for (; valid_samples < samples; ++valid_samples) {
        const FifoPacket &packet = packets[valid_samples];
        icm42688p::fifo::DecodedSample decoded{};
        if (!icm42688p::fifo::decode_sample(packet, decoded)) {
            break;
        }
        temperatures[valid_samples] = decoded.temperature;
        temperature_sum += static_cast<float>(decoded.temperature);

        accel_wide = accel_wide ||
                     icm42688p::fifo::requires_wide_scale(decoded.accel);
        gyro_wide = gyro_wide ||
                    icm42688p::fifo::requires_wide_scale(decoded.gyro);
    }
    if (valid_samples == 0U) {
        ++stats_.fifo_invalid;
        return false;
    }
    if (valid_samples != samples) {
        ++stats_.fifo_invalid;
    }

    const float temperature_average =
        temperature_sum / static_cast<float>(valid_samples);
    for (std::size_t index = 0U; index < valid_samples; ++index) {
        if (std::fabs(static_cast<float>(temperatures[index]) -
                      temperature_average) > 1000.0F) {
            ++stats_.fifo_invalid;
            return false;
        }
    }
    /* 温度公式 T[°C]=raw/132.48+25；同批原始温度离均值>1000 LSB 视作包错位。 */
    temperature_c_ = temperature_average / kTemperatureSensitivity +
                     kTemperatureOffsetC;
    if (!std::isfinite(temperature_c_)) {
        ++stats_.fifo_invalid;
        return false;
    }

    SensorBatch accel_batch{};
    SensorBatch gyro_batch{};
    accel_batch.scale = icm42688p::fifo::accel_scale(accel_wide);
    gyro_batch.scale = icm42688p::fifo::gyro_scale(gyro_wide);

    for (std::size_t index = 0U; index < valid_samples; ++index) {
        const FifoPacket &packet = packets[index];
        icm42688p::fifo::DecodedSample decoded{};
        (void)icm42688p::fifo::decode_sample(packet, decoded);
        accel_batch.x[index] = icm42688p::fifo::compact_accel(
            decoded.accel[0], combine_i16(packet.accel_x_high,
                                          packet.accel_x_low),
            accel_wide);
        accel_batch.y[index] = icm42688p::fifo::compact_accel(
            decoded.accel[1], combine_i16(packet.accel_y_high,
                                          packet.accel_y_low),
            accel_wide);
        accel_batch.z[index] = icm42688p::fifo::compact_accel(
            decoded.accel[2], combine_i16(packet.accel_z_high,
                                          packet.accel_z_low),
            accel_wide);
        gyro_batch.x[index] = icm42688p::fifo::compact_gyro(
            decoded.gyro[0], combine_i16(packet.gyro_x_high,
                                         packet.gyro_x_low),
            gyro_wide);
        gyro_batch.y[index] = icm42688p::fifo::compact_gyro(
            decoded.gyro[1], combine_i16(packet.gyro_y_high,
                                         packet.gyro_y_low),
            gyro_wide);
        gyro_batch.z[index] = icm42688p::fifo::compact_gyro(
            decoded.gyro[2], combine_i16(packet.gyro_z_high,
                                         packet.gyro_z_low),
            gyro_wide);

        /* 板上安装坐标为 (x,-y,-z)，在原始整数域变换，scale 仍保持每轴一致。 */
        accel_batch.y[index] = negate_saturated(accel_batch.y[index]);
        accel_batch.z[index] = negate_saturated(accel_batch.z[index]);
        gyro_batch.y[index] = negate_saturated(gyro_batch.y[index]);
        gyro_batch.z[index] = negate_saturated(gyro_batch.z[index]);
    }

    // 普通 IMU 输出继续使用批末采样时间；批首时间只服务于已退役 FIFO Topic。
    const std::uint64_t now_us = hrt_absolute_time();

    sensor_accel_s accel{};
    sensor_gyro_s gyro{};
    accel.timestamp = now_us;
    gyro.timestamp = now_us;
    accel.timestamp_sample = timestamp_sample;
    gyro.timestamp_sample = timestamp_sample;
    accel.device_id = kDeviceId;
    gyro.device_id = kDeviceId;
    accel.temperature = temperature_c_;
    gyro.temperature = temperature_c_;
    accel.samples = static_cast<std::uint8_t>(valid_samples);
    gyro.samples = static_cast<std::uint8_t>(valid_samples);
    accel.error_count = sensor_error_count();
    gyro.error_count = accel.error_count;

    accel.x = batch_average(accel_batch.x, valid_samples, accel_batch.scale,
                            last_accel_[0], have_last_accel_);
    accel.y = batch_average(accel_batch.y, valid_samples, accel_batch.scale,
                            last_accel_[1], have_last_accel_);
    accel.z = batch_average(accel_batch.z, valid_samples, accel_batch.scale,
                            last_accel_[2], have_last_accel_);
    gyro.x = batch_average(gyro_batch.x, valid_samples, gyro_batch.scale,
                           last_gyro_[0], have_last_gyro_);
    gyro.y = batch_average(gyro_batch.y, valid_samples, gyro_batch.scale,
                           last_gyro_[1], have_last_gyro_);
    gyro.z = batch_average(gyro_batch.z, valid_samples, gyro_batch.scale,
                           last_gyro_[2], have_last_gyro_);
    have_last_accel_ = true;
    have_last_gyro_ = true;

    accel.clip_counter[0] = clipping(accel_batch.x, valid_samples);
    accel.clip_counter[1] = clipping(accel_batch.y, valid_samples);
    accel.clip_counter[2] = clipping(accel_batch.z, valid_samples);
    gyro.clip_counter[0] = clipping(gyro_batch.x, valid_samples);
    gyro.clip_counter[1] = clipping(gyro_batch.y, valid_samples);
    gyro.clip_counter[2] = clipping(gyro_batch.z, valid_samples);

    // 两条真实输出都必须尝试发布，不能用短路求值跳过加速度；恢复计数仍以
    // 两者同时成功为准，避免把部分输出故障当成传感器已经恢复。
    const bool gyro_published = gyro_pub_.publish(gyro);
    const bool accel_published = accel_pub_.publish(accel);
    const bool published = gyro_published && accel_published;
    if (published) {
        ++stats_.publications;
        if (restart_fault_active_) {
            if (healthy_publications_after_fault_ != UINT32_MAX) {
                ++healthy_publications_after_fault_;
            }
            if (healthy_publications_after_fault_ >=
                kRestartLogRecoveryPublications) {
                restart_fault_active_ = false;
                healthy_publications_after_fault_ = 0U;
            }
        }
    } else {
        ++stats_.publication_failures;
    }
    return published;
}

std::uint32_t ICM42688P::sensor_error_count() const noexcept
{
    /* 向上层暴露驱动累计错误总和，先用 64-bit 求和再饱和到 UINT32_MAX，避免
     * 多个 32-bit 计数相加回绕成“健康”。 */
    const std::uint64_t total =
        static_cast<std::uint64_t>(stats_.register_failures) +
        stats_.transfer_failures + stats_.fifo_empty +
        stats_.fifo_overflows + stats_.fifo_invalid + stats_.dma_timeouts;
    return total > UINT32_MAX ? UINT32_MAX
                              : static_cast<std::uint32_t>(total);
}

} // namespace dima::drivers::imu
