/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * FlashFS 实现 —— 移植自 PX4 v1.17.0 flashfs.c，适配 dima 平台。
 *
 * 核心改动：
 * - STM32H743 32 字节 Flash 编程粒度（PX4 原始为 NuttX progmem 字节级）
 * - 条目头分两个 Flash 字存放，使 flag 字可独立编程实现软擦除
 * - 使用 FlashPartition 抽象替代 up_progmem_* 系列函数
 * - 使用 FlashTransactionManager / ArmedFlashCoordinator 保护操作
 * - C++ 面向对象封装，dima 命名空间
 */

#include "flashfs.h"
#include "Crc32.hpp"
#include "api/Execution.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dima::parameters {
namespace {

/* 对齐到 Flash 字边界（32 字节） */
constexpr std::size_t align_flashword(std::size_t v) noexcept
{
    return (v + 31U) & ~std::size_t{31U};
}

} // namespace

/* ─────────────────────────── 构造 / 初始化 ─────────────────────────── */

FlashFS::FlashFS(platform::FlashPartition &partition,
                 platform::FlashTransactionManager &transactions,
                 platform::ArmedFlashCoordinator &armed_flash,
                 platform::Synchronization &synchronization) noexcept
    : partition_(partition), transactions_(transactions),
      armed_flash_(armed_flash), synchronization_(synchronization)
{
}

void FlashFS::reset_layout() noexcept
{
    write_offset_ = 0U;
    status_.used_bytes = 0U;
    status_.free_bytes = static_cast<std::uint32_t>(partition_size_);
}

bool FlashFS::initialize() noexcept
{
    if (platform::in_interrupt_context()) {
        return false;
    }
    if (!initialized_) {
        const std::size_t partition_size = partition_.size();
        if (partition_.program_size() != kFlashWordBytes ||
            (partition_.base() % kFlashWordBytes) != 0U ||
            partition_size < kHeaderFlashBytes ||
            (partition_size % kFlashWordBytes) != 0U ||
            !mutex_.initialize(synchronization_)) {
            return false;
        }
        partition_size_ = partition_size;
        initialized_ = true;
    }
    platform::MutexGuard lock{mutex_};
    if (!lock) { return false; }
    const int result = scan();
    ready_ = result == 0 || result == -ENOENT;
    return ready_;
}

/* ─────────────────────────── 扫描 ─────────────────────────── */

int FlashFS::scan() noexcept
{
    std::uint32_t valid_entries = 0U;
    std::size_t high_water = 0U;
    std::size_t offset = 0U;

    while (offset + kFlashWordBytes <= partition_size_) {
        /* 读取 Word 0（magic + crc + size + token + header checksum + flag） */
        HeaderFields hdr{};
        if (!partition_.read(offset, &hdr, sizeof(hdr))) {
            return -EIO;
        }

        /* 擦除或损坏形成的空洞不能证明后续区域也为空。完整扫描以恢复
         * 掉电擦除后仍位于更高地址的有效记录，并据此计算 high-water。 */
        if (hdr.magic == kMagicBlank) {
            const auto *bytes = reinterpret_cast<const std::uint8_t *>(&hdr);
            const bool word_erased = std::all_of(
                bytes, bytes + sizeof(hdr),
                [](std::uint8_t value) { return value == 0xFFU; });
            offset += kFlashWordBytes;
            if (!word_erased) {
                high_water = std::max(high_water, offset);
            }
            continue;
        }

        if (offset + kHeaderFlashBytes > partition_size_) {
            offset += kFlashWordBytes;
            high_water = std::max(high_water, offset);
            continue;
        }

        /* 非有效 magic：逐字前进寻找下一个有效条目 */
        if (hdr.magic != kMagicValid) {
            offset += kFlashWordBytes;
            high_water = std::max(high_water, offset);
            continue;
        }

        /* Never use a damaged header's size to skip across later records. */
        if (!header_crc_valid(hdr)) {
            ++status_.crc_failures;
            offset += kFlashWordBytes;
            high_water = std::max(high_water, offset);
            continue;
        }

        /* 读取 Word 1（flag 字） */
        std::uint32_t flag_word = 0U;
        if (!partition_.read(offset + kFlashWordBytes, &flag_word,
                             sizeof(flag_word))) {
            return -EIO;
        }

        const bool size_valid =
            hdr.size > 0U &&
            hdr.size <= partition_size_ - offset - kHeaderFlashBytes;
        if (!size_valid) {
            offset += kFlashWordBytes;
            high_water = std::max(high_water, offset);
            continue;
        }
        const std::size_t total = compute_total_size(hdr.size);

        /* 有效条目：验证 CRC */
        if (hdr.flag == kFlagValid && flag_word == kFlagValid &&
            total <= partition_size_ - offset) {

            if (verify_crc(hdr, offset)) {
                ++valid_entries;
            } else {
                ++status_.crc_failures;
            }
            offset += total;
            high_water = std::max(high_water, offset);
            continue;
        }

        /* 未 commit 的条目也已经占用其完整范围，下一次从其后追加。 */
        offset += total;
        high_water = std::max(high_water, offset);
    }

    write_offset_ = std::min(align_flashword(high_water), partition_size_);
    status_.used_bytes = static_cast<std::uint32_t>(write_offset_);
    status_.free_bytes = static_cast<std::uint32_t>(partition_size_ - write_offset_);

    return valid_entries > 0U ? 0 : -ENOENT;
}

/* ─────────────────────────── CRC ─────────────────────────── */

std::uint32_t FlashFS::payload_crc_seed(
    const HeaderFields &header) noexcept
{
    std::uint32_t crc = UINT32_MAX;
    crc = crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.size),
        sizeof(header.size));
    return crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.token),
        sizeof(header.token));
}

bool FlashFS::verify_crc(const HeaderFields &hdr,
                         std::size_t entry_offset) noexcept
{
    if (hdr.size == 0U || entry_offset > partition_size_ ||
        kHeaderFlashBytes > partition_size_ - entry_offset ||
        hdr.size > partition_size_ - entry_offset - kHeaderFlashBytes) {
        return false;
    }
    const std::size_t data_size = hdr.size;

    std::uint32_t crc = payload_crc_seed(hdr);

    /* 从 Flash 读取数据并增量计算 CRC */
    const std::size_t data_offset = entry_offset + kHeaderFlashBytes;
    std::uint8_t buf[kFlashWordBytes];
    std::size_t consumed = 0U;
    while (consumed < data_size) {
        const std::size_t chunk =
            std::min(sizeof(buf), data_size - consumed);
        if (!partition_.read(data_offset + consumed, buf, chunk)) {
            return false;
        }
        crc = crc32_update(crc, buf, chunk);
        consumed += chunk;
    }
    return (crc ^ UINT32_MAX) == hdr.crc;
}

std::uint32_t FlashFS::header_crc(const HeaderFields &header) noexcept
{
    std::uint32_t crc = UINT32_MAX;
    crc = crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.magic),
        sizeof(header.magic));
    crc = crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.crc),
        sizeof(header.crc));
    crc = crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.size),
        sizeof(header.size));
    crc = crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.token),
        sizeof(header.token));
    crc = crc32_update(
        crc, reinterpret_cast<const std::uint8_t *>(&header.flag),
        sizeof(header.flag));
    return crc ^ UINT32_MAX;
}

bool FlashFS::header_crc_valid(const HeaderFields &header) noexcept
{
    return header.header_checksum == header_crc(header);
}

/* ─────────────────────────── 大小计算 ─────────────────────────── */

std::size_t FlashFS::compute_total_size(std::size_t data_size) const noexcept
{
    /* header(Word0 + Word1) + data，对齐到 Flash 字 */
    return kHeaderFlashBytes + align_flashword(data_size);
}

/* ─────────────────────────── 查找条目 ─────────────────────────── */

int FlashFS::find_entry_locked(flash_file_token_t token,
                               std::size_t &out_offset,
                               HeaderFields &out_header) noexcept
{
    std::size_t offset = 0U;
    bool found = false;

    while (offset + kHeaderFlashBytes <= write_offset_) {
        HeaderFields hdr{};
        if (!partition_.read(offset, &hdr, sizeof(hdr))) {
            return -EIO;
        }
        if (hdr.magic == kMagicBlank) {
            offset += kFlashWordBytes;
            continue;
        }
        if (hdr.magic != kMagicValid) {
            offset += kFlashWordBytes;
            continue;
        }

        if (!header_crc_valid(hdr)) {
            offset += kFlashWordBytes;
            continue;
        }

        std::uint32_t flag_word = 0U;
        if (!partition_.read(offset + kFlashWordBytes, &flag_word,
                             sizeof(flag_word))) {
            return -EIO;
        }

        const bool size_valid =
            hdr.size > 0U &&
            hdr.size <= partition_size_ - offset - kHeaderFlashBytes;
        if (!size_valid) {
            offset += kFlashWordBytes;
            continue;
        }
        const std::size_t total = compute_total_size(hdr.size);

        if (hdr.flag == kFlagValid && flag_word == kFlagValid) {
            if (hdr.token == token) {
                if (verify_crc(hdr, offset)) {
                    out_offset = offset;
                    out_header = hdr;
                    found = true;
                } else {
                    ++status_.crc_failures;
                }
            }
            offset += total;
            continue;
        }

        /* 损坏或未 commit：按头中的 payload 长度跳过。 */
        offset += total;
    }
    return found ? 0 : -ENOENT;
}

/* ─────────────────────────── 写入条目 ─────────────────────────── */

int FlashFS::begin_write_entry(flash_file_token_t token,
                               const void *data, std::size_t size) noexcept
{
    if (platform::in_interrupt_context()) {
        return -EPERM;
    }
    if (!initialized_) { return -ENODEV; }
    if (!ready_) { return -EIO; }
    if (data == nullptr || size == 0U) {
        return -EINVAL;
    }

    platform::MutexGuard lock{mutex_};
    if (!lock) { return -EDEADLK; }
    if (operation_ != Operation::Idle) {
        return -EBUSY;
    }
    if (partition_size_ < kHeaderFlashBytes ||
        size > partition_size_ - kHeaderFlashBytes) {
        return -EFBIG;
    }

    const std::size_t total_size = compute_total_size(size);
    /* 自动擦除会在掉电时同时丢失新旧快照；满区交给上层显式处理。 */
    if (total_size > partition_size_ ||
        write_offset_ > partition_size_ - total_size) {
        ++status_.write_failures;
        return -ENOSPC;
    }

    std::memset(&operation_header_, 0xFF, sizeof(operation_header_));
    operation_header_.magic = kMagicValid;
    operation_header_.size = static_cast<std::uint32_t>(size);
    operation_header_.token = token;
    operation_header_.flag = kFlagValid;

    std::uint32_t crc = payload_crc_seed(operation_header_);
    crc = crc32_update(crc, static_cast<const std::uint8_t *>(data), size);
    operation_header_.crc = crc ^ UINT32_MAX;
    operation_header_.header_checksum = header_crc(operation_header_);

    operation_data_ = static_cast<const std::uint8_t *>(data);
    operation_size_ = size;
    operation_total_size_ = total_size;
    operation_entry_offset_ = write_offset_;
    operation_offset_ = 0U;
    operation_crc_ = UINT32_MAX;
    operation_ = Operation::ProgramHeader;
    return 0;
}

int FlashFS::validate_exclusive_erase_locked(
    flash_file_token_t exclusive_token) noexcept
{
    std::size_t offset = 0U;
    while (offset + kFlashWordBytes <= partition_size_) {
        HeaderFields header{};
        if (!partition_.read(offset, &header, sizeof(header))) {
            return -EIO;
        }
        if (header.magic == kMagicBlank) {
            offset += kFlashWordBytes;
            continue;
        }
        if (offset + kHeaderFlashBytes > partition_size_ ||
            header.magic != kMagicValid || !header_crc_valid(header)) {
            offset += kFlashWordBytes;
            continue;
        }

        std::uint32_t commit_marker = 0U;
        if (!partition_.read(offset + kFlashWordBytes, &commit_marker,
                             sizeof(commit_marker))) {
            return -EIO;
        }
        const bool size_valid =
            header.size > 0U &&
            header.size <= partition_size_ - offset - kHeaderFlashBytes;
        if (!size_valid) {
            offset += kFlashWordBytes;
            continue;
        }
        const std::size_t total = compute_total_size(header.size);
        if (total > partition_size_ - offset) {
            offset += kFlashWordBytes;
            continue;
        }
        if (header.flag == kFlagValid && commit_marker == kFlagValid &&
            verify_crc(header, offset) &&
            !(header.token == exclusive_token)) {
            return -ENOTEMPTY;
        }
        offset += total;
    }
    return 0;
}

int FlashFS::begin_erase_all(flash_file_token_t exclusive_token) noexcept
{
    if (platform::in_interrupt_context()) {
        return -EPERM;
    }
    if (!initialized_) { return -ENODEV; }
    if (!ready_) { return -EIO; }
    platform::MutexGuard lock{mutex_};
    if (!lock) { return -EDEADLK; }
    if (operation_ != Operation::Idle) {
        return -EBUSY;
    }
    const int scope_result =
        validate_exclusive_erase_locked(exclusive_token);
    if (scope_result != 0) {
        return scope_result;
    }
    operation_ = Operation::Erase;
    return 0;
}

int FlashFS::continue_operation() noexcept
{
    if (platform::in_interrupt_context()) {
        return -EPERM;
    }
    if (!initialized_) { return -ENODEV; }
    if (!ready_) { return -EIO; }
    platform::MutexGuard lock{mutex_};
    if (!lock) { return -EDEADLK; }
    if (operation_ == Operation::Idle) {
        return -EINVAL;
    }

    if (operation_ == Operation::VerifyPayload) {
        std::uint8_t data[kFlashWordBytes]{};
        const std::size_t chunk =
            std::min(kFlashWordBytes, operation_size_ - operation_offset_);
        if (!partition_.read(operation_entry_offset_ + kHeaderFlashBytes +
                                 operation_offset_,
                             data, chunk)) {
            ++status_.write_failures;
            return fail_operation(-EIO);
        }
        operation_crc_ = crc32_update(operation_crc_, data, chunk);
        operation_offset_ += chunk;
        if (operation_offset_ == operation_size_) {
            if ((operation_crc_ ^ UINT32_MAX) != operation_header_.crc) {
                ++status_.write_failures;
                return fail_operation(-EIO);
            }
            operation_ = Operation::Commit;
        }
        return -EAGAIN;
    }

    platform::FlashTransaction transaction{
        transactions_, platform::Timeout::from_ms(50U)};
    if (!transaction) {
        return -EBUSY;
    }
    platform::FlashWriteLease write_lease{armed_flash_};
    if (!write_lease) {
        return fail_operation(-EPERM);
    }

    if (operation_ == Operation::ProgramHeader) {
        std::memcpy(flashword_, &operation_header_, sizeof(operation_header_));
        /* A program attempt may partially consume the flashword. Reserve the
         * complete record before touching Flash so a retry can never overlap
         * it, even when the HAL reports failure. */
        write_offset_ = operation_entry_offset_ + operation_total_size_;
        status_.used_bytes = static_cast<std::uint32_t>(write_offset_);
        status_.free_bytes =
            static_cast<std::uint32_t>(partition_size_ - write_offset_);
        if (!partition_.program(operation_entry_offset_, flashword_,
                                kFlashWordBytes)) {
            ++status_.write_failures;
            return fail_operation(-EIO);
        }
        operation_offset_ = 0U;
        operation_ = Operation::ProgramPayload;
        return -EAGAIN;
    }

    if (operation_ == Operation::ProgramPayload) {
        std::memset(flashword_, 0xFF, sizeof(flashword_));
        const std::size_t chunk =
            std::min(kFlashWordBytes, operation_size_ - operation_offset_);
        std::memcpy(flashword_, operation_data_ + operation_offset_, chunk);
        if (!partition_.program(operation_entry_offset_ + kHeaderFlashBytes +
                                    operation_offset_,
                                flashword_, kFlashWordBytes)) {
            ++status_.write_failures;
            return fail_operation(-EIO);
        }
        operation_offset_ += kFlashWordBytes;
        if (operation_offset_ >= align_flashword(operation_size_)) {
            operation_crc_ = payload_crc_seed(operation_header_);
            operation_offset_ = 0U;
            operation_ = Operation::VerifyPayload;
        }
        return -EAGAIN;
    }

    if (operation_ == Operation::Commit) {
        std::memset(flashword_, 0xFF, sizeof(flashword_));
        const std::uint32_t flag_valid = kFlagValid;
        std::memcpy(flashword_, &flag_valid, sizeof(flag_valid));
        if (!partition_.program(operation_entry_offset_ + kFlashWordBytes,
                                flashword_, kFlashWordBytes)) {
            ++status_.write_failures;
            return fail_operation(-EIO);
        }
        reset_operation();
        return 0;
    }

    if (operation_ == Operation::Erase) {
        if (!partition_.erase()) {
            ++status_.write_failures;
            return fail_operation(-EIO);
        }
        reset_layout();
        reset_operation();
        return 0;
    }

    return fail_operation(-EINVAL);
}

void FlashFS::cancel_operation() noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return;
    }
    platform::MutexGuard lock{mutex_};
    if (lock) {
        reset_operation();
    }
}

int FlashFS::fail_operation(int error) noexcept
{
    reset_operation();
    return error;
}

void FlashFS::reset_operation() noexcept
{
    operation_data_ = nullptr;
    operation_header_ = {};
    operation_size_ = 0U;
    operation_total_size_ = 0U;
    operation_entry_offset_ = 0U;
    operation_offset_ = 0U;
    operation_crc_ = UINT32_MAX;
    operation_ = Operation::Idle;
}

/* ─────────────────────────── 读取条目 ─────────────────────────── */

int FlashFS::read_entry(flash_file_token_t token,
                        void *data, std::size_t capacity) noexcept
{
    if (platform::in_interrupt_context()) {
        return -EPERM;
    }
    if (!initialized_) { return -ENODEV; }
    if (!ready_) { return -EIO; }

    platform::MutexGuard lock{mutex_};
    if (!lock) { return -EDEADLK; }

    std::size_t entry_offset = 0U;
    HeaderFields hdr{};
    const int rc = find_entry_locked(token, entry_offset, hdr);
    if (rc != 0) { return rc; }

    const std::size_t data_size = hdr.size;
    if (data_size > capacity) {
        return -ENOBUFS;
    }
    if (data == nullptr && data_size != 0U) {
        return -EINVAL;
    }

    /* 读取数据 */
    if (data_size > 0U) {
        if (!partition_.read(entry_offset + kHeaderFlashBytes,
                             data, data_size)) {
            return -EIO;
        }
        std::uint32_t crc = payload_crc_seed(hdr);
        crc = crc32_update(
            crc, static_cast<const std::uint8_t *>(data), data_size);
        if ((crc ^ UINT32_MAX) != hdr.crc) {
            ++status_.crc_failures;
            return -EILSEQ;
        }
    }

    return static_cast<int>(data_size);
}

/* ─────────────────────────── 状态 ─────────────────────────── */

FlashFSStatus FlashFS::status() noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return {};
    }
    platform::MutexGuard lock{mutex_};
    return lock ? status_ : FlashFSStatus{};
}

} // namespace dima::parameters
