#pragma once
/*
 * FlashFS — PX4 flashfs 风格的多条目 Flash 存储。
 *
 * 采用追加写和最终 commit 字，避免参数保存期间覆盖上一份有效记录。
 * 针对 STM32H743 32 字节 Flash 编程粒度适配，commit marker 独占一个
 * Flash 字并在 payload 完整写入后最后编程。
 *
 * 来源参考：PX4-Autopilot v1.17.0 src/lib/parameters/flashparams/flashfs.c
 * 适配：dima 平台 FlashPartition 抽象，32 字节编程对齐。
 */

#include "platform/api/Flash.hpp"
#include "platform/api/Synchronization.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::parameters {

/* 文件 token —— 4 字节标识，类似文件名 */
struct flash_file_token_t {
    std::uint8_t bytes[4];

    bool operator==(const flash_file_token_t &o) const noexcept {
        return bytes[0] == o.bytes[0] && bytes[1] == o.bytes[1] &&
               bytes[2] == o.bytes[2] && bytes[3] == o.bytes[3];
    }
};

/* 参数文件 token */
static constexpr flash_file_token_t FLASH_TOKEN_PARAMS = {'p', 'a', 'r', 'm'};

/* FlashFS 运行状态 */
struct FlashFSStatus {
    std::uint32_t free_bytes{0U};       /* 剩余可用字节 */
    std::uint32_t used_bytes{0U};       /* 已用字节 */
    std::uint32_t write_failures{0U};   /* 累计写入失败次数 */
    std::uint32_t crc_failures{0U};     /* CRC 校验失败次数 */
};

class FlashFS final {
public:
    FlashFS(platform::FlashPartition &partition,
            platform::FlashTransactionManager &transactions,
            platform::ArmedFlashCoordinator &armed_flash,
            platform::Synchronization &synchronization) noexcept;

    /* 初始化：扫描 Flash 恢复状态 */
    bool initialize() noexcept;

    /* 运行期写入是分步事务；每次 continue 最多编程或回读一个 Flash 字。 */
    int begin_write_entry(flash_file_token_t token,
                          const void *data, std::size_t size) noexcept;
    int begin_erase_all() noexcept;
    int continue_operation() noexcept;
    void cancel_operation() noexcept;

    /* 读取条目（最新有效版本） */
    int read_entry(flash_file_token_t token,
                   void *data, std::size_t capacity) noexcept;

    /* 获取运行状态 */
    FlashFSStatus status() noexcept;

private:
    /* Flash 字大小（STM32H743 = 32 字节） */
    static constexpr std::size_t kFlashWordBytes = 32U;

    /* 条目头字段（32 字节，物理结构）
     *
     * Flash 物理布局（两个 32 字节 Flash 字）：
     *   Word 0 [offset  0..31]: magic(4) + crc(4) + data_size(4) + token(4)
     *                            + header_crc(4) + reserved(8) + flag(4)
     *   Word 1 [offset 32..63]: commit marker (0xA5AC5CA5=有效)
     *   Data   [offset 64..  ]: 用户数据，填充至 32 字节对齐
     */
    struct HeaderFields {
        std::uint32_t magic;
        std::uint32_t crc;
        std::uint32_t size;             /* payload bytes, excludes header/padding */
        flash_file_token_t token;
        std::uint32_t header_checksum;
        std::uint8_t  reserved[8];
        std::uint32_t flag;
    };
    static_assert(sizeof(HeaderFields) == 32U, "HeaderFields must be 32 bytes");

    static constexpr std::uint32_t kMagicValid  = 0xAA553CC3U;
    static constexpr std::uint32_t kMagicBlank   = 0xFFFFFFFFU;
    static constexpr std::uint32_t kFlagValid    = 0xA5AC5CA5U;

    /* 头部在 Flash 中占用的总字节（Word 0 + Word 1） */
    static constexpr std::size_t kHeaderFlashBytes = 64U;

    enum class Operation : std::uint8_t {
        Idle,
        ProgramHeader,
        ProgramPayload,
        VerifyPayload,
        Commit,
        Erase,
    };

    /* 内部方法 */
    int  scan() noexcept;
    int  find_entry_locked(flash_file_token_t token,
                            std::size_t &offset,
                            HeaderFields &header) noexcept;
    std::size_t compute_total_size(std::size_t data_size) const noexcept;
    static std::uint32_t payload_crc_seed(
        const HeaderFields &header) noexcept;
    bool verify_crc(const HeaderFields &hdr,
                    std::size_t entry_offset) noexcept;
    static std::uint32_t header_crc(const HeaderFields &header) noexcept;
    static bool header_crc_valid(const HeaderFields &header) noexcept;
    int fail_operation(int error) noexcept;
    void reset_operation() noexcept;
    void reset_layout() noexcept;

    platform::FlashPartition &partition_;
    platform::FlashTransactionManager &transactions_;
    platform::ArmedFlashCoordinator &armed_flash_;
    platform::Synchronization &synchronization_;
    platform::RecursiveMutex mutex_{};

    std::size_t partition_size_{0U};
    std::size_t write_offset_{0U};      /* 高水位：下一个可写位置 */
    FlashFSStatus status_{};
    bool initialized_{false};
    bool ready_{false};

    const std::uint8_t *operation_data_{nullptr};
    HeaderFields operation_header_{};
    std::size_t operation_size_{0U};
    std::size_t operation_total_size_{0U};
    std::size_t operation_entry_offset_{0U};
    std::size_t operation_offset_{0U};
    std::uint32_t operation_crc_{UINT32_MAX};
    Operation operation_{Operation::Idle};

    /* 编程用对齐缓冲区 */
    alignas(32) std::uint8_t flashword_[kFlashWordBytes]{};
};

} // namespace dima::parameters
