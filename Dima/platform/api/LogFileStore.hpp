#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

/**
 * SD 日志文件 capability。
 *
 * 业务层只看到“新建 ULog、生成稳定列表、按 ID 分块读取、整目录擦除”这些角色
 * 操作；FatFs 路径、FIL/DIR 对象和介质恢复都留在平台后端。这样 MAVLink 处理器
 * 不会直接持有文件系统对象，也不会在通信 WorkQueue 中执行 SDMMC 阻塞调用。
 */
struct LogFileEntry {
    std::uint32_t time_utc{0U};
    std::uint32_t size_bytes{0U};
};

struct StorageInformation {
    std::uint64_t total_bytes{0U};
    std::uint64_t available_bytes{0U};
};

class LogFileStore {
public:
    virtual ~LogFileStore() = default;

    virtual int initialize() noexcept = 0;

    /**
     * 读取最近一次有界主动探测仍可用的 SD volume 容量。无 card-detect GPIO
     * 时该结果不等于物理在位证明。该调用可进入 FatFs，只允许由 wq:storage
     * 调用；通信队列必须通过有界请求/响应 Ring 获取结果。
     */
    virtual int storage_information(StorageInformation &information) noexcept = 0;

    virtual int start_log() noexcept = 0;
    virtual int append_log(const std::uint8_t *data,
                           std::size_t size) noexcept = 0;
    virtual int sync_log() noexcept = 0;
    virtual int close_log() noexcept = 0;
    virtual bool log_open() noexcept = 0;

    virtual int create_log_list(std::uint16_t &count) noexcept = 0;
    virtual int read_log_entry(std::uint16_t id,
                               LogFileEntry &entry) noexcept = 0;
    virtual int open_log(std::uint16_t id,
                         LogFileEntry &entry) noexcept = 0;
    virtual int read_log(std::uint32_t offset, std::uint8_t *destination,
                         std::size_t requested,
                         std::size_t &output_size) noexcept = 0;
    virtual void close_log_transfer() noexcept = 0;
    virtual int erase_logs() noexcept = 0;
};

} // namespace dima::platform
