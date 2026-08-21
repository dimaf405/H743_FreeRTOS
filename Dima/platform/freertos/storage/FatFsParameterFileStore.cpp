#include "platform/freertos/Backend.hpp"

#include "diskio.h"
#include "ff.h"

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dima::platform::freertos {
namespace {

constexpr const char *kVolumePath = "0:";
constexpr const char *kDirectoryPath = "0:/dima";
constexpr const char *kPrimaryPath = "0:/dima/params.bin";
constexpr const char *kBackupPath = "0:/dima/params.bak";
constexpr const char *kTemporaryPath = "0:/dima/params.tmp";

const char *file_path(ParameterFile file) noexcept
{
    switch (file) {
    case ParameterFile::Primary:
        return kPrimaryPath;
    case ParameterFile::Backup:
        return kBackupPath;
    case ParameterFile::Temporary:
        return kTemporaryPath;
    }
    return nullptr;
}

int fatfs_error(FRESULT result) noexcept
{
    switch (result) {
    case FR_OK:              return 0;
    case FR_INT_ERR:         return -EIO;
    case FR_DISK_ERR:        return -EIO;
    case FR_NOT_READY:       return -ENODEV;
    case FR_NO_FILE:         return -ENOENT;
    case FR_NO_PATH:         return -ENOENT;
    case FR_DENIED:          return -EACCES;
    case FR_EXIST:           return -EEXIST;
    case FR_INVALID_OBJECT:  return -EBADF;
    case FR_WRITE_PROTECTED: return -EROFS;
    case FR_INVALID_DRIVE:   return -ENXIO;
    case FR_NOT_ENABLED:     return -ENODEV;
    case FR_NO_FILESYSTEM:   return -ENODEV;
    case FR_TIMEOUT:         return -ETIMEDOUT;
    case FR_LOCKED:          return -EBUSY;
    case FR_NOT_ENOUGH_CORE: return -ENOMEM;
    default:                 return -EINVAL;
    }
}

bool media_failure(FRESULT result) noexcept
{
    return result == FR_DISK_ERR || result == FR_INT_ERR ||
           result == FR_NOT_READY || result == FR_NO_FILESYSTEM ||
           result == FR_INVALID_DRIVE || result == FR_NOT_ENABLED ||
           result == FR_INVALID_OBJECT;
}

class FatFsParameterFileStore final : public ParameterFileStore {
public:
    int initialize() noexcept override
    {
        if (mounted_ && disk_status(0) == 0U) {
            return 0;
        }

        invalidate_mount();

        FRESULT result = f_mount(&filesystem_, kVolumePath, 1);
        if (result != FR_OK) {
            invalidate_mount();
            return fatfs_error(result);
        }

        FILINFO information{};
        result = f_stat(kDirectoryPath, &information);
        if (result == FR_NO_FILE) {
            result = f_mkdir(kDirectoryPath);
        }
        if (result != FR_OK && result != FR_EXIST) {
            invalidate_mount();
            return fatfs_error(result);
        }

        mounted_ = true;
        return 0;
    }

    int begin_write(ParameterFile file, const std::uint8_t *data,
                    std::size_t size) noexcept override
    {
        const char *path = file_path(file);
        if (!mounted_) {
            return -ENODEV;
        }
        if (path == nullptr || data == nullptr || size == 0U) {
            return -EINVAL;
        }
        if (operation_ != Operation::Idle) {
            return -EBUSY;
        }

        const FRESULT result =
            f_open(&file_, path, FA_WRITE | FA_CREATE_ALWAYS);
        if (result != FR_OK) {
            return handle(result);
        }

        operation_data_ = data;
        operation_size_ = size;
        operation_offset_ = 0U;
        operation_ = Operation::WriteData;
        return 0;
    }

    int continue_write() noexcept override
    {
        if (operation_ != Operation::WriteData &&
            operation_ != Operation::SyncWrite &&
            operation_ != Operation::CloseWrite) {
            return -EINVAL;
        }

        if (operation_ == Operation::WriteData) {
            const UINT chunk = static_cast<UINT>(
                std::min(kChunkBytes, operation_size_ - operation_offset_));
            UINT written{};
            const FRESULT result = f_write(
                &file_, operation_data_ + operation_offset_, chunk, &written);
            if (result != FR_OK || written != chunk) {
                (void)f_close(&file_);
                reset_operation();
                return result == FR_OK ? -ENOSPC : handle(result);
            }
            operation_offset_ += written;
            if (operation_offset_ == operation_size_) {
                operation_ = Operation::SyncWrite;
            }
            return -EAGAIN;
        }

        if (operation_ == Operation::SyncWrite) {
            const FRESULT result = f_sync(&file_);
            if (result != FR_OK) {
                (void)f_close(&file_);
                reset_operation();
                return handle(result);
            }
            operation_ = Operation::CloseWrite;
            return -EAGAIN;
        }

        const FRESULT result = f_close(&file_);
        reset_operation();
        if (result != FR_OK) {
            return handle(result);
        }
        return 0;
    }

    int read(ParameterFile file, std::uint8_t *destination,
             std::size_t capacity,
             std::size_t &output_size) noexcept override
    {
        const char *path = file_path(file);
        output_size = 0U;
        if (!mounted_) {
            return -ENODEV;
        }
        if (path == nullptr || destination == nullptr || capacity == 0U) {
            return -EINVAL;
        }
        if (operation_ != Operation::Idle) {
            return -EBUSY;
        }

        FRESULT result = f_open(&file_, path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }

        const FSIZE_t file_size = f_size(&file_);
        if (file_size == 0U || file_size > capacity) {
            (void)f_close(&file_);
            return file_size == 0U ? -ENOENT : -EFBIG;
        }

        UINT read_size{};
        result = f_read(&file_, destination, static_cast<UINT>(file_size),
                        &read_size);
        (void)f_close(&file_);
        if (result != FR_OK) {
            return handle(result);
        }
        if (read_size != static_cast<UINT>(file_size)) {
            return -EIO;
        }
        output_size = read_size;
        return 0;
    }

    int begin_verify(ParameterFile file, const std::uint8_t *expected,
                     std::size_t size) noexcept override
    {
        const char *path = file_path(file);
        if (!mounted_) {
            return -ENODEV;
        }
        if (path == nullptr || expected == nullptr || size == 0U) {
            return -EINVAL;
        }
        if (operation_ != Operation::Idle) {
            return -EBUSY;
        }

        const FRESULT result =
            f_open(&file_, path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        if (f_size(&file_) != static_cast<FSIZE_t>(size)) {
            (void)f_close(&file_);
            return -EIO;
        }

        operation_data_ = expected;
        operation_size_ = size;
        operation_offset_ = 0U;
        operation_ = Operation::VerifyData;
        return 0;
    }

    int continue_verify() noexcept override
    {
        if (operation_ != Operation::VerifyData &&
            operation_ != Operation::CloseVerify) {
            return -EINVAL;
        }

        if (operation_ == Operation::VerifyData) {
            std::uint8_t buffer[kChunkBytes]{};
            const UINT chunk = static_cast<UINT>(
                std::min(sizeof(buffer),
                         operation_size_ - operation_offset_));
            UINT read_size{};
            const FRESULT result =
                f_read(&file_, buffer, chunk, &read_size);
            if (result != FR_OK || read_size != chunk ||
                std::memcmp(buffer, operation_data_ + operation_offset_,
                            chunk) != 0) {
                (void)f_close(&file_);
                reset_operation();
                return result == FR_OK ? -EIO : handle(result);
            }
            operation_offset_ += read_size;
            if (operation_offset_ == operation_size_) {
                operation_ = Operation::CloseVerify;
            }
            return -EAGAIN;
        }

        const FRESULT result = f_close(&file_);
        reset_operation();
        return result == FR_OK ? 0 : handle(result);
    }

    void cancel_operation() noexcept override
    {
        if (operation_ != Operation::Idle) {
            (void)f_close(&file_);
            reset_operation();
        }
    }

    int rename(ParameterFile from, ParameterFile to) noexcept override
    {
        const char *source = file_path(from);
        const char *destination = file_path(to);
        if (!mounted_) {
            return -ENODEV;
        }
        if (operation_ != Operation::Idle) {
            return -EBUSY;
        }
        if (source == nullptr || destination == nullptr || from == to) {
            return -EINVAL;
        }
        return handle(f_rename(source, destination));
    }

    int erase(ParameterFile file) noexcept override
    {
        const char *path = file_path(file);
        if (!mounted_) {
            return -ENODEV;
        }
        if (operation_ != Operation::Idle) {
            return -EBUSY;
        }
        return path == nullptr ? -EINVAL : handle(f_unlink(path));
    }

private:
    static constexpr std::size_t kChunkBytes = 512U;

    enum class Operation : std::uint8_t {
        Idle,
        WriteData,
        SyncWrite,
        CloseWrite,
        VerifyData,
        CloseVerify,
    };

    void reset_operation() noexcept
    {
        operation_data_ = nullptr;
        operation_size_ = 0U;
        operation_offset_ = 0U;
        operation_ = Operation::Idle;
    }

    void invalidate_mount() noexcept
    {
        if (operation_ != Operation::Idle) {
            (void)f_close(&file_);
            reset_operation();
        }
        (void)f_mount(nullptr, kVolumePath, 0);
        mounted_ = false;
    }

    int handle(FRESULT result) noexcept
    {
        if (media_failure(result)) {
            invalidate_mount();
        }
        return fatfs_error(result);
    }

    FATFS filesystem_{};
    FIL file_{};
    const std::uint8_t *operation_data_{nullptr};
    std::size_t operation_size_{0U};
    std::size_t operation_offset_{0U};
    Operation operation_{Operation::Idle};
    bool mounted_{false};
};

} // namespace

ParameterFileStore &parameter_file_store() noexcept
{
    static FatFsParameterFileStore instance;
    return instance;
}

} // namespace dima::platform::freertos
