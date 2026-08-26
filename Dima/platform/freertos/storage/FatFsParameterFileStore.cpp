#include "freertos/Backend.hpp"

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

/* 三文件事务角色：tmp 接收新快照，校验后由上层轮换 primary/backup；本后端只
 * 提供单步文件原语，不擅自决定代际提交策略。 */
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

constexpr std::size_t kChunkBytes = 512U;

enum class Operation : std::uint8_t {
    Idle = 0U,
    WriteData,
    SyncWrite,
    CloseWrite,
    VerifyData,
    CloseVerify,
};

static_assert(static_cast<std::uint8_t>(Operation::Idle) == 0U,
              "FatFs idle operation must remain zero-initializable");

struct FatFsParameterFileStoreState {
    FATFS filesystem{};
    FIL file{};
    const std::uint8_t *operation_data{nullptr};
    std::size_t operation_size{0U};
    std::size_t operation_offset{0U};
    Operation operation{Operation::Idle};
    bool mounted{false};
};

FatFsParameterFileStoreState g_file_store_state{};

class FatFsParameterFileStore final : public ParameterFileStore {
public:
    explicit FatFsParameterFileStore(
        FatFsParameterFileStoreState &state) noexcept
        : state_(state)
    {
    }

    int initialize() noexcept override
    {
        /* 已挂载且块设备仍 ready 时保持幂等；介质变化先彻底卸载旧 FATFS 对象，
         * 再同步挂载并确保 /dima 目录存在。 */
        if (state_.mounted && disk_status(0) == 0U) {
            return 0;
        }

        invalidate_mount();

        FRESULT result = f_mount(&state_.filesystem, kVolumePath, 1);
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

        state_.mounted = true;
        return 0;
    }

    int begin_write(ParameterFile file, const std::uint8_t *data,
                    std::size_t size) noexcept override
    {
        const char *path = file_path(file);
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (path == nullptr || data == nullptr || size == 0U) {
            return -EINVAL;
        }
        if (state_.operation != Operation::Idle) {
            return -EBUSY;
        }

        /* begin 只取得文件并保存调用方缓冲区引用；缓冲区所有权持续到
         * continue_write 返回 0/错误或 cancel，调用方期间不得修改或释放。 */
        const FRESULT result =
            f_open(&state_.file, path, FA_WRITE | FA_CREATE_ALWAYS);
        if (result != FR_OK) {
            return handle(result);
        }

        state_.operation_data = data;
        state_.operation_size = size;
        state_.operation_offset = 0U;
        state_.operation = Operation::WriteData;
        return 0;
    }

    int continue_write() noexcept override
    {
        /* 协作式状态机每次最多推进一个 512 B 数据块或一个 sync/close 阶段。
         * -EAGAIN 表示“仍在正常进行”，让维护任务在块间喂狗和服务通信。 */
        if (state_.operation != Operation::WriteData &&
            state_.operation != Operation::SyncWrite &&
            state_.operation != Operation::CloseWrite) {
            return -EINVAL;
        }

        if (state_.operation == Operation::WriteData) {
            const UINT chunk = static_cast<UINT>(
                std::min(kChunkBytes,
                         state_.operation_size - state_.operation_offset));
            UINT written{};
            const FRESULT result = f_write(
                &state_.file,
                state_.operation_data + state_.operation_offset,
                chunk, &written);
            if (result != FR_OK || written != chunk) {
                (void)f_close(&state_.file);
                reset_operation();
                return result == FR_OK ? -ENOSPC : handle(result);
            }
            state_.operation_offset += written;
            if (state_.operation_offset == state_.operation_size) {
                state_.operation = Operation::SyncWrite;
            }
            return -EAGAIN;
        }

        if (state_.operation == Operation::SyncWrite) {
            /* 必须先 f_sync 再 f_close；close 成功但介质缓存未同步不能算持久提交。 */
            const FRESULT result = f_sync(&state_.file);
            if (result != FR_OK) {
                (void)f_close(&state_.file);
                reset_operation();
                return handle(result);
            }
            state_.operation = Operation::CloseWrite;
            return -EAGAIN;
        }

        const FRESULT result = f_close(&state_.file);
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
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (path == nullptr || destination == nullptr || capacity == 0U) {
            return -EINVAL;
        }
        if (state_.operation != Operation::Idle) {
            return -EBUSY;
        }

        FRESULT result =
            f_open(&state_.file, path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }

        /* 参数快照必须完整装入调用方容量，空文件视作不存在，超长文件拒绝截断。 */
        const FSIZE_t file_size = f_size(&state_.file);
        if (file_size == 0U || file_size > capacity) {
            (void)f_close(&state_.file);
            return file_size == 0U ? -ENOENT : -EFBIG;
        }

        UINT read_size{};
        result = f_read(&state_.file, destination,
                        static_cast<UINT>(file_size), &read_size);
        (void)f_close(&state_.file);
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
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (path == nullptr || expected == nullptr || size == 0U) {
            return -EINVAL;
        }
        if (state_.operation != Operation::Idle) {
            return -EBUSY;
        }

        const FRESULT result =
            f_open(&state_.file, path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        /* 先锁定精确长度，再分块逐字节比对；仅 CRC 相同不足以代替落盘回读。 */
        if (f_size(&state_.file) != static_cast<FSIZE_t>(size)) {
            (void)f_close(&state_.file);
            return -EIO;
        }

        state_.operation_data = expected;
        state_.operation_size = size;
        state_.operation_offset = 0U;
        state_.operation = Operation::VerifyData;
        return 0;
    }

    int continue_verify() noexcept override
    {
        /* 与写入相同，每次只读 512 B；任何短读或内容差异都关闭文件并复位状态。 */
        if (state_.operation != Operation::VerifyData &&
            state_.operation != Operation::CloseVerify) {
            return -EINVAL;
        }

        if (state_.operation == Operation::VerifyData) {
            std::uint8_t buffer[kChunkBytes]{};
            const UINT chunk = static_cast<UINT>(
                std::min(sizeof(buffer),
                         state_.operation_size - state_.operation_offset));
            UINT read_size{};
            const FRESULT result =
                f_read(&state_.file, buffer, chunk, &read_size);
            if (result != FR_OK || read_size != chunk ||
                std::memcmp(buffer,
                            state_.operation_data + state_.operation_offset,
                            chunk) != 0) {
                (void)f_close(&state_.file);
                reset_operation();
                return result == FR_OK ? -EIO : handle(result);
            }
            state_.operation_offset += read_size;
            if (state_.operation_offset == state_.operation_size) {
                state_.operation = Operation::CloseVerify;
            }
            return -EAGAIN;
        }

        const FRESULT result = f_close(&state_.file);
        reset_operation();
        return result == FR_OK ? 0 : handle(result);
    }

    void cancel_operation() noexcept override
    {
        if (state_.operation != Operation::Idle) {
            (void)f_close(&state_.file);
            reset_operation();
        }
    }

    int rename(ParameterFile from, ParameterFile to) noexcept override
    {
        const char *source = file_path(from);
        const char *destination = file_path(to);
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (state_.operation != Operation::Idle) {
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
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (state_.operation != Operation::Idle) {
            return -EBUSY;
        }
        return path == nullptr ? -EINVAL : handle(f_unlink(path));
    }

private:
    void reset_operation() noexcept
    {
        state_.operation_data = nullptr;
        state_.operation_size = 0U;
        state_.operation_offset = 0U;
        state_.operation = Operation::Idle;
    }

    void invalidate_mount() noexcept
    {
        /* 媒体级错误会取消在途操作并卸载卷，后续调用必须重新 initialize；
         * 普通 ENOENT/EEXIST 不应误触发整卷失效。 */
        if (state_.operation != Operation::Idle) {
            (void)f_close(&state_.file);
            reset_operation();
        }
        (void)f_mount(nullptr, kVolumePath, 0);
        state_.mounted = false;
    }

    int handle(FRESULT result) noexcept
    {
        if (media_failure(result)) {
            invalidate_mount();
        }
        return fatfs_error(result);
    }

    FatFsParameterFileStoreState &state_;
};

} // namespace

ParameterFileStore &parameter_file_store() noexcept
{
    static FatFsParameterFileStore instance{g_file_store_state};
    return instance;
}

} // namespace dima::platform::freertos
