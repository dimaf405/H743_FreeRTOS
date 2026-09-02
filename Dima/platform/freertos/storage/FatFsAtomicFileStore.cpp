#include "freertos/Backend.hpp"

#include "diskio.h"
#include "ff.h"
#include "api/LogFileStore.hpp"
#include "api/Synchronization.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>

namespace dima::platform::freertos {
namespace {

constexpr const char *kVolumePath = "0:";
constexpr const char *kDirectoryPath = "0:/dima";
constexpr const char *kPrimaryPath = "0:/dima/params.bin";
constexpr const char *kBackupPath = "0:/dima/params.bak";
constexpr const char *kTemporaryPath = "0:/dima/params.tmp";
constexpr const char *kMissionPrimaryPath = "0:/dima/mission.bin";
constexpr const char *kMissionBackupPath = "0:/dima/mission.bak";
constexpr const char *kMissionTemporaryPath = "0:/dima/mission.tmp";
constexpr const char *kLogDirectoryPath = "0:/log";
constexpr const char *kLogListPath = "0:/logdata.bin";
constexpr const char *kLogListTemporaryPath = "0:/logtmp.bin";
constexpr std::size_t kMaximumLogPathLength = 60U;

struct LogIndexEntry {
    std::uint32_t time_utc{0U};
    std::uint32_t size_bytes{0U};
    char filepath[kMaximumLogPathLength]{};
};

static_assert(sizeof(LogIndexEntry) == 68U,
              "SD log index layout must stay fixed within one firmware");

bool join_path(const char *directory, const char *name, char *destination,
               std::size_t capacity) noexcept
{
    if (directory == nullptr || name == nullptr || destination == nullptr ||
        capacity == 0U) {
        return false;
    }
    const std::size_t directory_length = std::strlen(directory);
    const std::size_t name_length = std::strlen(name);
    if (directory_length + 1U + name_length + 1U > capacity) {
        return false;
    }
    std::memcpy(destination, directory, directory_length);
    destination[directory_length] = '/';
    std::memcpy(destination + directory_length + 1U, name, name_length);
    destination[directory_length + 1U + name_length] = '\0';
    return true;
}

bool session_path(std::uint16_t number, char *destination,
                  std::size_t capacity) noexcept
{
    constexpr char kTemplate[] = "0:/log/sess000";
    if (number == 0U || number > 999U || destination == nullptr ||
        capacity < sizeof(kTemplate)) {
        return false;
    }
    std::memcpy(destination, kTemplate, sizeof(kTemplate));
    destination[11] = static_cast<char>('0' + (number / 100U));
    destination[12] = static_cast<char>('0' + ((number / 10U) % 10U));
    destination[13] = static_cast<char>('0' + (number % 10U));
    return true;
}

bool valid_log_path(const char *path) noexcept
{
    constexpr char kPrefix[] = "0:/log/";
    if (path == nullptr || std::strncmp(path, kPrefix, sizeof(kPrefix) - 1U) != 0) {
        return false;
    }
    const void *terminator = std::memchr(path, '\0', kMaximumLogPathLength);
    if (terminator == nullptr || std::strstr(path, "..") != nullptr) {
        return false;
    }
    return true;
}

/* 三文件事务角色：tmp 接收新快照，校验后由上层轮换 primary/backup；本后端只
 * 提供单步文件原语，不擅自决定代际提交策略。 */
const char *file_path(AtomicFileDomain domain, AtomicFile file) noexcept
{
    // 两个域共用同一 FATFS/FIL 和操作状态，但每个域拥有独立三代文件；这样
    // 参数事务与任务事务无法并行踩踏，也不会因相同角色名覆盖彼此数据。
    switch (domain) {
    case AtomicFileDomain::Parameters:
        switch (file) {
        case AtomicFile::Primary:   return kPrimaryPath;
        case AtomicFile::Backup:    return kBackupPath;
        case AtomicFile::Temporary: return kTemporaryPath;
        }
        break;
    case AtomicFileDomain::Mission:
        switch (file) {
        case AtomicFile::Primary:   return kMissionPrimaryPath;
        case AtomicFile::Backup:    return kMissionBackupPath;
        case AtomicFile::Temporary: return kMissionTemporaryPath;
        }
        break;
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
           result == FR_INVALID_OBJECT || result == FR_TIMEOUT;
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

struct FatFsAtomicFileStoreState {
    FATFS filesystem{};
    FIL file{};
    FIL log_writer{};
    FIL log_reader{};
    FIL log_index{};
    DIR log_root_directory{};
    DIR log_child_directory{};
    const std::uint8_t *operation_data{nullptr};
    std::size_t operation_size{0U};
    std::size_t operation_offset{0U};
    std::uint32_t log_reader_size{0U};
    Operation operation{Operation::Idle};
    dima::platform::Synchronization *synchronization{nullptr};
    dima::platform::MutexHandle mutex{};
    bool log_writer_open{false};
    bool log_reader_open{false};
    bool log_index_open{false};
    bool log_root_open{false};
    bool log_child_open{false};
    bool atomic_operation_aborted{false};
    bool mounted{false};
};

FatFsAtomicFileStoreState g_file_store_state{};

class FileStoreGuard final {
public:
    explicit FileStoreGuard(FatFsAtomicFileStoreState &state) noexcept
        : state_(state),
          locked_(state.synchronization != nullptr && state.mutex &&
                  state.synchronization->lock(
                      state.mutex, dima::platform::Timeout::forever()))
    {
    }

    ~FileStoreGuard()
    {
        if (locked_) {
            state_.synchronization->unlock(state_.mutex);
        }
    }

    explicit operator bool() const noexcept { return locked_; }

private:
    FatFsAtomicFileStoreState &state_;
    bool locked_{false};
};

class FatFsFileStore final : public AtomicFileStore, public LogFileStore {
public:
    FatFsFileStore(FatFsAtomicFileStoreState &state,
                   dima::platform::Synchronization &synchronization) noexcept
        : state_(state)
    {
        state_.synchronization = &synchronization;
        state_.mutex = synchronization.create_mutex(
            dima::platform::MutexKind::Normal);
    }

    int initialize() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        /* 本板没有 card-detect GPIO，disk_status==0 只证明旧 SDMMC 会话未被
         * 判错，不能回答“卡此刻是否在位”。已挂载路径执行一次最长 500 ms 的
         * CTRL_SYNC 主动探测；只有探测成功才复用旧 FATFS 对象。 */
        if (state_.mounted && probe_media_locked() == 0) {
            return 0;
        }

        invalidate_mount();

        FRESULT result = f_mount(&state_.filesystem, kVolumePath, 1);
        if (result != FR_OK) {
            invalidate_mount();
            return fatfs_error(result);
        }

        constexpr const char *kRequiredDirectories[]{
            kDirectoryPath, kLogDirectoryPath};
        for (const char *directory : kRequiredDirectories) {
            const int directory_result = ensure_directory_locked(directory);
            if (directory_result != 0) {
                invalidate_mount();
                return directory_result;
            }
        }

        state_.mounted = true;
        return 0;
    }

    int storage_information(
        dima::platform::StorageInformation &information) noexcept override
    {
        FileStoreGuard lock{state_};
        information = {};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted || probe_media_locked() != 0) {
            return -ENODEV;
        }

        DWORD free_clusters = 0U;
        FATFS *filesystem = nullptr;
        const FRESULT result = f_getfree(
            kVolumePath, &free_clusters, &filesystem);
        if (result != FR_OK) {
            return handle(result);
        }
        if (filesystem == nullptr || filesystem->n_fatent < 2U ||
            filesystem->csize == 0U) {
            return invalidate_with_io_error();
        }

        // PX4 STORAGE_INFORMATION 以 MiB 上报。FatFs csize 的单位是逻辑
        // sector，本产品的 FatFs 与 SDMMC 合同均固定为 512 B/sector。
        static_assert(_MIN_SS == 512 && _MAX_SS == 512,
                      "SD capacity conversion requires 512-byte sectors");
        constexpr std::uint64_t kSectorBytes = 512U;
        const std::uint64_t cluster_bytes =
            static_cast<std::uint64_t>(filesystem->csize) * kSectorBytes;
        const std::uint64_t data_clusters =
            static_cast<std::uint64_t>(filesystem->n_fatent - 2U);
        if (static_cast<std::uint64_t>(free_clusters) > data_clusters) {
            return invalidate_with_io_error();
        }

        information.total_bytes = data_clusters * cluster_bytes;
        information.available_bytes =
            static_cast<std::uint64_t>(free_clusters) * cluster_bytes;
        return information.total_bytes != 0U ? 0 : -ENODEV;
    }

    int begin_write(AtomicFileDomain domain, AtomicFile file,
                    const std::uint8_t *data,
                    std::size_t size) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (state_.atomic_operation_aborted) {
            state_.atomic_operation_aborted = false;
            return -EIO;
        }
        const char *path = file_path(domain, file);
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
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (state_.atomic_operation_aborted) {
            state_.atomic_operation_aborted = false;
            return -EIO;
        }
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
            if (result != FR_OK) {
                return close_atomic_file_locked(result);
            }
            if (written != chunk) {
                // 成功状态下的短写同短读一样说明当前文件/介质会话不可再信任；
                // 立即卸载，禁止上层在旧挂载上继续原子轮换。
                return close_atomic_file_locked(FR_OK, -EIO, true);
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
                return close_atomic_file_locked(result);
            }
            state_.operation = Operation::CloseWrite;
            return -EAGAIN;
        }

        return close_atomic_file_locked();
    }

    int read(AtomicFileDomain domain, AtomicFile file,
             std::uint8_t *destination,
             std::size_t capacity,
             std::size_t &output_size) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            output_size = 0U;
            return -EDEADLK;
        }
        const char *path = file_path(domain, file);
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
            const FRESULT close_result = f_close(&state_.file);
            // 即使文件长度本身不合法，close 的介质级错误仍优先传播并撤销挂载；
            // 否则上层会继续尝试 backup/tmp，把已经失效的会话误判成普通坏文件。
            if (close_result != FR_OK) {
                return handle(close_result);
            }
            return file_size == 0U ? -ENOENT : -EFBIG;
        }

        UINT read_size{};
        result = f_read(&state_.file, destination,
                        static_cast<UINT>(file_size), &read_size);
        const FRESULT close_result = f_close(&state_.file);
        // 读取数据成功不等于文件事务成功；拔卡或底层同步故障可能只在 close
        // 阶段暴露。此时禁止把缓冲区交给 validator 并标记成已验证恢复来源。
        const int operation_result =
            finish_fatfs_results_locked(result, close_result);
        if (operation_result != 0) {
            return operation_result;
        }
        if (read_size != static_cast<UINT>(file_size)) {
            return invalidate_with_io_error();
        }
        output_size = read_size;
        return 0;
    }

    int begin_verify(AtomicFileDomain domain, AtomicFile file,
                     const std::uint8_t *expected,
                     std::size_t size) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (state_.atomic_operation_aborted) {
            state_.atomic_operation_aborted = false;
            return -EIO;
        }
        const char *path = file_path(domain, file);
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
            return close_atomic_file_locked(FR_OK, -EIO, true);
        }

        state_.operation_data = expected;
        state_.operation_size = size;
        state_.operation_offset = 0U;
        state_.operation = Operation::VerifyData;
        return 0;
    }

    int continue_verify() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (state_.atomic_operation_aborted) {
            state_.atomic_operation_aborted = false;
            return -EIO;
        }
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
                return result == FR_OK
                           ? close_atomic_file_locked(FR_OK, -EIO, true)
                           : close_atomic_file_locked(result);
            }
            state_.operation_offset += read_size;
            if (state_.operation_offset == state_.operation_size) {
                state_.operation = Operation::CloseVerify;
            }
            return -EAGAIN;
        }

        return close_atomic_file_locked();
    }

    void cancel_operation() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return;
        }
        if (state_.operation != Operation::Idle) {
            /* cancel 没有返回值，但 close 暴露的介质错误仍必须撤销挂载；
             * 否则下一笔事务会复用一个已经失效的 FIL/SDMMC 会话。 */
            (void)close_atomic_file_locked();
        }
        state_.atomic_operation_aborted = false;
    }

    int rename(AtomicFileDomain domain, AtomicFile from,
               AtomicFile to) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        const char *source = file_path(domain, from);
        const char *destination = file_path(domain, to);
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

    int erase(AtomicFileDomain domain, AtomicFile file) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        const char *path = file_path(domain, file);
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (state_.operation != Operation::Idle) {
            return -EBUSY;
        }
        return path == nullptr ? -EINVAL : handle(f_unlink(path));
    }

    int start_log() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (state_.log_writer_open) {
            return 0;
        }

        /* PX4 无 UTC 时使用 sessNNN/logNNN.ulg。当前产品每次建立新介质会话只
         * 打开一个 full log，因此从 sess001 向上寻找首个空目录，并沿用 PX4
         * v1.17 的首个文件号 log100。目录/文件名均满足当前 FatFs 8.3 限制。 */
        char directory[kMaximumLogPathLength]{};
        char filepath[kMaximumLogPathLength]{};
        for (std::uint16_t session = 1U; session <= 999U; ++session) {
            if (!session_path(session, directory, sizeof(directory))) {
                return -ENAMETOOLONG;
            }
            const FRESULT directory_result = f_mkdir(directory);
            if (directory_result == FR_EXIST) {
                continue;
            }
            if (directory_result != FR_OK) {
                return handle(directory_result);
            }
            if (!join_path(directory, "log100.ulg", filepath,
                           sizeof(filepath))) {
                const FRESULT cleanup_result = f_unlink(directory);
                return finish_fatfs_results_locked(
                    FR_OK, cleanup_result, -ENAMETOOLONG);
            }
            const FRESULT open_result = f_open(
                &state_.log_writer, filepath,
                static_cast<BYTE>(FA_WRITE | FA_CREATE_NEW));
            if (open_result != FR_OK) {
                if (media_failure(open_result)) {
                    return handle(open_result);
                }
                const FRESULT cleanup_result = f_unlink(directory);
                return finish_fatfs_results_locked(
                    open_result, cleanup_result);
            }
            state_.log_writer_open = true;
            return 0;
        }
        return -ENOSPC;
    }

    int append_log(const std::uint8_t *data,
                   std::size_t size) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted || !state_.log_writer_open) {
            return -ENODEV;
        }
        if (data == nullptr || size == 0U ||
            size > std::numeric_limits<UINT>::max()) {
            return -EINVAL;
        }
        /* MAVLink LOG_ENTRY 和当前 FatFs 文件长度都是 32 bit。达到边界时让
         * LogWriter 正常关闭并新建下一会话，不能依赖 4 GiB 溢出后的短写判错。 */
        const std::uint64_t current_size =
            static_cast<std::uint64_t>(f_tell(&state_.log_writer));
        if (current_size > std::numeric_limits<std::uint32_t>::max() ||
            size > std::numeric_limits<std::uint32_t>::max() - current_size) {
            return -EFBIG;
        }
        UINT written{};
        const FRESULT result = f_write(
            &state_.log_writer, data, static_cast<UINT>(size), &written);
        if (result != FR_OK) {
            return handle(result);
        }
        if (written != size) {
            return invalidate_with_io_error();
        }
        return 0;
    }

    int sync_log() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        return sync_log_writer_locked();
    }

    int close_log() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        return close_log_writer_locked(true);
    }

    bool log_open() noexcept override
    {
        FileStoreGuard lock{state_};
        return lock && state_.mounted && state_.log_writer_open;
    }

    int create_log_list(std::uint16_t &count) noexcept override
    {
        FileStoreGuard lock{state_};
        count = 0U;
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted) {
            return -ENODEV;
        }

        // 列表中的 size 必须是稳定快照；先同步当前 ULog，再扫描目录元数据。
        const int synchronized = sync_log_writer_locked();
        if (synchronized != 0) {
            return synchronized;
        }
        int closed = close_log_reader_locked();
        if (closed == 0) {
            closed = close_log_index_locked();
        }
        if (closed == 0) {
            closed = close_log_directories_locked();
        }
        if (closed != 0) {
            return closed;
        }
        const int stale_temporary_removed =
            unlink_if_exists_locked(kLogListTemporaryPath);
        if (stale_temporary_removed != 0) {
            return stale_temporary_removed;
        }

        FRESULT result = f_open(
            &state_.log_index, kLogListTemporaryPath,
            static_cast<BYTE>(FA_WRITE | FA_CREATE_ALWAYS));
        if (result != FR_OK) {
            return handle(result);
        }
        state_.log_index_open = true;

        result = f_opendir(&state_.log_root_directory, kLogDirectoryPath);
        if (result != FR_OK) {
            return abort_log_list_locked(result);
        }
        state_.log_root_open = true;

        for (;;) {
            FILINFO directory_info{};
            result = f_readdir(&state_.log_root_directory, &directory_info);
            if (result != FR_OK) {
                return abort_log_list_locked(result);
            }
            if (directory_info.fname[0] == '\0') {
                break;
            }
            if ((directory_info.fattrib & AM_DIR) == 0U ||
                std::strcmp(directory_info.fname, ".") == 0 ||
                std::strcmp(directory_info.fname, "..") == 0) {
                continue;
            }

            char directory_path[kMaximumLogPathLength]{};
            if (!join_path(kLogDirectoryPath, directory_info.fname,
                           directory_path, sizeof(directory_path))) {
                continue;
            }
            result = f_opendir(&state_.log_child_directory, directory_path);
            if (result != FR_OK) {
                return abort_log_list_locked(result);
            }
            state_.log_child_open = true;

            for (;;) {
                FILINFO file_info{};
                result = f_readdir(&state_.log_child_directory, &file_info);
                if (result != FR_OK) {
                    return abort_log_list_locked(result);
                }
                if (file_info.fname[0] == '\0') {
                    break;
                }
                if ((file_info.fattrib & AM_DIR) != 0U ||
                    file_info.fsize >
                        std::numeric_limits<std::uint32_t>::max()) {
                    continue;
                }
                if (count == std::numeric_limits<std::uint16_t>::max()) {
                    return abort_log_list_locked(FR_DENIED, -EOVERFLOW);
                }

                LogIndexEntry entry{};
                /* 板上没有 RTC，FatFs mtime 是编译期固定日期，不能冒充真实 UTC；
                 * 用 0 让 QGC 明确显示 UnknownDate，文件内仍保留单调启动时间。 */
                entry.time_utc = 0U;
                entry.size_bytes = static_cast<std::uint32_t>(file_info.fsize);
                if (!join_path(directory_path, file_info.fname,
                               entry.filepath, sizeof(entry.filepath))) {
                    continue;
                }
                UINT written{};
                result = f_write(&state_.log_index, &entry,
                                 static_cast<UINT>(sizeof(entry)), &written);
                if (result != FR_OK) {
                    return abort_log_list_locked(result);
                }
                if (written != sizeof(entry)) {
                    (void)abort_log_list_locked(FR_OK);
                    return invalidate_with_io_error();
                }
                ++count;
            }

            result = f_closedir(&state_.log_child_directory);
            state_.log_child_open = false;
            if (result != FR_OK) {
                return abort_log_list_locked(result);
            }
        }

        result = f_closedir(&state_.log_root_directory);
        state_.log_root_open = false;
        if (result != FR_OK) {
            return abort_log_list_locked(result);
        }
        result = f_sync(&state_.log_index);
        const FRESULT index_close_result = f_close(&state_.log_index);
        state_.log_index_open = false;
        const int index_result =
            finish_fatfs_results_locked(result, index_close_result);
        if (index_result != 0) {
            return abort_log_list_locked(FR_OK, index_result);
        }

        const int removed = unlink_if_exists_locked(kLogListPath);
        if (removed != 0) {
            const int cleanup_error = state_.mounted
                                          ? unlink_if_exists_locked(
                                                kLogListTemporaryPath)
                                          : 0;
            if (!state_.mounted && cleanup_error != 0) {
                return cleanup_error;
            }
            return removed;
        }
        result = f_rename(kLogListTemporaryPath, kLogListPath);
        if (result != FR_OK) {
            const int rename_error = handle(result);
            const int cleanup_error = state_.mounted
                                          ? unlink_if_exists_locked(
                                                kLogListTemporaryPath)
                                          : 0;
            if (!state_.mounted && cleanup_error != 0) {
                return cleanup_error;
            }
            return rename_error;
        }
        return 0;
    }

    int read_log_entry(std::uint16_t id,
                       LogFileEntry &entry) noexcept override
    {
        FileStoreGuard lock{state_};
        entry = LogFileEntry{};
        if (!lock) {
            return -EDEADLK;
        }
        LogIndexEntry indexed{};
        const int result = read_log_index_locked(id, indexed);
        if (result != 0) {
            return result;
        }
        entry.time_utc = indexed.time_utc;
        entry.size_bytes = indexed.size_bytes;
        return 0;
    }

    int open_log(std::uint16_t id,
                 LogFileEntry &entry) noexcept override
    {
        FileStoreGuard lock{state_};
        entry = LogFileEntry{};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted) {
            return -ENODEV;
        }
        const int reader_closed = close_log_reader_locked();
        if (reader_closed != 0) {
            return reader_closed;
        }

        LogIndexEntry indexed{};
        const int index_result = read_log_index_locked(id, indexed);
        if (index_result != 0) {
            return index_result;
        }
        const FRESULT result = f_open(
            &state_.log_reader, indexed.filepath, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        state_.log_reader_open = true;
        if (f_size(&state_.log_reader) < indexed.size_bytes) {
            const int close_error = close_log_reader_locked();
            const int size_error = invalidate_with_io_error();
            return close_error != 0 ? close_error : size_error;
        }
        state_.log_reader_size = indexed.size_bytes;
        entry.time_utc = indexed.time_utc;
        entry.size_bytes = indexed.size_bytes;
        return 0;
    }

    int read_log(std::uint32_t offset, std::uint8_t *destination,
                 std::size_t requested,
                 std::size_t &output_size) noexcept override
    {
        FileStoreGuard lock{state_};
        output_size = 0U;
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted || !state_.log_reader_open) {
            return -ENODEV;
        }
        if (destination == nullptr || requested == 0U ||
            requested > std::numeric_limits<UINT>::max()) {
            return -EINVAL;
        }
        if (offset >= state_.log_reader_size) {
            return 0;
        }
        const std::size_t remaining = state_.log_reader_size - offset;
        const UINT amount = static_cast<UINT>(std::min(requested, remaining));
        FRESULT result = FR_OK;
        if (f_tell(&state_.log_reader) != offset) {
            result = f_lseek(&state_.log_reader, offset);
        }
        if (result != FR_OK) {
            return handle(result);
        }
        UINT read_size{};
        result = f_read(&state_.log_reader, destination, amount, &read_size);
        if (result != FR_OK) {
            return handle(result);
        }
        if (read_size != amount) {
            return invalidate_with_io_error();
        }
        output_size = read_size;
        return 0;
    }

    void close_log_transfer() noexcept override
    {
        FileStoreGuard lock{state_};
        if (lock) {
            close_log_reader_locked();
        }
    }

    int erase_logs() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted) {
            return -ENODEV;
        }

        /* LOG_ERASE 只触及 PX4 /log 树。先关闭当前 writer/reader，避免 FatFs
         * 在 _FS_LOCK=0 时删除仍由本进程打开的文件；参数和任务目录绝不受影响。 */
        int first_error = close_log_writer_locked(true);
        if (!state_.mounted) {
            return first_error != 0 ? first_error : -EIO;
        }
        const int reader_closed = close_log_reader_locked();
        if (first_error == 0 && reader_closed != 0) {
            first_error = reader_closed;
        }
        const int index_closed = close_log_index_locked();
        if (first_error == 0 && index_closed != 0) {
            first_error = index_closed;
        }
        const int directories_closed = close_log_directories_locked();
        if (first_error == 0 && directories_closed != 0) {
            first_error = directories_closed;
        }
        if (!state_.mounted) {
            return first_error != 0 ? first_error : -EIO;
        }
        const int list_removed = unlink_if_exists_locked(kLogListPath);
        if (first_error == 0 && list_removed != 0) {
            first_error = list_removed;
        }
        if (!state_.mounted) {
            return first_error != 0 ? first_error : -EIO;
        }
        const int temporary_removed =
            unlink_if_exists_locked(kLogListTemporaryPath);
        if (first_error == 0 && temporary_removed != 0) {
            first_error = temporary_removed;
        }
        if (!state_.mounted) {
            return first_error != 0 ? first_error : -EIO;
        }

        FRESULT result = f_opendir(&state_.log_root_directory,
                                   kLogDirectoryPath);
        if (result != FR_OK) {
            const int open_error = record_fatfs_error_locked(
                result, first_error);
            return open_error != 0 ? open_error : first_error;
        }
        state_.log_root_open = true;

        for (;;) {
            FILINFO root_entry{};
            result = f_readdir(&state_.log_root_directory, &root_entry);
            if (result != FR_OK || root_entry.fname[0] == '\0') {
                break;
            }
            if (std::strcmp(root_entry.fname, ".") == 0 ||
                std::strcmp(root_entry.fname, "..") == 0) {
                continue;
            }
            char root_path[kMaximumLogPathLength]{};
            if (!join_path(kLogDirectoryPath, root_entry.fname,
                           root_path, sizeof(root_path))) {
                if (first_error == 0) {
                    first_error = -ENAMETOOLONG;
                }
                continue;
            }

            if ((root_entry.fattrib & AM_DIR) != 0U) {
                const int directory_error = erase_log_directory_locked(
                    root_path, first_error);
                if (directory_error != 0) {
                    return directory_error;
                }
            } else {
                const int file_error = record_fatfs_error_locked(
                    f_unlink(root_path), first_error);
                if (file_error != 0) {
                    return file_error;
                }
            }
        }

        const FRESULT close_root = f_closedir(&state_.log_root_directory);
        state_.log_root_open = false;
        const int root_error =
            finish_fatfs_results_locked(result, close_root);
        if (root_error != 0) {
            if (!state_.mounted) {
                return root_error;
            }
            if (first_error == 0) {
                first_error = root_error;
            }
        }
        return first_error;
    }

private:
    int finish_fatfs_results_locked(FRESULT primary_result,
                                    FRESULT cleanup_result,
                                    int explicit_error = 0,
                                    bool invalidate_explicit = false) noexcept
    {
        const int primary_error = primary_result == FR_OK
                                      ? explicit_error
                                      : fatfs_error(primary_result);
        const int cleanup_error = cleanup_result == FR_OK
                                      ? 0
                                      : fatfs_error(cleanup_result);
        const bool primary_media_error = media_failure(primary_result);
        const bool cleanup_media_error = media_failure(cleanup_result);

        /* FatFs 主操作失败后仍可能在 close/closedir/unlink 才暴露真正的拔卡。
         * 任一介质级错误都撤销整个会话；若只有 cleanup 属于介质错误，则优先
         * 返回它，避免上层把失效挂载误判成普通文件格式或权限错误。 */
        if (primary_media_error || cleanup_media_error ||
            (invalidate_explicit && explicit_error != 0)) {
            invalidate_mount();
        }
        if (cleanup_media_error && !primary_media_error) {
            return cleanup_error;
        }
        return primary_error != 0 ? primary_error : cleanup_error;
    }

    int close_atomic_file_locked(FRESULT operation_result = FR_OK,
                                 int explicit_error = 0,
                                 bool invalidate_explicit = false) noexcept
    {
        const FRESULT close_result = f_close(&state_.file);
        reset_operation();
        return finish_fatfs_results_locked(
            operation_result, close_result, explicit_error,
            invalidate_explicit);
    }

    int record_fatfs_error_locked(FRESULT result,
                                  int &first_error) noexcept
    {
        if (result == FR_OK) {
            return 0;
        }
        const int error = handle(result);
        if (!state_.mounted) {
            return error;
        }
        if (first_error == 0) {
            first_error = error;
        }
        return 0;
    }

    int erase_log_directory_locked(const char *directory_path,
                                   int &first_error) noexcept
    {
        FRESULT result = f_opendir(
            &state_.log_child_directory, directory_path);
        if (result != FR_OK) {
            return record_fatfs_error_locked(result, first_error);
        }
        state_.log_child_open = true;

        /* PX4 的正常布局只有 sessNNN/logNNN.ulg 一层。先只读扫描，确认没有
         * 更深目录后再进入删除遍历；遇到未知目录返回 ENOTEMPTY 并原样保留，
         * 防止 LOG_ERASE 在无法完整递归时先删掉同目录内的一部分日志。 */
        bool can_delete = true;
        for (;;) {
            FILINFO entry{};
            result = f_readdir(&state_.log_child_directory, &entry);
            if (result != FR_OK) {
                const int error = record_fatfs_error_locked(
                    result, first_error);
                if (error != 0) {
                    return error;
                }
                can_delete = false;
                break;
            }
            if (entry.fname[0] == '\0') {
                break;
            }
            if ((entry.fattrib & AM_DIR) != 0U &&
                std::strcmp(entry.fname, ".") != 0 &&
                std::strcmp(entry.fname, "..") != 0) {
                if (first_error == 0) {
                    first_error = -ENOTEMPTY;
                }
                can_delete = false;
                break;
            }
        }

        if (can_delete) {
            result = f_readdir(&state_.log_child_directory, nullptr);
            const int rewind_error = record_fatfs_error_locked(
                result, first_error);
            if (rewind_error != 0) {
                return rewind_error;
            }
            can_delete = result == FR_OK;
        }

        while (can_delete) {
            FILINFO entry{};
            result = f_readdir(&state_.log_child_directory, &entry);
            if (result != FR_OK) {
                const int error = record_fatfs_error_locked(
                    result, first_error);
                if (error != 0) {
                    return error;
                }
                can_delete = false;
                break;
            }
            if (entry.fname[0] == '\0') {
                break;
            }
            if ((entry.fattrib & AM_DIR) != 0U) {
                if (std::strcmp(entry.fname, ".") != 0 &&
                    std::strcmp(entry.fname, "..") != 0 &&
                    first_error == 0) {
                    first_error = -ENOTEMPTY;
                }
                if (std::strcmp(entry.fname, ".") != 0 &&
                    std::strcmp(entry.fname, "..") != 0) {
                    can_delete = false;
                }
                continue;
            }

            char child_path[kMaximumLogPathLength]{};
            if (!join_path(directory_path, entry.fname,
                           child_path, sizeof(child_path))) {
                if (first_error == 0) {
                    first_error = -ENAMETOOLONG;
                }
                can_delete = false;
                continue;
            }
            const int removed = record_fatfs_error_locked(
                f_unlink(child_path), first_error);
            if (removed != 0) {
                return removed;
            }
        }

        const FRESULT close_result =
            f_closedir(&state_.log_child_directory);
        state_.log_child_open = false;
        const int close_error = record_fatfs_error_locked(
            close_result, first_error);
        if (close_error != 0) {
            return close_error;
        }
        if (close_result != FR_OK) {
            can_delete = false;
        }
        if (!can_delete) {
            return 0;
        }
        return record_fatfs_error_locked(
            f_unlink(directory_path), first_error);
    }

    int probe_media_locked() noexcept
    {
        if (!state_.mounted || disk_status(0) != 0U ||
            disk_ioctl(0, CTRL_SYNC, nullptr) != RES_OK) {
            /* 无 NCD 板只能把成功的有界命令/I/O称为“当前可用”。探测失败立即
             * 撤销全部 FIL/DIR 和挂载；下一次 initialize 必须 HAL 重新初始化。 */
            invalidate_mount();
            return -ENODEV;
        }
        return 0;
    }

    int ensure_directory_locked(const char *path) noexcept
    {
        FILINFO information{};
        FRESULT result = f_stat(path, &information);
        if (result == FR_NO_FILE || result == FR_NO_PATH) {
            result = f_mkdir(path);
            if (result != FR_OK && result != FR_EXIST) {
                return handle(result);
            }
            information = FILINFO{};
            result = f_stat(path, &information);
        }
        if (result != FR_OK) {
            return handle(result);
        }
        /* 同名普通文件不能满足目录合同；明确返回 ENOTDIR，避免后续 open 的
         * FR_NO_PATH 被误判成“卡不在位”并反复重初始化健康介质。 */
        return (information.fattrib & AM_DIR) != 0U ? 0 : -ENOTDIR;
    }

    int sync_log_writer_locked() noexcept
    {
        if (!state_.log_writer_open) {
            return 0;
        }
        const FRESULT result = f_sync(&state_.log_writer);
        return result == FR_OK ? 0 : handle(result);
    }

    int close_log_writer_locked(bool synchronize) noexcept
    {
        if (!state_.log_writer_open) {
            return 0;
        }
        FRESULT result = FR_OK;
        if (synchronize) {
            result = f_sync(&state_.log_writer);
        }
        const FRESULT close_result = f_close(&state_.log_writer);
        state_.log_writer_open = false;
        return finish_fatfs_results_locked(result, close_result);
    }

    int close_log_reader_locked() noexcept
    {
        FRESULT result = FR_OK;
        if (state_.log_reader_open) {
            result = f_close(&state_.log_reader);
            state_.log_reader_open = false;
        }
        state_.log_reader_size = 0U;
        return result == FR_OK ? 0 : handle(result);
    }

    int close_log_index_locked() noexcept
    {
        FRESULT result = FR_OK;
        if (state_.log_index_open) {
            result = f_close(&state_.log_index);
            state_.log_index_open = false;
        }
        return result == FR_OK ? 0 : handle(result);
    }

    int close_log_directories_locked() noexcept
    {
        int first_error = 0;
        if (state_.log_child_open) {
            const FRESULT result = f_closedir(&state_.log_child_directory);
            state_.log_child_open = false;
            if (result != FR_OK) {
                first_error = handle(result);
                if (!state_.mounted) {
                    return first_error;
                }
            }
        }
        if (state_.log_root_open) {
            const FRESULT result = f_closedir(&state_.log_root_directory);
            state_.log_root_open = false;
            if (result != FR_OK) {
                const int error = handle(result);
                if (first_error == 0 || !state_.mounted) {
                    first_error = error;
                }
            }
        }
        return first_error;
    }

    int unlink_if_exists_locked(const char *path) noexcept
    {
        const FRESULT result = f_unlink(path);
        if (result == FR_OK || result == FR_NO_FILE || result == FR_NO_PATH) {
            return 0;
        }
        return handle(result);
    }

    int abort_log_list_locked(FRESULT result,
                              int explicit_error = 0) noexcept
    {
        const int primary_error =
            result == FR_OK ? explicit_error : handle(result);
        int cleanup_error = close_log_directories_locked();
        const int index_error = close_log_index_locked();
        if (cleanup_error == 0) {
            cleanup_error = index_error;
        }
        if (state_.mounted) {
            const int unlink_error =
                unlink_if_exists_locked(kLogListTemporaryPath);
            if (cleanup_error == 0) {
                cleanup_error = unlink_error;
            }
        }
        /* close/closedir/unlink 若发现介质失效，必须优先返回该错误并保持卸载；
         * 普通解析/容量错误则仍保留最初失败原因。 */
        if (!state_.mounted && cleanup_error != 0) {
            return cleanup_error;
        }
        return primary_error != 0 ? primary_error : cleanup_error;
    }

    int read_log_index_locked(std::uint16_t id,
                              LogIndexEntry &entry) noexcept
    {
        entry = LogIndexEntry{};
        if (!state_.mounted) {
            return -ENODEV;
        }
        const int stale_index_closed = close_log_index_locked();
        if (stale_index_closed != 0) {
            return stale_index_closed;
        }
        FRESULT result = f_open(
            &state_.log_index, kLogListPath, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        state_.log_index_open = true;

        const FSIZE_t offset =
            static_cast<FSIZE_t>(id) * static_cast<FSIZE_t>(sizeof(entry));
        if (offset > f_size(&state_.log_index) ||
            f_size(&state_.log_index) - offset < sizeof(entry)) {
            const int closed = close_log_index_locked();
            return closed != 0 ? closed : -ENOENT;
        }
        result = f_lseek(&state_.log_index, offset);
        if (result != FR_OK) {
            const int operation_error = handle(result);
            const int close_error = close_log_index_locked();
            return !state_.mounted && close_error != 0
                       ? close_error
                       : operation_error;
        }
        UINT read_size{};
        result = f_read(&state_.log_index, &entry,
                        static_cast<UINT>(sizeof(entry)), &read_size);
        const FRESULT close_result = f_close(&state_.log_index);
        state_.log_index_open = false;
        const int operation_result =
            finish_fatfs_results_locked(result, close_result);
        if (operation_result != 0) {
            return operation_result;
        }
        if (read_size != sizeof(entry) || !valid_log_path(entry.filepath)) {
            return invalidate_with_io_error();
        }
        return 0;
    }

    void reset_operation() noexcept
    {
        state_.operation_data = nullptr;
        state_.operation_size = 0U;
        state_.operation_offset = 0U;
        state_.operation = Operation::Idle;
    }

    void invalidate_mount() noexcept
    {
        /* 媒体级错误会同时取消原子事务、ULog writer、下载 reader 与列表扫描，
         * 再卸载唯一卷。下一轮必须完整 SDMMC/FatFs 重建，任何旧卡路径、文件
         * 位置或已打开对象都不能跨介质会话复用。 */
        if (state_.log_child_open) {
            (void)f_closedir(&state_.log_child_directory);
            state_.log_child_open = false;
        }
        if (state_.log_root_open) {
            (void)f_closedir(&state_.log_root_directory);
            state_.log_root_open = false;
        }
        if (state_.log_index_open) {
            (void)f_close(&state_.log_index);
            state_.log_index_open = false;
        }
        if (state_.log_reader_open) {
            (void)f_close(&state_.log_reader);
            state_.log_reader_open = false;
        }
        state_.log_reader_size = 0U;
        if (state_.log_writer_open) {
            (void)f_close(&state_.log_writer);
            state_.log_writer_open = false;
        }
        if (state_.operation != Operation::Idle) {
            (void)f_close(&state_.file);
            reset_operation();
            /* 日志或列表路径也可能发现介质故障；若此时原子事务跨 Run 保持打开，
             * 下一次 continue 必须收到 -EIO，而不是把被动复位误报成 -EINVAL。 */
            state_.atomic_operation_aborted = true;
        }
        (void)f_mount(nullptr, kVolumePath, 0);
        state_.mounted = false;
    }

    int invalidate_with_io_error() noexcept
    {
        // 长度、短读或逐字节校验失败说明当前文件/介质视图不可继续信任；与 FatFs
        // FR_DISK_ERR 一样先撤销挂载，使下一轮热插拔探测完成全量重初始化。
        invalidate_mount();
        return -EIO;
    }

    int handle(FRESULT result) noexcept
    {
        if (media_failure(result)) {
            invalidate_mount();
        }
        return fatfs_error(result);
    }

    FatFsAtomicFileStoreState &state_;
};

} // namespace

AtomicFileStore &atomic_file_store() noexcept
{
    static FatFsFileStore instance{g_file_store_state, synchronization()};
    return static_cast<AtomicFileStore &>(instance);
}

LogFileStore &log_file_store() noexcept
{
    /* 两个 capability 必须指向同一个后端实例，才能共享 FATFS、介质失效状态和
     * 物理调用 mutex。先经 atomic accessor 完成唯一构造，再取其 Log 视图。 */
    auto &atomic = atomic_file_store();
    return static_cast<LogFileStore &>(
        static_cast<FatFsFileStore &>(atomic));
}

} // namespace dima::platform::freertos
