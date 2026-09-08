#include "freertos/Backend.hpp"

#include "diskio.h"
#include "ff.h"
#include "api/LogFileStore.hpp"
#include "api/Synchronization.hpp"
#include "api/Time.hpp"
#include "logger_sidecar.hpp"

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
constexpr const char *kUlogFilename = "log100.ulg";
constexpr const char *kMetadataFilename = "meta.bin";
constexpr std::size_t kMaximumLogSessions = 999U;
constexpr std::size_t kRecoveryChunkBytes = 4096U;
constexpr std::uint64_t kSpaceCorrectionIntervalUs = 60000000ULL;
constexpr std::uint64_t kMinimumFreeBytes = 64ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaximumFreeBytes = 512ULL * 1024ULL * 1024ULL;
constexpr std::uint8_t kUlogMagic[]{
    'U', 'L', 'o', 'g', 0x01U, 0x12U, 0x35U, 0x01U};

namespace log_sidecar = dima::modules::logging::generated::sidecar;

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

bool delete_path(std::uint16_t number, char *destination,
                 std::size_t capacity) noexcept
{
    constexpr char kTemplate[] = "0:/log/del000";
    if (number == 0U || number > 999U || destination == nullptr ||
        capacity < sizeof(kTemplate)) {
        return false;
    }
    std::memcpy(destination, kTemplate, sizeof(kTemplate));
    destination[10] = static_cast<char>('0' + (number / 100U));
    destination[11] = static_cast<char>('0' + ((number / 10U) % 10U));
    destination[12] = static_cast<char>('0' + (number % 10U));
    return true;
}

char ascii_lower(char value) noexcept
{
    return value >= 'A' && value <= 'Z'
               ? static_cast<char>(value - 'A' + 'a')
               : value;
}

bool ascii_equal(const char *left, const char *right) noexcept
{
    if (left == nullptr || right == nullptr) {
        return false;
    }
    while (*left != '\0' && *right != '\0') {
        if (ascii_lower(*left) != ascii_lower(*right)) {
            return false;
        }
        ++left;
        ++right;
    }
    return *left == '\0' && *right == '\0';
}

bool numbered_directory(const char *name, const char *prefix,
                        std::uint16_t &number) noexcept
{
    number = 0U;
    if (name == nullptr || prefix == nullptr || std::strlen(name) != 7U ||
        std::strlen(prefix) != 4U) {
        return false;
    }
    for (std::size_t index = 0U; index < 4U; ++index) {
        if (ascii_lower(name[index]) != ascii_lower(prefix[index])) {
            return false;
        }
    }
    if (name[4] < '0' || name[4] > '9' ||
        name[5] < '0' || name[5] > '9' ||
        name[6] < '0' || name[6] > '9') {
        return false;
    }
    number = static_cast<std::uint16_t>(
        (name[4] - '0') * 100 + (name[5] - '0') * 10 +
        (name[6] - '0'));
    return number != 0U;
}

bool session_file_path(std::uint16_t number, const char *filename,
                       char *destination, std::size_t capacity) noexcept
{
    char directory[kMaximumLogPathLength]{};
    return session_path(number, directory, sizeof(directory)) &&
           join_path(directory, filename, destination, capacity);
}

bool valid_log_path(const char *path) noexcept
{
    constexpr char kPrefix[] = "0:/log/";
    constexpr char kSuffix[] = "/log100.ulg";
    if (path == nullptr ||
        std::strncmp(path, kPrefix, sizeof(kPrefix) - 1U) != 0) {
        return false;
    }
    const void *terminator = std::memchr(path, '\0', kMaximumLogPathLength);
    if (terminator == nullptr) {
        return false;
    }
    const std::size_t length = static_cast<const char *>(terminator) - path;
    if (length != (sizeof(kPrefix) - 1U) + 7U + (sizeof(kSuffix) - 1U) ||
        !ascii_equal(path + sizeof(kPrefix) - 1U + 7U, kSuffix)) {
        return false;
    }
    char directory_name[8]{};
    std::memcpy(directory_name, path + sizeof(kPrefix) - 1U, 7U);
    std::uint16_t number = 0U;
    return numbered_directory(directory_name, "sess", number);
}

std::uint16_t session_number_from_log_path(const char *path) noexcept
{
    if (!valid_log_path(path)) {
        return 0U;
    }
    char directory_name[8]{};
    std::memcpy(directory_name, path + 7U, 7U);
    std::uint16_t number = 0U;
    return numbered_directory(directory_name, "sess", number) ? number : 0U;
}

enum : std::uint8_t {
    kCatalogValidUlog = 1U << 0U,
    kCatalogHasMetadata = 1U << 1U,
    kCatalogNeedsRepair = 1U << 2U,
    kCatalogDeleteEmpty = 1U << 3U,
    kCatalogProtectedUnknown = 1U << 4U,
    kCatalogHasUlogFile = 1U << 5U,
    kCatalogSizeUnrepresentable = 1U << 6U,
};

struct SessionCatalogEntry {
    std::uint64_t sequence{0U};
    std::uint32_t time_utc{0U};
    std::uint32_t file_size{0U};
    std::uint16_t session_number{0U};
    std::uint8_t sidecar_flags{0U};
    std::uint8_t metadata_records{0U};
    std::uint8_t state{0U};
    std::uint8_t reserved{0U};
};

static_assert(sizeof(SessionCatalogEntry) == 24U,
              "bounded SD log catalogue RAM contract changed");

enum class LogMaintenancePhase : std::uint8_t {
    OpenScan,
    ScanDirectories,
    PrepareRepair,
    RepairEntries,
    RecoveryCrc,
    Ready,
};

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

int new_session_reclaim_result(int result) noexcept
{
    /* 新会话预留目录或空间时，没有可删候选与 reader/未知文件保护都表现为
     * ENOSPC；真实介质错误必须原样上送，让 LogWriter 撤销挂载并走 3 s 重试。 */
    if (result == 0) {
        return -EAGAIN;
    }
    if (result == -ENOENT || result == -EBUSY) {
        return -ENOSPC;
    }
    return result;
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
    FIL log_recovery{};
    DIR log_root_directory{};
    DIR log_child_directory{};
    SessionCatalogEntry log_catalog[kMaximumLogSessions]{};
    log_sidecar::Metadata active_metadata{};
    dima::platform::LogSessionContext pending_log_context{};
    const std::uint8_t *operation_data{nullptr};
    std::size_t operation_size{0U};
    std::size_t operation_offset{0U};
    std::uint32_t log_reader_size{0U};
    /* 大型 catalog 状态必须保持全零静态初始化以落入 BSS；CRC 初值分别在
     * reset/start 与 begin_recovery 路径显式设置，避免整块对象生成 Flash 副本。 */
    std::uint32_t active_crc_state{0U};
    std::uint32_t recovery_crc_state{0U};
    std::uint32_t recovery_size{0U};
    std::uint64_t maximum_session_sequence{0U};
    std::uint64_t total_bytes{0U};
    std::uint64_t available_bytes_estimate{0U};
    std::uint64_t free_space_threshold_bytes{0U};
    std::uint64_t cluster_bytes{0U};
    std::uint64_t last_space_correction_us{0U};
    std::size_t log_catalog_count{0U};
    std::size_t maintenance_cursor{0U};
    std::size_t recovery_catalog_index{0U};
    Operation operation{Operation::Idle};
    LogMaintenancePhase log_maintenance_phase{LogMaintenancePhase::OpenScan};
    dima::platform::Synchronization *synchronization{nullptr};
    dima::platform::MutexHandle mutex{};
    std::uint16_t active_session_number{0U};
    std::uint16_t log_reader_session_number{0U};
    std::uint16_t maximum_log_directories{0U};
    std::uint8_t active_metadata_records{0U};
    bool log_writer_open{false};
    bool log_reader_open{false};
    bool log_index_open{false};
    bool log_root_open{false};
    bool log_child_open{false};
    bool recovery_file_open{false};
    bool pending_log_context_valid{false};
    bool erase_logs_requested{false};
    bool atomic_operation_aborted{false};
    bool mounted{false};
};

FatFsAtomicFileStoreState g_file_store_state{};
/* wq:storage 的任务栈总共只有 4 KiB；恢复块必须独立放在零初始化 BSS，不能在
 * service Run 栈上放同尺寸数组，也不能塞进含非零初值的 state 而额外占用 Flash。 */
alignas(32) std::uint8_t g_log_recovery_buffer[kRecoveryChunkBytes]{};

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
        reset_log_maintenance_locked();
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

        std::uint64_t cluster_bytes = 0U;
        const int result = read_space_locked(information, cluster_bytes);
        if (result != 0) {
            return result;
        }
        return information.total_bytes != 0U ? 0 : -ENODEV;
    }

    int configure_log_maintenance(
        std::uint16_t maximum_directories) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (maximum_directories == 0U ||
            maximum_directories > kMaximumLogSessions) {
            return -EINVAL;
        }
        /* 此接口只发布重启参数快照，不访问介质。上限跨重新挂载保留，使无记录
         * 意图下的首次恢复扫描也能收敛目录，而不会创建一个空 ULog 会话。 */
        state_.maximum_log_directories = maximum_directories;
        return 0;
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

    int start_log(
        const dima::platform::LogSessionContext &context) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (context.maximum_directories == 0U ||
            context.maximum_directories > kMaximumLogSessions ||
            context.start_monotonic_us == 0U) {
            return -EINVAL;
        }
        state_.pending_log_context = context;
        state_.pending_log_context_valid = true;
        state_.maximum_log_directories = context.maximum_directories;
        if (state_.log_writer_open) {
            return 0;
        }

        const int maintained = service_log_maintenance_locked();
        if (maintained != 0) {
            return maintained;
        }

        /* 上限包含即将建立的活动会话；先把物理 sess 目录压到 max-1。损坏项
         * 优先、其余按全局 sequence 最旧删除，reader 和未知文件目录永不选中。 */
        if (logger_session_count_locked() >= context.maximum_directories) {
            const int reclaimed = delete_oldest_session_locked();
            return new_session_reclaim_result(reclaimed);
        }

        const int space = refresh_space_locked(hrt_absolute_time());
        if (space != 0) {
            return space;
        }
        if (state_.available_bytes_estimate <
            state_.free_space_threshold_bytes) {
            const int reclaimed = delete_oldest_session_locked();
            return new_session_reclaim_result(reclaimed);
        }

        char directory[kMaximumLogPathLength]{};
        char filepath[kMaximumLogPathLength]{};
        std::uint16_t selected_session = 0U;
        for (std::uint16_t session = 1U; session <= 999U; ++session) {
            if (session_number_in_use_locked(session)) {
                continue;
            }
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
            if (!join_path(directory, kUlogFilename, filepath,
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
            selected_session = session;
            break;
        }
        if (selected_session == 0U) {
            return -ENOSPC;
        }

        if (state_.maximum_session_sequence ==
            std::numeric_limits<std::uint64_t>::max()) {
            const FRESULT close_result = f_close(&state_.log_writer);
            const FRESULT file_removed = f_unlink(filepath);
            const FRESULT directory_removed = f_unlink(directory);
            return finish_fatfs_results_locked(
                close_result != FR_OK ? close_result : file_removed,
                directory_removed, -EOVERFLOW);
        }

        state_.active_metadata = {};
        state_.active_metadata.record_generation = 1U;
        state_.active_metadata.session_sequence =
            state_.maximum_session_sequence + 1U;
        state_.active_metadata.start_monotonic_us =
            context.start_monotonic_us;
        state_.active_metadata.hardware_uid = context.hardware_uid;
        if (context.time_reference.valid &&
            context.time_reference.boot_utc_us <=
                std::numeric_limits<std::uint64_t>::max() -
                    context.start_monotonic_us) {
            const std::uint64_t start_utc =
                context.time_reference.boot_utc_us +
                context.start_monotonic_us;
            if (start_utc >= log_sidecar::kMinimumUtcUs &&
                start_utc / 1000000ULL <=
                std::numeric_limits<std::uint32_t>::max()) {
                state_.active_metadata.flags |= log_sidecar::kUtcValidFlag;
                state_.active_metadata.boot_utc_us =
                    context.time_reference.boot_utc_us;
                state_.active_metadata.start_utc_us = start_utc;
            }
        }
        state_.active_metadata.file_crc32 =
            log_sidecar::crc32_finish(log_sidecar::kCrc32Initial);
        state_.active_metadata_records = 0U;
        state_.active_session_number = selected_session;
        state_.active_crc_state = log_sidecar::kCrc32Initial;
        state_.log_writer_open = true;

        const int metadata_result = append_active_metadata_locked();
        if (metadata_result != 0) {
            const int close_result = close_log_writer_locked(false);
            if (state_.mounted) {
                (void)unlink_if_exists_locked(filepath);
                (void)unlink_if_exists_locked(directory);
            }
            reset_active_log_locked();
            return close_result != 0 ? close_result : metadata_result;
        }

        /* sess 目录和首条 sidecar 也会消耗 FAT 分配簇。创建后立即用真实
         * f_getfree 校正，避免只按后续 ULog payload 扣减而跨过保护空闲线。若
         * 已低于阈值，此会话尚未对 producer 可见，可安全删除空文件并返回
         * ENOSPC；任何清理介质错误则优先原样上送。 */
        const int post_create_space = refresh_space_locked(hrt_absolute_time());
        if (post_create_space != 0 ||
            state_.available_bytes_estimate <
                state_.free_space_threshold_bytes) {
            const int close_result = close_log_writer_locked(false);
            int cleanup_result = 0;
            if (state_.mounted) {
                char metadata_path[kMaximumLogPathLength]{};
                if (!join_path(directory, kMetadataFilename, metadata_path,
                               sizeof(metadata_path))) {
                    cleanup_result = -ENAMETOOLONG;
                } else {
                    cleanup_result = unlink_if_exists_locked(metadata_path);
                }
                if (cleanup_result == 0 && state_.mounted) {
                    cleanup_result = unlink_if_exists_locked(filepath);
                }
                if (cleanup_result == 0 && state_.mounted) {
                    const FRESULT removed = f_unlink(directory);
                    if (removed != FR_OK && removed != FR_NO_FILE &&
                        removed != FR_NO_PATH) {
                        cleanup_result = handle(removed);
                    }
                }
            }
            reset_active_log_locked();
            if (post_create_space != 0) {
                return post_create_space;
            }
            if (close_result != 0) {
                return close_result;
            }
            return cleanup_result != 0 ? cleanup_result : -ENOSPC;
        }

        SessionCatalogEntry &entry =
            state_.log_catalog[state_.log_catalog_count++];
        entry = {};
        entry.sequence = state_.active_metadata.session_sequence;
        entry.file_size = 0U;
        entry.session_number = selected_session;
        entry.sidecar_flags = state_.active_metadata.flags;
        entry.metadata_records = state_.active_metadata_records;
        /* 文件刚创建时尚无 ULog magic，不能提前向 QGC 宣称有效；首个 append
         * 完整校验 8-byte header 后才置 kCatalogValidUlog。 */
        entry.state = kCatalogHasMetadata | kCatalogHasUlogFile;
        if ((state_.active_metadata.flags & log_sidecar::kUtcValidFlag) != 0U) {
            entry.time_utc = static_cast<std::uint32_t>(
                state_.active_metadata.start_utc_us / 1000000ULL);
        }
        state_.maximum_session_sequence = entry.sequence;
        state_.last_space_correction_us = hrt_absolute_time();
        return 0;
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
        if (current_size == 0U &&
            (size < sizeof(kUlogMagic) ||
             std::memcmp(data, kUlogMagic, sizeof(kUlogMagic)) != 0)) {
            /* 每个轮转文件必须从完整 ULog header 开始；拒绝把中途 D/L 数据
             * 拼进新会话。该错误不写任何字节，旧文件仍可按损坏项恢复。 */
            return -EPROTO;
        }
        if (current_size > std::numeric_limits<std::uint32_t>::max() ||
            size > std::numeric_limits<std::uint32_t>::max() - current_size) {
            return -EFBIG;
        }
        if (state_.cluster_bytes == 0U) {
            return -EIO;
        }
        if (state_.last_space_correction_us == 0U) {
            /* 历史会话删除后旧估算只会偏小；下一次 payload 写入前先重新读取 FAT
             * 空闲簇，避免实际已经回收成功却误报 ENOSPC 并关闭当前会话。 */
            const int refreshed = refresh_space_locked(hrt_absolute_time());
            if (refreshed != 0) {
                return refreshed;
            }
        }
        const std::uint64_t next_size = current_size + size;
        const std::uint64_t old_clusters =
            (current_size + state_.cluster_bytes - 1U) /
            state_.cluster_bytes;
        const std::uint64_t new_clusters =
            (next_size + state_.cluster_bytes - 1U) /
            state_.cluster_bytes;
        const std::uint64_t allocation_cost =
            (new_clusters - old_clusters) * state_.cluster_bytes;
        const std::uint64_t minimum_after_write =
            state_.free_space_threshold_bytes > state_.cluster_bytes
                ? state_.free_space_threshold_bytes - state_.cluster_bytes
                : 0U;
        if (allocation_cost > state_.available_bytes_estimate ||
            state_.available_bytes_estimate - allocation_cost <
                minimum_after_write) {
            /* 以 FAT 分配簇而非 payload 字节扣减估算；在下一次分配会越过保护
             * 线前拒绝整块，保证关闭后至少保留 threshold-1 cluster。 */
            return -ENOSPC;
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
        state_.active_crc_state = log_sidecar::crc32_update(
            state_.active_crc_state, data, size);
        state_.active_metadata.file_size = static_cast<std::uint32_t>(next_size);
        state_.active_metadata.file_crc32 =
            log_sidecar::crc32_finish(state_.active_crc_state);
        state_.available_bytes_estimate -= allocation_cost;
        if (current_size == 0U) {
            for (std::size_t index = 0U; index < state_.log_catalog_count;
                 ++index) {
                SessionCatalogEntry &entry = state_.log_catalog[index];
                if (entry.session_number == state_.active_session_number) {
                    entry.state |= kCatalogValidUlog;
                    break;
                }
            }
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
        if (!state_.log_writer_open) {
            return 0;
        }

        const FRESULT sync_result = f_sync(&state_.log_writer);
        const FSIZE_t final_size = f_size(&state_.log_writer);
        const FRESULT close_result = f_close(&state_.log_writer);
        state_.log_writer_open = false;
        const int file_result = finish_fatfs_results_locked(
            sync_result, close_result,
            final_size <= std::numeric_limits<std::uint32_t>::max()
                ? 0
                : -EFBIG);
        if (file_result != 0 || !state_.mounted) {
            reset_active_log_locked();
            return file_result != 0 ? file_result : -EIO;
        }

        state_.active_metadata.flags |= log_sidecar::kClosedFlag;
        ++state_.active_metadata.record_generation;
        state_.active_metadata.file_size = static_cast<std::uint32_t>(final_size);
        state_.active_metadata.file_crc32 =
            log_sidecar::crc32_finish(state_.active_crc_state);
        const int metadata_result = append_active_metadata_locked();
        update_active_catalog_locked();
        reset_active_log_locked();
        return metadata_result;
    }

    bool log_open() noexcept override
    {
        FileStoreGuard lock{state_};
        return lock && state_.mounted && state_.log_writer_open;
    }

    int update_log_time(
        const dima::platform::LogTimeReference &reference) noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        if (!state_.mounted || !state_.log_writer_open) {
            return -ENODEV;
        }
        if (!reference.valid ||
            (state_.active_metadata.flags & log_sidecar::kUtcValidFlag) != 0U) {
            return 0;
        }
        if (reference.boot_utc_us >
            std::numeric_limits<std::uint64_t>::max() -
                state_.active_metadata.start_monotonic_us) {
            return -ERANGE;
        }
        const std::uint64_t start_utc =
            reference.boot_utc_us +
            state_.active_metadata.start_monotonic_us;
        if (start_utc < log_sidecar::kMinimumUtcUs ||
            start_utc / 1000000ULL >
            std::numeric_limits<std::uint32_t>::max()) {
            return -ERANGE;
        }

        const log_sidecar::Metadata previous = state_.active_metadata;
        state_.active_metadata.flags |= log_sidecar::kUtcValidFlag;
        state_.active_metadata.boot_utc_us = reference.boot_utc_us;
        state_.active_metadata.start_utc_us = start_utc;
        ++state_.active_metadata.record_generation;
        state_.active_metadata.file_size = static_cast<std::uint32_t>(
            f_size(&state_.log_writer));
        state_.active_metadata.file_crc32 =
            log_sidecar::crc32_finish(state_.active_crc_state);
        const int result = append_active_metadata_locked();
        if (result != 0) {
            /* 媒体错误会在 handle() 内撤销整份挂载和活动会话；只有 writer
             * 仍有效时才回滚 RAM 元数据，不能在卸载后复活旧卡的身份。 */
            if (state_.mounted && state_.log_writer_open) {
                state_.active_metadata = previous;
            }
            return result;
        }
        update_active_catalog_locked();
        return 0;
    }

    int service_log_maintenance() noexcept override
    {
        FileStoreGuard lock{state_};
        if (!lock) {
            return -EDEADLK;
        }
        return service_log_maintenance_locked();
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

        if (state_.log_maintenance_phase != LogMaintenancePhase::Ready) {
            const int maintained = service_log_maintenance_locked();
            return maintained == 0 ? -EAGAIN : maintained;
        }
        return create_log_index_from_catalog_locked(count);
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
        state_.log_reader_session_number =
            session_number_from_log_path(indexed.filepath);
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

        /* LOG_ERASE 只登记“删除已关闭 Logger 会话”的维护请求。活动 writer
         * 继续记录；reader 已由 MAVLink handler 结束，未知目录/文件不进入目录
         * catalog。每次 service 最多重命名并清理一个 sessNNN。 */
        state_.erase_logs_requested = true;
        (void)close_log_index_locked();
        (void)unlink_if_exists_locked(kLogListPath);
        (void)unlink_if_exists_locked(kLogListTemporaryPath);
        const int maintained = service_log_maintenance_locked();
        return maintained == -EAGAIN ? -EAGAIN : maintained;
    }

private:
    void reset_active_log_locked() noexcept
    {
        state_.active_metadata = {};
        state_.active_metadata_records = 0U;
        state_.active_session_number = 0U;
        state_.active_crc_state = log_sidecar::kCrc32Initial;
    }

    void reset_log_maintenance_locked() noexcept
    {
        if (state_.recovery_file_open) {
            (void)f_close(&state_.log_recovery);
            state_.recovery_file_open = false;
        }
        for (SessionCatalogEntry &entry : state_.log_catalog) {
            entry = {};
        }
        state_.log_catalog_count = 0U;
        state_.maximum_session_sequence = 0U;
        state_.maintenance_cursor = 0U;
        state_.recovery_catalog_index = 0U;
        state_.recovery_crc_state = log_sidecar::kCrc32Initial;
        state_.recovery_size = 0U;
        state_.total_bytes = 0U;
        state_.available_bytes_estimate = 0U;
        state_.free_space_threshold_bytes = 0U;
        state_.cluster_bytes = 0U;
        state_.last_space_correction_us = 0U;
        state_.log_maintenance_phase = LogMaintenancePhase::OpenScan;
        reset_active_log_locked();
    }

    int read_metadata_locked(std::uint16_t session,
                             log_sidecar::Metadata &latest,
                             std::uint8_t &record_count) noexcept
    {
        latest = {};
        record_count = 0U;
        char path[kMaximumLogPathLength]{};
        if (!session_file_path(session, kMetadataFilename,
                               path, sizeof(path))) {
            return -ENAMETOOLONG;
        }
        FILINFO information{};
        FRESULT result = f_stat(path, &information);
        if (result == FR_NO_FILE || result == FR_NO_PATH) {
            return 0;
        }
        if (result != FR_OK) {
            return handle(result);
        }
        if ((information.fattrib & AM_DIR) != 0U) {
            /* 同名目录属于未知用户内容，既不能截断也不能改造成 Logger 侧车。
             * 调用方会保护该 sess 目录并从 QGC 隐藏，而不是让全卡恢复卡死。 */
            return -EISDIR;
        }

        FIL metadata_file{};
        result = f_open(&metadata_file, path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        std::uint8_t records[
            log_sidecar::kRecordSize * log_sidecar::kMaximumRecords]{};
        const UINT requested = static_cast<UINT>(std::min<FSIZE_t>(
            f_size(&metadata_file), sizeof(records)));
        UINT read_size = 0U;
        result = f_read(&metadata_file, records, requested, &read_size);
        const FRESULT close_result = f_close(&metadata_file);
        const int operation_result = finish_fatfs_results_locked(
            result, close_result, read_size == requested ? 0 : -EIO,
            read_size != requested);
        if (operation_result != 0) {
            return operation_result;
        }

        for (std::size_t offset = 0U;
             offset + log_sidecar::kRecordSize <= read_size &&
             record_count < log_sidecar::kMaximumRecords;
             offset += log_sidecar::kRecordSize) {
            log_sidecar::Metadata decoded{};
            if (!log_sidecar::decode(records + offset, decoded) ||
                decoded.record_generation !=
                    static_cast<std::uint32_t>(record_count) + 1U) {
                /* 断电尾部或 CRC 错误只终止后续解析；此前最后一条有效记录仍是
                 * 会话事实，不把整个 ULog 因半条 sidecar 写入而丢弃。 */
                break;
            }
            latest = decoded;
            ++record_count;
        }
        return 0;
    }

    int append_metadata_locked(std::uint16_t session,
                               const log_sidecar::Metadata &metadata,
                               std::uint8_t &record_count) noexcept
    {
        if (record_count >= log_sidecar::kMaximumRecords) {
            return -EOVERFLOW;
        }
        char path[kMaximumLogPathLength]{};
        if (!session_file_path(session, kMetadataFilename,
                               path, sizeof(path))) {
            return -ENAMETOOLONG;
        }
        FIL metadata_file{};
        FRESULT result = f_open(
            &metadata_file, path,
            static_cast<BYTE>(FA_READ | FA_WRITE | FA_OPEN_ALWAYS));
        if (result != FR_OK) {
            return handle(result);
        }
        const FSIZE_t valid_length = static_cast<FSIZE_t>(record_count) *
                                    log_sidecar::kRecordSize;
        result = f_lseek(&metadata_file, valid_length);
        if (result == FR_OK && f_size(&metadata_file) != valid_length) {
            /* 仅裁掉解析器已经判定无效的断电尾部，再从最后一条有效记录后
             * 追加；有效 64-byte 记录自身从不覆写。 */
            result = f_truncate(&metadata_file);
        }
        std::uint8_t encoded[log_sidecar::kRecordSize]{};
        log_sidecar::encode(metadata, encoded);
        UINT written = 0U;
        if (result == FR_OK) {
            result = f_write(&metadata_file, encoded,
                             static_cast<UINT>(sizeof(encoded)), &written);
        }
        if (result == FR_OK && written != sizeof(encoded)) {
            const FRESULT closed = f_close(&metadata_file);
            return finish_fatfs_results_locked(FR_OK, closed, -EIO, true);
        }
        if (result == FR_OK) {
            result = f_sync(&metadata_file);
        }
        const FRESULT closed = f_close(&metadata_file);
        const int operation_result =
            finish_fatfs_results_locked(result, closed);
        if (operation_result == 0) {
            ++record_count;
        }
        return operation_result;
    }

    int append_active_metadata_locked() noexcept
    {
        return append_metadata_locked(
            state_.active_session_number, state_.active_metadata,
            state_.active_metadata_records);
    }

    void update_active_catalog_locked() noexcept
    {
        for (std::size_t index = 0U; index < state_.log_catalog_count;
             ++index) {
            SessionCatalogEntry &entry = state_.log_catalog[index];
            if (entry.session_number != state_.active_session_number) {
                continue;
            }
            entry.sequence = state_.active_metadata.session_sequence;
            entry.file_size = state_.active_metadata.file_size;
            entry.sidecar_flags = state_.active_metadata.flags;
            entry.metadata_records = state_.active_metadata_records;
            entry.state |= kCatalogHasUlogFile | kCatalogHasMetadata;
            entry.time_utc =
                (state_.active_metadata.flags & log_sidecar::kUtcValidFlag) != 0U
                    ? static_cast<std::uint32_t>(
                          state_.active_metadata.start_utc_us / 1000000ULL)
                    : 0U;
            return;
        }
    }

    int inspect_session_directory_locked(
        std::uint16_t session, SessionCatalogEntry &entry) noexcept
    {
        entry = {};
        entry.session_number = session;
        char directory[kMaximumLogPathLength]{};
        if (!session_path(session, directory, sizeof(directory))) {
            return -ENAMETOOLONG;
        }
        FRESULT result = f_opendir(&state_.log_child_directory, directory);
        if (result != FR_OK) {
            return handle(result);
        }
        state_.log_child_open = true;
        bool unknown = false;
        for (;;) {
            FILINFO child{};
            result = f_readdir(&state_.log_child_directory, &child);
            if (result != FR_OK || child.fname[0] == '\0') {
                break;
            }
            if (ascii_equal(child.fname, ".") || ascii_equal(child.fname, "..")) {
                continue;
            }
            if ((child.fattrib & AM_DIR) != 0U) {
                unknown = true;
                break;
            }
            if (!ascii_equal(child.fname, kUlogFilename) &&
                !ascii_equal(child.fname, kMetadataFilename)) {
                unknown = true;
                break;
            }
        }
        const FRESULT closed = f_closedir(&state_.log_child_directory);
        state_.log_child_open = false;
        const int directory_result =
            finish_fatfs_results_locked(result, closed);
        if (directory_result != 0) {
            return directory_result;
        }
        if (unknown) {
            entry.state |= kCatalogProtectedUnknown;
        }

        /* 合法 Logger 目录最多有两个唯一文件，遇到首个未知项即可停止枚举；
         * 再用精确路径查询 ULog，既保持扫描有界，也不会因目录顺序漏掉有效日志。 */
        char ulog_path[kMaximumLogPathLength]{};
        if (!session_file_path(session, kUlogFilename,
                               ulog_path, sizeof(ulog_path))) {
            return -ENAMETOOLONG;
        }
        FILINFO information{};
        result = f_stat(ulog_path, &information);
        if (result == FR_NO_FILE || result == FR_NO_PATH) {
            if (!unknown) {
                entry.state |= kCatalogDeleteEmpty;
            } else {
                entry.sidecar_flags |= log_sidecar::kCorruptFlag;
            }
            return 0;
        }
        if (result != FR_OK) {
            return handle(result);
        }
        if ((information.fattrib & AM_DIR) != 0U) {
            entry.state |= kCatalogProtectedUnknown;
            entry.sidecar_flags |= log_sidecar::kCorruptFlag;
            return 0;
        }
        entry.state |= kCatalogHasUlogFile;
        if (information.fsize <= std::numeric_limits<std::uint32_t>::max()) {
            entry.file_size = static_cast<std::uint32_t>(information.fsize);
        } else {
            entry.sidecar_flags |= log_sidecar::kCorruptFlag;
            /* LOG_ENTRY 与 sidecar 大小字段均为 32 bit；外部遗留的超界文件
             * 只能隐藏并作为已关闭损坏候选回收，不能做会溢出的 CRC 迁移。 */
            entry.state |= kCatalogSizeUnrepresentable;
            entry.sidecar_flags |= static_cast<std::uint8_t>(
                log_sidecar::kRecoveredFlag |
                log_sidecar::kClosedFlag);
        }
        FIL ulog{};
        result = f_open(&ulog, ulog_path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        std::uint8_t magic[sizeof(kUlogMagic)]{};
        UINT read_size = 0U;
        result = f_read(&ulog, magic, sizeof(magic), &read_size);
        const FRESULT ulog_closed = f_close(&ulog);
        const int ulog_result = finish_fatfs_results_locked(
            result, ulog_closed);
        if (ulog_result != 0) {
            return ulog_result;
        }
        if (read_size == sizeof(magic) &&
            std::memcmp(magic, kUlogMagic, sizeof(kUlogMagic)) == 0) {
            entry.state |= kCatalogValidUlog;
        } else {
            entry.sidecar_flags |= log_sidecar::kCorruptFlag;
        }

        log_sidecar::Metadata metadata{};
        std::uint8_t records = 0U;
        const int metadata_result = read_metadata_locked(
            session, metadata, records);
        if (metadata_result == -EISDIR) {
            entry.state |= kCatalogProtectedUnknown;
            entry.sidecar_flags |= log_sidecar::kCorruptFlag;
            return 0;
        }
        if (metadata_result != 0) {
            return metadata_result;
        }
        if (records != 0U) {
            entry.state |= kCatalogHasMetadata;
            entry.sequence = metadata.session_sequence;
            entry.sidecar_flags |= metadata.flags;
            entry.metadata_records = records;
            if ((metadata.flags & log_sidecar::kUtcValidFlag) != 0U &&
                metadata.start_utc_us / 1000000ULL <=
                    std::numeric_limits<std::uint32_t>::max()) {
                entry.time_utc = static_cast<std::uint32_t>(
                    metadata.start_utc_us / 1000000ULL);
            }
        }
        if (records == 0U ||
            (entry.sidecar_flags & log_sidecar::kClosedFlag) == 0U ||
            metadata.file_size != entry.file_size ||
            ((entry.sidecar_flags & log_sidecar::kCorruptFlag) != 0U &&
             (metadata.flags & log_sidecar::kCorruptFlag) == 0U)) {
            entry.state |= kCatalogNeedsRepair;
        }
        return 0;
    }

    void sort_catalog_by_session_locked() noexcept
    {
        for (std::size_t index = 1U; index < state_.log_catalog_count; ++index) {
            const SessionCatalogEntry value = state_.log_catalog[index];
            std::size_t destination = index;
            while (destination > 0U &&
                   state_.log_catalog[destination - 1U].session_number >
                       value.session_number) {
                state_.log_catalog[destination] =
                    state_.log_catalog[destination - 1U];
                --destination;
            }
            state_.log_catalog[destination] = value;
        }
    }

    void sort_catalog_by_sequence_locked() noexcept
    {
        for (std::size_t index = 1U; index < state_.log_catalog_count; ++index) {
            const SessionCatalogEntry value = state_.log_catalog[index];
            std::size_t destination = index;
            while (destination > 0U &&
                   state_.log_catalog[destination - 1U].sequence >
                       value.sequence) {
                state_.log_catalog[destination] =
                    state_.log_catalog[destination - 1U];
                --destination;
            }
            state_.log_catalog[destination] = value;
        }
    }

    int prepare_catalog_repair_locked() noexcept
    {
        sort_catalog_by_session_locked();
        bool have_valid_sidecar = false;
        std::uint64_t maximum_sequence = 0U;
        for (std::size_t index = 0U; index < state_.log_catalog_count; ++index) {
            SessionCatalogEntry &entry = state_.log_catalog[index];
            if ((entry.state & kCatalogHasMetadata) == 0U) {
                continue;
            }
            have_valid_sidecar = true;
            maximum_sequence = std::max(maximum_sequence, entry.sequence);
            for (std::size_t earlier = 0U; earlier < index; ++earlier) {
                if ((state_.log_catalog[earlier].state &
                     kCatalogHasMetadata) != 0U &&
                    state_.log_catalog[earlier].sequence == entry.sequence) {
                    /* 重复 sequence 不能让两个目录在 QGC 中交换年龄；后出现的
                     * sess 通过 RECOVERED 记录获得大于全卡最大值的新序号。 */
                    entry.state |= kCatalogNeedsRepair;
                    entry.sequence = 0U;
                    break;
                }
            }
        }
        if (have_valid_sidecar && maximum_sequence ==
                                      std::numeric_limits<std::uint64_t>::max()) {
            return -EOVERFLOW;
        }
        std::uint64_t next_sequence = have_valid_sidecar
                                          ? maximum_sequence + 1U
                                          : 1U;
        for (std::size_t index = 0U; index < state_.log_catalog_count; ++index) {
            SessionCatalogEntry &entry = state_.log_catalog[index];
            if ((entry.state & kCatalogDeleteEmpty) != 0U) {
                continue;
            }
            if ((entry.state & kCatalogHasUlogFile) == 0U) {
                /* 只有未知用户内容的 sessNNN 留在 catalogue 中，仅用于占用其
                 * 目录编号；它不计入 Logger 上限，也绝不进入 sidecar 迁移。 */
                continue;
            }
            if ((entry.state & kCatalogHasMetadata) == 0U ||
                entry.sequence == 0U) {
                if (next_sequence == 0U) {
                    return -EOVERFLOW;
                }
                entry.sequence = next_sequence++;
                entry.state |= kCatalogNeedsRepair;
            }
            if ((entry.state & kCatalogSizeUnrepresentable) != 0U) {
                /* 不能把 >UINT32_MAX 的长度或 CRC 写进 v1 sidecar。该项已在 RAM
                 * 标为 CLOSED|RECOVERED|CORRUPT，跳过逐块恢复，避免 4 GiB 外部
                 * 文件让启动维护无限运行；仍可作为损坏会话优先回收。 */
                entry.state &= static_cast<std::uint8_t>(
                    ~kCatalogNeedsRepair);
            }
            maximum_sequence = std::max(maximum_sequence, entry.sequence);
        }
        state_.maximum_session_sequence = maximum_sequence;
        state_.maintenance_cursor = 0U;
        state_.log_maintenance_phase = LogMaintenancePhase::RepairEntries;
        return -EAGAIN;
    }

    int begin_catalog_recovery_locked(std::size_t index) noexcept
    {
        if (index >= state_.log_catalog_count) {
            return -EINVAL;
        }
        char path[kMaximumLogPathLength]{};
        if (!session_file_path(state_.log_catalog[index].session_number,
                               kUlogFilename, path, sizeof(path))) {
            return -ENAMETOOLONG;
        }
        const FRESULT result = f_open(
            &state_.log_recovery, path, FA_READ | FA_OPEN_EXISTING);
        if (result != FR_OK) {
            return handle(result);
        }
        state_.recovery_file_open = true;
        state_.recovery_catalog_index = index;
        state_.recovery_crc_state = log_sidecar::kCrc32Initial;
        state_.recovery_size = 0U;
        state_.log_maintenance_phase = LogMaintenancePhase::RecoveryCrc;
        return -EAGAIN;
    }

    int finish_catalog_recovery_locked() noexcept
    {
        const std::size_t index = state_.recovery_catalog_index;
        if (index >= state_.log_catalog_count) {
            return -EINVAL;
        }
        SessionCatalogEntry &entry = state_.log_catalog[index];
        log_sidecar::Metadata metadata{};
        std::uint8_t records = 0U;
        const int read_result = read_metadata_locked(
            entry.session_number, metadata, records);
        if (read_result != 0) {
            return read_result;
        }
        if (records >= log_sidecar::kMaximumRecords) {
            /* 三条均已占用却仍未形成可恢复终态，无法再保持 append-only。该项
             * 从 QGC 隐藏，但不覆写有效记录或阻塞其他会话。 */
            entry.sidecar_flags |= log_sidecar::kCorruptFlag;
            entry.state &= static_cast<std::uint8_t>(~kCatalogNeedsRepair);
            ++state_.maintenance_cursor;
            state_.log_maintenance_phase = LogMaintenancePhase::RepairEntries;
            return -EAGAIN;
        }
        if (records == 0U) {
            metadata = {};
            metadata.record_generation = 1U;
            metadata.session_sequence = entry.sequence;
            metadata.hardware_uid = state_.pending_log_context_valid
                                        ? state_.pending_log_context.hardware_uid
                                        : 0U;
        } else {
            ++metadata.record_generation;
            metadata.session_sequence = entry.sequence;
        }
        metadata.flags |= static_cast<std::uint8_t>(
            log_sidecar::kRecoveredFlag | log_sidecar::kClosedFlag);
        if ((entry.state & kCatalogValidUlog) == 0U ||
            (entry.sidecar_flags & log_sidecar::kCorruptFlag) != 0U) {
            metadata.flags |= log_sidecar::kCorruptFlag;
        }
        metadata.file_size = state_.recovery_size;
        metadata.file_crc32 =
            log_sidecar::crc32_finish(state_.recovery_crc_state);
        const int appended = append_metadata_locked(
            entry.session_number, metadata, records);
        if (appended != 0) {
            return appended;
        }
        entry.state |= kCatalogHasMetadata;
        entry.state &= static_cast<std::uint8_t>(~kCatalogNeedsRepair);
        entry.sequence = metadata.session_sequence;
        entry.sidecar_flags = metadata.flags;
        entry.metadata_records = records;
        entry.file_size = metadata.file_size;
        entry.time_utc =
            (metadata.flags & log_sidecar::kUtcValidFlag) != 0U &&
                    metadata.start_utc_us / 1000000ULL <=
                        std::numeric_limits<std::uint32_t>::max()
                ? static_cast<std::uint32_t>(
                      metadata.start_utc_us / 1000000ULL)
                : 0U;
        ++state_.maintenance_cursor;
        state_.log_maintenance_phase = LogMaintenancePhase::RepairEntries;
        return -EAGAIN;
    }

    int cleanup_delete_directory_locked(std::uint16_t session) noexcept
    {
        char directory[kMaximumLogPathLength]{};
        if (!delete_path(session, directory, sizeof(directory))) {
            return -ENAMETOOLONG;
        }
        char path[kMaximumLogPathLength]{};
        if (!join_path(directory, kUlogFilename, path, sizeof(path))) {
            return -ENAMETOOLONG;
        }
        int first_error = unlink_if_exists_locked(path);
        if (!state_.mounted) {
            return first_error;
        }
        if (!join_path(directory, kMetadataFilename, path, sizeof(path))) {
            return first_error != 0 ? first_error : -ENAMETOOLONG;
        }
        const int metadata_error = unlink_if_exists_locked(path);
        if (first_error == 0) {
            first_error = metadata_error;
        }
        if (!state_.mounted) {
            return first_error;
        }
        const FRESULT directory_result = f_unlink(directory);
        if (directory_result != FR_OK && directory_result != FR_NO_FILE &&
            directory_result != FR_NO_PATH && directory_result != FR_DENIED) {
            const int error = handle(directory_result);
            if (first_error == 0) {
                first_error = error;
            }
        }
        /* FR_DENIED 表示 delNNN 中出现非 Logger 文件；保留未知内容。正常断电
         * 遗留只有 log100.ulg/meta.bin，会在本步骤完整收敛。 */
        return first_error;
    }

    int logger_owned_directory_locked(std::uint16_t session,
                                      bool &owned) noexcept
    {
        owned = false;
        char directory[kMaximumLogPathLength]{};
        if (!session_path(session, directory, sizeof(directory))) {
            return -ENAMETOOLONG;
        }
        FRESULT result = f_opendir(&state_.log_child_directory, directory);
        if (result != FR_OK) {
            return handle(result);
        }
        state_.log_child_open = true;
        owned = true;
        for (;;) {
            FILINFO child{};
            result = f_readdir(&state_.log_child_directory, &child);
            if (result != FR_OK || child.fname[0] == '\0') {
                break;
            }
            if (ascii_equal(child.fname, ".") || ascii_equal(child.fname, "..")) {
                continue;
            }
            if ((child.fattrib & AM_DIR) != 0U ||
                (!ascii_equal(child.fname, kUlogFilename) &&
                 !ascii_equal(child.fname, kMetadataFilename))) {
                owned = false;
                /* 合法目录只有两个固定文件；首个未知项已足以撤销删除权，立即
                 * 结束枚举，避免用户内容让 storage worker 长时间占用。 */
                break;
            }
        }
        const FRESULT closed = f_closedir(&state_.log_child_directory);
        state_.log_child_open = false;
        return finish_fatfs_results_locked(result, closed);
    }

    void remove_catalog_entry_locked(std::size_t index) noexcept
    {
        if (index >= state_.log_catalog_count) {
            return;
        }
        for (std::size_t next = index + 1U;
             next < state_.log_catalog_count; ++next) {
            state_.log_catalog[next - 1U] = state_.log_catalog[next];
        }
        --state_.log_catalog_count;
        state_.log_catalog[state_.log_catalog_count] = {};
        if (state_.maintenance_cursor > index) {
            --state_.maintenance_cursor;
        }
    }

    int delete_catalog_entry_locked(std::size_t index) noexcept
    {
        if (index >= state_.log_catalog_count) {
            return -ENOENT;
        }
        SessionCatalogEntry &entry = state_.log_catalog[index];
        if (entry.session_number == state_.active_session_number ||
            entry.session_number == state_.log_reader_session_number ||
            (entry.state & kCatalogProtectedUnknown) != 0U) {
            return -EBUSY;
        }
        bool owned = false;
        const int ownership = logger_owned_directory_locked(
            entry.session_number, owned);
        if (ownership != 0) {
            return ownership;
        }
        if (!owned) {
            entry.state |= kCatalogProtectedUnknown;
            /* 目录在扫描后出现未知内容时保护当前项，并以 EAGAIN 让维护下一轮
             * 选择其他候选；一个受保护目录不能阻断全卡其余会话的回收。 */
            return -EAGAIN;
        }
        char source[kMaximumLogPathLength]{};
        char destination[kMaximumLogPathLength]{};
        if (!session_path(entry.session_number, source, sizeof(source)) ||
            !delete_path(entry.session_number, destination,
                         sizeof(destination))) {
            return -ENAMETOOLONG;
        }
        (void)cleanup_delete_directory_locked(entry.session_number);
        if (!state_.mounted) {
            return -EIO;
        }
        const FRESULT renamed = f_rename(source, destination);
        if (renamed != FR_OK) {
            if (renamed == FR_EXIST) {
                /* 同号 delNNN 含未知内容时不能覆盖。保护源 sess，下一轮改删
                 * 其他候选；换卡/重启重新扫描后仍可在冲突解除时恢复。 */
                entry.state |= kCatalogProtectedUnknown;
                return -EAGAIN;
            }
            return handle(renamed);
        }
        const int cleaned = cleanup_delete_directory_locked(
            entry.session_number);
        if (cleaned != 0) {
            if (state_.mounted) {
                /* rename 已提交后 sessNNN 已不存在，不能把旧 catalog 项留给
                 * 下一轮重复打开。delNNN 由后续启动扫描继续清理。 */
                remove_catalog_entry_locked(index);
                (void)unlink_if_exists_locked(kLogListPath);
                (void)unlink_if_exists_locked(kLogListTemporaryPath);
                state_.last_space_correction_us = 0U;
            }
            return cleaned;
        }
        remove_catalog_entry_locked(index);
        (void)unlink_if_exists_locked(kLogListPath);
        (void)unlink_if_exists_locked(kLogListTemporaryPath);
        state_.last_space_correction_us = 0U;
        return 0;
    }

    int delete_oldest_session_locked() noexcept
    {
        std::size_t corrupt = state_.log_catalog_count;
        std::size_t oldest = state_.log_catalog_count;
        std::uint64_t corrupt_sequence =
            std::numeric_limits<std::uint64_t>::max();
        std::uint64_t oldest_sequence =
            std::numeric_limits<std::uint64_t>::max();
        for (std::size_t index = 0U; index < state_.log_catalog_count; ++index) {
            const SessionCatalogEntry &entry = state_.log_catalog[index];
            if (entry.session_number == state_.active_session_number ||
                entry.session_number == state_.log_reader_session_number ||
                (entry.state & kCatalogProtectedUnknown) != 0U ||
                (entry.sidecar_flags & log_sidecar::kClosedFlag) == 0U) {
                continue;
            }
            if ((entry.sidecar_flags & log_sidecar::kCorruptFlag) != 0U &&
                entry.sequence < corrupt_sequence) {
                corrupt = index;
                corrupt_sequence = entry.sequence;
            }
            if (entry.sequence < oldest_sequence) {
                oldest = index;
                oldest_sequence = entry.sequence;
            }
        }
        const std::size_t candidate =
            corrupt != state_.log_catalog_count ? corrupt : oldest;
        return candidate == state_.log_catalog_count
                   ? -ENOENT
                   : delete_catalog_entry_locked(candidate);
    }

    bool session_number_in_use_locked(std::uint16_t session) const noexcept
    {
        for (std::size_t index = 0U; index < state_.log_catalog_count; ++index) {
            if (state_.log_catalog[index].session_number == session) {
                return true;
            }
        }
        return false;
    }

    std::size_t logger_session_count_locked() const noexcept
    {
        std::size_t count = 0U;
        for (std::size_t index = 0U; index < state_.log_catalog_count;
             ++index) {
            if ((state_.log_catalog[index].state &
                 kCatalogHasUlogFile) != 0U) {
                ++count;
            }
        }
        return count;
    }

    int read_space_locked(dima::platform::StorageInformation &information,
                          std::uint64_t &cluster_bytes) noexcept
    {
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

        // QGC 容量与 Logger 空间保护共用同一换算：簇字节数=csize*512，
        // 总容量=(n_fatent-2)*簇字节数。只读取/校验，不刷新 Logger 的估算时钟。
        static_assert(_MIN_SS == 512 && _MAX_SS == 512,
                      "SD capacity conversion requires 512-byte sectors");
        constexpr std::uint64_t kSectorBytes = 512U;
        cluster_bytes =
            static_cast<std::uint64_t>(filesystem->csize) * kSectorBytes;
        const std::uint64_t data_clusters =
            static_cast<std::uint64_t>(filesystem->n_fatent - 2U);
        if (static_cast<std::uint64_t>(free_clusters) > data_clusters) {
            return invalidate_with_io_error();
        }

        information.total_bytes = data_clusters * cluster_bytes;
        information.available_bytes =
            static_cast<std::uint64_t>(free_clusters) * cluster_bytes;
        return 0;
    }

    int refresh_space_locked(std::uint64_t now_us) noexcept
    {
        dima::platform::StorageInformation information{};
        std::uint64_t cluster_bytes = 0U;
        const int result = read_space_locked(information, cluster_bytes);
        if (result != 0) {
            return result;
        }
        const std::uint64_t total = information.total_bytes;
        const std::uint64_t five_percent = total / 20U;
        /* 空闲保护线 = clamp(SD容量*5%, 64 MiB, 512 MiB)，单位为 byte。
         * 运行中按簇扣减，60 s 才用 f_getfree 校正，避免高频 FAT 扫描。 */
        state_.cluster_bytes = cluster_bytes;
        state_.total_bytes = total;
        state_.available_bytes_estimate = information.available_bytes;
        state_.free_space_threshold_bytes = std::max(
            kMinimumFreeBytes, std::min(five_percent, kMaximumFreeBytes));
        state_.last_space_correction_us = now_us;
        return total != 0U ? 0 : -ENODEV;
    }

    int service_recovery_crc_locked() noexcept
    {
        if (!state_.recovery_file_open) {
            return -EINVAL;
        }
        UINT read_size = 0U;
        const FRESULT result = f_read(
            &state_.log_recovery, g_log_recovery_buffer,
            static_cast<UINT>(sizeof(g_log_recovery_buffer)), &read_size);
        if (result != FR_OK) {
            const FRESULT closed = f_close(&state_.log_recovery);
            state_.recovery_file_open = false;
            return finish_fatfs_results_locked(result, closed);
        }
        if (read_size != 0U) {
            if (state_.recovery_size >
                std::numeric_limits<std::uint32_t>::max() - read_size) {
                (void)f_close(&state_.log_recovery);
                state_.recovery_file_open = false;
                return -EFBIG;
            }
            state_.recovery_crc_state = log_sidecar::crc32_update(
                state_.recovery_crc_state, g_log_recovery_buffer, read_size);
            state_.recovery_size += read_size;
            return -EAGAIN;
        }
        const FRESULT closed = f_close(&state_.log_recovery);
        state_.recovery_file_open = false;
        if (closed != FR_OK) {
            return handle(closed);
        }
        return finish_catalog_recovery_locked();
    }

    int service_log_maintenance_locked() noexcept
    {
        if (!state_.mounted) {
            return -ENODEV;
        }
        if (state_.operation != Operation::Idle) {
            return -EAGAIN;
        }
        switch (state_.log_maintenance_phase) {
        case LogMaintenancePhase::OpenScan: {
            const FRESULT result = f_opendir(
                &state_.log_root_directory, kLogDirectoryPath);
            if (result != FR_OK) {
                return handle(result);
            }
            state_.log_root_open = true;
            state_.log_maintenance_phase =
                LogMaintenancePhase::ScanDirectories;
            return -EAGAIN;
        }
        case LogMaintenancePhase::ScanDirectories: {
            FILINFO directory{};
            const FRESULT result = f_readdir(
                &state_.log_root_directory, &directory);
            if (result != FR_OK) {
                (void)close_log_directories_locked();
                return handle(result);
            }
            if (directory.fname[0] == '\0') {
                const FRESULT closed = f_closedir(&state_.log_root_directory);
                state_.log_root_open = false;
                if (closed != FR_OK) {
                    return handle(closed);
                }
                state_.log_maintenance_phase =
                    LogMaintenancePhase::PrepareRepair;
                return -EAGAIN;
            }
            if ((directory.fattrib & AM_DIR) == 0U ||
                ascii_equal(directory.fname, ".") ||
                ascii_equal(directory.fname, "..")) {
                return -EAGAIN;
            }
            std::uint16_t number = 0U;
            if (numbered_directory(directory.fname, "del", number)) {
                const int cleaned = cleanup_delete_directory_locked(number);
                return cleaned == 0 ? -EAGAIN : cleaned;
            }
            if (!numbered_directory(directory.fname, "sess", number)) {
                return -EAGAIN;
            }
            if (state_.log_catalog_count >= kMaximumLogSessions) {
                return -EOVERFLOW;
            }
            SessionCatalogEntry &entry =
                state_.log_catalog[state_.log_catalog_count];
            const int inspected = inspect_session_directory_locked(
                number, entry);
            if (inspected != 0) {
                return inspected;
            }
            ++state_.log_catalog_count;
            return -EAGAIN;
        }
        case LogMaintenancePhase::PrepareRepair:
            return prepare_catalog_repair_locked();
        case LogMaintenancePhase::RepairEntries:
            while (state_.maintenance_cursor < state_.log_catalog_count) {
                SessionCatalogEntry &entry =
                    state_.log_catalog[state_.maintenance_cursor];
                if ((entry.state & kCatalogDeleteEmpty) != 0U) {
                    const int deleted = delete_catalog_entry_locked(
                        state_.maintenance_cursor);
                    if (deleted == -EBUSY) {
                        entry.state &= static_cast<std::uint8_t>(
                            ~kCatalogDeleteEmpty);
                        ++state_.maintenance_cursor;
                        return -EAGAIN;
                    }
                    return deleted == 0 ? -EAGAIN : deleted;
                }
                if ((entry.state & kCatalogNeedsRepair) != 0U) {
                    return begin_catalog_recovery_locked(
                        state_.maintenance_cursor);
                }
                ++state_.maintenance_cursor;
            }
            sort_catalog_by_sequence_locked();
            state_.log_maintenance_phase = LogMaintenancePhase::Ready;
            return -EAGAIN;
        case LogMaintenancePhase::RecoveryCrc:
            return service_recovery_crc_locked();
        case LogMaintenancePhase::Ready:
            break;
        }

        if (state_.erase_logs_requested) {
            const int deleted = delete_oldest_session_locked();
            if (deleted == 0) {
                return -EAGAIN;
            }
            if (deleted != -ENOENT && deleted != -EBUSY) {
                return deleted;
            }
            state_.erase_logs_requested = false;
        }
        if (state_.maximum_log_directories != 0U &&
            logger_session_count_locked() >
                state_.maximum_log_directories) {
            const int deleted = delete_oldest_session_locked();
            if (deleted == 0) {
                return -EAGAIN;
            }
            if (deleted != -ENOENT && deleted != -EBUSY) {
                return deleted;
            }
        }

        const std::uint64_t now = hrt_absolute_time();
        if (state_.log_writer_open &&
            (state_.last_space_correction_us == 0U ||
             now < state_.last_space_correction_us ||
             now - state_.last_space_correction_us >=
                 kSpaceCorrectionIntervalUs)) {
            const int refreshed = refresh_space_locked(now);
            if (refreshed != 0) {
                return refreshed;
            }
        }
        if (state_.log_writer_open &&
            state_.available_bytes_estimate <
                state_.free_space_threshold_bytes) {
            /* append_log 最多允许本次分配把空闲量推进到 threshold-1 cluster。
             * 到达保护带后先逐个回收最旧已关闭会话；没有安全候选时才返回
             * ENOSPC，让 LogWriter 停止接收并同步关闭当前有效前缀。 */
            const int deleted = delete_oldest_session_locked();
            if (deleted == 0 || deleted == -EAGAIN) {
                return -EAGAIN;
            }
            if (deleted == -ENOENT || deleted == -EBUSY) {
                return -ENOSPC;
            }
            return deleted;
        }
        return 0;
    }

    int create_log_index_from_catalog_locked(std::uint16_t &count) noexcept
    {
        count = 0U;
        const int synchronized = sync_log_writer_locked();
        if (synchronized != 0) {
            return synchronized;
        }
        if (state_.log_writer_open) {
            state_.active_metadata.file_size = static_cast<std::uint32_t>(
                f_size(&state_.log_writer));
            update_active_catalog_locked();
        }
        int closed = close_log_reader_locked();
        if (closed == 0) {
            closed = close_log_index_locked();
        }
        if (closed != 0) {
            return closed;
        }
        const int stale_removed = unlink_if_exists_locked(
            kLogListTemporaryPath);
        if (stale_removed != 0) {
            return stale_removed;
        }
        FRESULT result = f_open(
            &state_.log_index, kLogListTemporaryPath,
            static_cast<BYTE>(FA_WRITE | FA_CREATE_ALWAYS));
        if (result != FR_OK) {
            return handle(result);
        }
        state_.log_index_open = true;
        sort_catalog_by_sequence_locked();
        for (std::size_t index = 0U; index < state_.log_catalog_count; ++index) {
            const SessionCatalogEntry &catalog = state_.log_catalog[index];
            if ((catalog.state & kCatalogValidUlog) == 0U ||
                (catalog.sidecar_flags & log_sidecar::kCorruptFlag) != 0U) {
                continue;
            }
            LogIndexEntry entry{};
            entry.time_utc = catalog.time_utc;
            entry.size_bytes = catalog.file_size;
            if (!session_file_path(catalog.session_number, kUlogFilename,
                                   entry.filepath, sizeof(entry.filepath))) {
                continue;
            }
            UINT written = 0U;
            result = f_write(&state_.log_index, &entry,
                             static_cast<UINT>(sizeof(entry)), &written);
            if (result != FR_OK || written != sizeof(entry)) {
                return abort_log_list_locked(
                    result, result == FR_OK ? -EIO : 0);
            }
            ++count;
        }
        result = f_sync(&state_.log_index);
        const FRESULT closed_index = f_close(&state_.log_index);
        state_.log_index_open = false;
        const int index_result = finish_fatfs_results_locked(
            result, closed_index);
        if (index_result != 0) {
            return index_result;
        }
        const int removed = unlink_if_exists_locked(kLogListPath);
        if (removed != 0) {
            return removed;
        }
        result = f_rename(kLogListTemporaryPath, kLogListPath);
        return result == FR_OK ? 0 : handle(result);
    }

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
        state_.log_reader_session_number = 0U;
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
        if (state_.recovery_file_open) {
            (void)f_close(&state_.log_recovery);
            state_.recovery_file_open = false;
        }
        if (state_.log_reader_open) {
            (void)f_close(&state_.log_reader);
            state_.log_reader_open = false;
        }
        state_.log_reader_size = 0U;
        state_.log_reader_session_number = 0U;
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
        /* LOG_ERASE 只授权当前介质；拔卡后不能把未完成删除意图带到下一张卡。
         * 记录意图/目录上限仍由 pending context 保留，以便同一 Mode 自动恢复。 */
        state_.erase_logs_requested = false;
        reset_log_maintenance_locked();
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
