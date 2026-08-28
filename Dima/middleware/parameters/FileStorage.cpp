#define MODULE_NAME "fstore"
#include "FileStorage.hpp"

#include "logging/logging.hpp"
#include "api/ParameterFileStore.hpp"
#include "api/Synchronization.hpp"

#include <cerrno>

namespace dima {
namespace {

enum class SavePhase : std::uint8_t {
    Idle,
    BeginTemporaryWrite,
    ContinueTemporaryWrite,
    BeginTemporaryVerify,
    ContinueTemporaryVerify,
    EraseBackup,
    MovePrimaryToBackup,
    MoveTemporaryToPrimary,
    RollbackPrimary,
    CleanupTemporary,
};

/* 保存状态机：tmp 分块写入 -> sync/close -> 逐字节回读验证 -> 删除旧 backup ->
 * primary 改名为 backup -> tmp 改名为 primary。只有新文件已验证后才移动现役
 * primary；最后一步失败且 primary 已移动时回滚 backup，保证至少保留一代。 */

struct FileStorageContext {
    platform::ParameterFileStore *store{nullptr};
    platform::Synchronization *synchronization{nullptr};
    platform::MutexHandle mutex{};
    const std::uint8_t *save_data{nullptr};
    std::size_t save_size{0U};
    int save_error{0};
    SavePhase save_phase{SavePhase::Idle};
    bool primary_moved{false};
    bool available{false};
};

FileStorageContext ctx;

class FileStorageGuard final {
public:
    explicit FileStorageGuard(FileStorageContext &context) noexcept
        : context_(context),
          /* 文件状态机及 FATFS 后端为单事务资源，使用普通 mutex 串行化 poll、
           * save 与 load；RAII 覆盖全部 errno 返回路径。 */
          locked_(context.synchronization != nullptr && context.mutex &&
                  context.synchronization->lock(
                      context.mutex, platform::Timeout::forever()))
    {
    }

    ~FileStorageGuard()
    {
        if (locked_) {
            context_.synchronization->unlock(context_.mutex);
        }
    }

    explicit operator bool() const noexcept { return locked_; }
    FileStorageGuard(const FileStorageGuard &) = delete;
    FileStorageGuard &operator=(const FileStorageGuard &) = delete;

private:
    FileStorageContext &context_;
    bool locked_{false};
};

int refresh_media_locked(FileStorageContext &context) noexcept
{
    if (context.store == nullptr) {
        context.available = false;
        return -ENODEV;
    }
    const int result = context.store->initialize();
    context.available = result == 0;
    return result;
}

void reset_save(FileStorageContext &context) noexcept
{
    context.save_data = nullptr;
    context.save_size = 0U;
    context.save_error = 0;
    context.save_phase = SavePhase::Idle;
    context.primary_moved = false;
}

bool media_failure(int error) noexcept
{
    return error == -ENODEV || error == -EIO || error == -ENXIO ||
           error == -EBADF || error == -ETIMEDOUT;
}

int fail_save(FileStorageContext &context, int error,
              bool cleanup_temporary) noexcept
{
    context.store->cancel_operation();
    context.save_error = error;
    // 保存链任一介质级错误都立即撤销 available；上层只可在重新 initialize 后
    // 开始下一次事务，不能把旧挂载对象继续当作可写介质。
    if (media_failure(error)) {
        context.available = false;
    }
    if (cleanup_temporary) {
        context.save_phase = SavePhase::CleanupTemporary;
        return -EAGAIN;
    }
    reset_save(context);
    return error;
}

int read_and_validate(FileStorageContext &context,
                      platform::ParameterFile file,
                      std::uint8_t *destination, std::size_t capacity,
                      std::size_t &output_size,
                      FileStorageValidator validator,
                      void *validator_context) noexcept
{
    const int result = context.store->read(
        file, destination, capacity, output_size);
    if (result != 0) {
        return result;
    }
    return validator == nullptr
               ? 0
               : validator(destination, output_size, validator_context);
}

} // namespace

int file_storage_initialize(platform::ParameterFileStore &store,
                            platform::Synchronization &synchronization,
                            bool &available) noexcept
{
    if (ctx.store != nullptr) {
        if (ctx.store != &store || ctx.synchronization != &synchronization) {
            return -EBUSY;
        }
        const int result = file_storage_poll(available);
        return result == -EDEADLK ? result : 0;
    }
    if (ctx.synchronization != nullptr || ctx.mutex) {
        return -EBUSY;
    }

    const platform::MutexHandle mutex =
        synchronization.create_mutex(platform::MutexKind::Normal);
    if (!mutex) {
        return -ENOMEM;
    }

    ctx.store = &store;
    ctx.synchronization = &synchronization;
    ctx.mutex = mutex;
    const int media_result = refresh_media_locked(ctx);
    available = ctx.available;
    if (media_result != 0) {
        PX4_WARN("file_storage: SD unavailable: %d", media_result);
    }
    return 0;
}

int file_storage_poll(bool &available) noexcept
{
    FileStorageGuard lock{ctx};
    if (!lock) {
        return -EDEADLK;
    }
    if (ctx.save_phase != SavePhase::Idle) {
        available = ctx.available;
        return -EBUSY;
    }
    const int result = refresh_media_locked(ctx);
    available = ctx.available;
    return result;
}

int file_storage_begin_save(const std::uint8_t *data,
                            std::size_t size) noexcept
{
    if (data == nullptr || size == 0U) {
        return -EINVAL;
    }
    FileStorageGuard lock{ctx};
    if (!lock) {
        return -EDEADLK;
    }
    if (ctx.save_phase != SavePhase::Idle) {
        return -EBUSY;
    }
    const int ready = refresh_media_locked(ctx);
    if (ready != 0) {
        return ready;
    }

    /* 这里只借用序列化缓冲；直到 continue_save 返回最终 0/错误或 cancel 前，
     * 调用方必须保持 data 内容和生命周期稳定。 */
    ctx.save_data = data;
    ctx.save_size = size;
    ctx.save_error = 0;
    ctx.primary_moved = false;
    ctx.save_phase = SavePhase::BeginTemporaryWrite;
    return 0;
}

int file_storage_continue_save() noexcept
{
    FileStorageGuard lock{ctx};
    if (!lock) {
        return -EDEADLK;
    }
    if (ctx.save_phase == SavePhase::Idle || ctx.store == nullptr) {
        return -EINVAL;
    }

    /* 每次调用只推进一个可阻塞边界并以 -EAGAIN 请求再次调度，避免一次 Run 内
     * 完成整文件 I/O 而饿死看门狗、MAVLink 和控制服务。 */
    int result = 0;
    switch (ctx.save_phase) {
    case SavePhase::BeginTemporaryWrite:
        result = ctx.store->begin_write(platform::ParameterFile::Temporary,
                                        ctx.save_data, ctx.save_size);
        if (result != 0) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::ContinueTemporaryWrite;
        return -EAGAIN;

    case SavePhase::ContinueTemporaryWrite:
        result = ctx.store->continue_write();
        if (result == -EAGAIN) {
            return result;
        }
        if (result != 0) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::BeginTemporaryVerify;
        return -EAGAIN;

    case SavePhase::BeginTemporaryVerify:
        result = ctx.store->begin_verify(platform::ParameterFile::Temporary,
                                         ctx.save_data, ctx.save_size);
        if (result != 0) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::ContinueTemporaryVerify;
        return -EAGAIN;

    case SavePhase::ContinueTemporaryVerify:
        result = ctx.store->continue_verify();
        if (result == -EAGAIN) {
            return result;
        }
        if (result != 0) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::EraseBackup;
        return -EAGAIN;

    case SavePhase::EraseBackup:
        result = ctx.store->erase(platform::ParameterFile::Backup);
        if (result != 0 && result != -ENOENT) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::MovePrimaryToBackup;
        return -EAGAIN;

    case SavePhase::MovePrimaryToBackup:
        result = ctx.store->rename(platform::ParameterFile::Primary,
                                   platform::ParameterFile::Backup);
        if (result == 0) {
            ctx.primary_moved = true;
        } else if (result != -ENOENT) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::MoveTemporaryToPrimary;
        return -EAGAIN;

    case SavePhase::MoveTemporaryToPrimary:
        result = ctx.store->rename(platform::ParameterFile::Temporary,
                                   platform::ParameterFile::Primary);
        if (result == 0) {
            reset_save(ctx);
            return 0;
        }
        /* 只有确实把 primary 移到 backup 后才执行回滚；首次保存没有 primary 时
         * 不可把不存在的 backup 当成可恢复来源。 */
        if (ctx.primary_moved) {
            ctx.save_error = result;
            if (media_failure(result)) {
                ctx.available = false;
            }
            ctx.save_phase = SavePhase::RollbackPrimary;
            return -EAGAIN;
        }
        return fail_save(ctx, result, false);

    case SavePhase::RollbackPrimary: {
        const int rollback = ctx.store->rename(
            platform::ParameterFile::Backup,
            platform::ParameterFile::Primary);
        // 保留原始提交错误作为返回值；但回滚也遭遇介质错误时，
        // 必须同时撤销 available，防止旧挂载被继续使用。
        if (media_failure(rollback)) {
            ctx.available = false;
        }
        result = ctx.save_error;
        reset_save(ctx);
        return result;
    }

    case SavePhase::CleanupTemporary: {
        const int cleanup =
            ctx.store->erase(platform::ParameterFile::Temporary);
        if (media_failure(cleanup)) {
            ctx.available = false;
        }
        result = ctx.save_error;
        reset_save(ctx);
        return result;
    }

    case SavePhase::Idle:
    default:
        return -EINVAL;
    }
}

void file_storage_cancel_save() noexcept
{
    FileStorageGuard lock{ctx};
    if (!lock || ctx.save_phase == SavePhase::Idle || ctx.store == nullptr) {
        return;
    }
    ctx.store->cancel_operation();
    reset_save(ctx);
}

int file_storage_load(std::uint8_t *destination, std::size_t capacity,
                      std::size_t &output_size,
                      FileStorageValidator validator,
                      void *validator_context) noexcept
{
    if (destination == nullptr || capacity == 0U) {
        return -EINVAL;
    }
    FileStorageGuard lock{ctx};
    if (!lock) {
        return -EDEADLK;
    }
    if (ctx.save_phase != SavePhase::Idle) {
        return -EBUSY;
    }
    const int ready = refresh_media_locked(ctx);
    if (ready != 0) {
        return ready;
    }

    /* 恢复优先级固定为 primary -> backup -> tmp；每个候选都必须同时通过文件读取
     * 和上层快照 validator，存在但 CRC/格式错误的 primary 不阻断后备恢复。 */
    const int primary_result = read_and_validate(
        ctx, platform::ParameterFile::Primary, destination, capacity,
        output_size, validator, validator_context);
    if (primary_result == 0) {
        return 0;
    }
    const int backup_result = read_and_validate(
        ctx, platform::ParameterFile::Backup, destination, capacity,
        output_size, validator, validator_context);
    if (backup_result == 0) {
        PX4_WARN("file_storage: recovered backup file");
        return 0;
    }
    const int temporary_result = read_and_validate(
        ctx, platform::ParameterFile::Temporary, destination, capacity,
        output_size, validator, validator_context);
    if (temporary_result == 0) {
        PX4_WARN("file_storage: recovered temporary file");
        return 0;
    }
    if (primary_result == -ENOENT && backup_result == -ENOENT &&
        temporary_result == -ENOENT) {
        return -ENOENT;
    }
    return primary_result != -ENOENT
               ? primary_result
               : (backup_result != -ENOENT ? backup_result
                                            : temporary_result);
}


} // namespace dima
