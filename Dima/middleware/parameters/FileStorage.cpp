#define MODULE_NAME "fstore"
#include "FileStorage.hpp"

#include "logging/logging.hpp"
#include "api/AtomicFileStore.hpp"
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

enum class LoadedSource : std::uint8_t {
    Unknown,
    None,
    Primary,
    Backup,
    Temporary,
};

constexpr std::size_t kAtomicFileDomainCount = 2U;

std::size_t domain_index(platform::AtomicFileDomain domain) noexcept
{
    return domain == platform::AtomicFileDomain::Mission ? 1U : 0U;
}

/* 保存状态机：tmp 分块写入 -> sync/close -> 逐字节回读验证 -> 删除旧 backup ->
 * primary 改名为 backup -> tmp 改名为 primary。只有新文件已验证后才移动现役
 * primary；最后一步失败且 primary 已移动时回滚 backup，保证至少保留一代。 */

struct FileStorageContext {
    platform::AtomicFileStore *store{nullptr};
    platform::Synchronization *synchronization{nullptr};
    platform::MutexHandle mutex{};
    const std::uint8_t *save_data{nullptr};
    std::size_t save_size{0U};
    int save_error{0};
    SavePhase save_phase{SavePhase::Idle};
    platform::AtomicFileDomain save_domain{
        platform::AtomicFileDomain::Parameters};
    LoadedSource loaded_source[kAtomicFileDomainCount]{};
    bool primary_moved{false};
    bool available{false};
};

FileStorageContext ctx;

void invalidate_loaded_sources(FileStorageContext &context) noexcept
{
    // 介质会话失效后，原先记录的 primary/backup/tmp 角色不再可信；重新挂载的
    // 甚至可能是另一张卡。后续 load 会重新发现来源，未重新 load 的域则按未知
    // 代际处理，不能拿旧路径结论删除新介质上的文件。
    for (LoadedSource &source : context.loaded_source) {
        source = LoadedSource::Unknown;
    }
}

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
    const bool was_available = context.available;
    const int result = context.store->initialize();
    context.available = result == 0;
    if (result != 0 || !was_available) {
        invalidate_loaded_sources(context);
    }
    return result;
}

void reset_save(FileStorageContext &context) noexcept
{
    context.save_data = nullptr;
    context.save_size = 0U;
    context.save_error = 0;
    context.save_phase = SavePhase::Idle;
    context.save_domain = platform::AtomicFileDomain::Parameters;
    context.primary_moved = false;
}

bool media_failure(int error) noexcept
{
    return error == -ENODEV || error == -EIO || error == -ENXIO ||
           error == -EBADF || error == -ETIMEDOUT;
}

void mark_media_unavailable(FileStorageContext &context) noexcept
{
    context.available = false;
    invalidate_loaded_sources(context);
}

int promote_loaded_source_locked(
    FileStorageContext &context,
    platform::AtomicFileDomain domain) noexcept
{
    LoadedSource &source = context.loaded_source[domain_index(domain)];
    platform::AtomicFile recovered_file{};
    if (source == LoadedSource::Backup) {
        recovered_file = platform::AtomicFile::Backup;
    } else if (source == LoadedSource::Temporary) {
        recovered_file = platform::AtomicFile::Temporary;
    } else {
        // Primary/None 可直接进入正常三文件事务；Unknown
        // 已在 begin_save 的前置门被拒绝，不会到达这里。
        return 0;
    }

    // backup/tmp 已成为当前 RAM 快照的唯一已验证代时，必须先把它安全提升为
    // primary，再允许新 tmp 覆写和 backup 轮换。删除的是已经验证失败的旧
    // primary；掉电发生在 erase 与 rename 之间时，恢复源仍保持完整。
    int result = context.store->erase(
        domain, platform::AtomicFile::Primary);
    if (result != 0 && result != -ENOENT) {
        if (media_failure(result)) {
            mark_media_unavailable(context);
        }
        return result;
    }

    result = context.store->rename(
        domain, recovered_file, platform::AtomicFile::Primary);
    if (result == 0) {
        source = LoadedSource::Primary;
    } else if (media_failure(result)) {
        mark_media_unavailable(context);
    }
    return result;
}

int fail_save(FileStorageContext &context, int error,
              bool cleanup_temporary) noexcept
{
    context.store->cancel_operation();
    context.save_error = error;
    // 保存链任一介质级错误都立即撤销 available；上层只可在重新 initialize 后
    // 开始下一次事务，不能把旧挂载对象继续当作可写介质。
    if (media_failure(error)) {
        mark_media_unavailable(context);
    }
    if (cleanup_temporary) {
        context.save_phase = SavePhase::CleanupTemporary;
        return -EAGAIN;
    }
    reset_save(context);
    return error;
}

int read_and_validate(FileStorageContext &context,
                      platform::AtomicFileDomain domain,
                      platform::AtomicFile file,
                      std::uint8_t *destination, std::size_t capacity,
                      std::size_t &output_size,
                      FileStorageValidator validator,
                      void *validator_context) noexcept
{
    const int result = context.store->read(
        domain, file, destination, capacity, output_size);
    if (result != 0) {
        // 读取层的介质错误会使当前 FatFs 会话失效；立即清除
        // 所有域的来源结论，不能继续读 backup/tmp 并误标为 None。
        if (media_failure(result)) {
            mark_media_unavailable(context);
        }
        return result;
    }
    return validator == nullptr
               ? 0
               : validator(destination, output_size, validator_context);
}

} // namespace

int file_storage_initialize(platform::AtomicFileStore &store,
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

int file_storage_begin_save(platform::AtomicFileDomain domain,
                            const std::uint8_t *data,
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

    if (ctx.loaded_source[domain_index(domain)] == LoadedSource::Unknown) {
        // 热插拔或任一文件域的介质故障会重建共享 FatFs 会话。
        // 此时必须由所有者先重读 primary/backup/tmp 并运行本域
        // validator；否则轮换可能删掉新介质上唯一有效的上一代。
        return -ESTALE;
    }

    const int promoted = promote_loaded_source_locked(ctx, domain);
    if (promoted != 0) {
        // 提升失败发生在新 temporary 打开之前，因而当前 backup/tmp 恢复代仍
        // 保持不变；上层收到错误后继续使用冻结 RAM 快照并拒绝最终提交 ACK。
        return promoted;
    }

    /* 这里只借用序列化缓冲；直到 continue_save 返回最终 0/错误或 cancel 前，
     * 调用方必须保持 data 内容和生命周期稳定。 */
    ctx.save_data = data;
    ctx.save_size = size;
    ctx.save_error = 0;
    ctx.save_domain = domain;
    ctx.primary_moved = false;
    ctx.save_phase = SavePhase::BeginTemporaryWrite;
    return 0;
}

int file_storage_continue_save(platform::AtomicFileDomain domain) noexcept
{
    FileStorageGuard lock{ctx};
    if (!lock) {
        return -EDEADLK;
    }
    if (ctx.save_phase == SavePhase::Idle || ctx.store == nullptr) {
        return -EINVAL;
    }
    if (ctx.save_domain != domain) {
        // begin/continue 必须由同一文件域推进；否则另一服务可能误提交当前
        // 事务借用的 RAM 缓冲，必须返回 busy 而不是接管 FatFs owner。
        return -EBUSY;
    }

    /* 每次调用只推进一个可阻塞边界并以 -EAGAIN 请求再次调度，避免一次 Run 内
     * 完成整文件 I/O 而饿死看门狗、MAVLink 和控制服务。 */
    int result = 0;
    switch (ctx.save_phase) {
    case SavePhase::BeginTemporaryWrite:
        result = ctx.store->begin_write(ctx.save_domain,
                                        platform::AtomicFile::Temporary,
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
        result = ctx.store->begin_verify(ctx.save_domain,
                                         platform::AtomicFile::Temporary,
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
        result = ctx.store->erase(ctx.save_domain,
                                  platform::AtomicFile::Backup);
        if (result != 0 && result != -ENOENT) {
            return fail_save(ctx, result, true);
        }
        ctx.save_phase = SavePhase::MovePrimaryToBackup;
        return -EAGAIN;

    case SavePhase::MovePrimaryToBackup:
        result = ctx.store->rename(ctx.save_domain,
                                   platform::AtomicFile::Primary,
                                   platform::AtomicFile::Backup);
        if (result == 0) {
            ctx.primary_moved = true;
            ctx.loaded_source[domain_index(ctx.save_domain)] =
                LoadedSource::Backup;
        } else if (result != -ENOENT) {
            return fail_save(ctx, result, true);
        } else {
            // 首次保存没有上一代 primary；此时唯一已验证候选是新 tmp。
            ctx.loaded_source[domain_index(ctx.save_domain)] =
                LoadedSource::None;
        }
        ctx.save_phase = SavePhase::MoveTemporaryToPrimary;
        return -EAGAIN;

    case SavePhase::MoveTemporaryToPrimary:
        result = ctx.store->rename(ctx.save_domain,
                                   platform::AtomicFile::Temporary,
                                   platform::AtomicFile::Primary);
        if (result == 0) {
            ctx.loaded_source[domain_index(ctx.save_domain)] =
                LoadedSource::Primary;
            reset_save(ctx);
            return 0;
        }
        /* 只有确实把 primary 移到 backup 后才执行回滚；首次保存没有 primary 时
         * 不可把不存在的 backup 当成可恢复来源。 */
        if (ctx.primary_moved) {
            ctx.save_error = result;
            if (media_failure(result)) {
                mark_media_unavailable(ctx);
            }
            ctx.save_phase = SavePhase::RollbackPrimary;
            return -EAGAIN;
        }
        return fail_save(ctx, result, false);

    case SavePhase::RollbackPrimary: {
        const int rollback = ctx.store->rename(
            ctx.save_domain, platform::AtomicFile::Backup,
            platform::AtomicFile::Primary);
        // 保留原始提交错误作为返回值；但回滚也遭遇介质错误时，
        // 必须同时撤销 available，防止旧挂载被继续使用。
        if (media_failure(rollback)) {
            mark_media_unavailable(ctx);
        } else if (rollback == 0) {
            ctx.loaded_source[domain_index(ctx.save_domain)] =
                LoadedSource::Primary;
        }
        result = ctx.save_error;
        reset_save(ctx);
        return result;
    }

    case SavePhase::CleanupTemporary: {
        const int cleanup =
            ctx.store->erase(ctx.save_domain,
                             platform::AtomicFile::Temporary);
        if (media_failure(cleanup)) {
            mark_media_unavailable(ctx);
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

void file_storage_cancel_save(platform::AtomicFileDomain domain) noexcept
{
    FileStorageGuard lock{ctx};
    if (!lock || ctx.save_phase == SavePhase::Idle || ctx.store == nullptr ||
        ctx.save_domain != domain) {
        return;
    }
    ctx.store->cancel_operation();
    reset_save(ctx);
}

int file_storage_load(platform::AtomicFileDomain domain,
                      std::uint8_t *destination, std::size_t capacity,
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
        ctx, domain, platform::AtomicFile::Primary, destination, capacity,
        output_size, validator, validator_context);
    if (primary_result == 0) {
        ctx.loaded_source[domain_index(domain)] = LoadedSource::Primary;
        return 0;
    }
    if (!ctx.available) {
        return primary_result;
    }
    const int backup_result = read_and_validate(
        ctx, domain, platform::AtomicFile::Backup, destination, capacity,
        output_size, validator, validator_context);
    if (backup_result == 0) {
        ctx.loaded_source[domain_index(domain)] = LoadedSource::Backup;
        PX4_WARN("file_storage: recovered backup file");
        return 0;
    }
    if (!ctx.available) {
        return backup_result;
    }
    const int temporary_result = read_and_validate(
        ctx, domain, platform::AtomicFile::Temporary, destination, capacity,
        output_size, validator, validator_context);
    if (temporary_result == 0) {
        ctx.loaded_source[domain_index(domain)] = LoadedSource::Temporary;
        PX4_WARN("file_storage: recovered temporary file");
        return 0;
    }
    if (!ctx.available) {
        return temporary_result;
    }
    ctx.loaded_source[domain_index(domain)] =
        ctx.available ? LoadedSource::None : LoadedSource::Unknown;
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
