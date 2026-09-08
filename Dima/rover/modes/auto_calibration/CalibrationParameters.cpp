#include "CalibrationParameters.hpp"
#include "api/Time.hpp"

#include <cerrno>
#include <cmath>
#include <limits>

namespace dima::rover::modes {

float CalibrationParameters::original_float(dima::params parameter) const noexcept
{
    const auto handle = param_handle(parameter);
    for (std::size_t i = 0U; i < count_; ++i)
        if (entries_[i].handle == handle && entries_[i].type == PARAM_TYPE_FLOAT) return entries_[i].old.f;
    return std::numeric_limits<float>::quiet_NaN();
}

bool CalibrationParameters::active() const noexcept
{
    return held_ || phase_ == Phase::Applying || phase_ == Phase::Provisional || phase_ == Phase::Rollback || phase_ == Phase::Saving || phase_ == Phase::Fault;
}

bool CalibrationParameters::prepare() noexcept
{
    if (active()) return false;
    count_ = 0U;
    phase_ = Phase::Idle;
    rollback_ = false;
    cancel_pending_ = false;
    provisional_ = false;
    generation_valid_ = false;
    return true;
}

bool CalibrationParameters::add(dima::params parameter, param_value_u value,
                                 param_type_t type) noexcept
{
    const param_t handle = param_handle(parameter);
    if (phase_ != Phase::Idle || count_ >= sizeof(entries_) / sizeof(entries_[0]) || param_type(handle) != type) return false;
    for (std::size_t i = 0U; i < count_; ++i) if (entries_[i].handle == handle) return false;
    Entry candidate{};
    candidate.handle = handle;
    candidate.type = type;
    candidate.next = value;
    if (param_get(handle, &candidate.old) != 0) return false;
    entries_[count_++] = candidate;
    return true;
}

bool CalibrationParameters::add_float(dima::params parameter, float value) noexcept
{
    if (!std::isfinite(value)) return false;
    param_value_u next{};
    next.f = value;
    return add(parameter, next, PARAM_TYPE_FLOAT);
}

bool CalibrationParameters::add_int(dima::params parameter, std::int32_t value) noexcept
{
    param_value_u next{};
    next.i = value;
    return add(parameter, next, PARAM_TYPE_INT32);
}

bool CalibrationParameters::matches(bool original) const noexcept
{
    px4::AtomicTransaction transaction;
    for (std::size_t i = 0U; i < count_; ++i) {
        param_value_u current{};
        const auto &entry = entries_[i];
        const auto expected = original ? entry.old : entry.next;
        if (param_get(entry.handle, &current) != 0 ||
            (entry.type == PARAM_TYPE_FLOAT ? current.f != expected.f : current.i != expected.i)) return false;
    }
    return count_ != 0U;
}

bool CalibrationParameters::write(bool original) noexcept
{
    bool success = true;
    for (std::size_t i = 0U; i < count_; ++i) {
        const auto &entry = entries_[i];
        const auto value = original ? entry.old : entry.next;
        success = (param_set_no_notification(entry.handle, &value) == 0) && success;
    }
    return success;
}

void CalibrationParameters::notify(std::uint64_t now) noexcept
{
    (void)update_.update();
    applied_at_ = hrt_absolute_time();
    set_count_ = param_set_count();
    generation_valid_ = false;
    deadline_ = now + 20000000ULL;
    param_notify_changes();
}

bool CalibrationParameters::apply_candidate(std::uint64_t now, bool provisional) noexcept
{
    // 调用者已持有 maintenance 与参数原子锁，并完成原值/代次核对。
    // 三个入口共用整组写入和失败回滚；双重失败保留 Fault 与互锁，不发布半套参数。
    if (!write(false)) {
        rollback_ = true;
        if (!write(true)) { phase_ = Phase::Fault; return false; }
    }
    phase_ = rollback_ ? Phase::Rollback : Phase::Applying;
    provisional_ = provisional;
    notify(now);
    return !rollback_;
}

bool CalibrationParameters::apply(std::uint64_t now, std::uint32_t expected_set_count, bool provisional) noexcept
{
    if (phase_ != Phase::Idle || count_ == 0U || !armed_.begin_maintenance()) return false;
    held_ = true;
    if (!param_storage_pause(this)) { release(); return false; }
    storage_paused_ = true;
    px4::AtomicTransaction transaction;
    // 开始采样后用户可能改过这组参数；只有原值仍一致才覆盖，避免吞掉并发编辑。
    if (param_set_count() != expected_set_count || !matches(true)) { release(); return false; }
    return apply_candidate(now, provisional);
}

bool CalibrationParameters::refine(std::uint64_t now, std::uint32_t expected_set_count,
                                    const float (&offsets)[3]) noexcept
{
    // 本产品磁事务固定为 ID+三个 float offset；更新 provisional 候选时保留
    // 最初 old snapshot，避免最终失败只能回到 bootstrap 而回不到原磁参数。
    if (phase_ != Phase::Provisional || count_ != 4U || entries_[0].type != PARAM_TYPE_INT32) return false;
    for (unsigned i = 0U; i < 3U; ++i)
        if (entries_[i + 1U].type != PARAM_TYPE_FLOAT || !std::isfinite(offsets[i])) return false;
    if (!armed_.begin_maintenance()) return false;
    held_ = true;
    px4::AtomicTransaction atomic;
    if (param_set_count() != expected_set_count || !matches(false)) { phase_ = Phase::Fault; return false; }
    for (unsigned i = 0U; i < 3U; ++i) entries_[i + 1U].next.f = offsets[i];
    provisional_ = false;
    return apply_candidate(now, provisional_);
}

bool CalibrationParameters::replace_provisional(std::uint64_t now, std::uint32_t expected_set_count,
                                                const float *values, std::size_t count) noexcept
{
    // 控制增益组是固定容量 float 组；只能替换原组的候选，不能改变成员、顺序
    // 或最初 old 快照。路径增益比较多个候选也始终能回到进入该组前的值。
    if (phase_ != Phase::Provisional || values == nullptr || count != count_ || count == 0U) return false;
    for (std::size_t i = 0U; i < count; ++i)
        if (entries_[i].type != PARAM_TYPE_FLOAT || !std::isfinite(values[i])) return false;
    if (!armed_.begin_maintenance()) return false;
    held_ = true;
    px4::AtomicTransaction atomic;
    if (!storage_paused_ || param_set_count() != expected_set_count || !matches(false)) {
        phase_ = Phase::Fault;
        return false;
    }
    for (std::size_t i = 0U; i < count; ++i) entries_[i].next.f = values[i];
    return apply_candidate(now, provisional_);
}

bool CalibrationParameters::finalize_provisional(std::uint64_t now, std::uint32_t expected_set_count) noexcept
{
    // 闭环验证只授权当前候选代。最终保存仍须停车/Disarm、参数值/代次匹配，
    // 再经 poll 重新确认消费者；不能因为统计通过就在 Armed 中恢复 autosave。
    if (phase_ != Phase::Provisional || !generation_valid_ || !armed_.begin_maintenance()) return false;
    held_ = true;
    px4::AtomicTransaction atomic;
    if (!storage_paused_ || param_set_count() != expected_set_count || !matches(false)) {
        phase_ = Phase::Fault;
        return false;
    }
    provisional_ = false;
    phase_ = Phase::Applying;
    deadline_ = now + 20000000ULL;
    return true;
}

bool CalibrationParameters::revise_float(dima::params parameter, float value) noexcept
{
    if (phase_ != Phase::Provisional || rollback_ || !std::isfinite(value)) return false;
    const auto handle = param_handle(parameter);
    for (std::size_t i = 0U; i < count_; ++i) {
        if (entries_[i].handle == handle && entries_[i].type == PARAM_TYPE_FLOAT) {
            // 先只改独立的候选槽，不改变用于所有权核对的 next，也不发参数通知。
            entries_[i].revised = value; entries_[i].revision_pending = true;
            return true;
        }
    }
    return false; // 不能在运动中扩展事务成员或覆盖组外参数。
}

bool CalibrationParameters::apply_revisions(std::uint64_t now, std::uint32_t expected_set_count) noexcept
{
    if (phase_ != Phase::Provisional || rollback_ || !armed_.begin_maintenance()) return false;
    held_ = true;
    px4::AtomicTransaction atomic;
    if (!storage_paused_ || param_set_count() != expected_set_count || !matches(false)) {
        phase_ = Phase::Fault; return false;
    }
    for (std::size_t i = 0U; i < count_; ++i) {
        if (entries_[i].revision_pending) entries_[i].next.f = entries_[i].revised;
        entries_[i].revision_pending = false;
    }
    // 所有关联字段共享一次写入/代次，原始 old 始终不变；不按数组序号手工
    // 拼一份第二参数列表，调用方只用生成标识指定本次候选的变化。
    return apply_candidate(now, true);
}

void CalibrationParameters::release() noexcept
{
    if (storage_paused_) {
        if (!param_storage_resume(this)) { phase_ = Phase::Fault; return; }
        storage_paused_ = false;
    }
    if (held_) { armed_.end_maintenance(); held_ = false; }
}

void CalibrationParameters::cancel(std::uint64_t now) noexcept
{
    if (rollback_ || phase_ == Phase::Fault || phase_ == Phase::Idle || phase_ == Phase::Failed) return;
    cancel_pending_ = true;
    if (!held_) {
        if (!armed_.begin_maintenance()) return;
        held_ = true;
    }
    if (!storage_paused_) {
        if (!param_storage_pause(this)) { phase_ = Phase::Fault; return; }
        storage_paused_ = true;
    }
    px4::AtomicTransaction transaction;
    // 不覆盖事务外改写的值；发生所有权冲突时保持禁 Arm，由操作者处理明确故障。
    if (!matches(false)) { phase_ = Phase::Fault; return; }
    rollback_ = true;
    if (!write(true)) { phase_ = Phase::Fault; return; }
    phase_ = Phase::Rollback;
    notify(now);
}

void CalibrationParameters::poll(bool frontend_confirmed, bool validated,
                                  std::uint64_t now) noexcept
{
    if (phase_ == Phase::Fault || !active()) return;
    if (cancel_pending_ && !rollback_ && !held_) cancel(now);
    if (!held_ && phase_ != Phase::Saving) return;
    if (update_.update() && update_.get().timestamp >= applied_at_) {
        // 本轮 frontend_confirmed 来自调用前的代次；收到新代次后至少等下一轮
        // 重新核对，避免用旧确认批准新参数事件。
        if (!generation_valid_ || generation_ != update_.get().instance) frontend_confirmed = false;
        generation_ = update_.get().instance;
        generation_valid_ = true;
    }
    if (phase_ == Phase::Applying || phase_ == Phase::Rollback) {
        if (!rollback_ && param_set_count() != set_count_) { cancel(now); return; }
        if (!matches(rollback_)) { phase_ = Phase::Fault; return; }
        if (generation_valid_ && frontend_confirmed && (rollback_ || validated)) {
            if (provisional_ && !rollback_) {
                // 已确认候选只进入 RAM provisional 状态：释放阶段运动互锁，
                // 但继续暂停所有物理保存。磁/IMU/关联整定共用这个提交边界，
                // 断电只能加载此前完整持久化的参数，不加载未验证的临时组。
                armed_.end_maintenance();
                held_ = false;
                phase_ = Phase::Provisional;
                return;
            }
            if (!param_storage_resume(this)) { phase_ = Phase::Fault; return; }
            storage_paused_ = false;
            // ParameterService 保存本身需要非重入 maintenance。此处把 lease
            // 转交给它；Commander 仍由 active 且非 awaiting_arm 禁止 Arm。
            armed_.end_maintenance();
            held_ = false;
            phase_ = Phase::Saving;
            deadline_ = now + 20000000ULL;
        } else if (now > deadline_) {
            if (rollback_) phase_ = Phase::Fault;
            else cancel(now);
        }
        return;
    }
    if (phase_ != Phase::Saving) return;
    if (!rollback_ && param_set_count() != set_count_) {
        cancel(now);
        // 异步保存尚持有 maintenance 时先继续推进它，取得释放后的 lease
        // 再回滚；不能依赖另一个 autosave worker 替本事务解除等待。
        if (phase_ != Phase::Saving) return;
    }
    // 复用现有异步存储推进，不在控制循环执行 Flash/SD 等待；尚未保存的
    // 阶段不开放下一次 Arm，防止“已应用”被误当成“掉电后仍可恢复”。
    const int result = param_save_default(false);
    if (result == 0 && matches(rollback_)) {
        if (cancel_pending_ && !rollback_) {
            phase_ = Phase::Applying;
            cancel(now);
            return;
        }
        phase_ = rollback_ ? Phase::Failed : Phase::Done;
        release();
    } else if (now > deadline_ ||
               (result != -EAGAIN && result != -EBUSY && result != -ESTALE && result != -EPERM)) {
        if (rollback_) phase_ = Phase::Fault;
        else cancel(now);
    }
}

float CalibrationParameters::expected_float(dima::params parameter) const noexcept
{
    for (std::size_t i = 0U; i < count_; ++i) {
        const auto &entry = entries_[i];
        if (entry.handle == param_handle(parameter) && entry.type == PARAM_TYPE_FLOAT)
            return rollback_ ? entry.old.f : entry.next.f;
    }
    return std::numeric_limits<float>::quiet_NaN();
}

std::int32_t CalibrationParameters::expected_int(dima::params parameter) const noexcept
{
    for (std::size_t i = 0U; i < count_; ++i) {
        const auto &entry = entries_[i];
        if (entry.handle == param_handle(parameter) && entry.type == PARAM_TYPE_INT32)
            return rollback_ ? entry.old.i : entry.next.i;
    }
    return 0;
}

} // namespace dima::rover::modes
