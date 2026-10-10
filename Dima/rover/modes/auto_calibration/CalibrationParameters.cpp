#include "CalibrationParameters.hpp"
#include "api/Time.hpp"
#include <parameters/parameter_contract.hpp>

#include <cerrno>
#include <cmath>
#include <limits>

namespace dima::rover::modes {

bool CalibrationParameters::active() const noexcept
{
    return held_ || phase_ == Phase::Applying || phase_ == Phase::Provisional || phase_ == Phase::Rollback || phase_ == Phase::Fault;
}

void CalibrationParameters::fault() noexcept
{
    // 故障终态只锁存所有权，不释放未确认的存储/运动保护。
    phase_ = Phase::Fault;
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

bool CalibrationParameters::begin_session() noexcept
{
    // 从 Level 之前到最终保存持续持有同一把存储暂停锁；组间只释放运动维护锁。
    if (session_active_ || active() || !param_storage_pause(this)) return false;
    session_count_ = 0U;
    session_active_ = true;
    session_committed_ = false;
    return true;
}

bool CalibrationParameters::capture(dima::params parameter) noexcept
{
    if (!session_active_) return false;
    const auto handle = param_handle(parameter);
    for (std::size_t i = 0U; i < session_count_; ++i)
        if (session_entries_[i].handle == handle) return true;
    if (session_count_ >= sizeof(session_entries_) / sizeof(session_entries_[0])) return false;
    Snapshot entry{};
    entry.handle = handle;
    entry.type = param_type(handle);
    if (param_get(handle, &entry.old) != 0) return false;
    session_entries_[session_count_++] = entry;
    // Level 等外部 worker 会使关联观测身份失效；依赖快照仍取正式生成合同。
    if (dima::generated::parameters::invalidates_observations(parameter))
        for (const auto observation : dima::generated::parameters::kFirmwareObservationParameters)
            if (!capture(observation)) return false;
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
    if (session_active_) {
        bool already = false;
        for (std::size_t i = 0U; i < session_count_; ++i)
            already = already || session_entries_[i].handle == handle;
        if (!already) {
            if (session_count_ >= sizeof(session_entries_) / sizeof(session_entries_[0])) return false;
            session_entries_[session_count_++] = candidate;
        }
    }
    if (dima::generated::parameters::invalidates_observations(parameter)) {
        // 源校正改变会原子失效观测 ID；连同 K/GEN 捕获最初快照，避免磁
        // bootstrap/refine/回滚丢失原代。列表与容量来自正式生成合同。
        for (const auto observation : dima::generated::parameters::kFirmwareObservationParameters) {
            bool present = false;
            for (std::size_t i = 0U; i < count_; ++i)
                present = present || entries_[i].handle == param_handle(observation);
            if (present) continue;
            param_value_u next{};
            const auto observation_handle = param_handle(observation);
            if (param_get(observation_handle, &next) != 0) return false;
            for (const auto &edge : dima::generated::parameters::kParameterInvalidations)
                if (edge.target == observation) next.i = 0;
            if (!add(observation, next, param_type(observation_handle))) return false;
        }
    }
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
    // 先写源校正，后写观测组；否则回滚恢复的 ID 会被随后旧校正写入再失效。
    for (unsigned pass = 0U; pass < 2U; ++pass) {
        for (std::size_t i = 0U; i < count_; ++i) {
            const auto &entry = entries_[i];
            const bool observation = dima::generated::parameters::firmware_observation(
                static_cast<dima::params>(entry.handle));
            if (observation != (pass == 1U)) continue;
            const auto value = original ? entry.old : entry.next;
            success = (param_set_no_notification(entry.handle, &value) == 0) && success;
        }
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

bool CalibrationParameters::session_save(std::uint32_t &expected_set_count, bool commit) noexcept
{
    if (!session_active_ || active() || param_set_count() != expected_set_count) {
        phase_ = Phase::Fault;
        return false;
    }
    // 全会话唯一一次保存。失败不冒充成功。EAGAIN 由参数模块定义为"本次
    // 未写入、待恢复后重试"——含满区（ENOSPC）且 SD 镜像门未过的场景：
    // RAM 候选与 unsaved 由参数模块保留，ParameterService 在解锁且 SD 可
    // 用时自主执行 镜像→擦除→重建 恢复，随后本次保存重试即成功。其余
    // 错误才恢复 RAM 并锁 Fault。
    const int result = param_storage_save(this);
    if (result == -EAGAIN) return false; // 未写入（含满区待恢复）；FINALIZE 继续等待。
    if (result != 0) {
        (void)session_rollback(expected_set_count);
        phase_ = Phase::Fault;
        return false;
    }
    if (!param_storage_resume(this)) { phase_ = Phase::Fault; return false; }
    session_committed_ = commit;
    session_active_ = false;
    return true;
}

bool CalibrationParameters::session_rollback(std::uint32_t &expected_set_count) noexcept
{
    if (!session_active_ || session_committed_ || active()) return false;
    px4::AtomicTransaction transaction;
    if (param_set_count() != expected_set_count) { phase_ = Phase::Fault; return false; }
    if (session_count_ == 0U) return true; // 恢复完成后仅等待保存，不重复写回同一快照。
    const auto before = expected_set_count;
    // 只恢复 RAM：先恢复源校正，再恢复观测身份；最终保存仍只有 session_save 一处。
    for (unsigned pass = 0U; pass < 2U; ++pass) {
        for (std::size_t i = 0U; i < session_count_; ++i) {
            const auto &entry = session_entries_[i];
            if (dima::generated::parameters::firmware_observation(static_cast<dima::params>(entry.handle)) != (pass == 1U)) continue;
            if (param_set_no_notification(entry.handle, &entry.old) != 0) { phase_ = Phase::Fault; return false; }
        }
    }
    // 全部原值恢复成功才消耗日志；部分失败仍保留原日志并锁存Fault。
    session_count_ = 0U;
    expected_set_count = set_count_ = param_set_count();
    if (expected_set_count != before) param_notify_changes();
    return true;
}

bool CalibrationParameters::session_active() const noexcept
{ return session_active_; }

bool CalibrationParameters::session_committed() const noexcept
{ return session_committed_; }

bool CalibrationParameters::apply(std::uint64_t now, std::uint32_t expected_set_count, bool provisional) noexcept
{
    if (!session_active_ || phase_ != Phase::Idle || count_ == 0U || !armed_.begin_maintenance(true)) return false;
    held_ = true;
    px4::AtomicTransaction transaction;
    // 开始采样后用户可能改过这组参数；只有原值仍一致才覆盖，避免吞掉并发编辑。
    if (param_set_count() != expected_set_count || !matches(true)) { release(); return false; }
    return apply_candidate(now, provisional);
}

bool CalibrationParameters::finalize_provisional(std::uint64_t now, std::uint32_t expected_set_count) noexcept
{
    // 闭环验证只授权当前候选代。最终保存须确认物理停波、参数值/代次匹配，
    // 再经 poll 确认消费者；保持 Armed 不代表可以带着有效电机输出写 Flash。
    if (phase_ != Phase::Provisional || !generation_valid_ || !armed_.begin_maintenance(true)) return false;
    held_ = true;
    px4::AtomicTransaction atomic;
    if (!session_active_ || param_set_count() != expected_set_count || !matches(false)) {
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

bool CalibrationParameters::apply_revisions(std::uint64_t now, std::uint32_t expected_set_count, bool provisional) noexcept
{
    if (phase_ != Phase::Provisional || rollback_ || !armed_.begin_maintenance(true)) return false;
    held_ = true;
    px4::AtomicTransaction atomic;
    if (!session_active_ || param_set_count() != expected_set_count || !matches(false)) {
        phase_ = Phase::Fault; return false;
    }
    for (std::size_t i = 0U; i < count_; ++i) {
        if (entries_[i].revision_pending) entries_[i].next.f = entries_[i].revised;
        entries_[i].revision_pending = false;
    }
    // 所有关联字段共享一次写入/代次，原始 old 始终不变；不按数组序号手工
    // 拼一份第二参数列表，调用方只用生成标识指定本次候选的变化。
    // 磁refine确认后直接Done，关联增益仍Provisional；写入/确认/回滚只有这一条链。
    return apply_candidate(now, provisional);
}

void CalibrationParameters::release() noexcept
{
    if (held_) { armed_.end_maintenance(); held_ = false; }
}

void CalibrationParameters::cancel(std::uint64_t now) noexcept
{
    if (rollback_ || phase_ == Phase::Fault || phase_ == Phase::Idle || phase_ == Phase::Failed) return;
    cancel_pending_ = true;
    if (!held_) {
        if (!armed_.begin_maintenance(true)) return;
        held_ = true;
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
    if (!held_) return;
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
            // 阶段收尾仅释放维护锁；会话的存储暂停锁持续到 FINALIZE。
            // 回滚确认是候选失败，不能按 Done 让调用方登记新候选成功。
            phase_ = rollback_ ? Phase::Failed : Phase::Done;
            release();
            return;
        } else if (now > deadline_) {
            if (rollback_) phase_ = Phase::Fault;
            else cancel(now);
        }
        return;
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


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::rover::modes {

CalibrationParameters::CalibrationParameters(dima::platform::ArmedFlashCoordinator &armed) noexcept
: armed_(armed)
{}

CalibrationParameters::Phase CalibrationParameters::phase() const noexcept
{ return phase_; }

bool CalibrationParameters::rolling_back() const noexcept
{ return rollback_; }

bool CalibrationParameters::generation_valid() const noexcept
{ return generation_valid_; }

std::uint32_t CalibrationParameters::generation() const noexcept
{ return generation_; }

std::uint64_t CalibrationParameters::applied_at() const noexcept
{ return applied_at_; }

std::uint32_t CalibrationParameters::set_count_snapshot() const noexcept
{ return set_count_; }

} // namespace dima::rover::modes
