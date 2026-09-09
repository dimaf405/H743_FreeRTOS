#include "DynamicSparseLayer.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

DynamicSparseLayer::DynamicSparseLayer(ParamLayer *parent, int n_prealloc, int n_grow)
: ParamLayer(parent), _n_slots(n_prealloc), _n_grow(n_grow > 0 ? n_grow : 4)
{
    if (_n_slots > 0) {
        if (!allocation_allowed()) {
            _n_slots = 0;
            return;
        }
        auto *slots = static_cast<Slot *>(dima::platform::allocate(sizeof(Slot) * _n_slots,
            dima::platform::AllocationDomain::Service));
        if (slots) { initialize(slots, 0, _n_slots); _slots.store(slots); }
        else { _n_slots = 0; }
    }
}

DynamicSparseLayer::~DynamicSparseLayer()
{ dima::platform::deallocate(_slots.load()); }

bool DynamicSparseLayer::store(param_t param, param_value_u value)
{
    /* lowerBound 定位替换/插入点；新键通过 memmove 保持有序。先 grow 成功再
     * 移动，分配失败时原数组和逻辑内容完全不变。 */
    px4::AtomicTransaction transaction;
    const int index = lowerBound(param);
    Slot *slots = _slots.load();
    if (index < _next_slot && slots[index].param == param) { slots[index].value = value; return true; }
    if (_next_slot >= _n_slots && !grow()) { return false; }
    slots = _slots.load();
    if (index < _next_slot) {
        std::memmove(&slots[index + 1], &slots[index], sizeof(Slot) * (_next_slot - index));
    }
    slots[index] = {param, value}; ++_next_slot; return true;
}

bool DynamicSparseLayer::contains(param_t param) const
{
    px4::AtomicTransaction transaction; const int index = lowerBound(param); Slot *slots = _slots.load();
    return index < _next_slot && slots[index].param == param;
}

px4::AtomicBitset<ParamLayer::PARAM_COUNT> DynamicSparseLayer::containedAsBitset() const
{
    px4::AtomicTransaction transaction; px4::AtomicBitset<PARAM_COUNT> set;
    for (int i = 0; i < _next_slot; ++i) { set.set(_slots.load()[i].param); } return set;
}

param_value_u DynamicSparseLayer::get(param_t param) const
{
    px4::AtomicTransaction transaction; const int index = lowerBound(param); Slot *slots = _slots.load();
    return index < _next_slot && slots[index].param == param ? slots[index].value : _parent->get(param);
}

void DynamicSparseLayer::reset(param_t param)
{
    /* 删除 override 后元素左移，末槽恢复 UINT16_MAX 哨兵；读取会自然回退 parent。 */
    px4::AtomicTransaction transaction; const int index = lowerBound(param); Slot *slots = _slots.load();
    if (index < _next_slot && slots[index].param == param) {
        if (index + 1 < _next_slot) {
            std::memmove(&slots[index], &slots[index + 1], sizeof(Slot) * (_next_slot - index - 1));
        }
        slots[--_next_slot] = {UINT16_MAX, {}};
    }
}

void DynamicSparseLayer::refresh(param_t param)
{ _parent->refresh(param); }

int DynamicSparseLayer::size() const
{ return _next_slot; }

int DynamicSparseLayer::byteSize() const
{ return _n_slots * static_cast<int>(sizeof(Slot)); }

bool DynamicSparseLayer::valid() const noexcept
{ return _slots.load() != nullptr && _n_slots > 0; }

void DynamicSparseLayer::swapContents(DynamicSparseLayer &other) noexcept
{
    /* 仅交换已完整构建层的缓冲和容量，不分配也不逐项复制，用于解码完成后的
     * 原子式代际提交；两层必须具有相同 parent 语义。 */
    Slot *mine = _slots.load();
    Slot *theirs = other._slots.load();
    _slots.store(theirs);
    other._slots.store(mine);
    std::swap(_next_slot, other._next_slot);
    std::swap(_n_slots, other._n_slots);
}

bool DynamicSparseLayer::allocation_allowed()
{ return !dima::platform::in_realtime_context(); }

void DynamicSparseLayer::initialize(Slot *slots, int begin, int end)
{ for (int i = begin; i < end; ++i) { slots[i] = {UINT16_MAX, {}}; } }

int DynamicSparseLayer::lowerBound(param_t param) const
{
    /* 标准半开区间 [left,right) 二分，返回首个 slot.param >= param 的位置。 */
    int left = 0, right = _next_slot; Slot *slots = _slots.load();
    while (left < right) {
        const int mid = left + (right - left) / 2;
        if (slots[mid].param < param) { left = mid + 1; }
        else { right = mid; }
    }
    return left;
}

bool DynamicSparseLayer::grow()
{
    /* 新容量 = old + n_grow；先复制/初始化新块并发布指针，再释放旧块。
     * AtomicTransaction 负责阻止并发读者观察交换窗口。 */
    if (!allocation_allowed()) { return false; }
    const int next_capacity = _n_slots > 0 ? _n_slots + _n_grow : _n_grow;
    auto *next = static_cast<Slot *>(dima::platform::allocate(sizeof(Slot) * next_capacity,
        dima::platform::AllocationDomain::Service));
    if (!next) { return false; }
    if (_next_slot > 0) { std::memcpy(next, _slots.load(), sizeof(Slot) * _next_slot); }
    initialize(next, _next_slot, next_capacity); Slot *previous = _slots.load(); _slots.store(next);
    _n_slots = next_capacity; dima::platform::deallocate(previous); return true;
}
