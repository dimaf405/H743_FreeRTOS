/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "ParamLayer.h"
#include "containers/atomic.h"
#include "platform/api/Execution.hpp"
#include "platform/api/Memory.hpp"

#include <cstring>
#include <utility>

// Upstream path: src/lib/parameters/DynamicSparseLayer.h @ d6f12ad1
class DynamicSparseLayer : public ParamLayer
{
public:
    DynamicSparseLayer(ParamLayer *parent, int n_prealloc = 32, int n_grow = 4)
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
    ~DynamicSparseLayer() { dima::platform::deallocate(_slots.load()); }

    bool store(param_t param, param_value_u value) override
    {
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
    bool contains(param_t param) const override
    {
        px4::AtomicTransaction transaction; const int index = lowerBound(param); Slot *slots = _slots.load();
        return index < _next_slot && slots[index].param == param;
    }
    px4::AtomicBitset<PARAM_COUNT> containedAsBitset() const override
    {
        px4::AtomicTransaction transaction; px4::AtomicBitset<PARAM_COUNT> set;
        for (int i = 0; i < _next_slot; ++i) { set.set(_slots.load()[i].param); } return set;
    }
    param_value_u get(param_t param) const override
    {
        px4::AtomicTransaction transaction; const int index = lowerBound(param); Slot *slots = _slots.load();
        return index < _next_slot && slots[index].param == param ? slots[index].value : _parent->get(param);
    }
    void reset(param_t param) override
    {
        px4::AtomicTransaction transaction; const int index = lowerBound(param); Slot *slots = _slots.load();
        if (index < _next_slot && slots[index].param == param) {
            if (index + 1 < _next_slot) {
                std::memmove(&slots[index], &slots[index + 1], sizeof(Slot) * (_next_slot - index - 1));
            }
            slots[--_next_slot] = {UINT16_MAX, {}};
        }
    }
    void refresh(param_t param) override { _parent->refresh(param); }
    int size() const override { return _next_slot; }
    int byteSize() const override { return _n_slots * static_cast<int>(sizeof(Slot)); }
    bool valid() const noexcept { return _slots.load() != nullptr && _n_slots > 0; }

    void swapContents(DynamicSparseLayer &other) noexcept
    {
        Slot *mine = _slots.load();
        Slot *theirs = other._slots.load();
        _slots.store(theirs);
        other._slots.store(mine);
        std::swap(_next_slot, other._next_slot);
        std::swap(_n_slots, other._n_slots);
    }

private:
    struct Slot { param_t param; param_value_u value; };
    static bool allocation_allowed() { return !dima::platform::in_realtime_context(); }
    static void initialize(Slot *slots, int begin, int end) { for (int i = begin; i < end; ++i) { slots[i] = {UINT16_MAX, {}}; } }
    int lowerBound(param_t param) const
    {
        int left = 0, right = _next_slot; Slot *slots = _slots.load();
        while (left < right) {
            const int mid = left + (right - left) / 2;
            if (slots[mid].param < param) { left = mid + 1; }
            else { right = mid; }
        }
        return left;
    }
    bool grow()
    {
        if (!allocation_allowed()) { return false; }
        const int next_capacity = _n_slots > 0 ? _n_slots + _n_grow : _n_grow;
        auto *next = static_cast<Slot *>(dima::platform::allocate(sizeof(Slot) * next_capacity,
            dima::platform::AllocationDomain::Service));
        if (!next) { return false; }
        if (_next_slot > 0) { std::memcpy(next, _slots.load(), sizeof(Slot) * _next_slot); }
        initialize(next, _next_slot, next_capacity); Slot *previous = _slots.load(); _slots.store(next);
        _n_slots = next_capacity; dima::platform::deallocate(previous); return true;
    }
    int _next_slot{0}; int _n_slots{0}; const int _n_grow; px4::atomic<Slot *> _slots{nullptr};
};
