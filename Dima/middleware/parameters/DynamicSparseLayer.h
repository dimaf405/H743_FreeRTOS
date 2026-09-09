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
#include "api/Execution.hpp"
#include "api/Memory.hpp"

#include <cstring>
#include <utility>

// Upstream path: src/lib/parameters/DynamicSparseLayer.h @ d6f12ad1
/* 稀疏层只保存“与 parent 不同”的参数，槽按 param_t 升序排列。当前层未命中时
 * get 递归读取 parent，形成 firmware defaults -> runtime defaults -> user config
 * 三层覆盖；容量增长只允许在非实时任务中发生。 */
class DynamicSparseLayer : public ParamLayer
{
public:
    DynamicSparseLayer(ParamLayer *parent, int n_prealloc = 32, int n_grow = 4);
    ~DynamicSparseLayer();

    bool store(param_t param, param_value_u value) override;
    bool contains(param_t param) const override;
    px4::AtomicBitset<PARAM_COUNT> containedAsBitset() const override;
    param_value_u get(param_t param) const override;
    void reset(param_t param) override;
    void refresh(param_t param) override;
    int size() const override;
    int byteSize() const override;
    bool valid() const noexcept;

    void swapContents(DynamicSparseLayer &other) noexcept;

private:
    struct Slot { param_t param; param_value_u value; };
    static bool allocation_allowed();
    static void initialize(Slot *slots, int begin, int end);
    int lowerBound(param_t param) const;
    bool grow();
    int _next_slot{0}; int _n_slots{0}; const int _n_grow; px4::atomic<Slot *> _slots{nullptr};
};
