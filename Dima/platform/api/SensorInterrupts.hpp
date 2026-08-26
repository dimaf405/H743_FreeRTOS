#pragma once

#include "PlatformTypes.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::platform {

enum InterruptSourceMask : std::uint32_t {
    InterruptSourceNone = 0U,
    InterruptSource1 = 1U << 0U,
    InterruptSource2 = 1U << 1U,
};

inline constexpr std::size_t kInterruptSourceCount{2U};

struct InterruptSourceSnapshot {
    /* pending_mask 表示本批次至少发生过一次的源；count 是累计边沿数，timestamp_us
     * 是各源最近一次 ISR 到达时间，用于驱动层检测丢中断与计算采样间隔。 */
    std::uint32_t pending_mask{0U};
    std::uint32_t count[kInterruptSourceCount]{};
    std::uint64_t timestamp_us[kInterruptSourceCount]{};
};

class InterruptSources {
public:
    virtual ~InterruptSources() = default;
    virtual bool register_sources(IsrCallback notification) noexcept = 0;
    virtual void unregister_sources() noexcept = 0;
    /* 原子取得 ISR 快照；具体后端负责定义并实现 pending 的消费语义。 */
    virtual InterruptSourceSnapshot consume() noexcept = 0;
};

} // namespace dima::platform
