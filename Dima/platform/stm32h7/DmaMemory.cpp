#include "HardwareServices.hpp"

#include <cstring>

#include "stm32h7xx.h"

namespace dima::platform::stm32h7 {
namespace {

constexpr std::size_t kBounceCount = 4U;
constexpr std::size_t kBounceBytes = 512U;
constexpr std::uintptr_t kTokenSeed = UINT32_C(0xD14AD00D);

extern "C" std::uint8_t __dima_dma_region_start__;
extern "C" std::uint8_t __dima_dma_region_end__;

alignas(32) std::uint8_t g_bounce[kBounceCount][kBounceBytes]
    __attribute__((section(".dima_dma")));
bool g_bounce_in_use[kBounceCount]{};

class Stm32DmaMemory final : public DmaMemory {
public:
    DmaBufferView view(void *buffer, std::size_t length) noexcept override
    {
        const std::uintptr_t begin = reinterpret_cast<std::uintptr_t>(buffer);
        const std::uintptr_t end = begin + length;
        const std::uintptr_t region_begin =
            reinterpret_cast<std::uintptr_t>(&__dima_dma_region_start__);
        const std::uintptr_t region_end =
            reinterpret_cast<std::uintptr_t>(&__dima_dma_region_end__);
        if (buffer == nullptr || length == 0U || end < begin ||
            (begin & 31U) != 0U || begin < region_begin || end > region_end) {
            return {};
        }
        return DmaBufferView{static_cast<std::uint8_t *>(buffer), length,
                             token_for(begin, length)};
    }

    bool valid(const DmaBufferView &buffer) const noexcept override
    {
        if (!buffer) {
            return false;
        }
        const std::uintptr_t begin =
            reinterpret_cast<std::uintptr_t>(buffer.data);
        const std::uintptr_t end = begin + buffer.size;
        const std::uintptr_t region_begin =
            reinterpret_cast<std::uintptr_t>(&__dima_dma_region_start__);
        const std::uintptr_t region_end =
            reinterpret_cast<std::uintptr_t>(&__dima_dma_region_end__);
        return end >= begin && (begin & 31U) == 0U &&
               begin >= region_begin && end <= region_end &&
               buffer.token == token_for(begin, buffer.size);
    }

    DmaBufferView acquire_bounce(const void *source, std::size_t length,
                                 DmaDirection direction) noexcept override
    {
        if (length == 0U || length > kBounceBytes ||
            ((direction == DmaDirection::MemoryToPeripheral ||
              direction == DmaDirection::Bidirectional) &&
             source == nullptr)) {
            return {};
        }

        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        std::size_t index = kBounceCount;
        for (std::size_t candidate = 0U; candidate < kBounceCount;
             ++candidate) {
            if (!g_bounce_in_use[candidate]) {
                g_bounce_in_use[candidate] = true;
                index = candidate;
                break;
            }
        }
        if (primask == 0U) {
            __enable_irq();
        }
        if (index == kBounceCount) {
            return {};
        }

        if (source != nullptr &&
            direction != DmaDirection::PeripheralToMemory) {
            std::memcpy(g_bounce[index], source, length);
        }
        return view(g_bounce[index], length);
    }

    void release_bounce(DmaBufferView &buffer, void *destination,
                        DmaDirection direction) noexcept override
    {
        if (!valid(buffer)) {
            buffer = {};
            return;
        }
        std::size_t index = kBounceCount;
        for (std::size_t candidate = 0U; candidate < kBounceCount;
             ++candidate) {
            if (buffer.data == g_bounce[candidate]) {
                index = candidate;
                break;
            }
        }
        if (index == kBounceCount) {
            buffer = {};
            return;
        }
        if (destination != nullptr &&
            direction != DmaDirection::MemoryToPeripheral) {
            std::memcpy(destination, buffer.data, buffer.size);
        }

        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        g_bounce_in_use[index] = false;
        if (primask == 0U) {
            __enable_irq();
        }
        buffer = {};
    }

private:
    static std::uintptr_t token_for(std::uintptr_t address,
                                    std::size_t length) noexcept
    {
        return kTokenSeed ^ address ^ static_cast<std::uintptr_t>(length);
    }
};

} // namespace

DmaMemory &dma_memory() noexcept
{
    static Stm32DmaMemory instance;
    return instance;
}

} // namespace dima::platform::stm32h7
