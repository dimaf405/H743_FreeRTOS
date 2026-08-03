#include "ArmingFlashInterlock.h"

#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace {

enum class InterlockState : std::uint8_t {
    Idle,
    Armed,
    FlashBusy,
};

struct ArmingFlashInterlock {
    InterlockState state{InterlockState::Idle};
};

ArmingFlashInterlock g_interlock{};

} // namespace

extern "C" bool dima_arming_flash_try_arm(void)
{
    taskENTER_CRITICAL();
    const bool accepted = g_interlock.state != InterlockState::FlashBusy;
    if (accepted) {
        g_interlock.state = InterlockState::Armed;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

extern "C" void dima_arming_flash_disarm(void)
{
    taskENTER_CRITICAL();
    if (g_interlock.state == InterlockState::Armed) {
        g_interlock.state = InterlockState::Idle;
    }
    taskEXIT_CRITICAL();
}

extern "C" int dima_arming_flash_begin(void)
{
    taskENTER_CRITICAL();
    const bool accepted = g_interlock.state == InterlockState::Idle;
    if (accepted) {
        g_interlock.state = InterlockState::FlashBusy;
    }
    taskEXIT_CRITICAL();
    return accepted ? DIMA_FLASH_BEGIN_ACQUIRED : DIMA_FLASH_BEGIN_DEFERRED;
}

extern "C" void dima_arming_flash_end(void)
{
    taskENTER_CRITICAL();
    if (g_interlock.state == InterlockState::FlashBusy) {
        g_interlock.state = InterlockState::Idle;
    }
    taskEXIT_CRITICAL();
}

extern "C" bool dima_arming_flash_is_armed(void)
{
    taskENTER_CRITICAL();
    const bool armed = g_interlock.state == InterlockState::Armed;
    taskEXIT_CRITICAL();
    return armed;
}

extern "C" bool dima_arming_flash_is_busy(void)
{
    taskENTER_CRITICAL();
    const bool busy = g_interlock.state == InterlockState::FlashBusy;
    taskEXIT_CRITICAL();
    return busy;
}
