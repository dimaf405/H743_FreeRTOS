#include "App/runtime/scheduling/scheduled_work_item.hpp"

namespace app::runtime::scheduling {

bool ScheduledWorkItem::ScheduleNow()
{
    return queue_.schedule_now(*this);
}

bool ScheduledWorkItem::ScheduleOnInterval(uint32_t interval_ms)
{
    return queue_.schedule_interval(*this, interval_ms);
}

void ScheduledWorkItem::ScheduleClear()
{
    queue_.clear(*this);
}

} // namespace app::runtime::scheduling
