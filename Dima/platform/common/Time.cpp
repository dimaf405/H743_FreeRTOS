#include "api/Time.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

hrt_abstime hrt_absolute_time() noexcept
{
    return dima::platform::platform_time_us();
}

hrt_abstime hrt_elapsed_time(const hrt_abstime *then) noexcept
{
    return then == nullptr ? 0U : hrt_absolute_time() - *then;
}

std::uint64_t hrt_absolute_time_ms() noexcept
{
    return dima::platform::platform_time_ms();
}
