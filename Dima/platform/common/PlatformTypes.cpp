#include "api/PlatformTypes.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::platform {

void IsrCallback::invoke() const noexcept
{
    if (function != nullptr) {
        function(context);
    }
}

} // namespace dima::platform
