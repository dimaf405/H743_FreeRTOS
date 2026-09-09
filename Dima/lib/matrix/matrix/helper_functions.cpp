#include "helper_functions.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace matrix {

float wrap(float x, float low, float high)
{
	return matrix::detail::wrap_floating(x, low, high);
}

double wrap(double x, double low, double high)
{
	return matrix::detail::wrap_floating(x, low, high);
}

} // namespace matrix
