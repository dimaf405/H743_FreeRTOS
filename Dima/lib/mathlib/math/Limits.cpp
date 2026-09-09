#include "Limits.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace math {

bool isZero(float val)
{
	return fabsf(val - 0.0f) < FLT_EPSILON;
}

bool isZero(double val)
{
	return fabs(val - 0.0) < DBL_EPSILON;
}

} // namespace math
