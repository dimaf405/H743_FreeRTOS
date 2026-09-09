#include "Functions.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace math {

int signFromBool(bool positive)
{
	return positive ? 1 : -1;
}

bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

bool isFinite(const matrix::Vector2f &value)
{
	return value.isAllFinite();
}

bool isFinite(const matrix::Vector3f &value)
{
	return value.isAllFinite();
}

} // namespace math
