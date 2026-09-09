#include "ConstLayer.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

bool ConstLayer::store(param_t param, param_value_u value)
{
	(void)param; (void)value;
	return false;
}

bool ConstLayer::contains(param_t param) const
{
	return param < PARAM_COUNT;
}

px4::AtomicBitset<ParamLayer::PARAM_COUNT> ConstLayer::containedAsBitset() const
{
	px4::AtomicBitset<PARAM_COUNT> set;

	for (int i = 0; i < PARAM_COUNT; i++) {
		set.set(i);
	}

	return set;
}

param_value_u ConstLayer::get(param_t param) const
{
	if (param >= PARAM_COUNT) {
		return {0};
	}

	return dima::parameter_catalog::parameters[param].val;
}

void ConstLayer::reset(param_t param)
{
	(void)param;
	// Do nothing
}

void ConstLayer::refresh(param_t param)
{
	(void)param;
	// Do nothing
}

int ConstLayer::size() const
{
	return PARAM_COUNT;
}

int ConstLayer::byteSize() const
{
	return PARAM_COUNT * sizeof(param_info_s);
}
