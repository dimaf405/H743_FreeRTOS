#include "height_bias_estimator.hpp"

// 明确公共类型来源，不依赖调用方头文件中的 using 指令。
using estimator::HeightSensor;


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

HeightBiasEstimator::HeightBiasEstimator(HeightSensor sensor, const HeightSensor &sensor_ref)
:
		BiasEstimator(0.f, 0.f),
		_sensor(sensor),
		_sensor_ref(sensor_ref)
{}

void HeightBiasEstimator::setFusionActive()
{ _is_sensor_fusion_active = true; }

void HeightBiasEstimator::setFusionInactive()
{ _is_sensor_fusion_active = false; }

void HeightBiasEstimator::predict(float dt)
{
	if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
		BiasEstimator::predict(dt);
	}
}

void HeightBiasEstimator::fuseBias(float bias, float bias_var)
{
	if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
		BiasEstimator::fuseBias(bias, bias_var);
	}
}
