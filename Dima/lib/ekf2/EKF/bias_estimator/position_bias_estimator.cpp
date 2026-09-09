#include "position_bias_estimator.hpp"

// 明确公共类型来源，不依赖调用方头文件中的 using 指令。
using estimator::PositionSensor;
using matrix::Vector2f;


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

PositionBiasEstimator::PositionBiasEstimator(PositionSensor sensor, const PositionSensor &sensor_ref)
:
		_sensor(sensor),
		_sensor_ref(sensor_ref)
{}

bool PositionBiasEstimator::fusionActive() const
{ return _is_sensor_fusion_active; }

void PositionBiasEstimator::setFusionActive()
{ _is_sensor_fusion_active = true; }

void PositionBiasEstimator::setFusionInactive()
{ _is_sensor_fusion_active = false; }

void PositionBiasEstimator::predict(float dt)
{
	if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
		_bias[0].predict(dt);
		_bias[1].predict(dt);
	}
}

void PositionBiasEstimator::fuseBias(Vector2f bias, Vector2f bias_var)
{
	if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
		_bias[0].fuseBias(bias(0), bias_var(0));
		_bias[1].fuseBias(bias(1), bias_var(1));
	}
}

void PositionBiasEstimator::setBias(const Vector2f &bias)
{
	_bias[0].setBias(bias(0));
	_bias[1].setBias(bias(1));
}

void PositionBiasEstimator::setProcessNoiseSpectralDensity(float nsd)
{
	_bias[0].setProcessNoiseSpectralDensity(nsd);
	_bias[1].setProcessNoiseSpectralDensity(nsd);
}

void PositionBiasEstimator::setMaxStateNoise(const Vector2f &max_noise)
{
	_bias[0].setMaxStateNoise(max_noise(0));
	_bias[1].setMaxStateNoise(max_noise(1));
}

Vector2f PositionBiasEstimator::getBias() const
{ return Vector2f(_bias[0].getBias(), _bias[1].getBias()); }

float PositionBiasEstimator::getBias(int index) const
{ return _bias[index].getBias(); }

Vector2f PositionBiasEstimator::getBiasVar() const
{ return Vector2f(_bias[0].getBiasVar(), _bias[1].getBiasVar()); }

float PositionBiasEstimator::getBiasVar(int index) const
{ return _bias[index].getBiasVar(); }

const BiasEstimator::status & PositionBiasEstimator::getStatus(int index) const
{ return _bias[index].getStatus(); }

void PositionBiasEstimator::reset()
{
	_bias[0].reset();
	_bias[1].reset();
}
