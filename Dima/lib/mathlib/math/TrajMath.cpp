#include "TrajMath.hpp"

#include <cmath>


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace math::trajectory {

float computeMaxSpeedFromDistance(const float jerk, const float accel, const float braking_distance,
		const float final_speed)
{
	auto sqr = [](float f) {return f * f;};
	float b =  4.0f * sqr(accel) / jerk;
	float c = - 2.0f * accel * braking_distance - sqr(final_speed);
	float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));

	// don't slow down more than the end speed, even if the conservative accel ramp time requests it
	return fmaxf(max_speed, final_speed);
}

float computeMaxSpeedInWaypoint(const float alpha, const float accel, const float d)
{
	float tan_alpha = tanf(alpha / 2.0f);
	float max_speed_in_turn = sqrtf(accel * d * tan_alpha);

	return max_speed_in_turn;
}

float computeBrakingDistanceFromVelocity(const float velocity, const float jerk, const float accel,
		const float accel_delay_max)
{
	return velocity * (velocity / (2.0f * accel) + accel_delay_max / jerk);
}

float getMaxDistanceToCircle(const matrix::Vector2f &pos, const matrix::Vector2f &circle_pos, float radius,
				    const matrix::Vector2f &direction)
{
	matrix::Vector2f center_to_pos = pos - circle_pos;
	const float b = 2.f * center_to_pos.dot(direction.unit_or_zero());
	const float c = center_to_pos.norm_squared() - radius * radius;
	const float delta = b * b - 4.f * c;

	float distance_to_circle;

	if (delta >= 0.f && direction.longerThan(0.f)) {
		distance_to_circle = fmaxf((-b + sqrtf(delta)) / 2.f, 0.f);

	} else {
		// Never intersecting the circle
		distance_to_circle = NAN;
	}

	return distance_to_circle;
}

} // namespace math::trajectory
