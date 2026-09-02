/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "PurePursuit.hpp"

#include <cfloat>
#include <cmath>

namespace dima::lib::rover {
namespace {

constexpr float kPi = 3.14159265358979323846F;

struct Vector2f {
    float north;
    float east;
};

float clamp(float value, float lower, float upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

float wrap_pi(float angle) noexcept
{
    // fmod 先把任意有限角度压到一个 2*pi 周期，再平移到 [-pi, pi)。
    // 这里不使用循环，避免异常大输入导致不可界定的运行时间。
    float wrapped = std::fmod(angle + kPi, 2.0F * kPi);
    if (wrapped < 0.0F) {
        wrapped += 2.0F * kPi;
    }
    return wrapped - kPi;
}

Vector2f subtract(const Position2f &to, const Position2f &from) noexcept
{
    return {to.north_m - from.north_m, to.east_m - from.east_m};
}

Vector2f add(Vector2f lhs, Vector2f rhs) noexcept
{
    return {lhs.north + rhs.north, lhs.east + rhs.east};
}

Vector2f scale(Vector2f value, float factor) noexcept
{
    return {value.north * factor, value.east * factor};
}

float dot(Vector2f lhs, Vector2f rhs) noexcept
{
    return lhs.north * rhs.north + lhs.east * rhs.east;
}

float norm(Vector2f value) noexcept
{
    return std::hypot(value.north, value.east);
}

Vector2f unit_or_zero(Vector2f value) noexcept
{
    const float length = norm(value);
    return length > FLT_EPSILON ? scale(value, 1.0F / length)
                                : Vector2f{};
}

float bearing(Vector2f value) noexcept
{
    return wrap_pi(std::atan2(value.east, value.north));
}

} // namespace

bool PurePursuit::finite(float value) noexcept
{
    return std::isfinite(value);
}

bool PurePursuit::finite(const Position2f &value) noexcept
{
    return finite(value.north_m) && finite(value.east_m);
}

bool PurePursuit::valid_config(const PurePursuitConfig &config) noexcept
{
    return finite(config.lookahead_gain) && config.lookahead_gain > 0.0F &&
           finite(config.lookahead_min_m) &&
           config.lookahead_min_m > 0.0F &&
           finite(config.lookahead_max_m) &&
           config.lookahead_max_m >= config.lookahead_min_m;
}

bool PurePursuit::configure(const PurePursuitConfig &config) noexcept
{
    configured_ = valid_config(config);
    if (configured_) {
        config_ = config;
    } else {
        config_ = {};
    }
    return configured_;
}

PurePursuitOutput PurePursuit::update(
    const Position2f &previous_waypoint_ne_m,
    const Position2f &target_waypoint_ne_m,
    const Position2f &vehicle_position_ne_m,
    float ground_speed_m_s) const noexcept
{
    if (!configured_ || !finite(previous_waypoint_ne_m) ||
        !finite(target_waypoint_ne_m) || !finite(vehicle_position_ne_m) ||
        !finite(ground_speed_m_s)) {
        return {};
    }

    // PX4 Pure Pursuit 的前视距离公式：L=clamp(k*|v|, Lmin, Lmax)。
    // 速度只影响观察距离，不改变 NED 坐标或航向符号。
    const float lookahead_distance = clamp(
        config_.lookahead_gain * std::fabs(ground_speed_m_s),
        config_.lookahead_min_m, config_.lookahead_max_m);
    const Vector2f vehicle_to_target =
        subtract(target_waypoint_ne_m, vehicle_position_ne_m);
    const Vector2f previous_to_target =
        subtract(target_waypoint_ne_m, previous_waypoint_ne_m);
    const Vector2f previous_to_vehicle =
        subtract(vehicle_position_ne_m, previous_waypoint_ne_m);
    const float path_length = norm(previous_to_target);
    const Vector2f path_unit = unit_or_zero(previous_to_target);
    const float progress_along_path = dot(previous_to_vehicle, path_unit);

    // 将车辆相对起点位置投影到路径方向，再反向得到车辆到路径的最短向量。
    // 叉积符号约定与 NED 北/东轴一致，左、右横向误差保持稳定符号。
    const Vector2f position_along_path =
        scale(path_unit, dot(previous_to_vehicle, path_unit));
    const Vector2f vehicle_to_path = {
        position_along_path.north - previous_to_vehicle.north,
        position_along_path.east - previous_to_vehicle.east};
    const float cross = previous_to_target.east * vehicle_to_path.north -
                        previous_to_target.north * vehicle_to_path.east;
    // 与 PX4 matrix::sign 保持一致：叉积为 0 时符号也为 0。尤其前后航点
    // 重合时不能把“车辆到重合点距离”误报成带正号的横向路径误差。
    const float cross_sign = cross > 0.0F
                                 ? 1.0F
                                 : (cross < 0.0F ? -1.0F : 0.0F);
    const float crosstrack_error =
        cross_sign * norm(vehicle_to_path);
    const float distance_to_waypoint = norm(vehicle_to_target);
    const float bearing_to_waypoint = bearing(vehicle_to_target);
    float target_bearing = bearing_to_waypoint;

    if (distance_to_waypoint < lookahead_distance ||
        path_length < FLT_EPSILON) {
        // 已进入前视圆或航点重合时直接指向目标，避免零长度路径单位向量。
        target_bearing = bearing_to_waypoint;
    } else if (progress_along_path >= path_length) {
        // 车辆投影已越过有限航段终点时，前视圆仍会与“无限延长线”
        // 在车头前方相交。若继续使用该交点，车辆会背离航点行驶；因此必须
        // 回到有限路径的真实终点。
        target_bearing = bearing_to_waypoint;
    } else if (std::fabs(crosstrack_error) > lookahead_distance) {
        // 路径不与前视圆相交：选择有限线段上的最近点；投影落在端点外时
        // 指向对应端点，不能沿无限延长线继续行驶。
        const Vector2f previous_to_closest =
            add(vehicle_to_path, previous_to_vehicle);
        const Vector2f target_to_closest = {
            vehicle_to_path.north - vehicle_to_target.north,
            vehicle_to_path.east - vehicle_to_target.east};
        if (dot(previous_to_closest, previous_to_target) < FLT_EPSILON) {
            target_bearing = bearing(
                {-previous_to_vehicle.north, -previous_to_vehicle.east});
        } else if (dot(target_to_closest, previous_to_target) >
                   FLT_EPSILON) {
            target_bearing = bearing_to_waypoint;
        } else {
            target_bearing = bearing(vehicle_to_path);
        }
    } else {
        // 常规 Pure Pursuit：路径到车辆的垂足与前视圆交点构成直角三角形，
        // sqrt(L^2-e_ct^2) 是沿路径向前的距离。max 防止浮点舍入产生负根号。
        const float radicand = std::fmax(
            0.0F, lookahead_distance * lookahead_distance -
                      norm(vehicle_to_path) * norm(vehicle_to_path));
        const float line_extension = std::sqrt(radicand);
        const Vector2f previous_to_intersection =
            add(position_along_path, scale(path_unit, line_extension));
        const Vector2f vehicle_to_intersection = {
            previous_to_intersection.north - previous_to_vehicle.north,
            previous_to_intersection.east - previous_to_vehicle.east};
        target_bearing = bearing(vehicle_to_intersection);
    }

    if (!finite(target_bearing) || !finite(crosstrack_error) ||
        !finite(distance_to_waypoint)) {
        return {};
    }
    return {target_bearing, lookahead_distance, crosstrack_error,
            distance_to_waypoint, bearing_to_waypoint, true};
}

} // namespace dima::lib::rover
