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

#pragma once

namespace dima::lib::rover {

struct Position2f {
    float north_m;
    float east_m;
};

struct PurePursuitConfig {
    float lookahead_gain;
    float lookahead_min_m;
    float lookahead_max_m;
};

struct PurePursuitOutput {
    float target_bearing_rad;
    float lookahead_distance_m;
    float crosstrack_error_m;
    float distance_to_waypoint_m;
    float bearing_to_waypoint_rad;
    bool valid;
};

/**
 * 与 PX4 v1.17 Rover 一致的 Pure Pursuit 纯路径制导核。
 *
 * 本类型只负责“当前位置应该朝向哪里”，不拥有航点任务、参数、消息或电机；
 * 因而可由后续 AutoMode 在不跨越产品安全边界的前提下组合使用。
 */
class PurePursuit {
public:
    bool configure(const PurePursuitConfig &config) noexcept;

    PurePursuitOutput update(const Position2f &previous_waypoint_ne_m,
                             const Position2f &target_waypoint_ne_m,
                             const Position2f &vehicle_position_ne_m,
                             float ground_speed_m_s) const noexcept;

private:
    static bool finite(float value) noexcept;
    static bool finite(const Position2f &value) noexcept;
    static bool valid_config(const PurePursuitConfig &config) noexcept;

    PurePursuitConfig config_{};
    bool configured_{false};
};

} // namespace dima::lib::rover
