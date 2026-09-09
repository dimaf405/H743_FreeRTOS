#pragma once

#include "rover/RoverControl.hpp"

#include <cmath>

namespace dima::rover::control {

// 只检查两个应用消费者共有的内环参数条件，不读参数服务、不保存状态。
// 各消费者仍在自己的快照上独立调用，并保留速度上限、巡航、转向及 Heading
// 的交叉约束；共享谓词不会改变参数未就绪时的停控语义或触碰控制算法内核。
bool speed_control_parameters_valid(
    const dima::lib::rover::SpeedControlConfig &speed) noexcept;

bool yaw_rate_control_parameters_valid(
    const dima::lib::rover::YawRateControlConfig &yaw_rate) noexcept;

} // namespace dima::rover::control
