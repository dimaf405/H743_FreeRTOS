#include "stm32h7/HardwareServices.hpp"

#include "motor_pwm.h"

namespace dima::platform::stm32h7 {
namespace {

ActuatorPwmResult translate(board_motor_pwm_result_t result) noexcept
{
    /* 保持板级三态语义：Retry 是可恢复资源/启动失败，Fault 是合同或硬件配置错误，
     * 上层安全状态机据此决定重试还是锁定输出，不能合并成 bool。 */
    switch (result) {
    case BOARD_MOTOR_PWM_APPLIED:
        return ActuatorPwmResult::Applied;
    case BOARD_MOTOR_PWM_RETRY:
        return ActuatorPwmResult::Retry;
    case BOARD_MOTOR_PWM_FAULT:
    default:
        return ActuatorPwmResult::Fault;
    }
}

class Stm32ActuatorPwm final : public ActuatorPwm {
public:
    ActuatorPwmResult start() noexcept override
    {
        return translate(board_motor_pwm_start());
    }

    ActuatorPwmResult stop() noexcept override
    {
        return translate(board_motor_pwm_stop());
    }

    ActuatorPwmResult write(const ActuatorPwmFrame &frame) noexcept override
    {
        /* 编译期锁定平台 API 与板级六通道布局；脉宽单位保持 us，不做隐式缩放。 */
        static_assert(kActuatorPwmChannelCount == BOARD_MOTOR_PWM_COUNT);
        return translate(board_motor_pwm_write(frame.pulse_us,
                                               frame.enabled_mask));
    }

    bool started() const noexcept override
    {
        return board_motor_pwm_started();
    }
};

Stm32ActuatorPwm &instance() noexcept
{
    static Stm32ActuatorPwm value;
    return value;
}

} // namespace

ActuatorPwm &actuator_pwm() noexcept { return instance(); }

} // namespace dima::platform::stm32h7
