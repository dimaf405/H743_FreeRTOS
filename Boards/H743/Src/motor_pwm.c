#include "Boards/H743/Inc/motor_pwm.h"

#include "tim.h"

static bool motor_pwm_started;

static void clear_compare_registers(void)
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U); /* S1: PB0, TIM8_CH2N */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U); /* S2: PB1, TIM8_CH3N */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0U); /* S3: PA0 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0U); /* S4: PA1 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0U); /* S5: PA2 */
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0U); /* S6: PA3 */
}

static void stop_outputs(void)
{
    (void)HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
}

bool board_motor_pwm_start(void)
{
    if (motor_pwm_started) {
        return true;
    }

    clear_compare_registers();
    if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK) {
        clear_compare_registers();
        stop_outputs();
        return false;
    }

    motor_pwm_started = true;
    return true;
}

void board_motor_pwm_stop(void)
{
    clear_compare_registers();
    stop_outputs();
    motor_pwm_started = false;
}

bool board_motor_pwm_write(const uint16_t pulse_us[BOARD_MOTOR_PWM_COUNT],
                           uint8_t valid_mask)
{
    if (!motor_pwm_started || pulse_us == 0) {
        return false;
    }

    const uint32_t tim5_period = __HAL_TIM_GET_AUTORELOAD(&htim5);
    const uint32_t tim8_period = __HAL_TIM_GET_AUTORELOAD(&htim8);
    const uint8_t supported_mask = (uint8_t)((1U << BOARD_MOTOR_PWM_COUNT) - 1U);
    if ((valid_mask & (uint8_t)~supported_mask) != 0U) {
        return false;
    }

    for (uint8_t index = 0U; index < BOARD_MOTOR_PWM_COUNT; ++index) {
        if ((valid_mask & (uint8_t)(1U << index)) == 0U) {
            continue;
        }
        const uint32_t period = index < 2U ? tim8_period : tim5_period;
        if (pulse_us[index] == 0U || pulse_us[index] > period) {
            return false;
        }
    }

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
                          (valid_mask & (1U << BOARD_MOTOR_PWM_S1)) != 0U
                              ? pulse_us[BOARD_MOTOR_PWM_S1] : 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
                          (valid_mask & (1U << BOARD_MOTOR_PWM_S2)) != 0U
                              ? pulse_us[BOARD_MOTOR_PWM_S2] : 0U);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,
                          (valid_mask & (1U << BOARD_MOTOR_PWM_S3)) != 0U
                              ? pulse_us[BOARD_MOTOR_PWM_S3] : 0U);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,
                          (valid_mask & (1U << BOARD_MOTOR_PWM_S4)) != 0U
                              ? pulse_us[BOARD_MOTOR_PWM_S4] : 0U);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
                          (valid_mask & (1U << BOARD_MOTOR_PWM_S5)) != 0U
                              ? pulse_us[BOARD_MOTOR_PWM_S5] : 0U);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4,
                          (valid_mask & (1U << BOARD_MOTOR_PWM_S6)) != 0U
                              ? pulse_us[BOARD_MOTOR_PWM_S6] : 0U);
    return true;
}

bool board_motor_pwm_started(void)
{
    return motor_pwm_started;
}
