#include "motor_pwm.h"

#include "main.h"
#include "api/ActuatorPwmLimits.h"
#include "tim.h"

#define MOTOR_PWM_TIMER_CLOCK_HZ 240000000U
#define MOTOR_PWM_COUNTER_HZ 1000000U
#define MOTOR_PWM_PRESCALER (MOTOR_PWM_TIMER_CLOCK_HZ / MOTOR_PWM_COUNTER_HZ - 1U)
#define MOTOR_PWM_PERIOD_TICKS 19999U

/* 定时器计数频率 = 240 MHz / (PSC + 1) = 1 MHz，因此 1 tick = 1 us；
 * 周期 = (ARR + 1) tick = 20 ms，即 50 Hz。CCR 可直接使用 pulse_us。 */
_Static_assert(DIMA_ACTUATOR_PWM_MAX_PULSE_US <= MOTOR_PWM_PERIOD_TICKS,
               "actuator PWM envelope exceeds timer period");

static bool motor_pwm_started;

static uint32_t timer_input_clock_hz(uint32_t pclk, bool divided)
{
    /* STM32H7 的 APB 预分频不为 1 时，定时器内核时钟为对应 PCLK 的 2 倍。 */
    return divided ? pclk * 2U : pclk;
}

static bool timer_configuration_valid(void)
{
    /* TIM8 以更新事件输出 TRGO，TIM5 以 ITR3 的 reset 模式跟随，保证两组输出
     * 共用 20 ms 帧边界；S1/S2 使用互补通道，故同时核对其反相极性位。 */
    RCC_ClkInitTypeDef clocks = {0};
    uint32_t flash_latency = 0U;
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);

    const uint32_t tim5_clock =
        timer_input_clock_hz(HAL_RCC_GetPCLK1Freq(),
                             clocks.APB1CLKDivider != RCC_APB1_DIV1);
    const uint32_t tim8_clock =
        timer_input_clock_hz(HAL_RCC_GetPCLK2Freq(),
                             clocks.APB2CLKDivider != RCC_APB2_DIV1);
    const uint32_t tim5_slave = htim5.Instance != NULL
                                    ? htim5.Instance->SMCR
                                    : 0U;
    const uint32_t tim8_master = htim8.Instance != NULL
                                     ? htim8.Instance->CR2
                                     : 0U;
    const uint32_t tim8_polarity = htim8.Instance != NULL
                                       ? htim8.Instance->CCER
                                       : 0U;

    return htim5.Instance == TIM5 && htim8.Instance == TIM8 &&
           tim5_clock == MOTOR_PWM_TIMER_CLOCK_HZ &&
           tim8_clock == MOTOR_PWM_TIMER_CLOCK_HZ &&
           htim5.Init.Prescaler == MOTOR_PWM_PRESCALER &&
           htim8.Init.Prescaler == MOTOR_PWM_PRESCALER &&
           htim5.Init.Period == MOTOR_PWM_PERIOD_TICKS &&
           htim8.Init.Period == MOTOR_PWM_PERIOD_TICKS &&
           htim5.Init.CounterMode == TIM_COUNTERMODE_UP &&
           htim8.Init.CounterMode == TIM_COUNTERMODE_UP &&
           (tim5_slave & TIM_SMCR_SMS) == TIM_SLAVEMODE_RESET &&
           (tim5_slave & TIM_SMCR_TS) == TIM_TS_ITR3 &&
           (tim8_master & TIM_CR2_MMS) == TIM_TRGO_UPDATE &&
           (tim8_polarity & (TIM_CCER_CC2NP | TIM_CCER_CC3NP)) ==
               (TIM_CCER_CC2NP | TIM_CCER_CC3NP);
}

static void configure_pins_low(void)
{
    /* 先写低电平再切 GPIO 输出模式，避免复用切换窗口产生意外脉冲。 */
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, S3_Pin | S4_Pin | S5_Pin | S6_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, S1_Pin | S2_Pin, GPIO_PIN_RESET);

    gpio.Pin = S3_Pin | S4_Pin | S5_Pin | S6_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = S1_Pin | S2_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void configure_pins_alternate(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(GPIOA, S3_Pin | S4_Pin | S5_Pin | S6_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, S1_Pin | S2_Pin, GPIO_PIN_RESET);

    gpio.Pin = S3_Pin | S4_Pin | S5_Pin | S6_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = S1_Pin | S2_Pin;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOB, &gpio);
}

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

static void prepare_zero_frame(void)
{
    /* 清空 CCR 和计数器后强制更新，使预装载寄存器中的零值在开放引脚前生效。 */
    clear_compare_registers();
    __HAL_TIM_SET_COUNTER(&htim5, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    htim5.Instance->EGR = TIM_EGR_UG;
    htim8.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
}

board_motor_pwm_result_t board_motor_pwm_start(void)
{
    if (motor_pwm_started) {
        if (timer_configuration_valid()) {
            return BOARD_MOTOR_PWM_APPLIED;
        }
        motor_pwm_started = false;
        configure_pins_low();
        return BOARD_MOTOR_PWM_FAULT;
    }
    if (!timer_configuration_valid()) {
        configure_pins_low();
        return BOARD_MOTOR_PWM_FAULT;
    }

    /* 启动顺序保持失效安全：GPIO 拉低 -> 停止通道 -> 零帧 -> 切复用 -> 启动。
     * 任一 HAL 启动失败都回退到 GPIO 低电平，并返回 RETRY 供上层决定重试。 */
    configure_pins_low();
    stop_outputs();
    prepare_zero_frame();
    configure_pins_alternate();
    if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK) {
        clear_compare_registers();
        stop_outputs();
        configure_pins_low();
        motor_pwm_started = false;
        return BOARD_MOTOR_PWM_RETRY;
    }

    htim8.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    motor_pwm_started = true;
    return BOARD_MOTOR_PWM_APPLIED;
}

board_motor_pwm_result_t board_motor_pwm_stop(void)
{
    motor_pwm_started = false;
    if (!timer_configuration_valid()) {
        configure_pins_low();
        return BOARD_MOTOR_PWM_FAULT;
    }

    stop_outputs();
    prepare_zero_frame();
    configure_pins_low();
    return BOARD_MOTOR_PWM_APPLIED;
}

board_motor_pwm_result_t board_motor_pwm_write(
    const uint16_t pulse_us[BOARD_MOTOR_PWM_COUNT], uint8_t valid_mask)
{
    if (pulse_us == NULL) {
        return BOARD_MOTOR_PWM_FAULT;
    }
    if (!timer_configuration_valid()) {
        motor_pwm_started = false;
        configure_pins_low();
        return BOARD_MOTOR_PWM_FAULT;
    }
    if (!motor_pwm_started) {
        return BOARD_MOTOR_PWM_RETRY;
    }

    const uint32_t tim5_period = __HAL_TIM_GET_AUTORELOAD(&htim5);
    const uint32_t tim8_period = __HAL_TIM_GET_AUTORELOAD(&htim8);
    const uint8_t supported_mask = (uint8_t)((1U << BOARD_MOTOR_PWM_COUNT) - 1U);
    if ((valid_mask & (uint8_t)~supported_mask) != 0U) {
        return BOARD_MOTOR_PWM_FAULT;
    }

    /* 在触碰任何 CCR 前完成整帧校验：mask 外位、无效通道非零值、脉宽包络
     * 或超过 ARR 均整帧拒绝，避免六路输出出现部分更新。 */
    for (uint8_t index = 0U; index < BOARD_MOTOR_PWM_COUNT; ++index) {
        if ((valid_mask & (uint8_t)(1U << index)) == 0U) {
            if (pulse_us[index] != 0U) {
                return BOARD_MOTOR_PWM_FAULT;
            }
            continue;
        }
        const uint32_t period = index < 2U ? tim8_period : tim5_period;
        if (pulse_us[index] < DIMA_ACTUATOR_PWM_MIN_PULSE_US ||
            pulse_us[index] > DIMA_ACTUATOR_PWM_MAX_PULSE_US ||
            pulse_us[index] > period) {
            return BOARD_MOTOR_PWM_FAULT;
        }
    }

    /* 临时禁止 update 事件，连续写完 TIM8/TIM5 的六个比较值后再恢复；这依赖
     * 通道预装载与同步帧边界，防止软件更新事件在半帧写入期间锁存新值。 */
    htim8.Instance->CR1 |= TIM_CR1_UDIS;
    htim5.Instance->CR1 |= TIM_CR1_UDIS;
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
    __DMB();
    htim5.Instance->CR1 &= ~TIM_CR1_UDIS;
    htim8.Instance->CR1 &= ~TIM_CR1_UDIS;
    return BOARD_MOTOR_PWM_APPLIED;
}

bool board_motor_pwm_started(void)
{
    return motor_pwm_started;
}
