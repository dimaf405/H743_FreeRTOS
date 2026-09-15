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

static uint32_t timer_input_clock_hz(uint32_t pclk, uint32_t apb_code)
{
    /* RM0433 TIMPRE：0 时 APB 分频后的定时器为 2*PCLK；1 时 APB<=4
     * 取 HCLK，否则取 4*PCLK。不能无条件按两倍 PCLK 判断 PWM 周期。 */
    if (apb_code < 4U) return pclk;
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U) return pclk * 2U;
    return apb_code <= 5U ? HAL_RCC_GetHCLKFreq() : pclk * 4U;
}

static bool pwm_channel_mode_valid(uint32_t ccmr, uint32_t shift)
{
    /* 两个 CCMR 的通道字段位置相同：必须是输出/PWM1/预装载开启。
     * 快速模式或外部清除不能偷偷改变脉宽；只读 HAL Init 无法证明这些位。 */
    const uint32_t mask = (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M |
        TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE | TIM_CCMR1_OC1CE) << shift;
    const uint32_t expected = (TIM_OCMODE_PWM1 | TIM_CCMR1_OC1PE) << shift;
    return (ccmr & mask) == expected;
}

static bool timer_configuration_valid(void)
{
    if (htim5.Instance != TIM5 || htim8.Instance != TIM8) return false;
    const uint32_t tim5_clock = timer_input_clock_hz(HAL_RCC_GetPCLK1Freq(),
        (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos);
    const uint32_t tim8_clock = timer_input_clock_hz(HAL_RCC_GetPCLK2Freq(),
        (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos);
    const uint32_t incompatible_mode = TIM_CR1_DIR | TIM_CR1_CMS |
        TIM_CR1_CKD | TIM_CR1_OPM | TIM_CR1_UDIS;
    const uint32_t enabled5 = TIM_CCER_CC1E | TIM_CCER_CC2E |
        TIM_CCER_CC3E | TIM_CCER_CC4E;
    const uint32_t enabled8 = TIM_CCER_CC2NE | TIM_CCER_CC3NE;

    /* APM ChibiOS 的 N-only ACTIVE_HIGH 对应 NE=1、NP=0、主输出 E=0。
     * S1/S2 与 S3..S6 均应高电平宽度=CCR；读回模式/极性/使能/时钟，
     * 不把缓存配置正确当作硬件寄存器仍然正确。低 16 位覆盖前四通道。 */
    return tim5_clock == MOTOR_PWM_TIMER_CLOCK_HZ && tim8_clock == MOTOR_PWM_TIMER_CLOCK_HZ &&
        TIM5->PSC == MOTOR_PWM_PRESCALER && TIM8->PSC == MOTOR_PWM_PRESCALER &&
        TIM5->ARR == MOTOR_PWM_PERIOD_TICKS && TIM8->ARR == MOTOR_PWM_PERIOD_TICKS && TIM8->RCR == 0U &&
        (TIM5->CR1 & incompatible_mode) == 0U && (TIM8->CR1 & incompatible_mode) == 0U &&
        (TIM5->CR1 & TIM_CR1_CEN) == (motor_pwm_started ? TIM_CR1_CEN : 0U) &&
        (TIM8->CR1 & TIM_CR1_CEN) == (motor_pwm_started ? TIM_CR1_CEN : 0U) &&
        (TIM5->SMCR & (TIM_SMCR_SMS | TIM_SMCR_TS)) == (TIM_SLAVEMODE_RESET | TIM_TS_ITR3) &&
        (TIM8->SMCR & TIM_SMCR_SMS) == 0U &&
        (TIM8->CR2 & TIM_CR2_MMS) == TIM_TRGO_UPDATE &&
        pwm_channel_mode_valid(TIM5->CCMR1, 0U) && pwm_channel_mode_valid(TIM5->CCMR1, 8U) &&
        pwm_channel_mode_valid(TIM5->CCMR2, 0U) && pwm_channel_mode_valid(TIM5->CCMR2, 8U) &&
        pwm_channel_mode_valid(TIM8->CCMR1, 8U) && pwm_channel_mode_valid(TIM8->CCMR2, 0U) &&
        (TIM5->CCER & 0xFFFFU) == (motor_pwm_started ? enabled5 : 0U) &&
        (TIM8->CCER & 0xFFFFU) == (motor_pwm_started ? enabled8 : 0U) &&
        (TIM8->BDTR & (TIM_BDTR_MOE | TIM_BDTR_DTG | TIM_BDTR_BKE | TIM_BDTR_BK2E | TIM_BDTR_AOE)) ==
            (motor_pwm_started ? TIM_BDTR_MOE : 0U);
}

static bool pins_in_mode(GPIO_TypeDef *port, uint32_t pins, uint32_t mode, uint32_t alternate)
{
    if ((port->OTYPER & pins) != 0U) return false;
    for (uint32_t pin = 0U; pin < 16U; ++pin) {
        if ((pins & (1U << pin)) == 0U) continue;
        if (((port->MODER >> (pin * 2U)) & 3U) != mode) return false;
        if (mode == 2U && ((port->AFR[pin / 8U] >> ((pin % 8U) * 4U)) & 15U) != alternate) return false;
    }
    return true;
}

static bool pins_alternate_valid(void)
{
    return pins_in_mode(GPIOA, S3_Pin | S4_Pin | S5_Pin | S6_Pin, 2U, GPIO_AF2_TIM5) &&
        pins_in_mode(GPIOB, S1_Pin | S2_Pin, 2U, GPIO_AF3_TIM8);
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
    /* 固定地址仅限本板后端；故障时不能通过可能已损坏的 HAL Instance 访问。 */
    TIM8->CCR2 = 0U;
    TIM8->CCR3 = 0U;
    TIM5->CCR1 = 0U;
    TIM5->CCR2 = 0U;
    TIM5->CCR3 = 0U;
    TIM5->CCR4 = 0U;
}

static bool stop_outputs(void)
{
    /* 停波不依赖原 PWM 配置有效：先把六个引脚变成低电平 GPIO，再停计数、
     * 禁用输出/DMA/IRQ、清除 CCR。即使 PSC/ARR/模式已异常也不能只拉低引脚
     * 却留下正在运行的定时器。HAL 调用只负责恢复合法句柄的通道状态。 */
    configure_pins_low();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motor_pwm_started = false;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    TIM8->CCER = 0U;
    TIM5->CCER = 0U;
    TIM8->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_UDIS);
    TIM5->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_UDIS);
    TIM8->DIER = 0U;
    TIM5->DIER = 0U;
    clear_compare_registers();
    TIM8->CNT = 0U;
    TIM5->CNT = 0U;
    TIM5->EGR = TIM_EGR_UG;
    TIM8->EGR = TIM_EGR_UG;
    TIM5->SR = 0U;
    TIM8->SR = 0U;
    __DSB();
    __set_PRIMASK(primask);
    if (htim8.Instance == TIM8) {
        (void)HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
        (void)HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    }
    if (htim5.Instance == TIM5) {
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
    }
    const uint32_t pins_a = S3_Pin | S4_Pin | S5_Pin | S6_Pin;
    const uint32_t pins_b = S1_Pin | S2_Pin;
    return (TIM8->CR1 & TIM_CR1_CEN) == 0U && (TIM5->CR1 & TIM_CR1_CEN) == 0U &&
        (TIM8->BDTR & TIM_BDTR_MOE) == 0U && TIM8->CCER == 0U && TIM5->CCER == 0U &&
        TIM8->CCR2 == 0U && TIM8->CCR3 == 0U && TIM5->CCR1 == 0U && TIM5->CCR2 == 0U &&
        TIM5->CCR3 == 0U && TIM5->CCR4 == 0U &&
        pins_in_mode(GPIOA, pins_a, 1U, 0U) && pins_in_mode(GPIOB, pins_b, 1U, 0U) &&
        ((GPIOA->ODR | GPIOA->IDR) & pins_a) == 0U && ((GPIOB->ODR | GPIOB->IDR) & pins_b) == 0U;
}

static board_motor_pwm_result_t output_fault(void)
{
    (void)stop_outputs();
    return BOARD_MOTOR_PWM_FAULT;
}

board_motor_pwm_result_t board_motor_pwm_start(void)
{
    if (!timer_configuration_valid()) return output_fault();
    if (motor_pwm_started) return pins_alternate_valid() ? BOARD_MOTOR_PWM_APPLIED : output_fault();
    if (!stop_outputs()) return BOARD_MOTOR_PWM_FAULT;

    /* 引脚保持低电平期间启动六路零 CCR，主定时器 UG 同步零帧；确认寄存器
     * 后才开放 AF，避免逐个 HAL Start 期间把半初始化输出暴露给电调。 */
    if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK) {
        return stop_outputs() ? BOARD_MOTOR_PWM_RETRY : BOARD_MOTOR_PWM_FAULT;
    }
    TIM8->EGR = TIM_EGR_UG;
    TIM5->SR = 0U;
    TIM8->SR = 0U;
    motor_pwm_started = true;
    if (!timer_configuration_valid()) return output_fault();
    configure_pins_alternate();
    return pins_alternate_valid() ? BOARD_MOTOR_PWM_APPLIED : output_fault();
}

board_motor_pwm_result_t board_motor_pwm_stop(void)
{
    return stop_outputs() ? BOARD_MOTOR_PWM_APPLIED : BOARD_MOTOR_PWM_FAULT;
}

board_motor_pwm_result_t board_motor_pwm_write(
    const uint16_t pulse_us[BOARD_MOTOR_PWM_COUNT], uint8_t valid_mask)
{
    if (pulse_us == NULL || !timer_configuration_valid()) return output_fault();
    if (!motor_pwm_started) return BOARD_MOTOR_PWM_RETRY;
    if (!pins_alternate_valid()) return output_fault();

    const uint32_t tim5_period = __HAL_TIM_GET_AUTORELOAD(&htim5);
    const uint32_t tim8_period = __HAL_TIM_GET_AUTORELOAD(&htim8);
    const uint8_t supported_mask = (uint8_t)((1U << BOARD_MOTOR_PWM_COUNT) - 1U);
    if ((valid_mask & (uint8_t)~supported_mask) != 0U) {
        return output_fault();
    }

    /* 在触碰任何 CCR 前完成整帧校验：mask 外位、无效通道非零值、脉宽包络
     * 或超过 ARR 均整帧拒绝，避免六路输出出现部分更新。 */
    for (uint8_t index = 0U; index < BOARD_MOTOR_PWM_COUNT; ++index) {
        if ((valid_mask & (uint8_t)(1U << index)) == 0U) {
            if (pulse_us[index] != 0U) {
                return output_fault();
            }
            continue;
        }
        const uint32_t period = index < 2U ? tim8_period : tim5_period;
        if (pulse_us[index] < DIMA_ACTUATOR_PWM_MIN_PULSE_US ||
            pulse_us[index] > DIMA_ACTUATOR_PWM_MAX_PULSE_US ||
            pulse_us[index] > period) {
            return output_fault();
        }
    }

    /* 短临界区防止两次 UDIS 操作和六路写入之间被抢占；DMB 不能替代互斥。
     * 计数器不停止、不强制 UG，避免拉长正在输出的高脉冲。这里提交并读回
     * 的仍是预装载值，实际边沿随自然更新事件生效，不冒充示波器测量。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
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
    const bool readback = TIM8->CCR2 == pulse_us[BOARD_MOTOR_PWM_S1] &&
        TIM8->CCR3 == pulse_us[BOARD_MOTOR_PWM_S2] && TIM5->CCR1 == pulse_us[BOARD_MOTOR_PWM_S3] &&
        TIM5->CCR2 == pulse_us[BOARD_MOTOR_PWM_S4] && TIM5->CCR3 == pulse_us[BOARD_MOTOR_PWM_S5] &&
        TIM5->CCR4 == pulse_us[BOARD_MOTOR_PWM_S6];
    __DSB();
    __set_PRIMASK(primask);
    return readback && timer_configuration_valid() ? BOARD_MOTOR_PWM_APPLIED : output_fault();
}

bool board_motor_pwm_started(void)
{
    return motor_pwm_started;
}
