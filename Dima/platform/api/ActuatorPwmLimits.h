#pragma once

/* Product-level ordinary PWM envelope shared by C and C++ consumers. */
#define DIMA_ACTUATOR_PWM_MIN_PULSE_US 500U
#define DIMA_ACTUATOR_PWM_MAX_PULSE_US 2500U

#if DIMA_ACTUATOR_PWM_MIN_PULSE_US == 0U || \
    DIMA_ACTUATOR_PWM_MIN_PULSE_US > DIMA_ACTUATOR_PWM_MAX_PULSE_US
#error "invalid Dima actuator PWM pulse envelope"
#endif
