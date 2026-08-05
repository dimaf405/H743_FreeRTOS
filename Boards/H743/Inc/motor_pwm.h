#ifndef H743_MOTOR_PWM_H
#define H743_MOTOR_PWM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BOARD_MOTOR_PWM_S1 = 0,
    BOARD_MOTOR_PWM_S2,
    BOARD_MOTOR_PWM_S3,
    BOARD_MOTOR_PWM_S4,
    BOARD_MOTOR_PWM_S5,
    BOARD_MOTOR_PWM_S6,
    BOARD_MOTOR_PWM_COUNT
};

typedef enum board_motor_pwm_result_e {
    BOARD_MOTOR_PWM_APPLIED = 0,
    BOARD_MOTOR_PWM_RETRY = 1,
    BOARD_MOTOR_PWM_FAULT = 2
} board_motor_pwm_result_t;

/* Starts all six physical outputs with compare values held at zero. */
board_motor_pwm_result_t board_motor_pwm_start(void);

/* Stops both timers, clears all compares, and drives every pin low. */
board_motor_pwm_result_t board_motor_pwm_stop(void);

/*
 * Writes one transactional six-channel frame. Bits clear in valid_mask are
 * disabled by writing a zero compare value. Call board_motor_pwm_start first.
 */
board_motor_pwm_result_t board_motor_pwm_write(
    const uint16_t pulse_us[BOARD_MOTOR_PWM_COUNT], uint8_t valid_mask);

bool board_motor_pwm_started(void);

#ifdef __cplusplus
}
#endif

#endif /* H743_MOTOR_PWM_H */
