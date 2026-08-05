/**
 * Stage 5 Rover manual drive and actuator shaping parameters.
 *
 * PX4 v1.17.0 provides the direct control architecture. ArduPilot commit
 * 3f2e4763accb is used only as a behavioral reference for reversible Rover
 * steering, skid-steer saturation, slew, thrust shaping, and reversal delay.
 */

/**
 * Rover motion request timeout
 *
 * Maximum age of both the request publication and its input sample.
 *
 * @unit s
 * @min 0.02
 * @max 1.00
 * @increment 0.01
 * @decimal 2
 * @group Rover Differential
 */
PARAM_DEFINE_FLOAT(RO_CMD_TIMEOUT, 0.10f);

/**
 * Reverse steering in Manual mode
 *
 * Keeps the commanded vehicle nose direction intuitive while reversing.
 *
 * @min 0
 * @max 1
 * @group Rover Differential
 */
PARAM_DEFINE_INT32(RD_REV_STEER, 1);

/**
 * Steering versus throttle saturation priority
 *
 * 0 preserves longitudinal command, 0.5 scales both axes proportionally,
 * and 1 preserves steering command.
 *
 * @min 0.00
 * @max 1.00
 * @increment 0.01
 * @decimal 2
 * @group Rover Differential
 */
PARAM_DEFINE_FLOAT(RD_STR_THR_MIX, 0.50f);

/**
 * Minimum nonzero motor output
 *
 * Static-friction compensation applied only after a nonzero motor command.
 * This is not an RC input deadzone.
 *
 * @min 0.00
 * @max 1.00
 * @increment 0.01
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_THR_MIN, 0.00f);

/**
 * Maximum motor output
 *
 * @min 0.05
 * @max 1.00
 * @increment 0.01
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_THR_MAX, 1.00f);

/**
 * Longitudinal command slew rate
 *
 * A value of zero disables slew limiting.
 *
 * @unit 1/s
 * @min 0.00
 * @max 10.00
 * @increment 0.05
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_SLEW_RATE, 1.00f);

/**
 * Per-side motor reversal delay
 *
 * Each side independently holds zero before accepting the opposite sign.
 *
 * @unit s
 * @min 0.00
 * @max 1.00
 * @increment 0.01
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_REV_DELAY, 0.20f);

/**
 * Motor thrust curve expo
 *
 * Zero is linear. Positive values compensate a weak low end; negative values
 * reduce low-end response.
 *
 * @min -1.00
 * @max 1.00
 * @increment 0.01
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_THR_EXPO, 0.00f);

/**
 * Forward to reverse thrust asymmetry
 *
 * Values above one increase negative motor commands before final limiting.
 *
 * @min 1.00
 * @max 10.00
 * @increment 0.05
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_THR_ASYM, 1.00f);

/**
 * Motor output ramp after arming
 *
 * @unit s
 * @min 0.00
 * @max 5.00
 * @increment 0.05
 * @decimal 2
 * @group Rover Motors
 */
PARAM_DEFINE_FLOAT(MOT_ARM_RAMP, 0.50f);

/**
 * PWM S1 function
 *
 * @value 0 Disabled
 * @value 101 Motor right
 * @value 102 Motor left
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S1_FUNC, 0);

/**
 * PWM S1 minimum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S1_MIN, 1000);
/**
 * PWM S1 center pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S1_CENT, 1500);
/**
 * PWM S1 maximum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S1_MAX, 2000);
/**
 * PWM S1 reverse
 *
 * @value 0 Normal
 * @value 1 Reversed
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S1_REV, 0);

/**
 * PWM S2 function
 *
 * @value 0 Disabled
 * @value 101 Motor right
 * @value 102 Motor left
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S2_FUNC, 0);

/**
 * PWM S2 minimum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S2_MIN, 1000);
/**
 * PWM S2 center pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S2_CENT, 1500);
/**
 * PWM S2 maximum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S2_MAX, 2000);
/**
 * PWM S2 reverse
 *
 * @value 0 Normal
 * @value 1 Reversed
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S2_REV, 0);

/**
 * PWM S3 function
 *
 * @value 0 Disabled
 * @value 101 Motor right
 * @value 102 Motor left
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S3_FUNC, 0);

/**
 * PWM S3 minimum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S3_MIN, 1000);
/**
 * PWM S3 center pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S3_CENT, 1500);
/**
 * PWM S3 maximum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S3_MAX, 2000);
/**
 * PWM S3 reverse
 *
 * @value 0 Normal
 * @value 1 Reversed
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S3_REV, 0);

/**
 * PWM S4 function
 *
 * @value 0 Disabled
 * @value 101 Motor right
 * @value 102 Motor left
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S4_FUNC, 0);

/**
 * PWM S4 minimum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S4_MIN, 1000);
/**
 * PWM S4 center pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S4_CENT, 1500);
/**
 * PWM S4 maximum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S4_MAX, 2000);
/**
 * PWM S4 reverse
 *
 * @value 0 Normal
 * @value 1 Reversed
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S4_REV, 0);

/**
 * PWM S5 function
 *
 * @value 0 Disabled
 * @value 101 Motor right
 * @value 102 Motor left
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S5_FUNC, 0);

/**
 * PWM S5 minimum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S5_MIN, 1000);
/**
 * PWM S5 center pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S5_CENT, 1500);
/**
 * PWM S5 maximum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S5_MAX, 2000);
/**
 * PWM S5 reverse
 *
 * @value 0 Normal
 * @value 1 Reversed
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S5_REV, 0);

/**
 * PWM S6 function
 *
 * @value 0 Disabled
 * @value 101 Motor right
 * @value 102 Motor left
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S6_FUNC, 0);

/**
 * PWM S6 minimum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S6_MIN, 1000);
/**
 * PWM S6 center pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S6_CENT, 1500);
/**
 * PWM S6 maximum pulse
 *
 * @unit us
 * @min 800
 * @max 2200
 * @increment 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S6_MAX, 2000);
/**
 * PWM S6 reverse
 *
 * @value 0 Normal
 * @value 1 Reversed
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_S6_REV, 0);
