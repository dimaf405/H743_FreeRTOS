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
