/****************************************************************************
 * Fixed Dima stage-compatibility parameters for PX4-compatible GCSs.
 *
 * These 11 parameters retain the stock QGC PX4 identity, calibration-summary,
 * and implemented safety-policy contracts. They are fixed and write-protected.
 * Unimplemented flight-mode and Return Mode parameters are deliberately absent.
 * Board serial parameters are generated separately from
 * Boards/H743/serial_ports.json.
 ****************************************************************************/

/**
 * PX4-compatible airframe identity.
 *
 * Dima is a fixed differential rover product.  This value matches PX4's
 * Generic Rover Differential airframe, but does not enable runtime airframe
 * selection or PX4 startup scripts.
 *
 * @min 50000
 * @max 50000
 * @value 50000 Generic Rover Differential
 * @group System
 */
PARAM_DEFINE_INT32(SYS_AUTOSTART, 50000);

/**
 * PX4 airframe autoconfiguration request.
 *
 * Dima has no PX4 airframe-default transaction.  The fixed zero value means
 * keep the current product configuration; requests to set one are rejected.
 *
 * @min 0
 * @max 0
 * @value 0 Keep current product configuration
 * @group System
 */
PARAM_DEFINE_INT32(SYS_AUTOCONFIG, 0);

/**
 * MAVLink system ID.
 *
 * The current Dima protocol surface uses the fixed system/component identity
 * 1/1 for every encoder and target filter.
 *
 * @min 1
 * @max 1
 * @reboot_required true
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_SYS_ID, 1);

/** Gyroscope calibration device ID; zero means unavailable/unconfigured.
 * @min 0
 * @max 0
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_GYRO0_ID, 0);

/** Accelerometer calibration device ID; zero means unavailable/unconfigured.
 * @min 0
 * @max 0
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC0_ID, 0);

/** Magnetometer 0 calibration device ID; zero means unavailable/unconfigured.
 * @min 0
 * @max 0
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG0_ID, 0);

/** Magnetometer 1 calibration device ID; zero means unavailable/unconfigured.
 * @min 0
 * @max 0
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG1_ID, 0);

/** Magnetometer 2 calibration device ID; zero means unavailable/unconfigured.
 * @min 0
 * @max 0
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG2_ID, 0);

/**
 * RC-loss action.
 *
 * Commander currently fails closed by disarming on RC loss.  Other PX4
 * actions require navigation or landing capabilities which are not present.
 *
 * @min 6
 * @max 6
 * @value 6 Disarm
 * @group Commander
 */
PARAM_DEFINE_INT32(NAV_RCL_ACT, 6);

/**
 * GCS data-link-loss action.
 *
 * USB/GCS is not a Dima control source in this stage, so disconnect has no
 * navigation action.
 *
 * @min 0
 * @max 0
 * @value 0 Disabled
 * @group Commander
 */
PARAM_DEFINE_INT32(NAV_DLL_ACT, 0);

/**
 * Low-battery action compatibility value.
 *
 * UI-only in this stage: no battery detector or low-battery action exists.
 *
 * @min 0
 * @max 0
 * @value 0 No implemented action
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_LOW_BAT_ACT, 0);
