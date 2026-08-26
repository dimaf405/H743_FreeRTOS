/****************************************************************************
 * Sensor and UM982 parameters for the H743 Rover product.
 *
 * DroneCAN parameters are generated from the protocol contract manifest.
 ****************************************************************************/

// 中文维护说明：本文件是传感器参数生成链的权威输入，参数值、范围和枚举只在
// PARAM_DEFINE/Metadata 中定义一次。SENS_MAG_RATE 仅限制校准后前端发布频率，
// 不改变远端磁力计采样率；IMU_INTEG_RATE 对应积分周期
// delta_t_us = 1,000,000 / rate_hz，而不是 ICM42688P 的 8 kHz FIFO ODR；
// GPS_YAW_OFFSET 以车体前向为 0 度并按顺时针增加。英文 Doxygen 会进入 QGC
// Metadata，普通中文注释不参与生成，因而不会改变参数 ABI 或地面站显示合同。

/**
 * Magnetometer maximum publication rate.
 *
 * This limits the calibrated vehicle_magnetometer publication rate. The raw
 * sensor_mag stream remains at the incoming DroneCAN rate for source
 * detection and calibration, matching the PX4 SENS_MAG_RATE data-flow role.
 * It does not configure the remote node's hardware sampling rate.
 *
 * @min 1
 * @max 200
 * @unit Hz
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_MAG_RATE, 15.0f);

/**
 * Primary GPS protocol.
 *
 * Auto detect and NMEA both use the PX4-compatible NMEA frontend with the
 * CRC-validated Unicore AGRICA/UNIHEADINGA extension.
 *
 * @value 0 Auto detect
 * @value 6 NMEA / Unicore UM982
 * @group GPS
 */
PARAM_DEFINE_INT32(GPS_1_PROTOCOL, 6);

/**
 * Heading/Yaw offset for dual antenna GPS.
 *
 * Set this to zero when the antennas are parallel to vehicle forward and the
 * Unicore primary antenna is in front. The offset increases clockwise.
 *
 * @unit deg
 * @min 0
 * @max 360
 * @increment 0.1
 * @decimal 3
 * @group GPS
 */
PARAM_DEFINE_FLOAT(GPS_YAW_OFFSET, 0.0f);

/**
 * Board sensor rotation using the PX4 rotation enum.
 *
 * @min 0
 * @max 40
 * @value 0 No rotation
 * @value 1 Yaw 45 degrees
 * @value 2 Yaw 90 degrees
 * @value 3 Yaw 135 degrees
 * @value 4 Yaw 180 degrees
 * @value 5 Yaw 225 degrees
 * @value 6 Yaw 270 degrees
 * @value 7 Yaw 315 degrees
 * @value 8 Roll 180 degrees
 * @value 9 Roll 180, yaw 45 degrees
 * @value 10 Roll 180, yaw 90 degrees
 * @value 11 Roll 180, yaw 135 degrees
 * @value 12 Pitch 180 degrees
 * @value 13 Roll 180, yaw 225 degrees
 * @value 14 Roll 180, yaw 270 degrees
 * @value 15 Roll 180, yaw 315 degrees
 * @value 16 Roll 90 degrees
 * @value 17 Roll 90, yaw 45 degrees
 * @value 18 Roll 90, yaw 90 degrees
 * @value 19 Roll 90, yaw 135 degrees
 * @value 20 Roll 270 degrees
 * @value 21 Roll 270, yaw 45 degrees
 * @value 22 Roll 270, yaw 90 degrees
 * @value 23 Roll 270, yaw 135 degrees
 * @value 24 Pitch 90 degrees
 * @value 25 Pitch 270 degrees
 * @value 26 Pitch 180, yaw 90 degrees
 * @value 27 Pitch 180, yaw 270 degrees
 * @value 28 Roll 90, pitch 90 degrees
 * @value 29 Roll 180, pitch 90 degrees
 * @value 30 Roll 270, pitch 90 degrees
 * @value 31 Roll 90, pitch 180 degrees
 * @value 32 Roll 270, pitch 180 degrees
 * @value 33 Roll 90, pitch 270 degrees
 * @value 34 Roll 180, pitch 270 degrees
 * @value 35 Roll 270, pitch 270 degrees
 * @value 36 Roll 90, pitch 180, yaw 90 degrees
 * @value 37 Roll 90, yaw 270 degrees
 * @value 38 Roll 90, pitch 68, yaw 293 degrees
 * @value 39 Pitch 315 degrees
 * @value 40 Roll 90, pitch 315 degrees
 * @qgc_required
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_BOARD_ROT, 0);

/**
 * IMU integration rate.
 *
 * Rate at which calibrated raw IMU data is integrated into delta angle and
 * delta velocity. The listed rates are the modes verified against the fixed
 * 8 kHz ICM42688P FIFO path in this product. Changes are applied by the
 * disarmed sensor-configuration maintenance transaction.
 *
 * @min 100
 * @max 400
 * @value 100 100 Hz
 * @value 200 200 Hz
 * @value 250 250 Hz
 * @value 400 400 Hz
 * @unit Hz
 * @group Sensors
 */
PARAM_DEFINE_INT32(IMU_INTEG_RATE, 200);

/**
 * IMU clipping notification.
 *
 * Emit a rate-limited warning when integrated accelerometer or gyroscope data
 * reports clipping. This does not alter or discard samples.
 *
 * @boolean
 * @category System
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_IMU_CLPNOTI, 1);

/**
 * Accelerometer device identity.
 *
 * @category System
 * @qgc_required
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC0_ID, 0);
/**
 * Accelerometer X offset.
 *
 * @unit m/s^2
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_XOFF, 0.0f);
/**
 * Accelerometer Y offset.
 *
 * @unit m/s^2
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_YOFF, 0.0f);
/**
 * Accelerometer Z offset.
 *
 * @unit m/s^2
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_ZOFF, 0.0f);
/**
 * Accelerometer X scale.
 *
 * @min 0.1
 * @max 3.0
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_XSCALE, 1.0f);
/**
 * Accelerometer Y scale.
 *
 * @min 0.1
 * @max 3.0
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_YSCALE, 1.0f);
/**
 * Accelerometer Z scale.
 *
 * @min 0.1
 * @max 3.0
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_ZSCALE, 1.0f);

/**
 * Gyroscope device identity.
 *
 * @category System
 * @qgc_required
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_GYRO0_ID, 0);
/**
 * Gyroscope X offset.
 *
 * @unit rad/s
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_XOFF, 0.0f);
/**
 * Gyroscope Y offset.
 *
 * @unit rad/s
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_YOFF, 0.0f);
/**
 * Gyroscope Z offset.
 *
 * @unit rad/s
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_ZOFF, 0.0f);

/**
 * Magnetometer device identity.
 *
 * @category System
 * @qgc_required
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG0_ID, 0);
/**
 * Magnetometer X offset.
 *
 * @unit gauss
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XOFF, 0.0f);
/**
 * Magnetometer Y offset.
 *
 * @unit gauss
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YOFF, 0.0f);
/**
 * Magnetometer Z offset.
 *
 * @unit gauss
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZOFF, 0.0f);
/**
 * Magnetometer X scale.
 *
 * @min 0.1
 * @max 3.0
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XSCALE, 1.0f);
/**
 * Magnetometer Y scale.
 *
 * @min 0.1
 * @max 3.0
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YSCALE, 1.0f);
/**
 * Magnetometer Z scale.
 *
 * @min 0.1
 * @max 3.0
 * @category System
 * @decimal 3
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZSCALE, 1.0f);
/**
 * Magnetometer rotation using the PX4 rotation enum.
 *
 * @min 0
 * @max 40
 * @value 0 No rotation
 * @value 1 Yaw 45 degrees
 * @value 2 Yaw 90 degrees
 * @value 3 Yaw 135 degrees
 * @value 4 Yaw 180 degrees
 * @value 5 Yaw 225 degrees
 * @value 6 Yaw 270 degrees
 * @value 7 Yaw 315 degrees
 * @value 8 Roll 180 degrees
 * @value 9 Roll 180, yaw 45 degrees
 * @value 10 Roll 180, yaw 90 degrees
 * @value 11 Roll 180, yaw 135 degrees
 * @value 12 Pitch 180 degrees
 * @value 13 Roll 180, yaw 225 degrees
 * @value 14 Roll 180, yaw 270 degrees
 * @value 15 Roll 180, yaw 315 degrees
 * @value 16 Roll 90 degrees
 * @value 17 Roll 90, yaw 45 degrees
 * @value 18 Roll 90, yaw 90 degrees
 * @value 19 Roll 90, yaw 135 degrees
 * @value 20 Roll 270 degrees
 * @value 21 Roll 270, yaw 45 degrees
 * @value 22 Roll 270, yaw 90 degrees
 * @value 23 Roll 270, yaw 135 degrees
 * @value 24 Pitch 90 degrees
 * @value 25 Pitch 270 degrees
 * @value 26 Pitch 180, yaw 90 degrees
 * @value 27 Pitch 180, yaw 270 degrees
 * @value 28 Roll 90, pitch 90 degrees
 * @value 29 Roll 180, pitch 90 degrees
 * @value 30 Roll 270, pitch 90 degrees
 * @value 31 Roll 90, pitch 180 degrees
 * @value 32 Roll 270, pitch 180 degrees
 * @value 33 Roll 90, pitch 270 degrees
 * @value 34 Roll 180, pitch 270 degrees
 * @value 35 Roll 270, pitch 270 degrees
 * @value 36 Roll 90, pitch 180, yaw 90 degrees
 * @value 37 Roll 90, yaw 270 degrees
 * @value 38 Roll 90, pitch 68, yaw 293 degrees
 * @value 39 Pitch 315 degrees
 * @value 40 Roll 90, pitch 315 degrees
 * @category System
 * @qgc_required
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG0_ROT, 0);
