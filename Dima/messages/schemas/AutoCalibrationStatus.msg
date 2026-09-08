# SPDX-License-Identifier: BSD-3-Clause
# Rover 自动校准状态机的唯一状态/结果/故障枚举权威源。
# ENUM_LABELS STATE RESULT FAILURE FENCE GAIN
uint8 ORB_QUEUE_LENGTH = 8

uint32 MESSAGE_VERSION = 0

uint8 STATE_IDLE = 0
uint8 STATE_PREFLIGHT_CHECK = 1
uint8 STATE_LEVEL_HOLD = 2
uint8 STATE_COMMIT_LEVEL = 3
uint8 STATE_RTK_BASELINE_COLLECT = 4
uint8 STATE_WAIT_ARM_FIRST = 5
uint8 STATE_STRAIGHT_OUT = 6
uint8 STATE_TURN_AROUND = 7
uint8 STATE_STRAIGHT_BACK = 8
uint8 STATE_STOP_DISARM_FIRST = 9
uint8 STATE_COMMIT_RTK = 10
uint8 STATE_WAIT_RTK_RELOCK = 11
uint8 STATE_WAIT_ARM_SECOND = 12
uint8 STATE_TURN_CW = 13
uint8 STATE_TURN_CCW = 14
uint8 STATE_STOP_DISARM_SECOND = 16
uint8 STATE_APPLY_MAG_BOOTSTRAP = 17
uint8 STATE_COMMIT_DYNAMICS = 18
uint8 STATE_COMMIT_MAG = 20
uint8 STATE_RESTORE_MAG = 21
uint8 STATE_WAIT_ARM_IDENTIFICATION = 30
uint8 STATE_IDENTIFY_SPEED = 31
uint8 STATE_IDENTIFY_RATE = 32
uint8 STATE_STOP_IDENTIFICATION = 33
uint8 STATE_APPLY_GAINS = 34
uint8 STATE_WAIT_ARM_VALIDATION = 35
uint8 STATE_VALIDATE_SPEED = 36
uint8 STATE_VALIDATE_RATE = 37
uint8 STATE_VALIDATE_HEADING = 38
uint8 STATE_VALIDATE_PATH = 39
uint8 STATE_STOP_VALIDATION = 40
uint8 STATE_SAVE_GAINS = 41
uint8 STATE_RESTORE_GAINS = 42
uint8 STATE_WAIT_ARM_PROFILE = 43
uint8 STATE_PROFILE_FORWARD = 44
uint8 STATE_PROFILE_REVERSE = 45
uint8 STATE_PROFILE_FULL = 46
uint8 STATE_STOP_PROFILE = 47
uint8 STATE_COMMIT_IMU = 48
uint8 STATE_WAIT_IMU_RELOCK = 49
uint8 STATE_VALIDATE_REVERSE = 50
uint8 STATE_PROFILE_RATE_CW = 51
uint8 STATE_PROFILE_RATE_CCW = 52
uint8 STATE_APPLY_RUNTIME = 53
uint8 STATE_VALIDATE_DRIVING = 54

uint8 RESULT_NONE = 0
uint8 RESULT_RUNNING = 1
uint8 RESULT_SUCCESS = 2
uint8 RESULT_PARTIAL = 3
uint8 RESULT_FAILED = 4
uint8 RESULT_CANCELLED = 5

uint8 FAILURE_NONE = 0
uint8 FAILURE_BUSY = 1
uint8 FAILURE_ARMED_AT_ENTRY = 2
uint8 FAILURE_PREFLIGHT = 3
uint8 FAILURE_SENSOR_STALE = 4
uint8 FAILURE_NOT_STATIONARY = 5
uint8 FAILURE_RTK_QUALITY = 6
uint8 FAILURE_RTK_INCONSISTENT = 7
uint8 FAILURE_MOTION_ENVELOPE = 8
uint8 FAILURE_CONTROL_LOSS = 9
uint8 FAILURE_ESTIMATOR = 10
uint8 FAILURE_PARAMETER = 11
uint8 FAILURE_FRONTEND_CONFIRMATION = 12
uint8 FAILURE_MECHANICAL_ASYMMETRY = 13
uint8 FAILURE_MAG_DISTURBED = 14
uint8 FAILURE_TIMEOUT = 15
uint8 FAILURE_OPERATOR_CANCEL = 16
uint8 FAILURE_DYNAMICS_UNOBSERVABLE = 17
uint8 FAILURE_ENTRY_POSITION = 18
uint8 FAILURE_FENCE_SPACE = 19
uint8 FAILURE_FENCE_BOUNDARY = 20
uint8 FAILURE_STOP_DISTANCE = 21
uint8 FAILURE_IDENTIFICATION = 22
uint8 FAILURE_GAIN_VALIDATION = 23
uint8 FAILURE_MOTION_UNAVAILABLE = 24
uint8 FAILURE_PATH_UNOBSERVABLE = 25
uint8 FAILURE_PROFILE_UNOBSERVABLE = 26
uint8 FAILURE_IMU_BIAS_UNOBSERVABLE = 27

uint8 FENCE_UNAVAILABLE = 0
uint8 FENCE_SAFE = 1
uint8 FENCE_STOP_MARGIN = 2
uint8 FENCE_OUTSIDE = 3
uint8 FENCE_POSITION_LOST = 4
uint8 FENCE_STOP_UNKNOWN = 5

uint8 GAIN_NONE = 0
uint8 GAIN_INNER = 1
uint8 GAIN_HEADING = 2
uint8 GAIN_PATH = 3
uint8 GAIN_NAVIGATION = 4

uint32 STAGE_LEVEL = 1
uint32 STAGE_RTK = 2
uint32 STAGE_SPEED = 4
uint32 STAGE_YAW = 8
uint32 STAGE_MAG = 16
uint32 STAGE_MAG_SKIPPED = 32
uint32 STAGE_INNER_GAINS = 64
uint32 STAGE_HEADING_GAIN = 128
uint32 STAGE_PATH_GAIN = 256
uint32 STAGE_IMU_BIAS = 512
uint32 STAGE_MOTOR_PROFILE = 1024
uint32 STAGE_RUNTIME = 2048
uint32 STAGE_NAV_STRATEGY = 4096
uint32 NAV_FIELD_LOOKAHEAD = 1
uint32 NAV_FIELD_JERK = 2
uint32 NAV_FIELD_SPEED_REDUCTION = 4
uint32 NAV_FIELD_TRANSITIONS = 8

uint64 timestamp
uint64 timestamp_sample
uint32 session_id
uint32 sequence
uint32 completed_stages
uint32 unavailable_stages
uint32 provisional_validated_stages
uint8 state
uint8 result
uint8 failure_reason
uint8 progress
bool active
bool motion_allowed
bool ready_for_entry
bool awaiting_arm
bool session_authorized # 协调器观察到首次 Arm 后的诊断镜像；实际授权只属于 Commander。
bool full_output_requested
bool full_output_reached
bool magnetometer_present
bool mag_bootstrap_active
bool closed_loop
bool gains_provisional
bool fence_center_valid
uint8 fence_state
uint8 gain_group
uint64 fence_center_timestamp
uint32 fence_device_id
float64 fence_latitude_deg
float64 fence_longitude_deg
float32 fence_origin_error_m
float32 fence_radius_m
float32 fence_stop_distance_m
float32 fence_distance_m
float32 fence_margin_m
float32 fence_working_radius_m
float32 session_speed_limit_m_s
float32 session_motor_limit
float32 entry_cruise_speed_m_s
float32 observed_forward_speed_m_s
float32 observed_reverse_speed_m_s
float32 observed_output_max
float32 level_roll_offset_deg
float32 level_pitch_offset_deg
float32 rtk_baseline_m
float32 rtk_yaw_offset_deg
float32 maximum_speed_m_s
float32 yaw_rate_correction
float32 speed_p
float32 speed_i
float32 yaw_rate_p
float32 yaw_rate_i
float32 heading_p
float32 lookahead_gain
float32 validation_error
uint32 navigation_observed_fields
