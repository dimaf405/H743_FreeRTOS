# SPDX-License-Identifier: BSD-3-Clause
# PX4 v1.17.0 SensorGps compatibility contract.
@queue 1

uint8 FIX_TYPE_NONE = 1
uint8 FIX_TYPE_2D = 2
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT = 5
uint8 FIX_TYPE_RTK_FIXED = 6
uint8 FIX_TYPE_EXTRAPOLATED = 8
uint8 JAMMING_STATE_UNKNOWN = 0
uint8 JAMMING_STATE_OK = 1
uint8 JAMMING_STATE_MITIGATED = 2
uint8 JAMMING_STATE_DETECTED = 3
uint8 SPOOFING_STATE_UNKNOWN = 0
uint8 SPOOFING_STATE_OK = 1
uint8 SPOOFING_STATE_MITIGATED = 2
uint8 SPOOFING_STATE_DETECTED = 3
uint8 AUTHENTICATION_STATE_UNKNOWN = 0
uint8 AUTHENTICATION_STATE_INITIALIZING = 1
uint8 AUTHENTICATION_STATE_ERROR = 2
uint8 AUTHENTICATION_STATE_OK = 3
uint8 AUTHENTICATION_STATE_DISABLED = 4
uint32 SYSTEM_ERROR_OK = 0
uint32 SYSTEM_ERROR_INCOMING_CORRECTIONS = 1
uint32 SYSTEM_ERROR_CONFIGURATION = 2
uint32 SYSTEM_ERROR_SOFTWARE = 4
uint32 SYSTEM_ERROR_ANTENNA = 8
uint32 SYSTEM_ERROR_EVENT_CONGESTION = 16
uint32 SYSTEM_ERROR_CPU_OVERLOAD = 32
uint32 SYSTEM_ERROR_OUTPUT_CONGESTION = 64
uint8 RTCM_MSG_USED_UNKNOWN = 0
uint8 RTCM_MSG_USED_NOT_USED = 1
uint8 RTCM_MSG_USED_USED = 2

uint64 timestamp
uint64 timestamp_sample
uint32 device_id
float64 latitude_deg
float64 longitude_deg
float64 altitude_msl_m
float64 altitude_ellipsoid_m
float32 s_variance_m_s
float32 c_variance_rad
uint8 fix_type
float32 eph
float32 epv
float32 hdop
float32 vdop
int32 noise_per_ms
uint16 automatic_gain_control
uint8 jamming_state
int32 jamming_indicator
uint8 spoofing_state
uint8 authentication_state
float32 vel_m_s
float32 vel_n_m_s
float32 vel_e_m_s
float32 vel_d_m_s
float32 cog_rad
bool vel_ned_valid
int32 timestamp_time_relative
uint64 time_utc_usec
uint8 satellites_used
uint32 system_error
float32 heading
float32 heading_offset
float32 heading_accuracy
float32 rtcm_injection_rate
uint8 selected_rtcm_instance
bool rtcm_crc_failed
uint8 rtcm_msg_used
