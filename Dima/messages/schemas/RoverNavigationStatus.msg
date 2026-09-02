# SPDX-License-Identifier: BSD-3-Clause
# Rover AUTO 导航状态。该权威 schema 由 uORB 工具生成 Topic/ID/registry，
# 禁止在消费者中复制字段表或手写消息列表。
uint8 ORB_QUEUE_LENGTH = 8

uint32 MESSAGE_VERSION = 0

uint8 CONTROL_INACTIVE = 0
uint8 CONTROL_DRIVING = 1
uint8 CONTROL_STOPPING_FOR_TURN = 2
uint8 CONTROL_SPOT_TURNING = 3
uint8 CONTROL_HOLD = 4

uint8 FAILURE_NONE = 0
uint8 FAILURE_NOT_AUTO = 1
uint8 FAILURE_MISSION_UNAVAILABLE = 2
uint8 FAILURE_PARAMETER_INVALID = 3
uint8 FAILURE_ESTIMATOR_UNHEALTHY = 4
uint8 FAILURE_ESTIMATOR_STALE = 5
uint8 FAILURE_ESTIMATOR_RESET = 6
uint8 FAILURE_TIME_REGRESSION = 7
uint8 FAILURE_OUTPUT_NONFINITE = 8

uint8 WAYPOINT_ACTIVE = 0
uint8 WAYPOINT_INSIDE_ACCEPTANCE = 1
uint8 WAYPOINT_REACHED = 2
uint8 WAYPOINT_MISSION_COMPLETE = 3

uint64 timestamp                 # 发布时刻 [us]
uint64 timestamp_sample          # 所有估计输入中最旧样本时刻 [us]
uint32 mission_generation        # 与 MAVLink mission_id 一致的任务代际
uint16 mission_current
uint16 mission_count
uint8 control_state
uint8 failure_reason
uint8 waypoint_state
bool request_valid
bool ready_for_auto
bool mission_committed
bool parameters_valid
bool estimator_healthy
float32 distance_to_waypoint_m
float32 crosstrack_error_m
float32 lookahead_distance_m
float32 heading_error_rad
float32 speed_setpoint_m_s
float32 yaw_rate_setpoint_rad_s
