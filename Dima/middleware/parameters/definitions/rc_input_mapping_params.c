/****************************************************************************
 *
 *   Copyright (c) 2012-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file rc_input_mapping_params.c
 *
 * Dima RC 输入策略、通道功能映射及安全阈值参数。
 * 参数定义及其元数据保持原样，仅收敛文件职责。
 */

/** RC 输入协议：0 禁用，2 SBUS。
 * @min 0
 * @max 2
 * @value 0 Disabled
 * @value 2 SBUS
 * @reboot_required true
 * @group Radio Configuration
 */
PARAM_DEFINE_INT32(RC_INPUT_PROTO, 2);

/**
 * Manual control input source policy.
 *
 * Dima currently has one production manual-control source: the local RC/SBUS
 * pipeline.  Values which select MAVLink or disable RC are deliberately not
 * accepted until those control-source paths exist.
 *
 * @min 0
 * @max 0
 * @value 0 RC only
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 0);

/** 已完成校准的 RC 通道数。
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_CHAN_CNT, 0);

/** Roll 对应物理通道，0 表示未映射。
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_ROLL, 0);
/** Pitch 对应物理通道。
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_PITCH, 0);
/** Throttle 对应物理通道；Dima 保持中心双向语义。
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 1);
/** Yaw 对应物理通道。
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_YAW, 2);
/** Arm 开关通道，默认不映射。
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_ARM_SW, 0);
/** Kill 开关通道，默认不映射。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_KILL_SW, 0);
/** Dima has no selectable flight modes; fixed PX4/QGC compatibility handle.
 * @min 0
 * @max 0
 * @value 0 Disabled
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_FLTMODE, 0);
/** Flaps 对应物理通道，0 表示未映射。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_FLAPS, 0);
/** Aux1 通道。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_AUX1, 0);
/** Aux2 通道。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_AUX2, 0);
/** Aux3 通道。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_AUX3, 0);
/** Aux4 通道。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_AUX4, 0);
/** Aux5 通道。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_AUX5, 0);
/** Aux6 通道。
 * @min 0
 * @max 18
 * @group RC Mapping
 */
PARAM_DEFINE_INT32(RC_MAP_AUX6, 0);
/** Arm 开关触发阈值；负值表示反向比较。
 * @min -1
 * @max 1
 * @increment 0.01
 * @group RC Mapping
 */
PARAM_DEFINE_FLOAT(RC_ARMSWITCH_TH, 0.75f);
/** Kill 开关触发阈值。
 * @min 0
 * @max 1
 * @increment 0.01
 * @group RC Mapping
 */
PARAM_DEFINE_FLOAT(RC_KILLSWITCH_TH, 0.75f);
/** RC 信号丢失超时。
 * @unit s
 * @min 0.1
 * @max 35
 * @increment 0.1
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);
