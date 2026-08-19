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
 * @file rc_params.c
 *
 * Dima RC 参数。校准、映射和阈值基于 PX4 v1.17.0 RCUpdate/ManualControl/Commander，
 * 并增加 STM32H743 SBUS UART 端口与按协议自动硬件反相配置。
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
/** RC 通道 1 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MIN, 1000.0f);

/** RC 通道 1 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_TRIM, 1500.0f);

/** RC 通道 1 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MAX, 2000.0f);

/** RC 通道 1 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_REV, 1.0f);

/** RC 通道 1 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_DZ, 0.0f);
/** RC 通道 2 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MIN, 1000.0f);

/** RC 通道 2 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_TRIM, 1500.0f);

/** RC 通道 2 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MAX, 2000.0f);

/** RC 通道 2 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_REV, 1.0f);

/** RC 通道 2 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_DZ, 0.0f);
/** RC 通道 3 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MIN, 1000.0f);

/** RC 通道 3 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_TRIM, 1500.0f);

/** RC 通道 3 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MAX, 2000.0f);

/** RC 通道 3 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_REV, 1.0f);

/** RC 通道 3 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_DZ, 0.0f);
/** RC 通道 4 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MIN, 1000.0f);

/** RC 通道 4 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_TRIM, 1500.0f);

/** RC 通道 4 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MAX, 2000.0f);

/** RC 通道 4 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_REV, 1.0f);

/** RC 通道 4 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_DZ, 0.0f);
/** RC 通道 5 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MIN, 1000.0f);

/** RC 通道 5 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_TRIM, 1500.0f);

/** RC 通道 5 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MAX, 2000.0f);

/** RC 通道 5 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_REV, 1.0f);

/** RC 通道 5 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_DZ, 0.0f);
/** RC 通道 6 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MIN, 1000.0f);

/** RC 通道 6 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_TRIM, 1500.0f);

/** RC 通道 6 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MAX, 2000.0f);

/** RC 通道 6 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_REV, 1.0f);

/** RC 通道 6 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_DZ, 0.0f);
/** RC 通道 7 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MIN, 1000.0f);

/** RC 通道 7 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_TRIM, 1500.0f);

/** RC 通道 7 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MAX, 2000.0f);

/** RC 通道 7 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_REV, 1.0f);

/** RC 通道 7 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_DZ, 0.0f);
/** RC 通道 8 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MIN, 1000.0f);

/** RC 通道 8 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_TRIM, 1500.0f);

/** RC 通道 8 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MAX, 2000.0f);

/** RC 通道 8 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_REV, 1.0f);

/** RC 通道 8 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_DZ, 0.0f);
/** RC 通道 9 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MIN, 1000.0f);

/** RC 通道 9 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_TRIM, 1500.0f);

/** RC 通道 9 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MAX, 2000.0f);

/** RC 通道 9 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_REV, 1.0f);

/** RC 通道 9 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_DZ, 0.0f);
/** RC 通道 10 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MIN, 1000.0f);

/** RC 通道 10 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_TRIM, 1500.0f);

/** RC 通道 10 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MAX, 2000.0f);

/** RC 通道 10 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_REV, 1.0f);

/** RC 通道 10 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_DZ, 0.0f);
/** RC 通道 11 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MIN, 1000.0f);

/** RC 通道 11 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_TRIM, 1500.0f);

/** RC 通道 11 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MAX, 2000.0f);

/** RC 通道 11 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_REV, 1.0f);

/** RC 通道 11 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_DZ, 0.0f);
/** RC 通道 12 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MIN, 1000.0f);

/** RC 通道 12 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_TRIM, 1500.0f);

/** RC 通道 12 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MAX, 2000.0f);

/** RC 通道 12 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_REV, 1.0f);

/** RC 通道 12 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_DZ, 0.0f);
/** RC 通道 13 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MIN, 1000.0f);

/** RC 通道 13 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_TRIM, 1500.0f);

/** RC 通道 13 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MAX, 2000.0f);

/** RC 通道 13 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_REV, 1.0f);

/** RC 通道 13 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_DZ, 0.0f);
/** RC 通道 14 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MIN, 1000.0f);

/** RC 通道 14 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_TRIM, 1500.0f);

/** RC 通道 14 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MAX, 2000.0f);

/** RC 通道 14 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_REV, 1.0f);

/** RC 通道 14 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_DZ, 0.0f);
/** RC 通道 15 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MIN, 1000.0f);

/** RC 通道 15 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_TRIM, 1500.0f);

/** RC 通道 15 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MAX, 2000.0f);

/** RC 通道 15 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_REV, 1.0f);

/** RC 通道 15 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_DZ, 0.0f);
/** RC 通道 16 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MIN, 1000.0f);

/** RC 通道 16 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_TRIM, 1500.0f);

/** RC 通道 16 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MAX, 2000.0f);

/** RC 通道 16 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_REV, 1.0f);

/** RC 通道 16 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_DZ, 0.0f);
/** RC 通道 17 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MIN, 1000.0f);

/** RC 通道 17 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_TRIM, 1500.0f);

/** RC 通道 17 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MAX, 2000.0f);

/** RC 通道 17 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_REV, 1.0f);

/** RC 通道 17 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_DZ, 0.0f);
/** RC 通道 18 最小脉宽。
 * @unit us
 * @min 800
 * @max 1500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MIN, 1000.0f);

/** RC 通道 18 中位脉宽。
 * @unit us
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_TRIM, 1500.0f);

/** RC 通道 18 最大脉宽。
 * @unit us
 * @min 1500
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MAX, 2000.0f);

/** RC 通道 18 方向，-1 反向，1 正向。
 * @min -1
 * @max 1
 * @value -1 Reverse
 * @value 1 Normal
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_REV, 1.0f);

/** RC 通道 18 中位死区半宽。
 * @unit us
 * @min 0
 * @max 500
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_DZ, 0.0f);
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
