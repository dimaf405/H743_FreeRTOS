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
 * @file rc_calibration_1_9_params.c
 *
 * Dima RC 通道 1～9 的校准参数。
 * 参数定义及其元数据保持原样，仅收敛文件职责。
 */

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
