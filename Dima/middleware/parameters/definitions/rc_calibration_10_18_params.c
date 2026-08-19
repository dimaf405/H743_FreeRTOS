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
 * @file rc_calibration_10_18_params.c
 *
 * Dima RC 通道 10～18 的校准参数。
 * 参数定义及其元数据保持原样，仅收敛文件职责。
 */

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
