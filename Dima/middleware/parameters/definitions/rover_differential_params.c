/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * Wheel track
 *
 * Distance from the center of the right wheel to the center of the left wheel.
 *
 * @unit m
 * @min 0
 * @max 100
 * @increment 0.001
 * @decimal 3
 * @group Rover Differential
 */
PARAM_DEFINE_FLOAT(RD_WHEEL_TRACK, 0.f);

/**
 * Yaw error threshold to switch from spot turning to driving
 *
 * @unit rad
 * @min 0.001
 * @max 3.14159
 * @increment 0.01
 * @decimal 3
 * @group Rover Differential
 */
PARAM_DEFINE_FLOAT(RD_TRANS_TRN_DRV, 0.0872665f);

/**
 * Yaw error threshold to switch from driving to spot turning
 *
 * @unit rad
 * @min 0.001
 * @max 3.14159
 * @increment 0.01
 * @decimal 3
 * @group Rover Differential
 */
PARAM_DEFINE_FLOAT(RD_TRANS_DRV_TRN, 0.174533f);

/**
 * Yaw stick gain for Manual mode
 *
 * Assign value below 1.0 to decrease stick response for yaw control.
 *
 * @min 0.1
 * @max 1
 * @increment 0.01
 * @decimal 3
 * @group Rover Differential
 */
PARAM_DEFINE_FLOAT(RD_YAW_STK_GAIN, 1.f);
