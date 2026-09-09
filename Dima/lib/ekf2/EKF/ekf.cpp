/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	if (!_initialised) {
		_initialised = initialise_interface(timestamp);
		reset();
	}

	return _initialised;
}

void Ekf::reset()
{
	ECL_INFO("reset");

	_state.quat_nominal.setIdentity();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.accel_bias.setZero();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I.setZero();
	_state.mag_B.setZero();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel.setZero();
#endif // CONFIG_EKF2_WIND
	//
#if defined(CONFIG_EKF2_TERRAIN)
	// assume a ground clearance
	_state.terrain = -_gpos.altitude() + _params.ekf2_min_rng;
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	_range_sensor.setPitchOffset(_params.ekf2_rng_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.ekf2_rng_qlty_t);
	_range_sensor.setMaxFogDistance(_params.ekf2_rng_fog);
#endif // CONFIG_EKF2_RANGE_FINDER

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

#if defined(CONFIG_EKF2_GNSS)
	_gnss_checks.resetHard();
#endif // CONFIG_EKF2_GNSS
	_local_origin_alt = NAN;

	_output_predictor.reset();

	// Ekf private fields
	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;
	_time_last_terrain_fuse = 0;

	_last_known_gpos.setZero();

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_counter = 0;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_mag_counter = 0;
#endif // CONFIG_EKF2_MAGNETOMETER

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;

	for (auto &clip_count : _clip_counter) {
		clip_count = 0;
	}

	_zero_velocity_update.reset();

	updateParameters();
}

bool Ekf::update()
{
	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		// protect against zero data
		if (imu_sample_delayed.delta_vel_dt < 1e-4f || imu_sample_delayed.delta_ang_dt < 1e-4f) {
			return false;
		}

		// calculate an average filter update time
		// limit input between -50% and +100% of nominal value
		const float filter_update_s = 1e-6f * _params.ekf2_predict_us;
		const float input = math::constrain(0.5f * (imu_sample_delayed.delta_vel_dt + imu_sample_delayed.delta_ang_dt),
						    0.5f * filter_update_s,
						    2.f * filter_update_s);

		if (_is_first_imu_sample) {
			_accel_lpf.reset(imu_sample_delayed.delta_vel / imu_sample_delayed.delta_vel_dt);
			_gyro_lpf.reset(imu_sample_delayed.delta_ang / imu_sample_delayed.delta_ang_dt);
			_dt_ekf_avg = input;

			_is_first_imu_sample = false;

		} else {
			_accel_lpf.update(imu_sample_delayed.delta_vel / imu_sample_delayed.delta_vel_dt);
			_gyro_lpf.update(imu_sample_delayed.delta_ang / imu_sample_delayed.delta_ang_dt);
			_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;
		}

		if (!_filter_initialised) {
			_filter_initialised = initialiseFilter();

			if (!_filter_initialised) {
				return false;
			}
		}

		updateIMUBiasInhibit(imu_sample_delayed);

		// perform state and covariance prediction for the main filter
		predictCovariance(imu_sample_delayed);
		predictState(imu_sample_delayed);

		// control fusion of observation data
		controlFusionModes(imu_sample_delayed);

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, _state.quat_nominal, _state.vel, _gpos,
						      _state.gyro_bias, _state.accel_bias);

		return true;
	}

	return false;
}

bool Ekf::initialiseFilter()
{
	if (!initialiseTilt()) {
		return false;
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// reset the output predictor state history to match the EKF initial values
	_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _gpos);

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial tilt estimate from delta velocity vector, assuming vehicle is static
	_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState(const imuSample &imu_delayed)
{
	if (std::fabs(_gpos.latitude_rad() - _earth_rate_lat_ref_rad) > math::radians(1.0)) {
		_earth_rate_lat_ref_rad = _gpos.latitude_rad();
		_earth_rate_NED = calcEarthRateNED((float)_earth_rate_lat_ref_rad);
	}

	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity, Coriolis and transport rate
	const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G); // simplistic model
	const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);
	const Vector3f transport_rate = -_gpos.computeAngularRateNavFrame(vel_last).cross(vel_last);
	_state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
	_state.pos(2) = -_gpos.altitude();

	// constrain states
	_state.vel = matrix::constrain(_state.vel, -_params.ekf2_vel_lim, _params.ekf2_vel_lim);

	// calculate a filtered horizontal acceleration this are used for manoeuvre detection elsewhere
	_accel_horiz_lpf.update(corrected_delta_vel_ef.xy() / imu_delayed.delta_vel_dt, imu_delayed.delta_vel_dt);
}

bool Ekf::resetGlobalPosToExternalObservation(const double latitude, const double longitude, const float altitude,
		const float eph,
		const float epv, uint64_t timestamp_observation)
{
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized()) {
		if (!resetLatLonTo(latitude, longitude, sq(eph))) {
			return false;
		}

		initialiseAltitudeTo(altitude, sq(epv));

		return true;
	}

	Vector3f pos_correction;

	// apply a first order correction using velocity at the delayed time horizon and the delta time
	if ((timestamp_observation > 0) && isLocalHorizontalPositionValid()) {

		timestamp_observation = math::min(_time_latest_us, timestamp_observation);

		float dt_us;

		if (_time_delayed_us >= timestamp_observation) {
			dt_us = static_cast<float>(_time_delayed_us - timestamp_observation);

		} else {
			dt_us = -static_cast<float>(timestamp_observation - _time_delayed_us);
		}

		const float dt_s = dt_us * 1e-6f;
		pos_correction = _state.vel * dt_s;
	}

	LatLonAlt gpos(latitude, longitude, altitude);
	bool alt_valid = true;

	if (!checkAltitudeValidity(gpos.altitude())) {
		gpos.setAltitude(_gpos.altitude());
		alt_valid = false;
	}

	const LatLonAlt gpos_corrected = gpos + pos_correction;

	{
		const float obs_var = math::max(sq(eph), sq(0.01f));

		const Vector2f innov = (_gpos - gpos_corrected).xy();
		const Vector2f innov_var = Vector2f(getStateVariance<State::pos>()) + obs_var;

		const float sq_gate = sq(5.f); // magic hardcoded gate
		const float test_ratio = sq(innov(0)) / (sq_gate * innov_var(0)) + sq(innov(1)) / (sq_gate * innov_var(1));

		const bool innov_rejected = (test_ratio > 1.f);

		if (!_control_status.flags.in_air || (eph > 0.f && eph < 1.f) || innov_rejected) {
			// When on ground or accuracy chosen to be very low, we hard reset position
			// this allows the user to still send hard resets at any time
			// Also reset when another position source is active as it it would otherwise have almost no
			// visible effect to the position estimate.
			ECL_INFO("reset position to external observation");
			_information_events.flags.reset_pos_to_ext_obs = true;

			resetHorizontalPositionTo(gpos_corrected.latitude_deg(), gpos_corrected.longitude_deg(), obs_var);
			_last_known_gpos.setLatLon(gpos_corrected);

		} else {
			ECL_INFO("fuse external observation as position measurement");

			VectorState H;
			VectorState K;

			for (unsigned index = 0; index < 2; index++) {
				K = VectorState(P.row(State::pos.idx + index)) / innov_var(index);
				H(State::pos.idx + index) = 1.f;

				// Artificially set the position Kalman gain to 1 in order to force a reset
				// of the position through fusion. This allows the EKF to use part of the information
				// to continue learning the correlated states (e.g.: velocity, heading, wind) while
				// performing a position reset.
				K(State::pos.idx + index) = 1.f;
				measurementUpdate(K, H, obs_var, innov(index));
				H(State::pos.idx + index) = 0.f; // Reset the whole vector to 0
			}

			// Use the reset counters to inform the controllers about a position jump
			updateHorizontalPositionResetStatus(-innov);

			// Reset the positon of the output predictor to avoid a transient that would disturb the
			// position controller
			_output_predictor.resetLatLonTo(_gpos.latitude_deg(), _gpos.longitude_deg());

			_time_last_hor_pos_fuse = _time_delayed_us;
			_last_known_gpos.setLatLon(gpos_corrected);
		}
	}

	if (alt_valid) {
		const float obs_var = math::max(sq(epv), sq(0.01f));

		ECL_INFO("reset height to external observation");
		initialiseAltitudeTo(gpos_corrected.altitude(), obs_var);
		_last_known_gpos.setAltitude(gpos_corrected.altitude());
	}

	return true;
}

void Ekf::updateParameters()
{
	_params.ekf2_gyr_noise = math::constrain(_params.ekf2_gyr_noise, 0.f, 1.f);
	_params.ekf2_acc_noise = math::constrain(_params.ekf2_acc_noise, 0.f, 1.f);

	_params.ekf2_gyr_b_noise = math::constrain(_params.ekf2_gyr_b_noise, 0.f, 1.f);
	_params.ekf2_acc_b_noise = math::constrain(_params.ekf2_acc_b_noise, 0.f, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_params.ekf2_mag_e_noise = math::constrain(_params.ekf2_mag_e_noise, 0.f, 1.f);
	_params.ekf2_mag_b_noise = math::constrain(_params.ekf2_mag_b_noise, 0.f, 1.f);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_params.ekf2_wind_nsd = math::constrain(_params.ekf2_wind_nsd, 0.f, 1.f);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.updateParameters();
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
}

template<typename T>
static void printRingBuffer(const char *name, RingBuffer<T> *rb)
{
	if (rb) {
		printf("%s: %d/%d entries (%d/%d Bytes) (%zu Bytes per entry)\n",
		       name,
		       rb->entries(), rb->get_length(), rb->get_used_size(), rb->get_total_size(),
		       sizeof(T));
	}
}

void Ekf::print_status()
{
	printf("\nStates: (%.4f seconds ago)\n", (_time_latest_us - _time_delayed_us) * 1e-6);
	printf("Orientation (%d-%d): [%.3f, %.3f, %.3f, %.3f] (Euler [%.1f, %.1f, %.1f] deg) var: [%.1e, %.1e, %.1e]\n",
	       State::quat_nominal.idx, State::quat_nominal.idx + State::quat_nominal.dof - 1,
	       (double)_state.quat_nominal(0), (double)_state.quat_nominal(1), (double)_state.quat_nominal(2),
	       (double)_state.quat_nominal(3),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).phi()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).theta()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).psi()),
	       (double)getStateVariance<State::quat_nominal>()(0), (double)getStateVariance<State::quat_nominal>()(1),
	       (double)getStateVariance<State::quat_nominal>()(2)
	      );

	printf("Velocity (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::vel.idx, State::vel.idx + State::vel.dof - 1,
	       (double)_state.vel(0), (double)_state.vel(1), (double)_state.vel(2),
	       (double)getStateVariance<State::vel>()(0), (double)getStateVariance<State::vel>()(1),
	       (double)getStateVariance<State::vel>()(2)
	      );

	const Vector3f position = getPosition();
	printf("Position (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::pos.idx, State::pos.idx + State::pos.dof - 1,
	       (double)position(0), (double)position(1), (double) position(2),
	       (double)getStateVariance<State::pos>()(0), (double)getStateVariance<State::pos>()(1),
	       (double)getStateVariance<State::pos>()(2)
	      );

	printf("Gyro Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::gyro_bias.idx, State::gyro_bias.idx + State::gyro_bias.dof - 1,
	       (double)_state.gyro_bias(0), (double)_state.gyro_bias(1), (double)_state.gyro_bias(2),
	       (double)getStateVariance<State::gyro_bias>()(0), (double)getStateVariance<State::gyro_bias>()(1),
	       (double)getStateVariance<State::gyro_bias>()(2)
	      );

	printf("Accel Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::accel_bias.idx, State::accel_bias.idx + State::accel_bias.dof - 1,
	       (double)_state.accel_bias(0), (double)_state.accel_bias(1), (double)_state.accel_bias(2),
	       (double)getStateVariance<State::accel_bias>()(0), (double)getStateVariance<State::accel_bias>()(1),
	       (double)getStateVariance<State::accel_bias>()(2)
	      );

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printf("Magnetic Field (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_I.idx, State::mag_I.idx + State::mag_I.dof - 1,
	       (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
	       (double)getStateVariance<State::mag_I>()(0), (double)getStateVariance<State::mag_I>()(1),
	       (double)getStateVariance<State::mag_I>()(2)
	      );

	printf("Magnetic Bias (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_B.idx, State::mag_B.idx + State::mag_B.dof - 1,
	       (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2),
	       (double)getStateVariance<State::mag_B>()(0), (double)getStateVariance<State::mag_B>()(1),
	       (double)getStateVariance<State::mag_B>()(2)
	      );
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	printf("Wind velocity (%d-%d): [%.3f, %.3f] var: [%.1e, %.1e]\n",
	       State::wind_vel.idx, State::wind_vel.idx + State::wind_vel.dof - 1,
	       (double)_state.wind_vel(0), (double)_state.wind_vel(1),
	       (double)getStateVariance<State::wind_vel>()(0), (double)getStateVariance<State::wind_vel>()(1)
	      );
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	printf("Terrain position (%d): %.3f var: %.1e\n",
	       State::terrain.idx,
	       (double)_state.terrain,
	       (double)getStateVariance<State::terrain>()(0)
	      );
#endif // CONFIG_EKF2_TERRAIN

	printf("\nP:\n");
	P.print();

	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);
	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	printRingBuffer("IMU buffer", &_imu_buffer);
	printRingBuffer("system flag buffer", _system_flag_buffer);

#if defined(CONFIG_EKF2_AIRSPEED)
	printRingBuffer("airspeed buffer", _airspeed_buffer);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
	printRingBuffer("aux vel buffer", _auxvel_buffer);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
	printRingBuffer("baro buffer", _baro_buffer);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	printRingBuffer("drag buffer", _drag_buffer);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	printRingBuffer("ext vision buffer", _ext_vision_buffer);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	printRingBuffer("gps buffer", _gps_buffer);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printRingBuffer("mag buffer", _mag_buffer);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	printRingBuffer("flow buffer", _flow_buffer);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	printRingBuffer("range buffer", _range_buffer);
#endif // CONFIG_EKF2_RANGE_FINDER


	_output_predictor.print_status();
}


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

Ekf::Ekf()
{
	reset();
}

const StateSample & Ekf::state() const
{ return _state; }

#if defined(CONFIG_EKF2_BAROMETER)
const estimator_aid_source1d_s & Ekf::aid_src_baro_hgt() const
{ return _aid_src_baro_hgt; }
#endif

#if defined(CONFIG_EKF2_BAROMETER)
const BiasEstimator::status & Ekf::getBaroBiasEstimatorStatus() const
{ return _baro_b_est.getStatus(); }
#endif

#if defined(CONFIG_EKF2_TERRAIN)
bool Ekf::isTerrainEstimateValid() const
{ return _terrain_valid; }
#endif

#if defined(CONFIG_EKF2_TERRAIN)
float Ekf::getTerrainVertPos() const
{ return _state.terrain + getEkfGlobalOriginAltitude(); }
#endif

#if defined(CONFIG_EKF2_TERRAIN)
float Ekf::getHagl() const
{ return _state.terrain + _gpos.altitude(); }
#endif

#if defined(CONFIG_EKF2_TERRAIN)
float Ekf::getTerrainVariance() const
{ return P(State::terrain.idx, State::terrain.idx); }
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)
const estimator_aid_source1d_s & Ekf::aid_src_rng_hgt() const
{ return _aid_src_rng_hgt; }
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)
float Ekf::getHaglRateInnov() const
{ return _rng_consistency_check.getInnov(); }
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)
float Ekf::getHaglRateInnovVar() const
{ return _rng_consistency_check.getInnovVar(); }
#endif

#if defined(CONFIG_EKF2_RANGE_FINDER)
float Ekf::getHaglRateInnovRatio() const
{ return _rng_consistency_check.getSignedTestRatioLpf(); }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const estimator_aid_source2d_s & Ekf::aid_src_optical_flow() const
{ return _aid_src_optical_flow; }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector2f & Ekf::getFlowVelBody() const
{ return _flow_vel_body; }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
Vector2f Ekf::getFlowVelNE() const
{ return Vector2f(_R_to_earth * Vector3f(getFlowVelBody()(0), getFlowVelBody()(1), 0.f)); }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector2f & Ekf::getFilteredFlowVelBody() const
{ return _flow_vel_body_lpf.getState(); }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
Vector2f Ekf::getFilteredFlowVelNE() const
{ return Vector2f(_R_to_earth * Vector3f(getFilteredFlowVelBody()(0), getFilteredFlowVelBody()(1), 0.f)); }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector2f & Ekf::getFlowCompensated() const
{ return _flow_rate_compensated; }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector2f & Ekf::getFlowUncompensated() const
{ return _flow_sample_delayed.flow_rate; }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector3f Ekf::getFlowGyro() const
{ return _flow_sample_delayed.gyro_rate; }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector3f & Ekf::getFlowGyroBias() const
{ return _flow_gyro_bias; }
#endif

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
const Vector3f & Ekf::getFlowRefBodyRate() const
{ return _ref_body_rate; }
#endif

#if defined(CONFIG_EKF2_DRAG_FUSION)
const estimator_aid_source2d_s & Ekf::aid_src_drag() const
{ return _aid_src_drag; }
#endif

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
const estimator_aid_source3d_s & Ekf::aid_src_gravity() const
{ return _aid_src_gravity; }
#endif

#if defined(CONFIG_EKF2_WIND)
const Vector2f & Ekf::getWindVelocity() const
{ return _state.wind_vel; }
#endif

#if defined(CONFIG_EKF2_WIND)
Vector2f Ekf::getWindVelocityVariance() const
{ return getStateVariance<State::wind_vel>(); }
#endif

const matrix::SquareMatrix<float, State::size> & Ekf::covariances() const
{ return P; }

float Ekf::stateCovariance(unsigned r, unsigned c) const
{ return P(r, c); }

matrix::Vector<float, State::size> Ekf::covariances_diagonal() const
{ return P.diag(); }

Vector3f Ekf::getVelocityVariance() const
{ return getStateVariance<State::vel>(); }

Vector3f Ekf::getPositionVariance() const
{ return getStateVariance<State::pos>(); }

bool Ekf::isGlobalHorizontalPositionValid() const
{
	return _local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid();
}

bool Ekf::isGlobalVerticalPositionValid() const
{
	return PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid();
}

bool Ekf::isLocalHorizontalPositionValid() const
{
	return !_horizontal_deadreckon_time_exceeded;
}

bool Ekf::isLocalVerticalPositionValid() const
{
	return !_vertical_position_deadreckon_time_exceeded;
}

bool Ekf::isLocalVerticalVelocityValid() const
{
	return !_vertical_velocity_deadreckon_time_exceeded;
}

bool Ekf::isYawFinalAlignComplete() const
{
#if defined(CONFIG_EKF2_MAGNETOMETER)
	const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg);
	const bool is_mag_alignment_in_flight_complete = is_using_mag
			&& _control_status.flags.mag_aligned_in_flight
			&& ((_time_delayed_us - _flt_mag_align_start_time) > (uint64_t)1e6);
	return _control_status.flags.yaw_align
	       && (is_mag_alignment_in_flight_complete || !is_using_mag);
#else
	return _control_status.flags.yaw_align;
#endif
}

const Vector3f & Ekf::getGyroBias() const
{ return _state.gyro_bias; }

Vector3f Ekf::getGyroBiasVariance() const
{ return getStateVariance<State::gyro_bias>(); }

float Ekf::getGyroBiasLimit() const
{ return _params.ekf2_gyr_b_lim; }

float Ekf::getGyroNoise() const
{ return _params.ekf2_gyr_noise; }

const Vector3f & Ekf::getAccelBias() const
{ return _state.accel_bias; }

Vector3f Ekf::getAccelBiasVariance() const
{ return getStateVariance<State::accel_bias>(); }

float Ekf::getAccelBiasLimit() const
{ return _params.ekf2_abl_lim; }

#if defined(CONFIG_EKF2_MAGNETOMETER)
const Vector3f & Ekf::getMagEarthField() const
{ return _state.mag_I; }
#endif

#if defined(CONFIG_EKF2_MAGNETOMETER)
const Vector3f & Ekf::getMagBias() const
{ return _state.mag_B; }
#endif

#if defined(CONFIG_EKF2_MAGNETOMETER)
Vector3f Ekf::getMagBiasVariance() const
{ return getStateVariance<State::mag_B>(); }
#endif

#if defined(CONFIG_EKF2_MAGNETOMETER)
float Ekf::getMagBiasLimit() const
{ return 0.5f; }
#endif

bool Ekf::accel_bias_inhibited() const
{ return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }

bool Ekf::gyro_bias_inhibited() const
{ return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }

const Ekf::StateResets & Ekf::state_reset_status() const
{ return _state_reset_status; }

uint8_t Ekf::get_posD_reset_count() const
{ return _state_reset_status.reset_count.posD; }

void Ekf::get_posD_reset(float *delta, uint8_t *counter) const
{
	*delta = _state_reset_status.posD_change;
	*counter = _state_reset_status.reset_count.posD;
}

uint8_t Ekf::get_hagl_reset_count() const
{ return _state_reset_status.reset_count.hagl; }

void Ekf::get_hagl_reset(float *delta, uint8_t *counter) const
{
	*delta = _state_reset_status.hagl_change;
	*counter = _state_reset_status.reset_count.hagl;
}

uint8_t Ekf::get_velD_reset_count() const
{ return _state_reset_status.reset_count.velD; }

void Ekf::get_velD_reset(float *delta, uint8_t *counter) const
{
	*delta = _state_reset_status.velD_change;
	*counter = _state_reset_status.reset_count.velD;
}

uint8_t Ekf::get_posNE_reset_count() const
{ return _state_reset_status.reset_count.posNE; }

void Ekf::get_posNE_reset(float delta[2], uint8_t *counter) const
{
	_state_reset_status.posNE_change.copyTo(delta);
	*counter = _state_reset_status.reset_count.posNE;
}

uint8_t Ekf::get_velNE_reset_count() const
{ return _state_reset_status.reset_count.velNE; }

void Ekf::get_velNE_reset(float delta[2], uint8_t *counter) const
{
	_state_reset_status.velNE_change.copyTo(delta);
	*counter = _state_reset_status.reset_count.velNE;
}

uint8_t Ekf::get_quat_reset_count() const
{ return _state_reset_status.reset_count.quat; }

void Ekf::get_quat_reset(float delta_quat[4], uint8_t *counter) const
{
	_state_reset_status.quat_change.copyTo(delta_quat);
	*counter = _state_reset_status.reset_count.quat;
}

HeightSensor Ekf::getHeightSensorRef() const
{ return _height_sensor_ref; }

#if defined(CONFIG_EKF2_AIRSPEED)
const estimator_aid_source1d_s & Ekf::aid_src_airspeed() const
{ return _aid_src_airspeed; }
#endif

#if defined(CONFIG_EKF2_SIDESLIP)
const estimator_aid_source1d_s & Ekf::aid_src_sideslip() const
{ return _aid_src_sideslip; }
#endif

const estimator_aid_source1d_s & Ekf::aid_src_fake_hgt() const
{ return _aid_src_fake_hgt; }

const estimator_aid_source2d_s & Ekf::aid_src_fake_pos() const
{ return _aid_src_fake_pos; }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
const estimator_aid_source1d_s & Ekf::aid_src_ev_hgt() const
{ return _aid_src_ev_hgt; }
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
const estimator_aid_source2d_s & Ekf::aid_src_ev_pos() const
{ return _aid_src_ev_pos; }
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
const estimator_aid_source3d_s & Ekf::aid_src_ev_vel() const
{ return _aid_src_ev_vel; }
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
const estimator_aid_source1d_s & Ekf::aid_src_ev_yaw() const
{ return _aid_src_ev_yaw; }
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
const BiasEstimator::status & Ekf::getEvHgtBiasEstimatorStatus() const
{ return _ev_hgt_b_est.getStatus(); }
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
const BiasEstimator::status & Ekf::getEvPosBiasEstimatorStatus(int i) const
{ return _ev_pos_b_est.getStatus(i); }
#endif

#if defined(CONFIG_EKF2_GNSS)
void Ekf::set_min_required_gps_health_time(uint32_t time_us)
{ _min_gps_health_time_us = time_us; }
#endif

#if defined(CONFIG_EKF2_GNSS)
const GnssChecks::gps_check_fail_status_u & Ekf::gps_check_fail_status() const
{ return _gnss_checks.getFailStatus(); }
#endif

#if defined(CONFIG_EKF2_GNSS)
const decltype(GnssChecks::gps_check_fail_status_u::flags) & Ekf::gps_check_fail_status_flags() const
{ return _gnss_checks.getFailStatus().flags; }
#endif

#if defined(CONFIG_EKF2_GNSS)
bool Ekf::gps_checks_passed() const
{ return _gnss_checks.passed(); }
#endif

#if defined(CONFIG_EKF2_GNSS)
const BiasEstimator::status & Ekf::getGpsHgtBiasEstimatorStatus() const
{ return _gps_hgt_b_est.getStatus(); }
#endif

#if defined(CONFIG_EKF2_GNSS)
const estimator_aid_source1d_s & Ekf::aid_src_gnss_hgt() const
{ return _aid_src_gnss_hgt; }
#endif

#if defined(CONFIG_EKF2_GNSS)
const estimator_aid_source2d_s & Ekf::aid_src_gnss_pos() const
{ return _aid_src_gnss_pos; }
#endif

#if defined(CONFIG_EKF2_GNSS)
const estimator_aid_source3d_s & Ekf::aid_src_gnss_vel() const
{ return _aid_src_gnss_vel; }
#endif

#if defined(CONFIG_EKF2_GNSS)
# if defined(CONFIG_EKF2_GNSS_YAW)
const estimator_aid_source1d_s & Ekf::aid_src_gnss_yaw() const
{ return _aid_src_gnss_yaw; }
#endif
#endif

#if defined(CONFIG_EKF2_MAGNETOMETER)
const estimator_aid_source3d_s & Ekf::aid_src_mag() const
{ return _aid_src_mag; }
#endif

#if defined(CONFIG_EKF2_AUXVEL)
const estimator_aid_source2d_s & Ekf::aid_src_aux_vel() const
{ return _aid_src_aux_vel; }
#endif

void Ekf::resetHeadingToExternalObservation(float heading, float heading_accuracy)
{
	if (_control_status.flags.yaw_align) {
		resetYawByFusion(heading, heading_accuracy);

	} else {
		resetQuatStateYaw(heading, heading_accuracy);
		_control_status.flags.yaw_align = true;
	}

	// Force the mag consistency check to pass again since an external heading reset is often done to
	// counter mag disturbances.
	_control_status.flags.mag_heading_consistent = false;
	_control_status.flags.yaw_manual = true;
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, float vel_var)
{ resetHorizontalVelocityTo(new_horz_vel, Vector2f(vel_var, vel_var)); }

void Ekf::resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude, const float pos_var)
{ resetHorizontalPositionTo(new_latitude, new_longitude, Vector2f(pos_var, pos_var)); }

#if defined(CONFIG_EKF2_TERRAIN)
float Ekf::getTerrainVPos() const
{ return isTerrainEstimateValid() ? _state.terrain : _last_on_ground_posD; }
#endif

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void Ekf::fuseBodyVelocity(estimator_aid_source1d_s &aid_src, float &innov_var, VectorState &H)
{
	VectorState Kfusion = P * H / innov_var;
	measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);
	aid_src.fused = true;
}
#endif

bool Ekf::isTimedOut(uint64_t last_sensor_timestamp, uint64_t timeout_period) const
{
	return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < _time_delayed_us);
}

bool Ekf::isRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
{
	return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_delayed_us);
}

bool Ekf::isNewestSampleRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
{
	return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_latest_us);
}
