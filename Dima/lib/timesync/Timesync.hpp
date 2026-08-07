#pragma once
/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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
 * @file Timesync.hpp
 * Time synchroniser — ported from PX4-Autopilot v1.17.0
 * (src/lib/timesync/Timesync.hpp/.cpp, commit d6f12ad).
 *
 * Dima adaptations:
 *   - Header-only; the timesync_status uORB publication is removed
 *     (no timesync_status topic on this platform).
 *   - PX4_DEBUG traces are omitted.
 * The filter algorithm, gains and outlier rejection are unchanged.
 */

#include "platform/api/Time.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace dima::lib::timesync {

// Filter gains
//
// Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead
// to a smoother estimate, but track time drift more slowly, introducing a bias
// in the estimate. Larger values will cause low-amplitude oscillations.
//
// Beta : Used to smooth the clock skew estimate. Smaller values will lead to
// a tighter estimation of the skew (derivative), but will negatively affect how fast
// the filter reacts to clock skewing (e.g cause by temperature changes to the oscillator).
// Larger values will cause large-amplitude oscillations.
static constexpr double ALPHA_GAIN_INITIAL = 0.05;
static constexpr double BETA_GAIN_INITIAL = 0.05;
static constexpr double ALPHA_GAIN_FINAL = 0.003;
static constexpr double BETA_GAIN_FINAL = 0.003;

// Filter gain scheduling
//
// The filter interpolates between the INITIAL and FINAL gains while the number of
// exhanged timesync packets is less than CONVERGENCE_WINDOW. A lower value will
// allow the timesync to converge faster, but with potentially less accurate initial
// offset and skew estimates.
static constexpr std::uint32_t CONVERGENCE_WINDOW = 500;

// Outlier rejection and filter reset
//
// Samples with round-trip time higher than MAX_RTT_SAMPLE are not used to update the filter.
// More than MAX_CONSECUTIVE_HIGH_RTT number of such events in a row will throw a debug message
// but not reset the filter.
// Samples whose calculated clock offset is more than MAX_DEVIATION_SAMPLE off from the current
// estimate are not used to update the filter. More than MAX_CONSECUTIVE_HIGH_DEVIATION number
// of such events in a row will reset the filter. This usually happens only due to a time jump
// on the remote system.
static constexpr std::uint64_t MAX_RTT_SAMPLE = 10U * 1000U;      // 10 ms [us]
static constexpr std::uint64_t MAX_DEVIATION_SAMPLE = 100U * 1000U; // 100 ms [us]
static constexpr std::uint32_t MAX_CONSECUTIVE_HIGH_RTT = 10;
static constexpr std::uint32_t MAX_CONSECUTIVE_HIGH_DEVIATION = 10;

class Timesync
{
public:
	Timesync() = default;
	~Timesync() = default;

	void update(const std::uint64_t now_us, const std::int64_t remote_timestamp_ns,
		    std::int64_t originate_timestamp_ns)
	{
		// Message originating from this system, compute time offset from it
		if (remote_timestamp_ns > 0) {
			// Calculate time offset between this system and the remote system, assuming RTT for
			// the timesync packet is roughly equal both ways.
			std::int64_t offset_us = (std::int64_t)((originate_timestamp_ns / 1000ULL) + now_us -
						  (remote_timestamp_ns / 1000ULL) * 2) / 2;

			// Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
			std::uint64_t rtt_us = now_us - (originate_timestamp_ns / 1000ULL);

			// Calculate the difference of this sample from the current estimate
			std::uint64_t deviation = std::llabs((std::int64_t)_time_offset - offset_us);

			if (rtt_us < MAX_RTT_SAMPLE) {	// Only use samples with low RTT

				if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE)) {

					// Increment the counter if we have a good estimate and are getting samples far from the estimate
					_high_deviation_count++;

					// We reset the filter if we received 5 consecutive samples which violate our present estimate.
					// This is most likely due to a time jump on the offboard system.
					if (_high_deviation_count > MAX_CONSECUTIVE_HIGH_DEVIATION) {
						// Reset the filter
						reset_filter();
					}

				} else {

					// Filter gain scheduling
					if (!sync_converged()) {
						// Interpolate with a sigmoid function
						double progress = (double)_sequence / (double)CONVERGENCE_WINDOW;
						double p = 1.0 - std::exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
						_filter_alpha = p * ALPHA_GAIN_FINAL + (1.0 - p) * ALPHA_GAIN_INITIAL;
						_filter_beta = p * BETA_GAIN_FINAL + (1.0 - p) * BETA_GAIN_INITIAL;

					} else {
						_filter_alpha = ALPHA_GAIN_FINAL;
						_filter_beta = BETA_GAIN_FINAL;
					}

					// Perform filter update
					add_sample(offset_us);

					// Increment sequence counter after filter update
					_sequence++;

					// Reset high deviation count after filter update
					_high_deviation_count = 0;

					// Reset high RTT count after filter update
					_high_rtt_count = 0;
				}
			} else {
				// Increment counter if round trip time is too high for accurate timesync
				_high_rtt_count++;
			}
		}
	}

	/**
	 * Convert remote timestamp to local hrt time (usec)
	 * Use synchronised time if available, monotonic boot time otherwise
	 */
	std::uint64_t sync_stamp(std::uint64_t usec)
	{
		// Only return synchronised stamp if we have converged to a good value
		if (sync_converged()) {
			return usec + (std::int64_t)_time_offset;

		} else {
			return hrt_absolute_time();
		}
	}

	std::int64_t offset() const { return (std::int64_t)_time_offset; }

	/**
	 * Return true if the timesync algorithm converged to a good estimate,
	 * return false otherwise
	 */
	bool sync_converged() const { return _sequence >= CONVERGENCE_WINDOW; }

	/**
	 * Reset the exponential filter and its states
	 */
	void reset_filter()
	{
		// Do a full reset of all statistics and parameters
		_sequence = 0;
		_time_offset = 0.0;
		_time_skew = 0.0;
		_filter_alpha = ALPHA_GAIN_INITIAL;
		_filter_beta = BETA_GAIN_INITIAL;
		_high_deviation_count = 0;
		_high_rtt_count = 0;
	}

private:
	/**
	 * Online exponential filter to smooth time offset
	 */
	void add_sample(std::int64_t offset_us)
	{
		// Online exponential smoothing filter. The derivative of the estimate is also
		// estimated in order to produce an estimate without steady state lag:
		// https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
		double time_offset_prev = _time_offset;

		if (_sequence == 0) {
			// First offset sample
			_time_offset = offset_us;

		} else {
			// Update the clock offset estimate
			_time_offset = _filter_alpha * offset_us + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

			// Update the clock skew estimate
			_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
		}
	}

	std::uint32_t _sequence{0};

	// Timesync statistics
	double _time_offset{0};
	double _time_skew{0};

	// Filter parameters
	double _filter_alpha{ALPHA_GAIN_INITIAL};
	double _filter_beta{BETA_GAIN_INITIAL};

	// Outlier rejection and filter reset
	std::uint32_t _high_deviation_count{0};
	std::uint32_t _high_rtt_count{0};
};

} // namespace dima::lib::timesync
