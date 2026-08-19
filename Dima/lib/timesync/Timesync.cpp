#include "Timesync.hpp"

#include "platform/api/Time.hpp"

#include <cmath>
#include <cstdlib>

namespace dima::lib::timesync {

void Timesync::update(const std::uint64_t now_us,
                      const std::int64_t remote_timestamp_ns,
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

std::uint64_t Timesync::sync_stamp(std::uint64_t usec)
{
	// Only return synchronised stamp if we have converged to a good value
	if (sync_converged()) {
		return usec + (std::int64_t)_time_offset;

	} else {
		return hrt_absolute_time();
	}
}

std::int64_t Timesync::offset() const
{
	return (std::int64_t)_time_offset;
}

bool Timesync::sync_converged() const
{
	return _sequence >= CONVERGENCE_WINDOW;
}

void Timesync::reset_filter()
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

void Timesync::add_sample(std::int64_t offset_us)
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

} // namespace dima::lib::timesync
