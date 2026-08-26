#include "Timesync.hpp"

#include "api/Time.hpp"

#include <cmath>
#include <cstdlib>

namespace dima::lib::timesync {

void Timesync::update(const std::uint64_t now_us,
                      const std::int64_t remote_timestamp_ns,
                      std::int64_t originate_timestamp_ns)
{
	// tc1>0 表示远端回送了本机 originate 时间。只在完整往返样本上估计偏移，
	// 远端发起的 tc1=0 请求由上层回包，不进入滤波器。
	if (remote_timestamp_ns > 0) {
		// 假设上下行时延近似对称：
		// offset_us=((t_originate_us+now_us)/2)-t_remote_us。
		// 该符号表示“远端时间加多少得到本地时间”，与 sync_stamp 的加法一致。
		std::int64_t offset_us = (std::int64_t)((originate_timestamp_ns / 1000ULL) + now_us -
					  (remote_timestamp_ns / 1000ULL) * 2) / 2;

		// RTT=本次接收时刻-本机原始发送时刻；所有内部时间统一为微秒。
		std::uint64_t rtt_us = now_us - (originate_timestamp_ns / 1000ULL);

		// deviation=|当前估计-本次样本|，只用于收敛后的远端跳时检测。
		std::uint64_t deviation = std::llabs((std::int64_t)_time_offset - offset_us);

		if (rtt_us < MAX_RTT_SAMPLE) {	// 高 RTT 的不对称误差过大，不更新滤波器。

			if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE)) {

				// 已收敛后连续出现大偏差，优先判断为远端时钟跳变而非缓慢漂移。
				_high_deviation_count++;

				// 超过 MAX_CONSECUTIVE_HIGH_DEVIATION 个连续异常样本才清空估计；
				// 单个网络抖动不会破坏已经收敛的 offset/skew。
				if (_high_deviation_count > MAX_CONSECUTIVE_HIGH_DEVIATION) {
					// Reset the filter
					reset_filter();
				}

			} else {

				// 收敛窗口内按 sigmoid 从初始大增益平滑过渡到最终小增益：
				// p=1-exp(0.5*(1-1/(1-progress)))，gain=p*final+(1-p)*initial。
				if (!sync_converged()) {
					// sequence<window 保证 progress<1，分母不会触及零。
					double progress = (double)_sequence / (double)CONVERGENCE_WINDOW;
					double p = 1.0 - std::exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
					_filter_alpha = p * ALPHA_GAIN_FINAL + (1.0 - p) * ALPHA_GAIN_INITIAL;
					_filter_beta = p * BETA_GAIN_FINAL + (1.0 - p) * BETA_GAIN_INITIAL;

				} else {
					_filter_alpha = ALPHA_GAIN_FINAL;
					_filter_beta = BETA_GAIN_FINAL;
				}

				// 接受样本后执行 Holt 双指数平滑，同时估计 offset 与每样本 skew。
				add_sample(offset_us);

				// sequence 只统计真正进入滤波器的低 RTT、非离群样本。
				_sequence++;

				// 一个健康样本即结束“连续异常”序列。
				_high_deviation_count = 0;

				// RTT 计数同样按连续序列统计；Dima 不发布 timesync_status。
				_high_rtt_count = 0;
			}
		} else {
			// 仅记数并拒绝样本；高 RTT 本身不会重置已有的稳定时间估计。
			_high_rtt_count++;
		}
	}
}

std::uint64_t Timesync::sync_stamp(std::uint64_t usec)
{
	// 未收敛时绝不输出看似有效的远端时间，改用本机单调时钟保持时间戳可排序。
	if (sync_converged()) {
		// remote_us + (local-remote) = local_us。
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
	// 链路重连或远端跳时后完整清零统计、趋势和动态增益，重新走收敛窗口。
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
	// Holt 双指数平滑同时估计 level(offset) 与 trend(skew)，减少持续晶振漂移下
	// 单指数滤波的稳态滞后：
	// https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
	double time_offset_prev = _time_offset;

	if (_sequence == 0) {
		// 首样本直接建立 level，不能把默认零偏移作为一次历史观测。
		_time_offset = offset_us;

	} else {
		// L_t=alpha*x_t+(1-alpha)*(L_{t-1}+T_{t-1})。
		_time_offset = _filter_alpha * offset_us + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

		// T_t=beta*(L_t-L_{t-1})+(1-beta)*T_{t-1}。
		_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
	}
}

} // namespace dima::lib::timesync
