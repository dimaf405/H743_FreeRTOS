#pragma once
/*
 * MAVLink Classic Parameter Protocol 处理器，移植自 PX4 v1.17.0。
 *
 * Dima 用发送回调替代 Mavlink 通道引用，链路缓冲核算归调用方；本板没有 UAVCAN 参数
 * 转发，也不宣称 PARAM_HASH。首次列表请求前必须激活并验证生成的产品参数目录，然后
 * 冻结本次 count/index/handle 映射，使 QGC 的缺失索引重试始终解析到同一参数。
 */

#include "parameter_update.hpp"
#include "mavlink/MavlinkBridge.h"
#include "parameters/param.h"
#include <parameters/parameter_contract.hpp>
#include "api/Time.hpp"
#include "uorb/SubscriptionData.hpp"

#include <cstdint>

namespace dima::modules::mavlink {

class MavlinkParameters {
public:
    /** 发送回调；false 表示传输层暂时未接收该帧，当前参数索引必须保留待重试。 */
    using SendFn = bool (*)(void *ctx, mavlink_message_t &msg);

    MavlinkParameters(SendFn send, void *send_ctx) noexcept;

    void reset() noexcept;

    /** 激活并验证生成的公开参数目录，禁止在此维护第二份参数清单。 */
    bool prepare_parameter_catalogue() noexcept;

    void handle_message(const mavlink_message_t *msg) noexcept;

    /**
     * 周期发送入口；USB 每轮最多尝试 20 个参数，避免参数下载独占 WorkQueue。
     */
    void send() noexcept;

private:
    using FixedParameterConstraint =
        dima::generated::parameters::FixedParameterConstraint;

    static const FixedParameterConstraint *fixed_parameter_constraint(
        param_t param) noexcept;

    static bool is_serial_baud_parameter(const char *name) noexcept;

    static bool supported_serial_baud(std::int32_t value) noexcept;

    static bool serial_function_write_allowed(
        const char *name, std::int32_t value) noexcept;

    bool write_and_acknowledge_parameter(
        param_t param, float wire_value) noexcept;

    bool set_serial_function(
        param_t param, const char *name, std::int32_t value) noexcept;

    static void append_used_parameter(void *context,
                                      param_t param) noexcept;

    void stop_parameter_stream() noexcept;

    void clear_parameter_snapshot() noexcept;

    bool snapshot_parameter_stream() noexcept;

    bool snapshot_contains_qgc_required_parameters() const noexcept;

    int parameter_snapshot_index(param_t param) const noexcept;

    static bool write_value_allowed(param_t param,
                                    float wire_value) noexcept;

    bool send_params() noexcept;

    bool send_untransmitted() noexcept;

    // 传输不可用或取值失败都保留当前快照索引；已经声明的 PARAM_VALUE 索引绝不跳过。
    bool send_one() noexcept;

    int send_param(param_t param) noexcept;

    int send_param(param_t param, std::uint16_t count,
                   std::uint16_t index) noexcept;

    void handle_param_ext_request_read(
        const mavlink_message_t *msg) noexcept;

    void send_param_ext_not_found(const char param_id[16],
                                  uint16_t count) noexcept;

    SendFn send_{nullptr};
    void *send_ctx_{nullptr};
    param_t _send_all_snapshot[px4::param_info_count]{};
    int _send_all_index{-1};
    unsigned _send_all_count{0U};
    hrt_abstime _param_update_time{0};
    int _param_update_index{0};
    int _last_read_failure_index{-1};
    bool _catalogue_reported{false};
    uORB::SubscriptionData<parameter_update_s> _parameter_update_sub{
        ORB_ID(parameter_update)};
};

}  // namespace dima::modules::mavlink
