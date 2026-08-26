#pragma once

#include "api/Can.hpp"

#include <DroneCanContract.hpp>

#include <cstddef>
#include <cstdint>

namespace dima::protocols::dronecan {

/** Fixed-memory DroneCAN v0 node core.
 *
 * Owns the libcanard session, Classic CAN bridge, NodeStatus and GetNodeInfo.
 * Device drivers provide only their broadcast subscriptions and decoders.
 * The public contract deliberately keeps libcanard and generated DSDL types
 * out of first-party headers.
 */
// 协议层固定内存的 DroneCAN v0 节点核心：统一拥有 libcanard 会话、Classic CAN
// 桥接、1 Hz NodeStatus、GetNodeInfo 和可选的集中式动态节点分配。设备驱动
// 只能通过生成合同订阅业务广播，首方头文件不暴露 libcanard/DSDL 具体类型。
class DroneCanNode final {
public:
    struct Identity {
        const char *name{nullptr};
        std::uint8_t software_major{0U};
        std::uint8_t software_minor{0U};
        std::uint8_t hardware_major{0U};
        std::uint8_t hardware_minor{0U};
        std::uint8_t unique_id[generated::kUniqueIdBytes]{};
    };

    /** Persistent backend used by the PX4-style centralized allocator.
     *
     * The image is fixed-size and remains owned by DroneCanNode until an
     * asynchronous save completes or is cancelled.
     */
    // 动态分配表采用异步持久化：begin_save 启动事务，continue_save 分步推进。
    // 保存完成或 cancel_save 前，data 始终由 DroneCanNode 固定缓冲持有，后端
    // 不得保存超出该生命周期的指针，也不得在回调中阻塞等待 Flash。
    struct AllocationStorage {
        /** Returns 0 for an exact image, -ENOENT when absent, or -errno. */
        int (*load)(void *context, std::uint8_t *data,
                    std::size_t capacity) noexcept{nullptr};
        int (*begin_save)(void *context, const std::uint8_t *data,
                          std::size_t size) noexcept{nullptr};
        int (*continue_save)(void *context) noexcept{nullptr};
        void (*cancel_save)(void *context) noexcept{nullptr};
        void *context{nullptr};
    };

    struct Configuration {
        // automatic_allocation=true 时本节点既是正常 DroneCAN 节点，也是集中式
        // 分配服务器；此模式必须提供完整 AllocationStorage 回调组。
        std::uint32_t bitrate{0U};
        std::uint8_t local_node_id{0U};
        bool automatic_allocation{false};
        Identity identity{};
        AllocationStorage allocation_storage{};
    };

    enum class AllocationEventKind : std::uint8_t {
        // 事件区分正常里程碑与协议/存储/冲突故障，供上层做有界日志而不泄露 UID。
        ServerReady,
        FirstAnonymousRequest,
        AllocationSucceeded,
        MalformedRequest,
        FollowupTimeout,
        AllocationExhausted,
        StorageFailure,
        NodeConflict,
    };

    struct AllocationEvent {
        AllocationEventKind kind{AllocationEventKind::StorageFailure};
        std::uint8_t node_id{0U};
        std::uint8_t preferred_node_id{0U};
        std::uint32_t unique_id_fingerprint{0U};
        std::int32_t error{0};
    };

    struct Transfer {
        std::uint64_t timestamp_us{0U};
        std::uint16_t data_type_id{0U};
        std::uint8_t source_node_id{0U};
        std::uint8_t transfer_id{0U};
        std::uint8_t transfer_type{0U};

        /** Valid only for the duration of the receive callback. */
        // native_handle 指向 libcanard 的临时接收对象；回调返回即释放 payload，
        // 消费者必须同步解码，不能跨回调保存该句柄或任何内部指针。
        void *native_handle() const noexcept { return native_handle_; }

    private:
        friend class DroneCanNode;
        void *native_handle_{nullptr};
    };

    struct Callbacks {
        bool (*accept_broadcast)(void *context, std::uint64_t &signature,
                                 std::uint16_t data_type_id,
                                 std::uint8_t source_node_id) noexcept{nullptr};
        void (*receive_broadcast)(void *context,
                                  Transfer &transfer) noexcept{nullptr};
        void (*allocation_event)(void *context,
                                 const AllocationEvent &event) noexcept{nullptr};
        void *context{nullptr};
    };

    struct Stats {
        std::uint32_t protocol_errors{0U};
        std::uint32_t node_status_transfers{0U};
        std::uint32_t node_info_responses{0U};
        std::uint32_t transport_failures{0U};
        std::uint32_t allocation_requests{0U};
        std::uint32_t allocation_successes{0U};
        std::uint32_t allocation_malformed{0U};
        std::uint32_t allocation_timeouts{0U};
        std::uint32_t allocation_storage_failures{0U};
        std::uint32_t discovered_nodes{0U};
    };

    explicit DroneCanNode(dima::platform::CanTransport &transport) noexcept;

    bool start(const Configuration &configuration,
               const Callbacks &callbacks, std::uint64_t now_us) noexcept;
    void stop() noexcept;
    // service 的固定次序为：底层总线恢复 -> RX -> 分配/持久化 -> 1 Hz 节点状态
    // 与过期会话清理 -> TX。调用者需周期调用，函数自身不阻塞。
    bool service(std::uint64_t now_us) noexcept;
    bool running() const noexcept { return running_; }
    void set_health_warning(bool warning) noexcept {
        health_warning_ = warning;
    }
    const Stats &stats() const noexcept { return stats_; }

private:
    // libcanard 实例和 3 KiB 会话池均由对象内静态内存提供；启动后无堆分配。
    static constexpr std::size_t kInstanceStorageBytes = 64U;
    static constexpr std::size_t kMemoryPoolBytes = 3072U;
    static constexpr std::uint64_t kNodeStatusIntervalUs = 1000000ULL;
    static constexpr std::size_t kNodeCount =
        static_cast<std::size_t>(generated::kMaximumNodeId) + 1U;

    enum class PendingCommitKind : std::uint8_t {
        // 内存分配表只有在异步镜像保存成功后才应用 PendingCommit，并且只在
        // Allocation commit 完成后才向匿名节点发送最终 node-ID 响应。
        None,
        AllocatorIdentity,
        Allocation,
        DiscoveredNode,
    };

    struct PendingCommit {
        PendingCommitKind kind{PendingCommitKind::None};
        std::uint8_t node_id{0U};
        std::uint8_t state{generated::kAllocationStateEmpty};
        std::uint8_t unique_id[generated::kUniqueIdBytes]{};
    };

    struct PendingAllocationResponse {
        bool valid{false};
        std::uint8_t node_id{0U};
        std::uint8_t unique_id_length{0U};
        std::uint8_t unique_id[generated::kUniqueIdBytes]{};
    };

    void process_rx() noexcept;
    void process_tx() noexcept;
    void send_node_status(std::uint64_t now_us) noexcept;
    void receive(void *native_transfer) noexcept;
    void respond_node_info(void *native_transfer) noexcept;
    bool accept(std::uint64_t *signature, std::uint16_t data_type_id,
                std::int32_t transfer_type,
                std::uint8_t source_node_id) const noexcept;
    bool initialize_allocation(std::uint64_t now_us) noexcept;
    void reset_allocation() noexcept;
    void service_allocation(std::uint64_t now_us) noexcept;
    void service_allocation_storage(std::uint64_t now_us) noexcept;
    void service_allocation_response(std::uint64_t now_us) noexcept;
    void service_pending_allocation_request(std::uint64_t now_us) noexcept;
    void service_discovery(std::uint64_t now_us) noexcept;
    void handle_allocation(void *native_transfer) noexcept;
    void handle_node_status(void *native_transfer) noexcept;
    void handle_node_info_response(void *native_transfer) noexcept;
    void complete_allocation_request(
        const std::uint8_t *unique_id, std::uint8_t preferred_node_id,
        std::uint64_t now_us) noexcept;
    bool stage_commit(PendingCommitKind kind, std::uint8_t node_id,
                      std::uint8_t state, const std::uint8_t *unique_id,
                      std::uint64_t now_us) noexcept;
    void apply_completed_commit(std::uint64_t now_us) noexcept;
    bool load_allocation_image() noexcept;
    void encode_allocation_image() noexcept;
    std::uint8_t node_id_for_unique_id(
        const std::uint8_t *unique_id) const noexcept;
    std::uint8_t select_node_id(std::uint8_t preferred) const noexcept;
    bool node_id_occupied(std::uint8_t node_id) const noexcept;
    bool send_get_node_info_request(std::uint8_t node_id) noexcept;
    void queue_allocation_response(std::uint8_t node_id,
                                   const std::uint8_t *unique_id,
                                   std::size_t unique_id_length) noexcept;
    void emit_allocation_event(AllocationEventKind kind,
                               std::uint8_t node_id,
                               std::uint8_t preferred_node_id,
                               const std::uint8_t *unique_id,
                               std::int32_t error) noexcept;
    void emit_bounded_allocation_error(AllocationEventKind kind,
                                       std::uint64_t now_us,
                                       std::uint8_t node_id,
                                       std::int32_t error) noexcept;
    static std::uint32_t unique_id_fingerprint(
        const std::uint8_t *unique_id) noexcept;

    dima::platform::CanTransport &transport_;
    Configuration configuration_{};
    Callbacks callbacks_{};
    alignas(8) std::uint8_t instance_storage_[kInstanceStorageBytes]{};
    alignas(8) std::uint8_t memory_pool_[kMemoryPoolBytes]{};
    Stats stats_{};
    std::uint64_t start_time_us_{0U};
    std::uint64_t next_node_status_us_{0U};
    std::uint8_t node_status_transfer_id_{0U};
    std::uint8_t allocation_transfer_id_{0U};
    std::uint8_t get_node_info_transfer_ids_[kNodeCount]{};
    std::uint8_t allocation_prefix_[generated::kUniqueIdBytes]{};
    std::uint8_t allocation_prefix_length_{0U};
    // 所有按 node-ID 索引的数组包含 0..kMaximumNodeId；0 保留为匿名/广播，
    // 正常分配与发现只遍历 1..kMaximumNodeId。
    std::uint8_t node_states_[kNodeCount]{};
    std::uint8_t node_unique_ids_[kNodeCount]
                                 [generated::kUniqueIdBytes]{};
    std::uint8_t discovery_attempts_[kNodeCount]{};
    std::uint32_t discovery_uptime_[kNodeCount]{};
    bool observed_nodes_[kNodeCount]{};
    bool discovery_pending_[kNodeCount]{};
    std::uint8_t pending_request_unique_id_[generated::kUniqueIdBytes]{};
    std::uint8_t pending_request_preferred_node_id_{0U};
    std::uint8_t pending_discovery_node_id_{0U};
    std::uint8_t pending_discovery_unique_id_[generated::kUniqueIdBytes]{};
    std::uint8_t discovery_query_node_id_{0U};
    std::uint64_t discovery_query_deadline_us_{0U};
    std::uint64_t next_discovery_poll_us_{0U};
    std::uint64_t last_allocation_message_us_{0U};
    std::uint64_t storage_retry_after_us_{0U};
    std::uint64_t next_allocation_error_event_us_{0U};
    std::int32_t last_allocation_storage_error_{0};
    PendingCommit pending_commit_{};
    PendingAllocationResponse pending_allocation_response_{};
    alignas(4) std::uint8_t
        allocation_storage_image_[generated::kAllocationStorageImageBytes]{};
    bool running_{false};
    bool health_warning_{false};
    bool automatic_allocation_{false};
    bool allocation_ready_{false};
    bool allocation_storage_active_{false};
    bool allocation_first_request_reported_{false};
    bool pending_request_valid_{false};
    bool pending_discovery_valid_{false};
};

} // namespace dima::protocols::dronecan
