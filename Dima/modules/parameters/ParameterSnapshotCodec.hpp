#pragma once

#include "parameters/param.h"

#include <cstddef>
#include <cstdint>

namespace dima::modules::parameters::snapshot_codec {

// 参数持久化快照的纯 codec：20 B 固定头 + flashparams payload。只处理内存缓冲，
// 不拥有 Flash/SD，也不改变参数；介质选择和事务状态机由 ParameterService 负责。
struct SnapshotInfo {
    std::uint32_t generation{0U};
    std::uint32_t payload_crc{0U};
    std::size_t payload_size{0U};
};

int encode(param_storage_enumerator_t enumerate, void *enumerate_context,
           std::uint8_t *destination, std::size_t destination_capacity,
           std::uint32_t generation, std::size_t &snapshot_size) noexcept;

int inspect(const std::uint8_t *data, std::size_t size,
            SnapshotInfo &info) noexcept;

int validate(const std::uint8_t *data, std::size_t size,
             void *context) noexcept;

int decode_mutable(const std::uint8_t *payload, std::size_t payload_size,
                   param_storage_visitor_t visitor,
                   void *visitor_context) noexcept;

bool payload_matches(const std::uint8_t *persisted,
                     const std::uint8_t *comparison,
                     std::size_t comparison_size) noexcept;

} // namespace dima::modules::parameters::snapshot_codec
