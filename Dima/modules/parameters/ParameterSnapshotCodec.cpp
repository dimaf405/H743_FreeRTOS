#include "ParameterSnapshotCodec.hpp"

#include "parameters/Crc32.hpp"
#include <parameters/parameter_contract.hpp>
#include "flashparams/flashparams.h"

#include <cerrno>
#include <cstring>

namespace dima::modules::parameters::snapshot_codec {
namespace {

struct FilteredLoadContext {
    param_storage_visitor_t visitor;
    void *visitor_context;
};

struct SnapshotHeader {
    // ABI 顺序固定为 magic/format/generation/payload_size/payload_crc，均 32 bit；
    // 不可插入 padding 字段或直接更改版本而继续读取旧镜像。
    std::uint32_t magic;
    std::uint32_t format;
    std::uint32_t generation;
    std::uint32_t payload_size;
    std::uint32_t payload_crc;
};
static_assert(sizeof(SnapshotHeader) == 20U);

constexpr std::uint32_t kSnapshotMagic = 0x5041524DU;
constexpr std::uint32_t kSnapshotFormat = 1U;

bool is_fixed_parameter(const char *name) noexcept
{
    // 固定参数集合只来自生成的 kFixedParameterConstraints；codec 不维护手写名称
    // 列表。加载用户层时跳过固定项，防止持久化快照覆盖产品强制合同。
    if (name == nullptr) {
        return false;
    }
    const param_t handle = param_find_no_notification(name);
    if (handle == PARAM_INVALID) {
        return false;
    }
    for (const auto &constraint :
         dima::generated::parameters::kFixedParameterConstraints) {
        if (param_handle(constraint.parameter) == handle) {
            return true;
        }
    }
    return false;
}

int load_mutable_parameter(const char *name, param_type_t type,
                           const void *value, void *context) noexcept
{
    if (name == nullptr || value == nullptr || context == nullptr) {
        return -EINVAL;
    }
    // 参数目录收缩后，旧快照可能仍携带已退役的 Aux/Flaps/QGC 兼容名称；
    // 只丢弃当前目录不存在的条目，保留同一快照中的 RC1～RC18 等有效配置。
    if (param_find_no_notification(name) == PARAM_INVALID) {
        return 0;
    }
    auto &filtered = *static_cast<FilteredLoadContext *>(context);
    if (is_fixed_parameter(name)) {
        return 0;
    }

    return filtered.visitor(name, type, value, filtered.visitor_context);
}

} // namespace

int encode(param_storage_enumerator_t enumerate, void *enumerate_context,
           std::uint8_t *destination, std::size_t destination_capacity,
           std::uint32_t generation, std::size_t &snapshot_size) noexcept
{
    if (enumerate == nullptr || destination == nullptr || generation == 0U ||
        destination_capacity < sizeof(SnapshotHeader)) {
        return -EINVAL;
    }

    // 先把可保存参数编码到 header 后方，再计算 payload CRC-32，最后 memcpy
    // 20 B 头。generation=0 保留为“无已提交快照”。
    std::size_t payload_size{};
    const int encoded = flashparams_encode_buffer(
        destination + sizeof(SnapshotHeader),
        destination_capacity - sizeof(SnapshotHeader),
        enumerate, enumerate_context, &payload_size);
    if (encoded != 0) {
        return encoded;
    }

    const SnapshotHeader header{
        kSnapshotMagic,
        kSnapshotFormat,
        generation,
        static_cast<std::uint32_t>(payload_size),
        dima::parameters::crc32(
            destination + sizeof(SnapshotHeader), payload_size),
    };
    std::memcpy(destination, &header, sizeof(header));
    snapshot_size = sizeof(SnapshotHeader) + payload_size;
    return 0;
}

int inspect(const std::uint8_t *data, std::size_t size,
            SnapshotInfo &info) noexcept
{
    if (data == nullptr || size < sizeof(SnapshotHeader)) {
        return -EILSEQ;
    }
    // 先验证 magic/format/非零 generation/精确长度/容量，再校验 payload CRC；
    // 任一不符统一 -EILSEQ，不能把半写或旧格式解释为参数记录。
    SnapshotHeader header{};
    std::memcpy(&header, data, sizeof(header));
    if (header.magic != kSnapshotMagic || header.format != kSnapshotFormat ||
        header.generation == 0U ||
        header.payload_size != size - sizeof(SnapshotHeader) ||
        header.payload_size >
            dima::generated::parameters::kParameterStorageMaxBytes) {
        return -EILSEQ;
    }
    const auto *payload = data + sizeof(SnapshotHeader);
    if (dima::parameters::crc32(payload, header.payload_size) !=
        header.payload_crc) {
        return -EILSEQ;
    }
    info.generation = header.generation;
    info.payload_crc = header.payload_crc;
    info.payload_size = header.payload_size;
    return 0;
}

int validate(const std::uint8_t *data, std::size_t size,
             void *context) noexcept
{
    SnapshotInfo info{};
    const int result = inspect(data, size, info);
    if (result != 0) {
        return result;
    }

    if (info.payload_size == 0U) {
        if (context != nullptr) {
            *static_cast<SnapshotInfo *>(context) = info;
        }
        return 0;
    }

    // CRC 正确仍不足够：逐条解析并确认当前权威参数表存在同名、同类型项，
    // 防止 schema 变化后的旧 payload 在运行期写入错误类型。
    const auto validate_parameter = [](const char *name, param_type_t type,
                                       const void *value,
                                       void *) noexcept {
        if (name == nullptr || value == nullptr) {
            return -EINVAL;
        }
        const param_t parameter = param_find_no_notification(name);
        if (parameter == PARAM_INVALID) {
            // CRC 正确但目录中已无此名称，视为已退役参数并允许其余条目迁移。
            return 0;
        }
        return param_type(parameter) == type ? 0 : -EINVAL;
    };
    const int decoded = flashparams_decode_buffer(
        data + sizeof(SnapshotHeader), info.payload_size,
        validate_parameter, nullptr);
    if (decoded != 0) {
        return decoded;
    }
    if (context != nullptr) {
        *static_cast<SnapshotInfo *>(context) = info;
    }
    return 0;
}

int decode_mutable(const std::uint8_t *payload, std::size_t payload_size,
                   param_storage_visitor_t visitor,
                   void *visitor_context) noexcept
{
    // decode 时以生成固定参数集合过滤，只把可变用户层交给 param visitor。
    FilteredLoadContext filtered{visitor, visitor_context};
    return flashparams_decode_buffer(
        payload, payload_size, load_mutable_parameter, &filtered);
}

bool payload_matches(const std::uint8_t *persisted,
                     const std::uint8_t *comparison,
                     std::size_t comparison_size) noexcept
{
    SnapshotHeader persisted_header{};
    SnapshotHeader comparison_header{};
    std::memcpy(&persisted_header, persisted, sizeof(persisted_header));
    std::memcpy(&comparison_header, comparison, sizeof(comparison_header));
    // generation 不参与等价判断：先比较 size+CRC，再 memcmp payload 防 CRC 碰撞；
    // 用于确认持久化期间是否又有参数变化。
    return persisted_header.payload_size == comparison_header.payload_size &&
           persisted_header.payload_crc == comparison_header.payload_crc &&
           comparison_size ==
               sizeof(SnapshotHeader) + persisted_header.payload_size &&
           std::memcmp(persisted + sizeof(SnapshotHeader),
                       comparison + sizeof(SnapshotHeader),
                       persisted_header.payload_size) == 0;
}

} // namespace dima::modules::parameters::snapshot_codec
