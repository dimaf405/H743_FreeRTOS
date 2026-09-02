#pragma once

#include "uORB.hpp"

#ifndef __EXPORT
#define __EXPORT __attribute__((visibility("default")))
#endif

#define ORB_MULTI_MAX_INSTANCES uORB::kMaximumInstances
#define ORB_ID(_name) (&__orb_##_name)
#define ORB_DECLARE(_name) extern const struct orb_metadata __orb_##_name

/* 官方模板负责传入结构尺寸、尾部 padding 前尺寸、字段 hash、连续 Topic ID 与
 * 队列长度；本宏只附加 Dima 实例状态，不重新计算任何消息合同。 */
#define ORB_DEFINE(_name, _struct, _size_no_padding, _message_hash,            \
                   _orb_id_enum, _queue_size)                                  \
    static_assert((_queue_size) > 0U && (_queue_size) <= 0xFFU,                \
                  "uORB queue size is outside metadata range");               \
    static_assert(sizeof(_struct) <= 0xFFFFU,                                  \
                  "uORB object size is outside metadata range");              \
    static_assert((_size_no_padding) <= sizeof(_struct),                       \
                  "uORB no-padding size exceeds object size");                \
    static uORB::orb_runtime_instance __orb_runtime_##_name[                  \
        uORB::kMaximumInstances]{};                                            \
    const struct orb_metadata __orb_##_name{                                   \
        #_name, static_cast<uint16_t>(sizeof(_struct)),                        \
        static_cast<uint16_t>(_size_no_padding),                               \
        static_cast<uint32_t>(_message_hash),                                  \
        static_cast<orb_id_size_t>(_orb_id_enum),                              \
        static_cast<uint8_t>(_queue_size), uORB::kMaximumInstances,            \
        __orb_runtime_##_name}

/**
 * 将 PX4 生成 JSON 中的单字节字段 token 还原成 ULog 规范 C 类型名。
 * token 表由锁定的官方 helper 产生，本函数必须与上游映射逐项一致。
 */
const char *orb_get_c_type(unsigned char short_type);

void orb_print_message_internal(const struct orb_metadata *meta,
                                const void *data,
                                bool print_topic_name);
