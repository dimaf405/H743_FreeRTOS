/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
/* Source: PX4-Autopilot d6f12ad1; fd/POSIX/realloc paths removed. */
#include "Dima/lib/tinybson/tinybson.h"
#include <cerrno>
#include <cstring>
namespace {
template <class T> void put(uint8_t *d, const T &v) {
    std::memcpy(d, &v, sizeof(v));
}
template <class T> T get(const uint8_t *s) {
    T v{};
    std::memcpy(&v, s, sizeof(v));
    return v;
}
int add(bson_encoder_t e, bson_type_t t, const char *n, const void *v, size_t z) {
    if (!e || !e->buffer || !n || e->finalized)
        return -EINVAL;
    size_t l = std::strlen(n);
    if (!l || l >= BSON_MAXNAME)
        return -ENAMETOOLONG;
    size_t r = 1 + l + 1 + z;
    if (e->offset > e->capacity || r > e->capacity - e->offset)
        return -ENOSPC;
    e->buffer[e->offset++] = (uint8_t)t;
    std::memcpy(e->buffer + e->offset, n, l + 1);
    e->offset += l + 1;
    std::memcpy(e->buffer + e->offset, v, z);
    e->offset += z;
    return 0;
}
} // namespace
extern "C" {
int bson_encoder_init_buf(bson_encoder_t e, void *b, size_t z) {
    if (!e || !b || z < 5)
        return -EINVAL;
    *e = {};
    e->buffer = (uint8_t *)b;
    e->capacity = z;
    e->offset = 4;
    return 0;
}
int bson_encoder_fini(bson_encoder_t e) {
    if (!e || !e->buffer || e->finalized)
        return -EINVAL;
    if (e->offset >= e->capacity || e->offset + 1 > UINT32_MAX)
        return -ENOSPC;
    e->buffer[e->offset++] = 0;
    uint32_t z = (uint32_t)e->offset;
    put(e->buffer, z);
    e->finalized = true;
    return 0;
}
size_t bson_encoder_buf_size(const bson_encoder_t e) {
    return e ? e->offset : 0;
}
void *bson_encoder_buf_data(const bson_encoder_t e) {
    return e ? e->buffer : nullptr;
}
int bson_encoder_append_bool(bson_encoder_t e, const char *n, bool v) {
    uint8_t x = v;
    return add(e, BSON_BOOL, n, &x, 1);
}
int bson_encoder_append_int32(bson_encoder_t e, const char *n, int32_t v) {
    return add(e, BSON_INT32, n, &v, 4);
}
int bson_encoder_append_int64(bson_encoder_t e, const char *n, int64_t v) {
    return add(e, BSON_INT64, n, &v, 8);
}
int bson_encoder_append_double(bson_encoder_t e, const char *n, double v) {
    return add(e, BSON_DOUBLE, n, &v, 8);
}
int bson_decoder_init_buf(bson_decoder_t d, const void *b, size_t z, bson_decoder_callback cb,
                          void *c) {
    if (!d || !b || z < 5)
        return -EINVAL;
    uint32_t declared = get<uint32_t>((const uint8_t *)b);
    if (declared < 5 || declared > z)
        return -EBADMSG;
    *d = {};
    d->buffer = (const uint8_t *)b;
    d->size = declared;
    d->offset = 4;
    d->callback = cb;
    d->context = c;
    return 0;
}
int bson_decoder_next(bson_decoder_t d) {
    if (!d || !d->buffer || d->offset >= d->size)
        return -EINVAL;
    uint8_t t = d->buffer[d->offset++];
    if (t == 0)
        return d->offset == d->size ? 0 : -EBADMSG;
    size_t s = d->offset;
    while (d->offset < d->size && d->buffer[d->offset])
        ++d->offset;
    size_t l = d->offset - s;
    if (d->offset >= d->size || !l || l >= BSON_MAXNAME)
        return -EBADMSG;
    std::memcpy(d->node.name, d->buffer + s, l);
    d->node.name[l] = 0;
    ++d->offset;
    size_t z = t == BSON_BOOL    ? 1
               : t == BSON_INT32 ? 4
                                 : (t == BSON_INT64 || t == BSON_DOUBLE ? 8 : 0);
    if (!z)
        return -ENOTSUP;
    if (z > d->size - d->offset)
        return -EBADMSG;
    d->node.type = (bson_type_t)t;
    d->node.data = d->buffer + d->offset;
    d->node.data_size = z;
    d->offset += z;
    if (d->callback) {
        int r = d->callback(d, &d->node, d->context);
        if (r)
            return r;
    }
    return 1;
}
int bson_decoder_copy_data(bson_decoder_t d, void *b, size_t z) {
    if (!d || !b)
        return -EINVAL;
    if (d->node.data_size > z)
        return -ENOBUFS;
    std::memcpy(b, d->node.data, d->node.data_size);
    return 0;
}
size_t bson_decoder_data_pending(bson_decoder_t d) {
    return d ? d->node.data_size : 0;
}
}
