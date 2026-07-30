/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
/* Source: PX4-Autopilot d6f12ad1; flashfs/POSIX paths removed. */
#include "Dima/middleware/parameters/flashparams/flashparams.h"
#include "Dima/lib/tinybson/tinybson.h"
#include <cerrno>
#include <cstring>
namespace {
struct EC {
    bson_encoder_s e;
};
int enc(const char *n, param_type_t t, const void *v, void *c) {
    if (!n || !v || !c)
        return -EINVAL;
    auto &e = ((EC *)c)->e;
    if (t == PARAM_TYPE_INT32) {
        int32_t x;
        std::memcpy(&x, v, 4);
        return bson_encoder_append_int32(&e, n, x);
    }
    if (t == PARAM_TYPE_FLOAT) {
        float x;
        std::memcpy(&x, v, 4);
        return bson_encoder_append_double(&e, n, (double)x);
    }
    return -ENOTSUP;
}
struct DC {
    param_storage_visitor_t v;
    void *c;
};
int dec(bson_decoder_t, bson_node_t n, void *c) {
    auto *d = (DC *)c;
    if (!d || !d->v)
        return -EINVAL;
    if (n->type == BSON_INT32) {
        int32_t x;
        std::memcpy(&x, n->data, 4);
        const int r = d->v(n->name, PARAM_TYPE_INT32, &x, d->c);
        return r <= 0 ? r : -ECANCELED;
    }
    if (n->type == BSON_DOUBLE) {
        double x;
        std::memcpy(&x, n->data, 8);
        float y = (float)x;
        const int r = d->v(n->name, PARAM_TYPE_FLOAT, &y, d->c);
        return r <= 0 ? r : -ECANCELED;
    }
    return -ENOTSUP;
}
} // namespace
extern "C" {
int flashparams_encode_buffer(void *b, size_t z, param_storage_enumerator_t f, void *c,
                              size_t *out) {
    if (!b || !f || !out)
        return -EINVAL;
    EC x{};
    int r = bson_encoder_init_buf(&x.e, b, z);
    if (!r)
        r = f(enc, &x, c);
    if (!r)
        r = bson_encoder_fini(&x.e);
    if (!r)
        *out = bson_encoder_buf_size(&x.e);
    return r;
}
int flashparams_decode_buffer(const void *b, size_t z, param_storage_visitor_t v, void *c) {
    if (!b || !v)
        return -EINVAL;
    DC x{v, c};
    bson_decoder_s d{};
    int r = bson_decoder_init_buf(&d, b, z, dec, &x);
    if (r)
        return r;
    do {
        r = bson_decoder_next(&d);
    } while (r > 0);
    return r;
}
}
