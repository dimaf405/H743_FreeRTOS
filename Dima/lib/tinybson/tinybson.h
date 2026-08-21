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
/* Source: PX4-Autopilot d6f12ad1 src/lib/tinybson; buffer-only adaptation. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    BSON_EOO = 0,
    BSON_DOUBLE = 1,
    BSON_BOOL = 8,
    BSON_INT32 = 16,
    BSON_INT64 = 18
} bson_type_t;
enum { BSON_MAXNAME = 32 };
typedef struct bson_node_s {
    char name[BSON_MAXNAME];
    bson_type_t type;
    const uint8_t *data;
    size_t data_size;
} bson_node_s, *bson_node_t;
struct bson_decoder_s;
typedef struct bson_decoder_s *bson_decoder_t;
typedef int (*bson_decoder_callback)(bson_decoder_t, bson_node_t, void *);
typedef struct bson_decoder_s {
    const uint8_t *buffer;
    size_t size, offset;
    bson_node_s node;
    bson_decoder_callback callback;
    void *context;
} bson_decoder_s;
typedef struct bson_encoder_s {
    uint8_t *buffer;
    size_t capacity, offset;
    bool finalized;
} bson_encoder_s, *bson_encoder_t;
int bson_decoder_init_buf(bson_decoder_t, const void *, size_t, bson_decoder_callback, void *);
int bson_decoder_next(bson_decoder_t);
int bson_encoder_init_buf(bson_encoder_t, void *, size_t);
int bson_encoder_fini(bson_encoder_t);
size_t bson_encoder_buf_size(const bson_encoder_t);
int bson_encoder_append_int32(bson_encoder_t, const char *, int32_t);
int bson_encoder_append_double(bson_encoder_t, const char *, double);
#ifdef __cplusplus
}
#endif
