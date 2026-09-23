/*
 * Copyright (c) 2018, Thomas Barbier
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef CAUDIORA_RADIO_H
#define CAUDIORA_RADIO_H

#define DEFAULT_DEVICE_INDEX 0
#define DURATION_S           5
#define CENTER_FREQUENCY     (uint32_t)868300000
#define BANDWIDTH            (uint32_t)125000
#define SAMPLE_RATE          (uint32_t)2400000
#define BUFFER_SIZE          (uint32_t) 1048576

typedef void (*radio_sample_cb_t)(const uint8_t *iq, size_t len, void *user_ctx);

/* ---------------------------------------------------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------------------------------------------------*/
int radio_config(void);
int radio_init(void);
void* radio_stream_thread(void* ctx);
uint32_t radio_stream_stop(void);
int radio_set_callback(radio_sample_cb_t cb, void *user_ctx);

#endif //CAUDIORA_RADIO_H
