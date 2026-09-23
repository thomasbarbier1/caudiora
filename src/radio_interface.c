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

/**
 * @file radio_interface.c
 * @brief Abstraction layer for the RTL-SDR driver. Facilitates the initialization and configuration of the radio.
 * Also implement the driver callback function.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <assert.h>
#include <semaphore.h>
#include <stdatomic.h>
#include "ringbuf.h" // https://github.com/szanni/ringbuf/tree/master
#include "radio_interface.h"
#include "rtl-sdr.h"

/* ---------------------------------------------------------------------------------------------------------------------
 * Local variables
 * -------------------------------------------------------------------------------------------------------------------*/

typedef struct {
    // private data
    atomic_bool       running;
    size_t            buf_size;
    struct ringbuf   *input_buf;
    sem_t             semaphore;
    // public data
    radio_sample_cb_t sample_cb;
    void             *cb_ctx;
    uint8_t           output_buf[BUFFER_SIZE];
    uint32_t          overflow_counter;
} radio_data_t;

static radio_data_t   radio_data;
static rtlsdr_dev_t  *radio_device = NULL;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/

static void on_samples(unsigned char *buf, uint32_t len, void *ctx);
static void *sample_consumer_loop(void *ctx);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

int radio_config(void)
{
    if (rtlsdr_open(&radio_device, DEFAULT_DEVICE_INDEX) < 0)
    {
        fprintf(stderr, "rtlsdr_open failed\n");
        return 1;
    }

    uint8_t err = 0;
    if (rtlsdr_set_center_freq(radio_device, CENTER_FREQUENCY) != 0)
    {
        fprintf(stderr, "Failed to set central frequency\n");
        err = 1;
    }
    if (rtlsdr_set_sample_rate(radio_device, SAMPLE_RATE) != 0)
    {
        fprintf(stderr, "Failed to set sample rate\n");
        err = 1;
    }
    if (rtlsdr_set_tuner_bandwidth(radio_device, BANDWIDTH) != 0)
    {
        fprintf(stderr, "Failed to set bandwidth\n");
        err = 1;
    }
    if (rtlsdr_set_tuner_gain_mode(radio_device, 0) != 0)
    {
        fprintf(stderr, "Failed to set tuner gain mode\n");
        err = 1;
    }
    if (rtlsdr_reset_buffer(radio_device) != 0)
    {
        fprintf(stderr, "Failed to reset RTLSDR buffer\n");
        err = 1;
    }

    if (err == 1 && radio_device != NULL)
    {
        rtlsdr_close(radio_device);
    }

    return err;
}

int radio_init(void)
{
    radio_data.buf_size = BUFFER_SIZE;
    radio_data.input_buf = ringbuf_new(radio_data.buf_size);
    if (radio_data.input_buf == NULL)
    {
        return 1;
    }
    radio_data.overflow_counter = 0;
    sem_init(&radio_data.semaphore, 0, 0);
    radio_data.running = true;

    return 0;
}

void* radio_stream_thread(void* ctx)
{
    pthread_t consume_thread;
    pthread_create(&consume_thread, NULL, sample_consumer_loop, NULL);

    rtlsdr_read_async(radio_device, on_samples, NULL, 0, 0);
    pthread_join(consume_thread, NULL);

    return NULL;
}

uint32_t radio_stream_stop(void)
{
    radio_data.running = false;
    rtlsdr_cancel_async(radio_device);
    rtlsdr_close(radio_device);
    sem_post(&radio_data.semaphore); // in case the semaphore is waiting, we unlock it first
    sem_destroy(&radio_data.semaphore);
    ringbuf_free(radio_data.input_buf);

    return radio_data.overflow_counter;
}

int radio_set_callback(radio_sample_cb_t cb, void *user_ctx)
{
    if (cb == NULL)
    {
        return 1;
    }

    radio_data.sample_cb = cb;
    radio_data.cb_ctx    = user_ctx;

    return 0;
}

/***********************************************************************************************************************
 * Private functions implementations
 **********************************************************************************************************************/

static void on_samples(unsigned char *buf, uint32_t len, void *ctx)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    uint32_t size = ringbuf_write(radio_data.input_buf, buf, len);
    if (size != len)
    {
        radio_data.overflow_counter++;
    }
    sem_post(&radio_data.semaphore);
}

static void *sample_consumer_loop(void *ctx)
{
    while (radio_data.running)
    {
        sem_wait(&radio_data.semaphore);
        uint32_t size = ringbuf_read(radio_data.input_buf, radio_data.output_buf, radio_data.buf_size);
        if (size > 0)
        {
            assert(size % 2 == 0);
            radio_data.sample_cb(radio_data.output_buf, size, radio_data.cb_ctx);
        }
    }
    return NULL;
}