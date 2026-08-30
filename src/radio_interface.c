#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include "radio_interface.h"

#include <unistd.h>

#include "rtl-sdr.h"

/* ---------------------------------------------------------------------------------------------------------------------
 * Local variables
 * -------------------------------------------------------------------------------------------------------------------*/

typedef struct radio_data {
    // private data
    atomic_bool     running;
    size_t          buf_size;
    struct ringbuf *input_buf;
    sem_t           semaphore;
    // public data
    uint8_t         output_buf[BUFFER_SIZE];
    uint32_t        overflow_counter;
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

int radio_config(uint32_t central_frequency, uint32_t bandwidth, uint32_t sample_rate)
{
    if (rtlsdr_open(&radio_device, DEFAULT_DEVICE_INDEX) < 0)
    {
        fprintf(stderr, "rtlsdr_open failed\n");
        return 1;
    }

    uint8_t err = 0;
    if (rtlsdr_set_center_freq(radio_device, central_frequency) != 0)
    {
        fprintf(stderr, "failed to set central frequency\n");
        err = 1;
    }
    if (rtlsdr_set_sample_rate(radio_device, sample_rate) != 0)
    {
        fprintf(stderr, "failed to set sample rate\n");
        err = 1;
    }
    if (rtlsdr_set_tuner_bandwidth(radio_device, bandwidth) != 0)
    {
        fprintf(stderr, "failed to set bandwidth\n");
        err = 1;
    }
    if (rtlsdr_set_tuner_gain_mode(radio_device, 0) != 0)
    {
        fprintf(stderr, "failed to set tuner gain mode\n");
        err = 1;
    }
    if (rtlsdr_reset_buffer(radio_device) != 0)
    {
        fprintf(stderr, "failed to reset buffer\n");
        err = 1;
    }

    if (err == 1 && radio_device != NULL)
    {
        rtlsdr_close(radio_device);
        return 1;
    }

    return 0;
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
    sem_init(&radio_data.semaphore, 1, 1);
    radio_data.running = true;

    return 0;
}
void* radio_stream_start(void* ctx)
{
    if (radio_data.input_buf == NULL)
    {
        fprintf(stderr, "failed to allocate buffer\n");
        return NULL;
    }

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
            if (size % 2 != 0)
            {
               radio_data.output_buf[size] = radio_data.output_buf[size-1];
               size++;
            }
        }
    }
    return NULL;
}