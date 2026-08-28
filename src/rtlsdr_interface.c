#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include "rtlsdr_interface.h"
#include "rtl-sdr.h"

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/

static void on_samples(unsigned char *buf, uint32_t len, void *ctx);
static void *radio_thread_loop(void *ctx);
static double compute_mean_complex_magnitude(const double *iq_samples, uint32_t len);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

int rtlsdr_init(const rtlsdr_config_t *config)
{
    if (rtlsdr_open(&radio_device, DEFAULT_DEVICE_INDEX) < 0)
    {
        fprintf(stderr, "rtlsdr_open failed\n");
        return 1;
    }
    if (rtlsdr_set_center_freq(radio_device, config->central_frequency) != 0)
    {
        fprintf(stderr, "failed to set central frequency\n");
        return 1;
    }
    if (rtlsdr_set_sample_rate(radio_device, config->sample_rate) != 0)
    {
        fprintf(stderr, "failed to set sample rate\n");
        return 1;
    }
    if (rtlsdr_set_tuner_bandwidth(radio_device, config->bandwidth) != 0)
    {
        fprintf(stderr, "failed to set bandwidth\n");
        return 1;
    }
    if (rtlsdr_set_tuner_gain_mode(radio_device, 0) != 0)
    {
        fprintf(stderr, "failed to set tuner gain mode\n");
        return 1;
    }
    if (rtlsdr_reset_buffer(radio_device) != 0)
    {
        fprintf(stderr, "failed to reset buffer\n");
        return 1;
    }

    return 0;
}

int rtlsdr_stream_start()
{
    pthread_t rtlsdr_thread_id;
    if (pthread_create(&rtlsdr_thread_id, NULL, radio_thread_loop, &radio_context) != 0)
    {
        fprintf(stderr, "pthread_create failed\n");
        rtlsdr_close(radio_device);
        return 1;
    }
    rtlsdr_read_async(radio_device, on_samples, &radio_context, 0, 0);
    pthread_join(rtlsdr_thread_id, NULL);

    return 0;
}

int rtlsdr_stream_stop()
{
    rtlsdr_close(radio_device);
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
    
    rtlsdr_context_t *radio_ctx = ctx;

    pthread_mutex_lock(&radio_ctx->lock);
    for (uint32_t i=0; i<len-1; i+=2)
    {
        radio_ctx->data[i]   = (float) (buf[i]   - 127.5);
        radio_ctx->data[i+1] = (float) (buf[i+1] - 127.5);
    }
    radio_ctx->data_len = len;
    radio_ctx->new_data_is_available = true;
    pthread_cond_signal(&radio_ctx->cv);
    pthread_mutex_unlock(&radio_ctx->lock);
}

static void *radio_thread_loop(void *ctx)
{
    rtlsdr_context_t *radio_ctx = ctx;

    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);   // REALTIME imposé par cond_timedwait
    deadline.tv_sec += DURATION_S;


    while (true)
    {
        pthread_mutex_lock(&radio_ctx->lock);

        if (!radio_ctx->running)
        {
            pthread_mutex_unlock(&radio_ctx->lock);
            break;
        }

        int rc = 0;
        while (!radio_ctx->new_data_is_available && radio_ctx->running && rc == 0)
        {
            rc = pthread_cond_timedwait(&radio_ctx->cv, &radio_ctx->lock, &deadline);
        }

        if (rc == ETIMEDOUT)
        {
            radio_ctx->running = false; break;
        }

        if (radio_ctx->new_data_is_available)
        {
            double v =  compute_mean_complex_magnitude(radio_ctx->data, radio_ctx->data_len);
            printf("%lf\n", v);
            radio_ctx->new_data_is_available = false;
            pthread_mutex_unlock(&radio_ctx->lock);
        }
    }
    pthread_mutex_unlock(&radio_ctx->lock);
    rtlsdr_cancel_async(radio_device);

    return NULL;
}

static double compute_mean_complex_magnitude(const double *iq_samples, uint32_t len)
{
    double sum = 0;
    for (uint32_t k=0; k<len-1; k += 2)
    {
        double I = iq_samples[k];
        double Q = iq_samples[k+1];
        sum += sqrt(I*I + Q*Q);
    }
    return sum / (len / 2);
}