#ifndef CAUDIORA_RADIO_H
#define CAUDIORA_RADIO_H

#include <pthread.h>
#include "rtl-sdr.h"

#define DEFAULT_DEVICE_INDEX 0
#define DURATION_S           5
#define MAX_BUFFER_SIZE      (uint32_t) 1048576
#define HALF_BUFFER_SIZE     (uint32_t) (MAX_BUFFER_SIZE / 2)

/* ---------------------------------------------------------------------------------------------------------------------
 * Structures
 * -------------------------------------------------------------------------------------------------------------------*/

typedef struct rtlsdr_context
{
    pthread_mutex_t lock;
    pthread_cond_t  cv;
    bool            running;
    bool            new_data_is_available;
    double          data[MAX_BUFFER_SIZE];
    uint32_t        data_len;

} rtlsdr_context_t;

typedef struct rtlsdr_config
{
    uint32_t central_frequency;
    uint32_t bandwidth;
    uint32_t sample_rate;
} rtlsdr_config_t;

/* ---------------------------------------------------------------------------------------------------------------------
 * Global variables
 * -------------------------------------------------------------------------------------------------------------------*/

extern rtlsdr_context_t radio_context;
extern rtlsdr_dev_t *radio_device;

/* ---------------------------------------------------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------------------------------------------------*/
int rtlsdr_init(const rtlsdr_config_t *config);
int rtlsdr_stream_start();
int rtlsdr_stream_stop();

#endif //CAUDIORA_RADIO_H
