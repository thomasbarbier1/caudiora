#ifndef CAUDIORA_RADIO_H
#define CAUDIORA_RADIO_H

#include <pthread.h>
#include "rtl-sdr.h"
#include "ringbuf.h" // https://github.com/szanni/ringbuf/tree/master
#include <semaphore.h>
#include <stdatomic.h>

#define DEFAULT_DEVICE_INDEX 0
#define DURATION_S           5
#define BUFFER_SIZE          (uint32_t) 1048576

/* ---------------------------------------------------------------------------------------------------------------------
 * Structures
 * -------------------------------------------------------------------------------------------------------------------*/

typedef struct radio_data {
    // private data
    atomic_bool     running;
    size_t          buf_size;
    struct ringbuf *input_buf;
    sem_t           semaphore;
    // public data
    uint8_t         output_buf[BUFFER_SIZE];
    uint32_t        overflow_ctn;
} radio_data_t;

/* ---------------------------------------------------------------------------------------------------------------------
 * Global variables
 * -------------------------------------------------------------------------------------------------------------------*/

extern radio_data_t radio_data;
extern rtlsdr_dev_t *radio_device;

/* ---------------------------------------------------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------------------------------------------------*/
int radio_config(uint32_t central_frequency, uint32_t bandwidth, uint32_t sample_rate);
void* radio_stream_start(void* ctx);
uint32_t radio_stream_stop(void);

#endif //CAUDIORA_RADIO_H
