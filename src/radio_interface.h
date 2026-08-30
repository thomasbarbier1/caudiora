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
 * Public API
 * -------------------------------------------------------------------------------------------------------------------*/
int radio_config(uint32_t central_frequency, uint32_t bandwidth, uint32_t sample_rate);
int radio_init(void);
void* radio_stream_start(void* ctx);
uint32_t radio_stream_stop(void);

#endif //CAUDIORA_RADIO_H
