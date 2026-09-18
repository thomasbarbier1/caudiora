#ifndef CAUDIORA_ACQUISITION_H
#define CAUDIORA_ACQUISITION_H

#include <stdint.h>
#include <semaphore.h>

int acquisition_init(void);
static void on_iq_samples(const uint8_t *iq, size_t len, void *user_ctx);
void unpause_acquisition(void);
uint8_t* get_acquisition_buffer(void);
sem_t* get_acquisition_semaphore(void);
void get_buffer_length(uint32_t *length);
double mean_power(const uint8_t *iq, size_t len);

#endif //CAUDIORA_ACQUISITION_H
